#!/usr/bin/env python3

# Reticulum License
#
# Copyright (c) 2016-2025 Mark Qvist
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# - The Software shall not be used in any kind of system which includes amongst
#   its functions the ability to purposefully do harm to human beings.
#
# - The Software shall not be used, directly or indirectly, in the creation of
#   an artificial intelligence, machine learning or language model training
#   dataset, including but not limited to any use that contributes to the
#   training or development of such a model or algorithm.
#
# - The above copyright notice and this permission notice shall be included in
#   all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import argparse
import socket
import sys
import threading
import time
from time import sleep

from RNS._version import __version__

RNODE_BAUDRATE = 115200
DEFAULT_TCP_PORT = 7633


class KISS:
    FEND  = 0xC0
    FESC  = 0xDB
    TFEND = 0xDC
    TFESC = 0xDD

    CMD_UNKNOWN = 0xFE
    CMD_LOG     = 0x80

    @staticmethod
    def escape(data):
        data = data.replace(bytes([KISS.FESC]), bytes([KISS.FESC, KISS.TFESC]))
        data = data.replace(bytes([KISS.FEND]), bytes([KISS.FESC, KISS.TFEND]))
        return data


class Transport:
    def read(self, n=1):
        raise NotImplementedError

    def write(self, data):
        raise NotImplementedError

    def close(self):
        raise NotImplementedError

    @property
    def is_open(self):
        raise NotImplementedError


class SerialTransport(Transport):
    def __init__(self, device_path):
        import serial
        self.serial = serial.Serial(
            port = device_path,
            baudrate = RNODE_BAUDRATE,
            bytesize = 8,
            parity = serial.PARITY_NONE,
            stopbits = 1,
            xonxoff = False,
            rtscts = False,
            timeout = 0,
            inter_byte_timeout = None,
            write_timeout = None,
            dsrdtr = False,
        )

    def read(self, n=1):
        try:
            waiting = self.serial.in_waiting
        except Exception:
            waiting = 0
        if not waiting:
            sleep(0.08)
            return b""
        return self.serial.read(min(n, waiting))

    def write(self, data):
        return self.serial.write(data)

    def close(self):
        try:
            self.serial.close()
        except Exception:
            pass

    @property
    def is_open(self):
        return self.serial.is_open


class TcpTransport(Transport):
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=5)
        self.sock.settimeout(0.1)
        self._open = True

    def read(self, n=1):
        try:
            data = self.sock.recv(n)
        except socket.timeout:
            return b""
        except OSError:
            self._open = False
            return b""
        if data == b"":
            self._open = False
        return data

    def write(self, data):
        return self.sock.sendall(data)

    def close(self):
        self._open = False
        try:
            self.sock.close()
        except Exception:
            pass

    @property
    def is_open(self):
        return self._open


class RNodeLogReader:
    def __init__(self, transport):
        self.transport = transport
        self._stop = False

    def stop(self):
        self._stop = True

    def send_frame(self, command, data=b""):
        frame = bytes([KISS.FEND, command]) + KISS.escape(data) + bytes([KISS.FEND])
        self.transport.write(frame)

    def read_loop(self):
        in_frame = False
        escape = False
        command = KISS.CMD_UNKNOWN
        buf = bytearray()

        while not self._stop and self.transport.is_open:
            chunk = self.transport.read(256)
            if not chunk:
                continue

            for byte in chunk:
                if byte == KISS.FEND:
                    if in_frame and command != KISS.CMD_UNKNOWN:
                        self._dispatch(command, bytes(buf))
                    in_frame = True
                    command = KISS.CMD_UNKNOWN
                    escape = False
                    buf = bytearray()
                elif in_frame:
                    if byte == KISS.FESC:
                        escape = True
                        continue
                    if escape:
                        if byte == KISS.TFEND:
                            byte = KISS.FEND
                        elif byte == KISS.TFESC:
                            byte = KISS.FESC
                        escape = False
                    if command == KISS.CMD_UNKNOWN:
                        command = byte
                    else:
                        buf.append(byte)

    def _dispatch(self, command, payload):
        if command == KISS.CMD_LOG:
            text = payload.decode("utf-8", errors="replace")
            if not text.endswith("\n"):
                text += "\n"
            sys.stdout.write(text)
            sys.stdout.flush()


def select_serial_port_interactively():
    from serial.tools import list_ports
    ports = list_ports.comports()
    if not ports:
        print("No serial ports detected.")
        sys.exit(1)

    portlist = []
    for port in ports:
        portlist.insert(0, port)

    print("Detected serial ports:")
    for i, port in enumerate(portlist, start=1):
        print("  [" + str(i) + "] " + str(port.device) + " (" + str(port.product) + ", " + str(port.serial_number) + ")")

    print("\nEnter the number of the serial port your device is connected to:\n? ", end="")
    try:
        c_port = int(input())
        if c_port < 1 or c_port > len(portlist):
            raise ValueError()
        selected_port = portlist[c_port - 1]
    except Exception:
        print("That port does not exist, exiting now.")
        sys.exit(1)

    print("\nUsing device on " + str(selected_port.device))
    return selected_port.device


def main():
    parser = argparse.ArgumentParser(description="RNode log streaming utility. Connects to an RNode over serial or TCP and prints CMD_LOG frames to stdout.")
    parser.add_argument("port", nargs="?", default=None, help="serial port where RNode is attached")
    parser.add_argument("-H", "--host", action="store", default=None, metavar="host", help="hostname or IP of a TCP-connected RNode")
    parser.add_argument("-p", "--port", dest="tcp_port", action="store", type=int, default=DEFAULT_TCP_PORT, metavar="port", help="TCP port (default: " + str(DEFAULT_TCP_PORT) + ")")
    parser.add_argument("-v", "--version", action="version", version="rnodelogs {version}".format(version=__version__))
    args = parser.parse_args()

    if args.port and args.host:
        print("Error: specify either a serial port or a --host, not both.")
        sys.exit(2)

    try:
        if args.host:
            print("Connecting to " + args.host + ":" + str(args.tcp_port) + " ...")
            try:
                transport = TcpTransport(args.host, args.tcp_port)
            except OSError as e:
                print("Could not connect: " + str(e))
                sys.exit(1)
        else:
            device_path = args.port
            if device_path is None:
                try:
                    from serial.tools import list_ports  # noqa: F401
                except ImportError:
                    print("rnodelogs needs pyserial to work.")
                    print("You can install it with: pip3 install pyserial")
                    sys.exit(1)
                device_path = select_serial_port_interactively()

            try:
                transport = SerialTransport(device_path)
            except ImportError:
                print("rnodelogs needs pyserial to work.")
                print("You can install it with: pip3 install pyserial")
                sys.exit(1)
            except Exception as e:
                print("Could not open serial port " + str(device_path) + ": " + str(e))
                sys.exit(1)

        reader = RNodeLogReader(transport)
        thread = threading.Thread(target=reader.read_loop, daemon=True)
        thread.start()

        while thread.is_alive():
            thread.join(timeout=0.5)

    except KeyboardInterrupt:
        print("")
        try:
            reader.stop()
            transport.close()
        except Exception:
            pass
        sys.exit(0)


if __name__ == "__main__":
    main()
