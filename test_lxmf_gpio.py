#!/usr/bin/env python3
"""
test_lxmf_gpio.py — LXMF test client for ESP32 GPIO control node

Sends LXMF messages to the GPIO node and displays responses.
Can be used from any machine running Reticulum (laptop, RPi, etc.)
that shares a transport path with the ESP32 (e.g. via shared RNode,
serial interface, or TCP transport).

Usage:
    python3 test_lxmf_gpio.py <gpio_node_address> [command]

Examples:
    python3 test_lxmf_gpio.py a1b2c3d4e5f6... HELP
    python3 test_lxmf_gpio.py a1b2c3d4e5f6... "SET 13 HIGH"
    python3 test_lxmf_gpio.py a1b2c3d4e5f6... STATUS
    python3 test_lxmf_gpio.py a1b2c3d4e5f6...           # interactive mode

Requirements:
    pip install rns lxmf
"""

import sys
import os
import time
import RNS
import LXMF


class GPIOTestClient:
    def __init__(self, destination_hex):
        self.destination_hex = destination_hex
        self.received_response = None

        # Initialize Reticulum
        self.reticulum = RNS.Reticulum()

        # Create our own identity (ephemeral for testing)
        identity_path = os.path.expanduser("~/.reticulum/gpio_test_identity")
        if os.path.exists(identity_path):
            self.identity = RNS.Identity.from_file(identity_path)
            RNS.log(f"Loaded identity from {identity_path}")
        else:
            self.identity = RNS.Identity()
            self.identity.to_file(identity_path)
            RNS.log(f"Created new identity, saved to {identity_path}")

        # Create LXMF router
        self.router = LXMF.LXMRouter(identity=self.identity, storagepath=os.path.expanduser("~/.reticulum/gpio_test_lxmf"))
        self.router.register_delivery_identity(self.identity, display_name="GPIO Test Client")
        self.router.register_delivery_callback(self.on_message)

        # Announce ourselves so the ESP32 can reply
        self.router.announce(self.router.delivery_destinations[list(self.router.delivery_destinations.keys())[0]].hash)

        # Parse destination hash
        self.dest_hash = bytes.fromhex(destination_hex)

        RNS.log(f"Test client ready")
        RNS.log(f"Our address: {RNS.prettyhexrep(list(self.router.delivery_destinations.keys())[0])}")
        RNS.log(f"Target GPIO node: {RNS.prettyhexrep(self.dest_hash)}")

    def on_message(self, message):
        """Callback for incoming LXMF messages (responses from GPIO node)"""
        sender = RNS.prettyhexrep(message.source_hash)
        timestamp = time.strftime("%H:%M:%S", time.localtime(message.timestamp))

        content = ""
        if message.content:
            if isinstance(message.content, bytes):
                content = message.content.decode("utf-8", errors="replace")
            else:
                content = str(message.content)

        print(f"\n{'='*60}")
        print(f"  Response from {sender}")
        print(f"  Time: {timestamp}")
        print(f"{'='*60}")
        print(f"  {content}")
        print(f"{'='*60}\n")

        self.received_response = content

    def send_command(self, command, timeout=30):
        """Send a GPIO command via LXMF and wait for response"""
        self.received_response = None

        # Look up destination identity
        dest_identity = RNS.Identity.recall(self.dest_hash)

        if dest_identity is None:
            # Request path and wait
            RNS.log(f"Destination identity not known, requesting path...")
            RNS.Transport.request_path(self.dest_hash)

            # Wait for path resolution
            wait_start = time.time()
            while dest_identity is None and time.time() - wait_start < 15:
                time.sleep(0.5)
                dest_identity = RNS.Identity.recall(self.dest_hash)

            if dest_identity is None:
                print(f"ERROR: Could not resolve identity for {self.destination_hex}")
                print(f"Make sure the GPIO node is running and has announced.")
                print(f"Try waiting for an announce cycle (up to 5 minutes).")
                return None

        # Create destination
        destination = RNS.Destination(
            dest_identity,
            RNS.Destination.OUT,
            RNS.Destination.SINGLE,
            "lxmf",
            "delivery"
        )

        # Create LXMF message
        lxm = LXMF.LXMessage(
            destination,
            list(self.router.delivery_destinations.values())[0],  # our delivery destination as source
            command.encode("utf-8"),
            title=b"",
            desired_method=LXMF.LXMessage.OPPORTUNISTIC
        )

        # Send
        print(f">> Sending: {command}")
        self.router.handle_outbound(lxm)

        # Wait for response
        print(f"   Waiting for response (timeout: {timeout}s)...")
        wait_start = time.time()
        while self.received_response is None and time.time() - wait_start < timeout:
            time.sleep(0.2)

        if self.received_response is None:
            print(f"   (no response within {timeout}s)")
            return None

        return self.received_response

    def interactive(self):
        """Interactive command mode"""
        print()
        print("=" * 60)
        print("  LXMF GPIO Control — Interactive Mode")
        print("=" * 60)
        print(f"  Target: {self.destination_hex}")
        print(f"  Commands: HELP, SET, GET, MODE, STATUS, PINS")
        print(f"  Type 'quit' to exit")
        print("=" * 60)
        print()

        while True:
            try:
                cmd = input("gpio> ").strip()
                if not cmd:
                    continue
                if cmd.lower() in ("quit", "exit", "q"):
                    break
                self.send_command(cmd)
            except (KeyboardInterrupt, EOFError):
                print()
                break

        print("Goodbye!")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 test_lxmf_gpio.py <gpio_node_address> [command]")
        print()
        print("  gpio_node_address: The hex LXMF address of the ESP32 GPIO node")
        print("                     (shown on serial output when the node boots)")
        print()
        print("Examples:")
        print('  python3 test_lxmf_gpio.py a1b2c3d4... HELP')
        print('  python3 test_lxmf_gpio.py a1b2c3d4... "SET 13 HIGH"')
        print('  python3 test_lxmf_gpio.py a1b2c3d4...   # interactive mode')
        sys.exit(1)

    dest_hex = sys.argv[1].strip().replace(" ", "").replace(":", "")

    if len(dest_hex) != 32:
        print(f"ERROR: Address must be 32 hex characters (16 bytes), got {len(dest_hex)}")
        sys.exit(1)

    client = GPIOTestClient(dest_hex)

    if len(sys.argv) > 2:
        # Single command mode
        command = " ".join(sys.argv[2:])
        client.send_command(command)
    else:
        # Interactive mode
        client.interactive()


if __name__ == "__main__":
    main()
