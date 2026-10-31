#!/usr/bin/env python3
"""MASN V1.2 configuration program.

Run without arguments for the interactive wizard:

    python masn/masn_setup.py

It configures, over USB:
  * radio parameters (defaults: 869.618 MHz, 62.5 kHz, SF5, CR5, 7 dBm),
  * operating mode: standalone Reticulum transport node or RNode,
  * node name and NomadNet page (description, contact, location),
  * visibility on interface-discovery maps such as https://rmap.world
    (location, operator LXMF address, announce interval).

Non-interactive sub-commands for scripting:

    python masn/masn_setup.py status|hash|rnode|tnc|show [--port P] [radio options]

Requirements: pip install pyserial rns lxmf
"""
import argparse
import hashlib
import json
import locale
import os
import struct
import sys
import threading
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial is required: pip install pyserial")

try:
    from RNS.vendor import umsgpack as msgpack
except ImportError:
    sys.exit("rns is required: pip install rns lxmf")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BIN = os.path.join(ROOT, ".pio/build/masn-ht-ra62/rnode_firmware_masn_ht_ra62.bin")

# ---------------------------------------------------------------------------
# Texts (Spanish / English)
# ---------------------------------------------------------------------------
LANG = "es" if (os.environ.get("LANG", "") or (locale.getlocale()[0] or "")).lower().startswith("es") else "en"
TXT = {
    "title":        {"es": "Configuración de nodo MASN V1.2", "en": "MASN V1.2 node configuration"},
    "port":         {"es": "Puerto", "en": "Port"},
    "no_board":     {"es": "No se encuentra la placa. Conéctala por USB o usa --port.", "en": "Board not found. Connect it over USB or use --port."},
    "no_reply":     {"es": "La placa no responde.", "en": "The board does not reply."},
    "hw_bad":       {"es": "El firmware no está validado (HW=0). ¿Grabar el hash del firmware compilado?", "en": "Firmware not validated (HW=0). Write the hash of the built firmware?"},
    "radio_title":  {"es": "Radio", "en": "Radio"},
    "radio_now":    {"es": "Parámetros de radio", "en": "Radio parameters"},
    "radio_change": {"es": "¿Quieres cambiarlos?", "en": "Do you want to change them?"},
    "radio_match":  {"es": "Deben coincidir con los del resto de nodos de tu red.", "en": "They must match the other nodes of your network."},
    "freq":         {"es": "Frecuencia (Hz)", "en": "Frequency (Hz)"},
    "bw":           {"es": "Ancho de banda (Hz)", "en": "Bandwidth (Hz)"},
    "sf":           {"es": "Spreading factor (5-12)", "en": "Spreading factor (5-12)"},
    "cr":           {"es": "Coding rate 4/N (5-8)", "en": "Coding rate 4/N (5-8)"},
    "txp":          {"es": "Potencia TX (dBm, máx 22)", "en": "TX power (dBm, max 22)"},
    "warn_eu_g3":   {"es": "Aviso: en 869,40-869,65 MHz se permiten 500 mW PRA con ciclo de trabajo del 10 %.", "en": "Note: 869.40-869.65 MHz allows 500 mW e.r.p. at a 10 % duty cycle."},
    "warn_eu_14":   {"es": "Aviso: fuera de 869,40-869,65 MHz la banda EU868 suele limitarse a 25 mW (14 dBm) y 1 % de ciclo de trabajo.", "en": "Note: outside 869.40-869.65 MHz, EU868 is usually limited to 25 mW (14 dBm) and a 1 % duty cycle."},
    "mode_title":   {"es": "Modo de funcionamiento", "en": "Operating mode"},
    "mode_q":       {"es": "¿Nodo Reticulum autónomo (s) o RNode para un ordenador (n)?", "en": "Standalone Reticulum node (y) or RNode for a host computer (n)?"},
    "site_title":   {"es": "Nombre y página NomadNet", "en": "Name and NomadNet page"},
    "name":         {"es": "Nombre del nodo", "en": "Node name"},
    "page_q":       {"es": "¿Publicar una página NomadNet con información del nodo?", "en": "Publish a NomadNet page with node information?"},
    "desc":         {"es": "Descripción (una línea, opcional, '-' para vaciar)", "en": "Description (one line, optional, '-' to clear)"},
    "contact":      {"es": "Contacto del operador (opcional, '-' para vaciar)", "en": "Operator contact (optional, '-' to clear)"},
    "vis_title":    {"es": "Visibilidad en mapas (rmap.world)", "en": "Map visibility (rmap.world)"},
    "vis_q":        {"es": "¿Hacer el nodo visible en mapas como rmap.world?", "en": "Make the node visible on maps such as rmap.world?"},
    "vis_note":     {"es": "El nodo enviará un anuncio de 'interface discovery' con su ubicación y radio. Solo llegará al mapa si tu red LoRa enlaza (por algún nodo de transporte) con Internet.", "en": "The node will send an 'interface discovery' announce with its location and radio. It only reaches the map if your LoRa network is bridged (through some transport node) to the Internet."},
    "loc":          {"es": "Ubicación 'latitud, longitud' en grados decimales con punto (p. ej. 42.8125, -1.6458; se puede pegar desde un mapa)", "en": "Location 'latitude, longitude' in decimal degrees with a dot (e.g. 42.8125, -1.6458; can be pasted from a map)"},
    "alt":          {"es": "Altitud (m)", "en": "Altitude (m)"},
    "loc_page_q":   {"es": "¿Mostrar también la ubicación en la página NomadNet?", "en": "Also show the location on the NomadNet page?"},
    "lxmf":         {"es": "Dirección LXMF del operador (32 hex, opcional, '-' = ninguna)", "en": "Operator LXMF address (32 hex, optional, '-' = none)"},
    "interval":     {"es": "Intervalo de anuncio (horas, 1-24; rmap penaliza más de 3/hora)", "en": "Announce interval (hours, 1-24; rmap penalises more than 3/hour)"},
    "summary":      {"es": "Resumen", "en": "Summary"},
    "apply_q":      {"es": "¿Aplicar esta configuración?", "en": "Apply this configuration?"},
    "cancel":       {"es": "Cancelado, no se ha cambiado nada.", "en": "Cancelled, nothing was changed."},
    "applying":     {"es": "Aplicando", "en": "Applying"},
    "stamp":        {"es": "Generando el sello (prueba de trabajo) del anuncio de discovery", "en": "Generating the discovery announce stamp (proof of work)"},
    "rebooting":    {"es": "Reiniciando la placa", "en": "Rebooting the board"},
    "waiting":      {"es": "Esperando a que arranque Reticulum en la placa", "en": "Waiting for Reticulum to start on the board"},
    "verify":       {"es": "Comprobando el arranque", "en": "Checking the boot"},
    "stored_ok":    {"es": "Configuración guardada y comprobada tras el reinicio.", "en": "Configuration stored and verified after the reboot."},
    "stored_bad":   {"es": "La placa no conservó la configuración; se vuelve a grabar", "en": "The board did not keep the configuration; writing it again"},
    "stored_fail":  {"es": "La placa no conserva la configuración tras 3 intentos. Revisa el log por USB.", "en": "The board does not keep the configuration after 3 attempts. Check the USB log."},
    "announce_info":{"es": "El nodo anuncia su sitio NomadNet al arrancar y cada {h} h; el anuncio para rmap.world (si es visible) sale 2 min después de arrancar y cada {h} h.", "en": "The node announces its NomadNet site at boot and every {h} h; the rmap.world announce (if visible) goes out 2 min after boot and every {h} h."},
    "done":         {"es": "Listo.", "en": "Done."},
    "saved_prof":   {"es": "Perfil guardado en", "en": "Profile saved to"},
    "nomad_addr":   {"es": "Dirección NomadNet del nodo", "en": "Node NomadNet address"},
    "transport_id": {"es": "Identidad de transporte", "en": "Transport identity"},
    "yes":          {"es": "s", "en": "y"},
    "invalid":      {"es": "Valor no válido", "en": "Invalid value"},
    "need_lxmf":    {"es": "Para la visibilidad en mapas hace falta el módulo lxmf: pip install lxmf", "en": "Map visibility needs the lxmf module: pip install lxmf"},
    "old_fw":       {"es": "Este firmware no tiene los campos de sitio/discovery. Flashea la versión actual.", "en": "This firmware lacks the site/discovery fields. Flash the current version."},
    "mode_node":    {"es": "nodo autónomo", "en": "standalone node"},
    "mode_rnode":   {"es": "RNode", "en": "RNode"},
    "yes_word":     {"es": "sí", "en": "yes"},
    "no_word":      {"es": "no", "en": "no"},
}


def t(key):
    return TXT[key][LANG]


# ---------------------------------------------------------------------------
# KISS / RNode protocol
# ---------------------------------------------------------------------------
FEND, FESC, TFEND, TFESC = 0xC0, 0xDB, 0xDC, 0xDD
CMD_FREQUENCY, CMD_BANDWIDTH, CMD_TXPOWER, CMD_SF, CMD_CR = 0x01, 0x02, 0x03, 0x04, 0x05
CMD_RADIO_STATE, CMD_DETECT, CMD_LOG = 0x06, 0x08, 0x80
CMD_CONF_SAVE, CMD_CONF_DELETE, CMD_RESET, CMD_FW_HASH = 0x53, 0x54, 0x55, 0x58
CMD_PROVISION_REQ, CMD_PROVISION_RSP = 0x86, 0x87

# Provisioning (see Provisioning.h and microReticulum Provisioning/Ops.h)
OP_GET_INFO, OP_GET_STATE, OP_SET_STATE, OP_COMMIT, OP_ERROR = 2, 4, 5, 6, 101
NS_GENERAL, NS_ADDRS = 100, 107
F_NOMAD_ENABLE, F_NOMAD_NAME, F_SITE_DESC, F_SITE_CONTACT = 4, 5, 20, 21
F_LAT, F_LON, F_ALT, F_LOC_PUBLIC = 22, 23, 24, 25
F_DISCOVERY, F_DISCOVERY_DATA, F_ANNOUNCE_INT = 26, 27, 28
F_ADDR_TRANSPORT, F_ADDR_NOMAD = 1, 4

# Interface discovery announce (RNS/Discovery.py)
D_NAME, D_TRANSPORT_ID, D_IMPL, D_VERS = 0xFF, 0xFE, 0xFD, 0xFC
D_TYPE, D_TRANSPORT, D_LAT, D_LON, D_HEIGHT = 0x00, 0x01, 0x03, 0x04, 0x05
D_FREQ, D_BW, D_SF, D_CR, D_OP_ADDR = 0x09, 0x0A, 0x0B, 0x0C, 0xF0
DISCOVERY_STAMP_COST, DISCOVERY_EXPAND_ROUNDS = 16, 20

DEFAULT_RADIO = {"freq": 869618000, "bw": 62500, "sf": 5, "cr": 5, "txp": 7}


def kiss_escape(data):
    return bytes(data).replace(b"\xdb", b"\xdb\xdd").replace(b"\xc0", b"\xdb\xdc")


class Board:
    """KISS connection to the board.

    While the port is open a background thread drains it continuously: the
    firmware logs a lot over USB, and if the host keeps the port open without
    reading, the USB buffer fills, the firmware blocks on Serial.write() and
    its 60 s watchdog reboots the board. Close the port while waiting for user
    input.
    """

    def __init__(self, port):
        self.port = port
        self.ser = None
        self.buf = bytearray()
        self.lock = threading.Lock()
        self.reader = None
        self.seq = int(time.time()) & 0xFFFF
        self.open()

    def open(self, wait=15.0):
        end = time.monotonic() + wait
        while True:
            try:
                self.ser = serial.Serial(self.port, 115200, timeout=0.1)
                break
            except (serial.SerialException, OSError):
                if time.monotonic() > end:
                    raise
                time.sleep(0.2)
                self.port = find_port(None, self.port)   # may come back renamed
        time.sleep(0.5)
        self.ser.reset_input_buffer()
        with self.lock:
            self.buf.clear()
        self.reader = threading.Thread(target=self._read_loop, args=(self.ser,), daemon=True)
        self.reader.start()

    def _read_loop(self, ser):
        while self.ser is ser:
            try:
                chunk = ser.read(4096)
            except (serial.SerialException, OSError, TypeError, AttributeError):
                break
            if chunk:
                with self.lock:
                    self.buf.extend(chunk)
                    if len(self.buf) > 1 << 20:      # nobody consuming: keep the tail
                        del self.buf[:-65536]

    def close(self):
        ser, self.ser = self.ser, None
        if ser:
            if self.reader:
                self.reader.join(timeout=1)
            ser.close()

    def send(self, cmd, payload=b""):
        try:
            self.ser.write(bytes([FEND, cmd]) + kiss_escape(payload) + bytes([FEND]))
            self.ser.flush()
        except (serial.SerialException, OSError, AttributeError):
            # The board rebooted under us: reconnect and retry once.
            self.close()
            self.reconnect()
            self.ser.write(bytes([FEND, cmd]) + kiss_escape(payload) + bytes([FEND]))
            self.ser.flush()

    def frames(self, timeout):
        """Yield (cmd, data) KISS frames received within timeout seconds."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            frame = None
            with self.lock:
                start = self.buf.find(FEND)
                if start < 0:
                    self.buf.clear()
                else:
                    stop = self.buf.find(FEND, start + 1)
                    if stop < 0:
                        del self.buf[:start]
                    else:
                        frame = bytes(self.buf[start + 1:stop])
                        del self.buf[:stop]
            if frame is None:
                time.sleep(0.02)
                continue
            if frame:
                data = frame[1:].replace(b"\xdb\xdc", b"\xc0").replace(b"\xdb\xdd", b"\xdb")
                yield frame[0], data

    # --- RNode commands ---------------------------------------------------
    def status(self, timeout=3.0, tries=3):
        # The board can be busy for a second or two (e.g. validating an
        # announce signature), so retry before giving up.
        for _ in range(tries):
            self.send(CMD_DETECT, b"\x60")
            for cmd, data in self.frames(timeout):
                if cmd == 0x60 and data.startswith(b"V="):
                    return dict(kv.split("=", 1) for kv in data.decode(errors="replace").split(";") if "=" in kv)
        return None

    def set_radio(self, r, on=True):
        self.send(CMD_FREQUENCY, struct.pack(">I", r["freq"])); time.sleep(0.2)
        self.send(CMD_BANDWIDTH, struct.pack(">I", r["bw"])); time.sleep(0.2)
        self.send(CMD_TXPOWER, bytes([r["txp"]])); time.sleep(0.2)
        self.send(CMD_SF, bytes([r["sf"]])); time.sleep(0.2)
        self.send(CMD_CR, bytes([r["cr"]])); time.sleep(0.2)
        if on:
            self.send(CMD_RADIO_STATE, b"\x01"); time.sleep(1.0)

    def reboot(self):
        self.send(CMD_RESET, b"\xf8")
        time.sleep(0.3)
        self.close()
        self.reconnect()

    def reconnect(self):
        """Wait for the USB port to drop (if it is going to) and come back."""
        end = time.monotonic() + 5
        while os.path.exists(self.port) and time.monotonic() < end:
            time.sleep(0.05)
        time.sleep(1.0)
        self.port = find_port(None, self.port)
        self.open(wait=30)

    def write_hash(self, path=BIN):
        digest = hashlib.sha256(open(path, "rb").read()).digest()
        self.send(CMD_FW_HASH, digest)
        time.sleep(2)
        return digest.hex()

    # --- Provisioning -----------------------------------------------------
    def prov(self, op, payload=None, timeout=8.0):
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        req = [op, self.seq] + ([payload] if payload is not None else [])
        self.send(CMD_PROVISION_REQ, msgpack.packb(req))
        for cmd, data in self.frames(timeout):
            if cmd != CMD_PROVISION_RSP:
                continue
            rsp = msgpack.unpackb(data)
            if len(rsp) < 2 or rsp[1] != self.seq:
                continue
            body = rsp[2] if len(rsp) > 2 else None
            if rsp[0] == OP_ERROR:
                raise RuntimeError(f"provisioning error: {body}")
            return body
        raise TimeoutError("no provisioning response")

    def wait_ready(self, timeout=90.0):
        """Provisioning starts with RNS, up to ~60 s after boot."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                return self.prov(OP_GET_INFO, {}, timeout=3.0)
            except (TimeoutError, RuntimeError):
                print(".", end="", flush=True)
            except (serial.SerialException, OSError):
                self.close()
                self.reconnect()
        raise TimeoutError("provisioning not available")

    def get_state(self, namespaces):
        body = self.prov(OP_GET_STATE, {1: list(namespaces)})
        return body.get(1, {}) if isinstance(body, dict) else {}

    def set_state(self, values, ns=NS_GENERAL, max_bytes=400):
        """SetState in chunks: the nRF52 request buffer is 512 bytes."""
        chunk, errors = {}, {}
        items = list(values.items())
        for i, (fid, val) in enumerate(items):
            trial = {**chunk, fid: val}
            if chunk and len(msgpack.packb({3: {ns: trial}})) > max_bytes:
                errors.update(self._set_chunk(ns, chunk))
                chunk = {}
            chunk[fid] = val
        if chunk:
            errors.update(self._set_chunk(ns, chunk))
        return errors

    def _set_chunk(self, ns, chunk):
        body = self.prov(OP_SET_STATE, {3: {ns: chunk}})
        errs = body.get(3) if isinstance(body, dict) else None
        return {"set": errs} if errs else {}

    def commit(self):
        return self.prov(OP_COMMIT, {})

    def info(self):
        return self.prov(OP_GET_INFO, {})


# ---------------------------------------------------------------------------
# Interface discovery announce data
# ---------------------------------------------------------------------------
def build_discovery_app_data(transport_id, fw_version, cfg):
    """flags(1) + msgpack(info) + LXMF stamp, as RNS/Discovery.py builds it."""
    from LXMF import LXStamper
    import RNS
    info = {
        D_TYPE: "RNodeInterface",            # LoRa interface, carries radio params
        D_TRANSPORT: cfg["mode"] == "tnc",
        D_TRANSPORT_ID: transport_id,
        D_IMPL: "microReticulum",
        D_VERS: fw_version,
        D_NAME: cfg["name"],
        D_LAT: float(cfg["lat"]),
        D_LON: float(cfg["lon"]),
        D_HEIGHT: float(cfg["alt"]),
        D_FREQ: cfg["radio"]["freq"],
        D_BW: cfg["radio"]["bw"],
        D_SF: cfg["radio"]["sf"],
        D_CR: cfg["radio"]["cr"],
    }
    if cfg.get("lxmf"):
        info[D_OP_ADDR] = bytes.fromhex(cfg["lxmf"])
    packed = msgpack.packb(info)
    infohash = RNS.Identity.full_hash(packed)
    stamp, value = LXStamper.generate_stamp(infohash, stamp_cost=DISCOVERY_STAMP_COST,
                                            expand_rounds=DISCOVERY_EXPAND_ROUNDS)
    workblock = LXStamper.stamp_workblock(infohash, expand_rounds=DISCOVERY_EXPAND_ROUNDS)
    if not LXStamper.stamp_valid(stamp, DISCOVERY_STAMP_COST, workblock):
        raise RuntimeError("generated stamp does not validate")
    return b"\x00" + packed + stamp


# ---------------------------------------------------------------------------
# Interactive helpers
# ---------------------------------------------------------------------------
def ask(prompt, default=None, conv=str, check=None):
    while True:
        shown = f" [{default}]" if default not in (None, "") else ""
        raw = input(f"  {prompt}{shown}: ").strip()
        if not raw:
            if default is None:
                continue
            raw = str(default)
        if raw == "-" and conv is str:      # '-' clears an optional text field
            raw = ""
        try:
            val = conv(raw)
            if check is None or check(val):
                return val
        except ValueError:
            pass
        print(f"    {t('invalid')}")


def ask_yes(prompt, default=False):
    d = t("yes").upper() + "/n" if default else t("yes") + "/N"
    raw = input(f"  {prompt} [{d}]: ").strip().lower()
    if not raw:
        return default
    return raw[0] in ("s", "y")


def section(title):
    print(f"\n== {title} ==")


def radio_str(r):
    return (f"{r['freq'] / 1e6:.3f} MHz, BW {r['bw'] / 1e3:g} kHz, SF{r['sf']}, "
            f"CR4/{r['cr']}, {r['txp']} dBm")


def radio_warnings(r):
    if 869400000 <= r["freq"] <= 869650000:
        print(f"    {t('warn_eu_g3')}")
    elif 863000000 <= r["freq"] <= 870000000 and r["txp"] > 14:
        print(f"    {t('warn_eu_14')}")


def find_port(arg, fallback=None):
    """Explicit port, else the first USB device with the nice!nano VID."""
    if arg:
        return arg
    ports = [p.device for p in serial.tools.list_ports.comports() if p.vid == 0x239A]
    if ports:
        return ports[0]
    if fallback:
        return fallback
    sys.exit(t("no_board"))


def parse_location(text):
    """'lat, lon' (also 'lat lon' or 'lat;lon') -> (lat, lon) in degrees."""
    parts = [p for p in text.replace(";", " ").replace(",", " ").split() if p]
    if len(parts) != 2:
        raise ValueError("expected two numbers")
    lat, lon = float(parts[0]), float(parts[1])
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        raise ValueError("out of range")
    return lat, lon


def is_hex32(s):
    return len(s) == 32 and all(c in "0123456789abcdef" for c in s.lower())


# ---------------------------------------------------------------------------
# Wizard
# ---------------------------------------------------------------------------
def wizard(args):
    print(f"\n{t('title')}\n" + "=" * len(t("title")))
    port = find_port(args.port)
    print(f"  {t('port')}: {port}")
    board = Board(port)

    st = board.status()
    if st is None:
        sys.exit(t("no_reply"))
    if st.get("HW") != "1":
        if ask_yes(t("hw_bad"), True):
            print("  hash", board.write_hash())
            board.reboot()
            st = board.status() or st

    # Current provisioning values (defaults for the questions)
    print(f"  {t('waiting')}", end="", flush=True)
    board.wait_ready()
    print(" OK")
    try:
        state = board.get_state([NS_GENERAL, NS_ADDRS])
    except (RuntimeError, TimeoutError):
        state = {}
    gen = state.get(NS_GENERAL, {})
    if F_DISCOVERY_DATA not in gen and F_LAT not in gen:
        sys.exit(t("old_fw"))
    addrs = state.get(NS_ADDRS, {})
    transport_id = addrs.get(F_ADDR_TRANSPORT, b"")
    board.close()       # never keep the port open while waiting for input

    profile = {}
    if args.profile and os.path.exists(args.profile):
        profile = json.load(open(args.profile))

    cfg = {"radio": dict(DEFAULT_RADIO, **profile.get("radio", {}))}

    # --- Radio
    section(t("radio_title"))
    print(f"  {t('radio_now')}: {radio_str(cfg['radio'])}")
    print(f"  {t('radio_match')}")
    radio_warnings(cfg["radio"])
    if ask_yes(t("radio_change"), False):
        r = cfg["radio"]
        r["freq"] = ask(t("freq"), r["freq"], int, lambda v: 137000000 <= v <= 1020000000)
        r["bw"] = ask(t("bw"), r["bw"], int, lambda v: 7800 <= v <= 500000)
        r["sf"] = ask(t("sf"), r["sf"], int, lambda v: 5 <= v <= 12)
        r["cr"] = ask(t("cr"), r["cr"], int, lambda v: 5 <= v <= 8)
        r["txp"] = ask(t("txp"), r["txp"], int, lambda v: -9 <= v <= 22)
        radio_warnings(r)

    # --- Mode
    section(t("mode_title"))
    cfg["mode"] = "tnc" if ask_yes(t("mode_q"), profile.get("mode", "tnc") == "tnc") else "rnode"

    # --- Name and page
    section(t("site_title"))
    cfg["name"] = ask(t("name"), profile.get("name") or gen.get(F_NOMAD_NAME) or "MASN node",
                      str, lambda v: 0 < len(v) <= 63)
    cfg["page"] = ask_yes(t("page_q"), profile.get("page", bool(gen.get(F_NOMAD_ENABLE, True))))
    cfg["desc"] = cfg["contact"] = ""
    if cfg["page"]:
        cfg["desc"] = ask(t("desc"), profile.get("desc", gen.get(F_SITE_DESC, "")) or "", str, lambda v: len(v) <= 160)
        cfg["contact"] = ask(t("contact"), profile.get("contact", gen.get(F_SITE_CONTACT, "")) or "", str, lambda v: len(v) <= 64)

    # --- Visibility
    section(t("vis_title"))
    print(f"  {t('vis_note')}")
    cfg["visible"] = ask_yes(t("vis_q"), profile.get("visible", bool(gen.get(F_DISCOVERY, False))))
    cfg["lat"] = cfg["lon"] = cfg["alt"] = 0.0
    cfg["loc_page"], cfg["lxmf"], cfg["interval_h"] = False, "", profile.get("interval_h", 6)
    if cfg["visible"]:
        try:
            import LXMF  # noqa: F401
        except ImportError:
            sys.exit(t("need_lxmf"))
        lat0 = profile.get("lat", gen.get(F_LAT) or None)
        lon0 = profile.get("lon", gen.get(F_LON) or None)
        default_loc = f"{lat0:.6f}, {lon0:.6f}" if lat0 is not None and lon0 is not None else None
        cfg["lat"], cfg["lon"] = ask(t("loc"), default_loc, parse_location)
        cfg["alt"] = ask(t("alt"), profile.get("alt", gen.get(F_ALT, 0.0)), float, lambda v: -500 <= v <= 10000)
        if cfg["page"]:
            cfg["loc_page"] = ask_yes(t("loc_page_q"), profile.get("loc_page", True))
        cfg["lxmf"] = ask(t("lxmf"), profile.get("lxmf", "") or "-", str,
                          lambda v: v == "" or is_hex32(v)).lower()
        cfg["interval_h"] = ask(t("interval"), cfg["interval_h"], int, lambda v: 1 <= v <= 24)

    # --- Summary
    section(t("summary"))
    yn = lambda b: t("yes_word") if b else t("no_word")
    print(f"  {t('radio_title')}: {radio_str(cfg['radio'])}")
    print(f"  {t('mode_title')}: {t('mode_node') if cfg['mode'] == 'tnc' else t('mode_rnode')}")
    print(f"  {t('name')}: {cfg['name']}")
    print(f"  NomadNet: {yn(cfg['page'])}" + (f" - {cfg['desc']}" if cfg["desc"] else ""))
    if cfg["contact"]:
        print(f"  {t('contact')}: {cfg['contact']}")
    print(f"  rmap.world: {yn(cfg['visible'])}")
    if cfg["visible"]:
        print(f"    {cfg['lat']:.5f}, {cfg['lon']:.5f}, {cfg['alt']:.0f} m; "
              f"NomadNet: {yn(cfg['loc_page'])}; {cfg['interval_h']} h"
              + (f"; LXMF {cfg['lxmf']}" if cfg["lxmf"] else ""))
    if not ask_yes(t("apply_q"), True):
        sys.exit(t("cancel"))

    board.open()
    apply_config(board, cfg, transport_id)

    out = args.profile or f"masn-{transport_id.hex()[:8] or 'node'}.json"
    json.dump(cfg, open(out, "w"), indent=2, ensure_ascii=False)
    print(f"  {t('saved_prof')} {out}")


def stored_matches(board, values):
    """True if the board's stored General namespace equals `values`."""
    stored = board.get_state([NS_GENERAL]).get(NS_GENERAL, {})
    for fid, want in values.items():
        have = stored.get(fid)
        if isinstance(want, float):
            if have is None or abs(float(have) - want) > 1e-6:
                return False
        elif have != want:
            return False
    return True


def apply_config(board, cfg, transport_id):
    section(t("applying"))
    values = {
        F_NOMAD_ENABLE: bool(cfg["page"]),
        F_NOMAD_NAME: cfg["name"],
        F_SITE_DESC: cfg["desc"],
        F_SITE_CONTACT: cfg["contact"],
        F_LAT: float(cfg["lat"]),
        F_LON: float(cfg["lon"]),
        F_ALT: float(cfg["alt"]),
        F_LOC_PUBLIC: bool(cfg["loc_page"]),
        F_DISCOVERY: bool(cfg["visible"]),
        F_ANNOUNCE_INT: int(cfg["interval_h"]) * 60,
    }
    if cfg["visible"]:
        print(f"  {t('stamp')}...")
        fw = board.info().get(1, "")
        fw = fw if isinstance(fw, str) else str(fw)
        values[F_DISCOVERY_DATA] = build_discovery_app_data(transport_id, fw, cfg)
    else:
        values[F_DISCOVERY_DATA] = b""

    # Radio and mode first: the radio config lives in the emulated EEPROM
    # file and writing it right after a provisioning commit has been seen
    # to lose the commit on nRF52, so the commit goes last.
    board.set_radio(cfg["radio"], on=True)
    board.send(CMD_CONF_SAVE if cfg["mode"] == "tnc" else CMD_CONF_DELETE, b"\x00")
    time.sleep(1)
    board.wait_ready()      # EEPROM emulation writes keep the board busy for a while
    print(f"  {t('radio_title')}: OK")

    for attempt in range(1, 4):
        errors = board.set_state(values)
        if errors:
            print("  SetState:", errors)
        board.commit()
        print(f"  {t('rebooting')}...")
        board.reboot()
        print(f"  {t('waiting')}", end="", flush=True)
        board.wait_ready()
        print(" OK")
        if cfg["mode"] == "rnode":
            board.set_radio(cfg["radio"], on=True)
        if stored_matches(board, values):
            print(f"  {t('stored_ok')}")
            break
        print(f"  {t('stored_bad')} ({attempt}/3)")
    else:
        sys.exit(t("stored_fail"))

    section(t("verify"))
    st = board.status() or {}
    print(f"  ONLINE={st.get('ONLINE', '?')} HW={st.get('HW', '?')} "
          f"FREQ={st.get('FREQ', '?')} TXP={st.get('TXP', '?')}")
    try:
        addrs = board.get_state([NS_ADDRS]).get(NS_ADDRS, {})
        if addrs.get(F_ADDR_NOMAD):
            print(f"  {t('nomad_addr')}: {addrs[F_ADDR_NOMAD].hex()}")
        if addrs.get(F_ADDR_TRANSPORT):
            print(f"  {t('transport_id')}: {addrs[F_ADDR_TRANSPORT].hex()}")
    except (RuntimeError, TimeoutError):
        pass
    print(f"  {t('announce_info')}".format(h=cfg["interval_h"]))
    print(f"\n{t('done')}")


# ---------------------------------------------------------------------------
# Non-interactive commands
# ---------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("action", nargs="?", default="wizard",
                   choices=["wizard", "status", "show", "hash", "rnode", "tnc", "power"])
    p.add_argument("--port", help="serial port (default: auto-detect nice!nano)")
    p.add_argument("--profile", help="JSON profile to pre-fill the wizard and save to")
    for k, v in DEFAULT_RADIO.items():
        p.add_argument(f"--{k}", type=int, default=v)
    a = p.parse_args()

    if a.action == "wizard":
        try:
            return wizard(a)
        except KeyboardInterrupt:
            sys.exit("\n" + t("cancel"))

    board = Board(find_port(a.port))
    radio = {k: getattr(a, k) for k in DEFAULT_RADIO}
    if a.action == "show":
        board.wait_ready()
    if a.action == "status":
        print(board.status() or t("no_reply"))
    elif a.action == "power":
        board.send(CMD_DETECT, b"\x6c")
        for cmd, data in board.frames(3.0):
            if cmd == 0x6C:
                raw = data.decode(errors="replace")
                kv = dict(x.split("=", 1) for x in raw.split(";") if "=" in x)
                print(raw)
                if "INA" in kv:
                    # INA3221 channels "bus_mV,shunt_uV"; 0.1 ohm shunts -> mA = uV / 100.
                    ch = {k: [int(v) for v in kv[k].split(",")] for k in ("CH1", "CH2", "CH3")}
                    print(f"  battery {ch['CH2'][0] / 1000:.2f} V, node {ch['CH1'][1] / 100:.0f} mA, "
                          f"charging {ch['CH2'][1] / 100:.0f} mA, "
                          f"solar {ch['CH3'][0] / 1000:.2f} V {ch['CH3'][1] / 100:.0f} mA")
                break
        else:
            print(t("no_reply"))
    elif a.action == "show":
        state = board.get_state([NS_GENERAL, NS_ADDRS])
        for ns, fields in state.items():
            for fid, val in sorted(fields.items()):
                print(ns, fid, val.hex() if isinstance(val, bytes) else val)
    elif a.action == "hash":
        print("firmware hash", board.write_hash())
    elif a.action == "rnode":
        board.send(CMD_CONF_DELETE, b"\x00")
        time.sleep(0.5)
        board.reboot()
        board.set_radio(radio)
        print(board.status() or t("no_reply"))
    elif a.action == "tnc":
        board.set_radio(radio)
        board.send(CMD_CONF_SAVE, b"\x00")
        time.sleep(1)
        board.reboot()
        print(f"TNC: {radio_str(radio)}")


if __name__ == "__main__":
    sys.exit(main())
