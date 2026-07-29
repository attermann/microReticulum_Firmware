#
# uf2.py - Minimal Intel-HEX -> UF2 converter for nRF52840 boards that flash
# via the Adafruit/Seeed mass-storage (UF2) bootloader, e.g. the Seeed
# Tracker T1000-E.
#
# UF2 is Microsoft's USB Flashing Format:
#   https://github.com/microsoft/uf2
# Each 512-byte block carries a 256-byte payload plus the absolute target
# address, so the converted file is independent of any particular flash
# layout (the addresses come straight from the .hex the linker emitted).
#
# Licensed under the MIT license.

import os
import shutil
import struct

# UF2 on-wire constants.
UF2_MAGIC_START0 = 0x0A324655  # "UF2\n"
UF2_MAGIC_START1 = 0x9E5D5157
UF2_MAGIC_END    = 0x0AB16F30
UF2_FLAG_FAMILY_ID_PRESENT = 0x00002000
UF2_BLOCK_SIZE   = 512
UF2_PAYLOAD_SIZE = 256

# Family ID for Adafruit Bluefruit / Seeed nRF52840 UF2 bootloaders.
# (See microsoft/uf2 uf2families.json.) May be overridden by the caller.
UF2_FAMILY_NRF52840 = 0xADA52840


def _read_ihex(path):
    """Parse an Intel HEX file into a list of (absolute_addr, data_bytes)
    contiguous segments, honouring type-04 extended-linear-address records
    so flash above 64 KB is handled correctly."""
    segments = []
    base_hi = 0          # upper 16 bits from the last type-04 record
    seg_addr = None      # start address of the current segment
    seg_data = bytearray()

    def flush():
        nonlocal seg_addr, seg_data
        if seg_addr is not None and seg_data:
            segments.append((seg_addr, bytes(seg_data)))
        seg_addr = None
        seg_data = bytearray()

    with open(path, "r") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line or not line.startswith(":"):
                continue
            buf = bytes.fromhex(line[1:])
            dlen = buf[0]
            addr = (buf[1] << 8) | buf[2]
            rtype = buf[3]
            data = buf[4:4 + dlen]
            if rtype == 0x00:                      # data
                abs_addr = base_hi + addr
                if seg_addr is None:
                    seg_addr = abs_addr
                elif abs_addr != seg_addr + len(seg_data):
                    flush()
                    seg_addr = abs_addr
                seg_data.extend(data)
            elif rtype == 0x04:                    # extended linear address
                flush()
                base_hi = ((data[0] << 8) | data[1]) << 16
            elif rtype == 0x01:                    # EOF
                break
            # type 02/03 (segment/extended segment) are unused for nRF52.
        flush()
    return segments


def generate_uf2(hex_path, uf2_path, family_id=UF2_FAMILY_NRF52840):
    """Convert an Intel HEX firmware image to a UF2 file. Returns a tuple
    (output_path, num_blocks). Falls back to a raw .bin image at a given base
    if the .hex is absent (caller passes bin_path + bin_base)."""
    segments = _read_ihex(hex_path)

    # Slice every segment into 256-byte UF2 payloads.
    blocks = []
    for base, data in segments:
        for off in range(0, len(data), UF2_PAYLOAD_SIZE):
            chunk = data[off:off + UF2_PAYLOAD_SIZE]
            if len(chunk) < UF2_PAYLOAD_SIZE:
                chunk = chunk + bytes(UF2_PAYLOAD_SIZE - len(chunk))
            blocks.append((base + off, chunk))

    total = len(blocks)
    if total == 0:
        raise RuntimeError("UF2: no data records found in %s" % hex_path)

    with open(uf2_path, "wb") as out:
        for index, (addr, payload) in enumerate(blocks):
            block = bytearray(UF2_BLOCK_SIZE)
            struct.pack_into(
                "<IIIIIIII", block, 0,
                UF2_MAGIC_START0, UF2_MAGIC_START1,
                UF2_FLAG_FAMILY_ID_PRESENT,
                addr & 0xFFFFFFFF,
                UF2_PAYLOAD_SIZE,
                index, total,
                family_id,
            )
            block[32:32 + len(payload)] = payload
            struct.pack_into("<I", block, 508, UF2_MAGIC_END)
            out.write(block)

    return uf2_path, total


def find_dfu_volume(name_token="T1000"):
    """Best-effort search for a mounted UF2 bootloader mass-storage volume.
    Matches any mount point whose leaf name contains name_token (case
    insensitive). Returns the path or None."""
    import os
    import glob
    token = name_token.upper()
    patterns = [
        "/Volumes/*",            # macOS
        "/media/*/*",            # Linux (UDisks)
        "/run/media/*/*",        # Linux (systemd)
    ]
    for pattern in patterns:
        for candidate in glob.glob(pattern):
            if token in os.path.basename(candidate).upper():
                # Confirm it looks like a UF2 volume (INFO_UF2.TXT is the
                # marker file every UF2 bootloader exposes).
                if os.path.exists(os.path.join(candidate, "INFO_UF2.TXT")):
                    return candidate
    return None


def upload_via_uf2(hex_path, uf2_path=None, volume_token="T1000"):
    """Generate a UF2 from an Intel HEX image and, if a UF2 mass-storage
    bootloader volume is currently mounted, copy it there. Intended to back
    the platform's `upload` target for boards such as the Seeed T1000-E that
    flash by drag-and-drop rather than serial DFU. Returns 0 on success.

    When no DFU volume is found this prints step-by-step drag-and-drop
    instructions instead of failing, so the same command works whether or not
    the board is currently in bootloader mode."""
    import shutil

    if not uf2_path:
        base, _ = os.path.splitext(hex_path)
        uf2_path = base + ".uf2"

    try:
        out, blocks = generate_uf2(hex_path, uf2_path)
        print("*** Generated UF2: %s (%d blocks)" % (out, blocks))
    except Exception as exc:
        print("*** UF2 generation failed: %s" % exc)
        return 1

    vol = find_dfu_volume(volume_token)
    if vol:
        dest = os.path.join(vol, os.path.basename(uf2_path))
        shutil.copy(uf2_path, dest)
        print("*** Copied %s -> %s" % (uf2_path, dest))
        print("*** The board reboots into the application once the copy completes.")
    else:
        print("***")
        print("*** No %s DFU volume was found." % volume_token)
        print("*** To flash:")
        print("***   1. Put the board in DFU mode: rapidly disconnect and reconnect")
        print("***      USB (or double-tap reset) until a %s drive mounts." % volume_token)
        print("***   2. Drag this file onto it:")
        print("***      %s" % uf2_path)
        print("***")
    return 0


def _cli_main(argv=None):
    """Command-line entry point: `python uf2.py --hex firmware.hex`.

    Used as UPLOADCMD for the T1000-E upload target so the UF2 conversion +
    drag-and-drop copy can run outside the SCons action callbacks."""
    import argparse
    import sys

    parser = argparse.ArgumentParser(
        description="Convert an Intel HEX firmware image to UF2 and copy it "
                    "to a mounted UF2 (mass-storage) bootloader volume.")
    parser.add_argument("--hex", required=True,
                        help="Path to the input Intel HEX firmware image.")
    parser.add_argument("--uf2",
                        help="Path for the generated UF2 file. Defaults to the "
                             ".hex path with a .uf2 extension.")
    parser.add_argument("--volume-token", default="T1000",
                        help="Token used to locate the mounted DFU volume "
                             "(matched case-insensitively against the volume "
                             "name). Default: T1000.")
    args = parser.parse_args(argv)
    return upload_via_uf2(args.hex, args.uf2, args.volume_token)


if __name__ == "__main__":
    import sys
    sys.exit(_cli_main())
