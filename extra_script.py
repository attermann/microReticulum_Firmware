import time
import hashlib
import shutil
import os
import platform as platformlib

from firmware_image import esp_image_sha256, firmware_hash_kiss_frame
from uf2 import generate_uf2, find_dfu_volume

#
# Helpier functions
#

def get_target():

    # Detect the operating system
    platform_system = platformlib.system().lower()
    #print("System:", platform_system)
    if "linux" in platform_system:
        os_name = "linux"
    elif "darwin" in platform_system:
        os_name = "darwin"
    else:
        os_name = "unknown"

    # Get OS release details
    try:
        platform_os_info = platformlib.freedesktop_os_release()
        #print("OS Release:", platform_os_info)
        if platform_os_info.get('VERSION_CODENAME'):
            distro_name = platform_os_info.get('ID') + "-" + platform_os_info.get('VERSION_CODENAME')
        else:
            distro_name = platform_os_info.get('ID')
        os_name += "-" + distro_name
    except Exception:
        pass

    # Detect the architecture
    platform_machine = platformlib.machine().lower()
    #print("Machine:", platform_machine)
    if platform_machine == "x86_64":
        arch_name = "amd64"
    elif "aarch" in platform_machine or "arm64" in platform_machine:
        arch_name = "arm64"
    elif "arm" in platform_machine:
        arch_name = "armhf"
    else:
        arch_name = "unknown"

    return os_name + "-" + arch_name

#
# Custom targets
#

def target_package(target, source, env):
    print("*** Executing target_package steps...")
    print("Platform:", env.GetProjectOption("platform"))
    print("Board:", env.GetProjectOption("board"))
    print("Variant:", env.GetProjectOption("custom_variant"))
    #if env.GetProjectOption("custom_variant").endswith('_local'):
    #    print("*** Skipping target_package for local build")
    #    return
    # do some actions
    platform = env.GetProjectOption("platform")
    board = env.GetProjectOption("board")
    firmware_package(env)

#
# Upload actions
#

# ---- UF2 (Adafruit/Seeed mass-storage bootloader, e.g. T1000-E) ----
#
# Boards such as the Seeed Tracker T1000-E are not flashed over serial
# DFU. They expose a UF2 mass-storage volume in bootloader (DFU) mode and
# the firmware is copied there as a .uf2 file. The converter in uf2.py turns
# the linked .hex into a .uf2 with absolute addresses, so no flash base has
# to be hardcoded.

def generate_uf2_action(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    progname  = env.subst("$PROGNAME")
    hex_path  = build_dir + "/" + progname + ".hex"
    uf2_path  = build_dir + "/" + progname + ".uf2"
    try:
        out, blocks = generate_uf2(hex_path, uf2_path)
        print("*** Generated UF2: %s (%d blocks)" % (out, blocks))
        return None
    except Exception as exc:
        print("*** UF2 generation failed: %s" % exc)
        return None

def pre_upload(source, target, env):
    print("*** Executing pre_upload steps...")
    # do some actions

def post_upload(source, target, env):
    print("*** Executing post_upload steps...")
    print("Platform:", env.GetProjectOption("platform"))
    print("Board:", env.GetProjectOption("board"))
    print("Variant:", env.GetProjectOption("custom_variant"))
    print("Serial port:", env.subst("$UPLOAD_PORT"))
    # do some actions
    platform = env.GetProjectOption("platform")
    board = env.GetProjectOption("board")
    if ("espressif32" in platform):
        time.sleep(10)
        # device provisioning is incomplete and only currently appropriate for 915MHz T-Beam
        #device_wipe(env)
        device_provision(env)
        firmware_hash(source, env)
        # firmware pacakaging is incomplete due to missing console image
        #firmware_package(env)
    elif ("nordicnrf52" in platform):
        time.sleep(10)
        # device provisioning is incomplete and only currently appropriate for 915MHz RAK4631
        #device_wipe(env)
        device_provision(env)
        time.sleep(5)
        firmware_hash(source, env)
        # firmware pacakaging is incomplete due to missing console image
        #firmware_package(env)

def pre_clean(env):
    print("*** Executing pre_clean steps...")
    print("Platform:", env.GetProjectOption("platform"))
    print("Board:", env.GetProjectOption("board"))
    print("Variant:", env.GetProjectOption("custom_variant"))
    project_dir = env.subst("$PROJECT_DIR")
    print("project_dir:", project_dir)
    env.Execute("rm -f " + project_dir + "/Release/" + env.subst("$PROGNAME") + ".zip")
    env.Execute("rm -f " + project_dir + "/Debug/" + env.subst("$PROGNAME") + ".elf")
    env.Execute("rm -f " + project_dir + "/Debug/" + env.subst("$PROGNAME") + ".map")
    env.Execute("rm -f " + project_dir + "/Release/" + env.subst("$PROGNAME") + "_debug.zip")

def full_clean(env):
    print("*** Executing full_clean steps...")
    project_dir = env.subst("$PROJECT_DIR")
    print("project_dir:", project_dir)
    env.Execute("rm -f " + project_dir + "/Release/release.json")

def device_wipe(env):
    # Device wipe
    print("--- Wiping Device ---")
    env.Execute("rnodeconf --eeprom-wipe " + env.subst("$UPLOAD_PORT"))

def device_set_firmware_hash(firmware_hash, env):
    import serial

    port_path = env.subst("$UPLOAD_PORT")
    frame = firmware_hash_kiss_frame(firmware_hash)
    print("Writing firmware hash directly over KISS for unsupported rnodeconf model...")
    with serial.Serial(port_path, 115200, timeout=0.1) as port:
        # Opening native USB resets the Tracker V2. Drain startup output and
        # wait until setup() has reached the serial command loop.
        ready_at = time.monotonic() + 4.0
        while time.monotonic() < ready_at:
            port.read(4096)
        port.write(frame)
        port.flush()
        time.sleep(1)

def device_provision(env):
    # Device provision
    print("--- Provisioning Device ---")
    platform = env.GetProjectOption("platform")
    print("Platform:", platform)
    board = env.GetProjectOption("board")
    print("Board:", board)
    variant = env.GetProjectOption("custom_variant")
    print("Variant:", variant)
    match variant:
        case "tbeam" | "tbeam_local":
            env.Execute("rnodeconf --product e0 --model e9 --hwrev 1 --rom " + env.subst("$UPLOAD_PORT"))
        case "lora32v21" | "lora32v21_local":
            env.Execute("rnodeconf --product b1 --model b9 --hwrev 1 --rom " + env.subst("$UPLOAD_PORT"))
        case "heltec32v4pa" | "heltec32v4pa_local":
            env.Execute("rnodeconf --product c3 --model c8 --hwrev 1 --rom " + env.subst("$UPLOAD_PORT"))
        case "heltec_tracker_v2" | "heltec_tracker_v2_local":
            env.Execute("rnodeconf --product c4 --model cb --hwrev 1 --rom " + env.subst("$UPLOAD_PORT"))
        case "rak4631" | "rak4631_local":
            env.Execute("rnodeconf --product 10 --model 12 --hwrev 1 --rom " + env.subst("$UPLOAD_PORT"))
        case "rak3401" | "rak3401_local":
            env.Execute("rnodeconf --product 10 --model 14 --hwrev 1 --rom " + env.subst("$UPLOAD_PORT"))
        case "techo" | "techo_local":
            env.Execute("rnodeconf --product 15 --model 17 --hwrev 1 --rom " + env.subst("$UPLOAD_PORT"))
        case "t1000e":
            env.Execute("rnodeconf --product 16 --model 19 --hwrev 1 --rom " + env.subst("$UPLOAD_PORT"))
        case "heltec_t114" | "heltec_t114_local":
            env.Execute("rnodeconf --product c2 --model c7 --hwrev 1 --rom " + env.subst("$UPLOAD_PORT"))
        case _:
            print(f"Unknown board variant {variant}, can not provision device!")

def firmware_hash(source, env):
    # Firmware hash
    print("--- Updating Firmware Hash ---")
    source_file = source[0].get_abspath()
    platform = env.GetProjectOption("platform")
    print("Platform:", platform)
    if (platform == "nordicnrf52"):
        build_dir = env.subst("$BUILD_DIR")
        env.Execute("cd " + build_dir + "; unzip -o " + source_file + " " + env.subst("$PROGNAME") + ".bin")
        #source_file.replace(".zip", ".bin")
        source_file = build_dir + "/" + env.subst("$PROGNAME") + ".bin";
        print("source_file:", source_file)
        firmware_data = open(source_file, "rb").read()
        calc_hash = hashlib.sha256(firmware_data).digest()
        hex_hash = calc_hash.hex()
        print("firmware_hash:", hex_hash)
        env.Execute("rnodeconf --firmware-hash " + hex_hash + " " + env.subst("$UPLOAD_PORT"))
    else:
        print("source_file:", source_file)
        firmware_data = open(source_file, "rb").read()
        if env.GetProjectOption("custom_variant") in ("heltec_tracker_v2", "heltec_tracker_v2_local"):
            try:
                calc_hash = esp_image_sha256(firmware_data)
            except ValueError as error:
                print(f"Unable to calculate firmware hash: {error}")
                return
            print("firmware_hash:", calc_hash.hex())
            device_set_firmware_hash(calc_hash, env)
        else:
            calc_hash = hashlib.sha256(firmware_data[0:-32]).digest()
            part_hash = firmware_data[-32:]
            hex_hash = calc_hash.hex()
            print("firmware_hash:", hex_hash)
            if calc_hash == part_hash:
                env.Execute("rnodeconf --firmware-hash " + hex_hash + " " + env.subst("$UPLOAD_PORT"))
            else:
                print("Calculated hash does not match!")

def firmware_package(env):
    # Firmware package
    print("--- Packaging Firmware ---")
    platform = env.GetProjectOption("platform")
    print("Platform:", platform)
    board = env.GetProjectOption("board")
    print("Board:", board)
    variant = env.GetProjectOption("custom_variant")
    print("Variant:", variant)
    core_dir = env.subst("$CORE_DIR")
    print("core_dir:", core_dir)
    packages_dir = env.subst("$PACKAGES_DIR")
    print("packages_dir:", packages_dir)
    workspace_dir = env.subst("$WORKSPACE_DIR")
    print("workspace_dir:", workspace_dir)
    project_dir = env.subst("$PROJECT_DIR")
    print("project_dir:", project_dir)
    #build_dir = env.subst("$BUILD_DIR").get_abspath()
    build_dir = env.subst("$BUILD_DIR")
    print("build_dir:", build_dir)
    env.Execute("mkdir -p " + project_dir + "/Release")
    env.Execute("mkdir -p " + project_dir + "/Debug")
    if (platform == "espressif32"):
        #env.Execute("cp " + packages_dir + "/framework-arduinoespressif32/tools/partitions/boot_app0.bin " + build_dir + "/rnode_firmware_" + variant + ".boot_app0")
        env.Execute("cp ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin " + build_dir + "/rnode_firmware_" + variant + ".boot_app0")
        env.Execute("cp " + build_dir + "/bootloader.bin " + build_dir + "/" + env.subst("$PROGNAME") + ".bootloader")
        env.Execute("cp " + build_dir + "/partitions.bin " + build_dir + "/" + env.subst("$PROGNAME") + ".partitions")
        env.Execute("rm -f " + project_dir + "/Release/" + env.subst("$PROGNAME") + ".zip")
        zip_cmd = "zip --junk-paths "
        zip_cmd += project_dir + "/Release/rnode_firmware_" + variant + ".zip "
        zip_cmd += project_dir + "/Release/esptool/esptool.py "
        zip_cmd += project_dir + "/Release/console_image.bin "
        zip_cmd += build_dir + "/" + env.subst("$PROGNAME") + ".bin "
        zip_cmd += build_dir + "/" + env.subst("$PROGNAME") + ".boot_app0 "
        zip_cmd += build_dir + "/" + env.subst("$PROGNAME") + ".bootloader "
        zip_cmd += build_dir + "/" + env.subst("$PROGNAME") + ".partitions "
        env.Execute(zip_cmd)
        env.Execute("cp " + build_dir + "/" + env.subst("$PROGNAME") + ".elf " + project_dir + "/Debug/.")
        env.Execute("cp " + build_dir + "/" + env.subst("$PROGNAME") + ".map " + project_dir + "/Debug/.")
        zip_cmd = "zip --junk-paths "
        zip_cmd += project_dir + "/Release/rnode_firmware_" + variant + "_debug.zip "
        zip_cmd += build_dir + "/" + env.subst("$PROGNAME") + ".elf "
        zip_cmd += build_dir + "/" + env.subst("$PROGNAME") + ".map "
        env.Execute(zip_cmd)
    elif (platform == "nordicnrf52"):
        # The T1000-E ships a UF2 image, not an nrfutil DFU .zip (its
        # upload_protocol is "custom"), so only copy the .zip for the
        # serial-DFU boards that actually produce one.
        if variant != "t1000e":
            env.Execute("cp " + build_dir + "/" + env.subst("$PROGNAME") + ".zip " + project_dir + "/Release/.")
        # UF2 image for Adafruit/Seeed mass-storage-bootloader boards (T1000-E).
        uf2_artifact = build_dir + "/" + env.subst("$PROGNAME") + ".uf2"
        if os.path.exists(uf2_artifact):
            env.Execute("cp " + uf2_artifact + " " + project_dir + "/Release/.")
    else:
        env.Execute("cp " + build_dir + "/" + env.subst("$PROGNAME") + " " + build_dir + "/rnoded")
        env.Execute("rm -f " + project_dir + "/Release/rnoded-" + get_target() + ".zip")
        zip_cmd = "zip --junk-paths "
        zip_cmd += project_dir + "/Release/rnoded-" + get_target() + ".zip "
        zip_cmd += build_dir + "/rnoded "
        zip_cmd += project_dir + "/rnoded.example.conf "
        zip_cmd += project_dir + "/rnoded.example.service "
        env.Execute(zip_cmd)
        get_target()
    env.Execute("python3 " + project_dir + "/release_hashes.py > " + project_dir + "/Release/release.json")

#
# Main script
#

Import("env")

env.Replace(PROGNAME="rnode_firmware_%s" % env.GetProjectOption("custom_variant"))
print("PROGNAME:", env.subst("$PROGNAME"))

print("*** Running custom script...")
platform = env.GetProjectOption("platform")
print("Platform:", platform)
targets = env.GetProjectOption("targets", [])
print("Targets:", targets)

# Clean
if env.IsCleanTarget():
    pre_clean(env)
    if "cleanall" in targets or "fullclean" in targets:
        full_clean(env)

# Add custom targets
if (platform == "espressif32"):
    env.AddCustomTarget(
        name="package",
        dependencies="$BUILD_DIR/${PROGNAME}.bin",
        actions=[
            target_package
        ],
        title="Package",
        description="Package esp32 firmware for delivery"
    )
elif (platform == "nordicnrf52"):
    # remove --specs=nano.specs to allow exceptions to work
    if '--specs=nano.specs' in env['LINKFLAGS']:
        env['LINKFLAGS'].remove('--specs=nano.specs')
    # Generate a UF2 image alongside the linked .hex for boards that flash
    # via the Adafruit/Seeed mass-storage bootloader (e.g. T1000-E). The
    # .hex is emitted by the Arduino core's objcopy step, so this post-action
    # fires once it exists. Harmless on boards that never use the .uf2.
    env.AddPostAction("$BUILD_DIR/${PROGNAME}.hex", generate_uf2_action)
    # The T1000-E is a UF2 board: with upload_protocol = custom the nrfutil
    # DFU .zip is never produced, so package the .hex (whose post-action above
    # also emits the distributable .uf2) instead of the .zip.
    _package_dep = ("$BUILD_DIR/${PROGNAME}.hex"
                    if env.GetProjectOption("custom_variant", "") == "t1000e"
                    else "$BUILD_DIR/${PROGNAME}.zip")
    env.AddCustomTarget(
        name="package",
        dependencies=_package_dep,
        actions=[
            target_package
        ],
        title="Package",
        description="Package nrf52 firmware for delivery"
    )
else:
    env.AddCustomTarget(
        name="package",
        dependencies="$BUILD_DIR/${PROGNAME}",
        actions=[
            target_package
        ],
        title="Package",
        description="Package native daemon for delivery"
    )

# Register upload actions.
#
# The T1000-E flashes by dragging a .uf2 onto its DFU mass-storage volume,
# not over serial DFU, so its `upload` target generates the .uf2 and copies
# it to the mounted volume (with drag-and-drop instructions as a fallback).
# The serial-DFU / rnodeconf pre- and post-upload steps are only wired up for
# the other boards that actually use nrfutil.
variant = env.GetProjectOption("custom_variant", "")
if variant == "t1000e":
    env.AddCustomTarget(
        name="uf2",
        dependencies="$BUILD_DIR/${PROGNAME}.hex",
        actions=[generate_uf2_action],
        title="Generate UF2",
        description="Build a .uf2 file for drag-and-drop flashing"
    )
    # The T1000-E flashes via UF2 mass storage, not serial DFU. Drive the
    # platform's own "upload" target (seeed_t1000e sets upload_protocol =
    # custom) with a UPLOADCMD that converts the linked .hex to UF2 and copies
    # it onto the mounted DFU volume.
    #
    # We deliberately do NOT call env.AddCustomTarget(name="upload", ...):
    # extra_script.py runs as a `pre:` script, i.e. *before* the nordicnrf52
    # platform registers its "upload" target in main.py. Registering the same
    # name twice trips `assert name not in env["__PIO_TARGETS"]` inside PIO's
    # piotarget.AddTarget and aborts the build.
    project_dir = env.subst("$PROJECT_DIR")
    env.Replace(
        UPLOADCMD='"$PYTHONEXE" "%s/uf2.py" --hex "$SOURCE"' % project_dir
    )
else:
    env.AddPreAction("upload", pre_upload)
    env.AddPostAction("upload", post_upload)
