import time
import hashlib
import shutil

Import("env")

# Helper: detect nRF52-compatible platforms (includes Seeed's XIAO nRF52840)
def is_nrf52_platform(platform_str):
    return platform_str == "nordicnrf52" or "platform-seeedboards" in platform_str

env.Replace(PROGNAME="rnode_firmware_%s" % env.GetProjectOption("custom_variant"))
print("PROGNAME:", env.subst("$PROGNAME"))

#
# Custom targets
#

def target_package(target, source, env):
    print("target_package...")
    print("Platform:", env.GetProjectOption("platform"))
    print("Board:", env.GetProjectOption("board"))
    print("Variant:", env.GetProjectOption("custom_variant"))
    # do some actions
    platform = env.GetProjectOption("platform")
    board = env.GetProjectOption("board")
    firmware_package(env)

platform = env.GetProjectOption("platform")
print("Platform:", platform)
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
elif (is_nrf52_platform(platform)):
    # remove --specs=nano.specs to allow exceptions to work
    if '--specs=nano.specs' in env['LINKFLAGS']:
        env['LINKFLAGS'].remove('--specs=nano.specs')
    env.AddCustomTarget(
        name="package",
        dependencies="$BUILD_DIR/${PROGNAME}.zip",
        actions=[
            target_package
        ],
        title="Package",
        description="Package nrf52 firmware for delivery"
    )

#
# Upload actions
#

def pre_upload(source, target, env):
    print("pre_upload...")
    # do some actions

def post_upload(source, target, env):
    print("post_upload...")
    print("Platform:", env.GetProjectOption("platform"))
    print("Board:", env.GetProjectOption("board"))
    print("Serial port:", env.subst("$UPLOAD_PORT"))
    # do some actions
    platform = env.GetProjectOption("platform")
    board = env.GetProjectOption("board")
    if (platform == "espressif32"):
        time.sleep(5)
        # device provisioning is incomplete and only currently appropriate for 915MHz T-Beam
        #device_provision(env)
        firmware_hash(source, env)
        # firmware pacakaging is incomplete due to missing console image
        #firmware_package(env)
    elif (is_nrf52_platform(platform)):
        time.sleep(5)
        # device provisioning is incomplete and only currently appropriate for 915MHz RAK4631
        #device_provision(env)
        firmware_hash(source, env)
        # firmware pacakaging is incomplete due to missing console image
        #firmware_package(env)

def post_clean(source, target, env):
    print("post_clean...")
    core_dir = env.subst("$CORE_DIR")
    print("core_dir:", core_dir)
    packages_dir = env.subst("$PACKAGES_DIR")
    print("packages_dir:", packages_dir)
    project_dir = env.subst("$PROJECT_DIR")
    print("project_dir:", project_dir)
    #build_dir = env.subst("$BUILD_DIR").get_abspath()
    build_dir = env.subst("$BUILD_DIR")
    print("build_dir:", build_dir)
    build_cache_dir = env.subst("$PLATFORMIO_BUILD_CACHE_DIR")
    print("build_cache_dir:", build_cache_dir)
    workspace_dir = env.subst("$PLATFORMIO_WORKSPACE_DIR")
    print("workspace_dir:", workspace_dir)
    #shutil.rmtree(directory_path)
    env.Execute("rm -f " + project_dir + "/Release/" + project_dir + "/" + env.subst("$PROGNAME") + ".zip")

env.AddPreAction("upload", pre_upload)
env.AddPostAction("upload", post_upload)
env.AddPostAction("clean", post_clean)

def device_wipe(env):
    # Device wipe
    print("Wiping device...")
    env.Execute("rnodeconf --eeprom-wipe " + env.subst("$UPLOAD_PORT"))

def device_provision(env):
    # Device provision
    print("Provisioning device...")
    platform = env.GetProjectOption("platform")
    print("Platform:", platform)
    board = env.GetProjectOption("board")
    print("Board:", board)
    if (platform == "espressif32"):
        if (board == "ttgo-t-beam"):
            print("Provisioning t-beam device...")
            env.Execute("rnodeconf --product e0 --model e9 --hwrev 1 --rom " + env.subst("$UPLOAD_PORT"))
    elif (is_nrf52_platform(platform)):
        if (board == "wiscore_rak4631"):
            print("Provisioning rak4631 device...")
            env.Execute("rnodeconf --product 10 --model 12 --hwrev 1 --rom " + env.subst("$UPLOAD_PORT"))

def firmware_hash(source, env):
    # Firmware hash
    print("Updating firmware hash...")
    source_file = source[0].get_abspath()
    platform = env.GetProjectOption("platform")
    print("Platform:", platform)
    if (is_nrf52_platform(platform)):
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
        calc_hash = hashlib.sha256(firmware_data[0:-32]).digest()
        part_hash = firmware_data[-32:]
        hex_hash = calc_hash.hex()
        print("firmware_hash:", hex_hash)
        if (calc_hash == part_hash):
            env.Execute("rnodeconf --firmware-hash " + hex_hash + " " + env.subst("$UPLOAD_PORT"))
        else:
            print("Calculated hash does not match!")

def firmware_package(env):
    platform = env.GetProjectOption("platform")
    board = env.GetProjectOption("board")
    # Firmware package
    print("Building firmware package...")
    platform = env.GetProjectOption("platform")
    print("Platform:", platform)
    board = env.GetProjectOption("board")
    print("Board:", board)
    variant = env.GetProjectOption("custom_variant")
    print("custom_variant:", variant)
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
    elif (is_nrf52_platform(platform)):
        env.Execute("cp " + build_dir + "/" + env.subst("$PROGNAME") + ".zip " + project_dir + "/Release/.")
    env.Execute("python " + project_dir + "/release_hashes.py > " + project_dir + "/Release/release.json")
