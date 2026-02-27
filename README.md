# microReticulum_Firmware

Fork of RNode_Firmware with integration of the [microReticulum](https://github.com/attermann/microReticulum) Network Stack to implement a completeley self-contained standalone Reticulum node.

## Installation

This firmware can be easily installed on devices in the same way as RNode using the new `fw-url` switch to `rnodeconf` which allows firmware images to be pulled from an alternate repository. RNS may need to be updated to the latest version to use this new switch.

The latest version of this firmware can be installed in the usual RNode way with the following command:
```
rnodeconf --autoinstall --fw-url https://github.com/attermann/microReticulum_Firmware/releases/
```

NOTE: If re-installing a new build of the same version installed previously, be sure to clear the rnodeconf cache first to force it to download the very latest.
```
rnodeconf --clear-cache
```

## Enabling Transport Mode

By default this firmware will operate just like any other RNode firmware allowing it to be used as just a radio by RNS installed on an attached machine.

To enable `Transport Mode` using the RNS embedded on the device, the device must be switched to TNC mode using a command like the following:
```
rnodeconf --tnc --freq 915000000 --bw 125000 --sf 8 --cr 5 --txp 17 /dev/ttyACM0
```
When in `Transport Mode`, the device will display "TRANSPORT" across the top of the AirTime panel of the display to indicate that the embedded RNS is active and routing packets.

Note that at the present time, when in TNC mode this firmware does not operate like a regular RNode does when in TNC mode due to logging from the embedded RNS that is output on the serial port. This can clobber KISS communication from the attached machine so do not attempt to attach another RNS to the device while in this mode. On the plus side, there is extensive logging available on the serial port to observe the embedded RNS in action and to aid in troubleshooting.

## Build Dependencies

Build environment is configured for use in [VSCode](https://code.visualstudio.com/) and [PlatformIO](https://platformio.org/).

## Building from Source

Building and uploading to hardware is simple through the VSCode PlatformIO IDE
- Install VSCode and PlatformIO
- Clone this repo
- Lanch PlatformIO and load repo
- In PlatformIO, select the environment for intended board
- Build, Upload, and Monitor to observe application logging

Uploading to devices requires access to the `rnodeconf` utility included in the official [Reticulum](https://github.com/markqvist/Reticulum) distribution to update the device firmware hash. Without this step the device will report invalid firmware and will fail to fully initialize.

Instructions for command line builds and packaging for firmware distribution.

## PlatformIO Command Line

Clean all environments (boards):
```
pio run -t clean
```

Full Clean (including libdeps) all environments (boards):
```
pio run -t fullclean
```

Build a single environment (board):
```
pio run -e ttgo-t-beam
pio run -e wiscore_rak4631
```

Build and upload a single environment (board):
```
pio run -e ttgo-t-beam -t upload
pio run -e wiscore_rak4631 -t upload
```

Build and package a single environment (board):
```
pio run -e ttgo-t-beam -t package
pio run -e wiscore_rak4631 -t package
```

Build all environments (boards):
```
pio run
```

Build and package all environments (boards):
```
pio run -t package
```

Write version info:
  python release_hashes.py > Release/release.json

## Firmware Release

New firmware release procedure:

  1. Ensure that microReticulum repo is updated for build (and package versioning is incremented if changed)

  2. Shutdown microReticulum_Firmware project in IDE (if open)

  3. Clean build directory
     ```
     pio run -t fullclean
     ```

  4. Clean release directory
     ```
     rm Release/release.json
     ```

  5. Build new releases
     ```
     pio run -t package
     ```

  6. Upload all files (except README.md and esptool) to github release

## Roadmap

- [ ] Extend KISS interface to support config/control of the integrated microReticulum stack
- [ ] Add interface for easy customization of firmware
- [ ] Add power management and sleep states to extend battery runtime
- [x] Add build targets for NRF52 boards

Please open an Issue if you have trouble building ior using the API, and feel free to start a new Discussion for anything else.

