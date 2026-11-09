## Problem

<!-- What is broken or missing? Who is affected? How can it be reproduced? -->

## Solution

<!-- How does this change solve it, why this approach, any tradeoffs or limitations? -->

## Compatibility

- [ ] Host Mode still works with stock `rnodeconf` and Python RNS (no change to KISS commands, EEPROM layout or firmware hash handling, or described below)
- [ ] No change to existing Provisioning namespace/field IDs or configuration (or described below and added to CHANGELOG)
- [ ] No hard-coded region-specific radio defaults (frequency, TX power, duty cycle)
- [ ] Protocol behavior belongs in this firmware, not in the microReticulum library
- [ ] No `-local` PlatformIO environments or other settings specific to my machine (local paths, upload ports, personal build flags)
- [ ] New board: product/board/model IDs match upstream RNode_Firmware, or are new and don't collide (explain below)

## Build and testing

Environments built (`pio run -e …`):

Boards / hosts tested on (and how):

- [ ] ESP32 (board: )
- [ ] nRF52 (board: )
- [ ] native (host / HAT: )
- [ ] Interop with Python RNS / `rnodeconf`

Affected boards not tested, and why:

## Docs

- [ ] README / `docs/` / `rnoded.example.conf` / CHANGELOG updated where user-visible behavior, build flags, configuration or supported boards changed

<!-- Please read CONTRIBUTING.md and .github/PR_POLICY.md before submitting. One logical concern per PR. -->
