# microReticulum_Firmware Pull Request Policy

This document explains what microReticulum_Firmware will and won't accept, so
you can tell before you start whether a change is likely to be merged. It
complements [CONTRIBUTING.md](../CONTRIBUTING.md), which covers *how* to
contribute.

Every pull request is reviewed against these rules. The maintainer uses
automated tooling to help prepare reviews, but every merge or close decision
is made by the maintainer personally.

---

## 1. What microReticulum_Firmware is

A fork of [RNode_Firmware](https://github.com/markqvist/RNode_Firmware) that
embeds the [microReticulum](https://github.com/attermann/microReticulum) C++
network stack, so a LoRa device can run as a self-contained
[Reticulum](https://reticulum.network/) transport node. It runs on ESP32 and
nRF52 boards, and as a native daemon on Linux (and macOS for development).

By default a device still behaves as a standard RNode modem for a host
running Python RNS (Host Mode). In TNC mode the embedded stack routes packets
itself (Transport Mode).

This repository is where **board support** belongs. The Reticulum protocol
itself lives in the microReticulum library, and persistence in microStore.

## 2. Project goals (what a PR should move toward)

1. **Stay a compatible RNode.** In Host Mode, the device must keep working
   with stock `rnodeconf` and Python RNS: KISS commands, EEPROM layout,
   firmware hash checks and product/board/model IDs behave as upstream
   RNode_Firmware expects.
2. **A reliable standalone transport node.** Transport Mode should run
   unattended for long periods: no leaks, deadlocks, watchdog resets or
   corrupted persistent state.
3. **Fitting constrained hardware.** RAM, flash and filesystem space are
   scarce, especially on nRF52. Code size, heap use, fragmentation and
   stack depth all matter.
4. **Opt-in features.** Optional subsystems (WiFi/UDP transport, filesystem
   persistence, web console, extra interfaces) are gated behind build flags
   or provisioning settings, so they cost nothing where they're not used.
5. **Broad, consistent board support.** New boards are welcome. Shared code
   behaves the same on every supported board and on the native daemon, with
   board differences kept in board definitions and variants.
6. **Remote management through Provisioning.** Configuration and metrics are
   exposed through the Provisioning subsystem and the web console in a way
   that stays backward compatible with deployed devices.
7. **Completing the roadmap.** Open items in the README roadmap (currently
   **power management and sleep states**) are especially welcome.

## 3. In scope

- Bug fixes in radio drivers, interfaces, power management, display,
  Bluetooth, WiFi, persistence, provisioning or the web console.
- **New board support**: board definitions, variants, pin maps, PlatformIO
  environments and product/board/model IDs, following §5.9.
- New interfaces or transports that fit the existing interface
  abstraction, gated so they're off where unsupported.
- Provisioning fields, metrics and web console improvements.
- Native daemon support for further Linux hosts and radio HATs.
- Memory, flash, airtime and power improvements.
- Build-system and packaging fixes for PlatformIO, the Makefile or `docker/`.
- Tests and documentation fixes and improvements.

## 4. Out of scope (likely to be closed or redirected)

- **Reticulum protocol, transport or link behavior** that belongs in the
  library → redirect to
  [microReticulum](https://github.com/attermann/microReticulum).
  Firmware-side glue that uses the library is fine.
- **Behavior that breaks RNode compatibility** in Host Mode (KISS
  commands, EEPROM layout, IDs that `rnodeconf` relies on) unless discussed
  in an issue first.
- **End-user applications on the device** (chat hubs, messaging clients,
  games, home-automation control) that go beyond running and managing a
  transport node → a separate project built on microReticulum.
- **New third-party dependencies**, or replacing existing ones, unless
  discussed in an issue first.
- **Large refactors, renames or reformatting** not tied to a stated
  problem, or not discussed first. This includes broad re-syncs with
  upstream RNode_Firmware, which the maintainer does.
- **Release artifacts**: changes to built binaries, hashes or
  `Release/release.json`. The maintainer produces releases.
- **Board-specific code in shared code paths** where it could live in the
  board's definition or variant instead.

## 5. Requirements for merging

A PR that's missing any of these will be sent back for changes before it
can be merged.

1. **Builds for the affected environments.** The PR states which
   PlatformIO environments were built (`pio run -e …`) and that they build
   cleanly. Changes to shared code build on at least one ESP32, one nRF52
   and the `native` environment.
2. **One logical concern per PR.** Unrelated fixes, formatting, refactors
   and new boards go in separate PRs.
3. **Problem stated first.** The description says what's broken or missing,
   who is affected and how to reproduce, before the solution.
4. **Tested on hardware, and the boards are listed.** The PR lists which
   boards (or native hosts) it was run on, how it was tested, and which
   affected boards weren't tested and why. Changes to shared code (radio
   drivers, interfaces, framing, persistence, provisioning, timing) are
   tested on every affected platform family. New board support is tested on
   that board.
5. **RNode compatibility preserved.** Host Mode still works with stock
   `rnodeconf` and Python RNS. Any change to KISS commands, EEPROM layout or
   firmware hash handling is called out and was discussed first.
6. **No breaking Provisioning or configuration changes** without prior
   discussion and a CHANGELOG entry. Provisioning namespace and field IDs
   are never changed or reused once released; only additions are allowed.
   Namespace IDs 1–99 are reserved for the microReticulum library.
7. **Docs updated** for user-visible behavior, build flags, configuration,
   supported boards or workflows (README, `docs/`, `rnoded.example.conf`,
   CHANGELOG as appropriate).
8. **Radio defaults are safe.** No hard-coded frequency, transmit power or
   duty cycle that could be illegal in some regions; radio parameters stay
   unset until the user configures them, as today.
9. **Board IDs are correct.** A board that upstream RNode_Firmware already
   supports uses upstream's product, board and model IDs. A new board's IDs
   don't collide with existing or upstream IDs, and the PR says how they
   were chosen.
10. **License:** contributions are accepted under GPL-3.0. Code copied from
    elsewhere (e.g. other firmware projects) must be license-compatible and
    credited.
11. **No local-environment changes.** PRs don't add or change `-local`
    PlatformIO environments (the ones using `symlink://` library paths), or
    other settings that only fit the contributor's own machine, such as
    local paths, upload ports or personal build flags. These are artifacts
    of a local setup; remove them before submitting. New boards add only
    their release environment.

## 6. Preferences

These won't block a PR on their own, but they make it faster to review and
more likely to be merged.

- Follows the existing coding style and naming.
- New optional features are off by default, or follow how similar
  features are enabled today.
- Heap allocation in hot paths and ISRs avoided; long-lived RNS containers
  use the `RNS_*_ALLOCATOR` framework.
- Logging uses the existing log levels and isn't noisy at INFO (in TNC mode
  logs share the serial port).
- Small enough to review in one sitting (roughly < 500 changed lines
  excluding board definitions, tests and docs; bigger needs a good reason).

## 7. What happens after you submit

Each pull request ends up in one of four places:

| Outcome | What it means for you |
|---|---|
| **Merged** | It's in scope and meets the requirements in section 5. |
| **Changes requested** | The idea is welcome, but specific things need fixing first (build failures, untested platforms, missing docs, a compatibility issue, too many concerns in one PR). You'll get a list. |
| **Discussion needed** | It raises a design, scope or compatibility question that has to be settled before code review. Expect the conversation to move to an issue. |
| **Closed** | It's out of scope (section 4), duplicates existing work, or conflicts with the project's goals. Where possible you'll be pointed to a better home for it, such as the microReticulum library. |

The maintainer has limited time, so reviews may take a while. A PR that
follows this document and the template is the fastest to get through.
If you're unsure whether something fits, open an issue first; that's
always welcome and often saves rework.
