# Changelog

## [1.86.4] - 2026-06-27

### Added

- Added WebSocket support for KISS interfaces
- Added public WebSocket operating mode
- Added RNS transport support in the web console for remote provisioning
- Added integration with MeshChatX API servers for remote link establishment
- Added native SX1262 board support
- Added extended provisioning and remote-management metrics
- Added expanded provisioning field definitions
- Added provisioning reboot as a first-class operation
- Added browser local-storage schema caching for provisioning
- Added schema and provisioning-data compression
- Added log filtering by substring in the web console
- Added configurable maximum log-level filtering
- Added command-line configuration path support for native builds
- Added additional native packet and diagnostic logging
- Added detailed provisioning and management documentation
- Added Docker workspace auto-clone behavior when repository is missing

### Changed

- Refactored build configuration to reduce firmware size
- Refactored provisioning draft, working, and effective value handling
- Refactored native TCXO enablement
- Improved radio initialization and reset sequencing
- Improved provisioning transfer efficiency for low-bandwidth links
- Updated native pin mappings and board configuration support
- Updated web console packaging and provisioning behavior
- Disabled periodic statistics dumps to logs
- Improved modem status and noise floor handling
- Improved transaction handling in radio drivers

### Fixed

- Fixed embedded single-transfer regression affecting SX radios
- Fixed TX queue lockup when interference avoidance was enabled
- Fixed `medium_free()` incorrectly returning false after startup
- Fixed RX LED remaining active when noise floor sampling was unavailable
- Fixed float-to-int conversion truncation for values below 1
- Fixed heatshrink decoding issues in the web console
- Fixed Linux reboot failure in native builds
- Fixed WebSocket failures occurring after reboot
- Removed ineffective pre-initialization board reset behavior

### Documentation

- Added comprehensive provisioning documentation
- Expanded remote management documentation
- Updated README examples and provisioning guidance

### Contributors

- @attermann
- @nilu96
