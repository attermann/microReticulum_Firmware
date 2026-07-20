# Changelog

## [Unreleased]

### Changed

- **Provisioning wire format** — reworked the wire protocol for lower LoRa airtime and fewer round-trips. Breaking change from the previous format:
    - `GET_STATE` now returns `{Values, Drafts?, Hash}`; the `Draft` request flag folds drafts into the same response.
    - `GET_STATE` supports a `PriorHash` short-circuit: clients echo the previous response's `Hash` back, and the server responds `{Unchanged: true}` when state hasn't changed.
    - `SET_STATE` requests use a `{State: {...}, IncludeState?, ReqCompress?}` envelope. With `IncludeState: true`, the response includes `PostOpValues` / `PostOpDrafts` / `PostOpHash` — Save no longer needs a follow-up `GET_STATE`.
    - `COMMIT` requests use a `{NamespaceFilter?, IncludeState?, ReqCompress?}` map (was a bare `[ns_ids]` array). With `IncludeState: true`, the response carries post-commit `PostOpValues` and `PostOpHash` — Commit no longer needs a follow-up refresh.
    - `GET_CAPABILITIES` returns a namespace hierarchy map (`{NsId, NsName, NsParent, NsFieldCount, NsSchemaHash}`) instead of a bare id array; combined with `GET_SCHEMA`'s new `NamespaceFilter` support, this enables lazy per-namespace schema loading.
- **Web console** — refactored to use the new wire protocol. Save and Commit each collapse from two transactions to one; namespace-panel refreshes cache-hit via `PriorHash` when state hasn't changed since the last poll.

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
