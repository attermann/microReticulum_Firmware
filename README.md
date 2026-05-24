# RNode Firmware — RAK4631 Ethernet Edition

This fork adds Ethernet support for the RAK4631 + RAK13800 (W5100S) WisBlock combination, enabling KISS-over-TCP host connections alongside the existing USB serial interface. The same firmware binary supports both plain RAK4631 boards and the ETH variant — the W5100S is detected at runtime.

To install via `rnodeconf`:

```bash
rnodeconf --autoinstall --fw-url https://github.com/metrafonic/RNode_Firmware/releases/
```

