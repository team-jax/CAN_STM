<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-10-14 | Updated: 2026-10-14 -->

# lib

## Purpose
PlatformIO's private-library directory. It currently holds only a vendored copy of ST's USB Host middleware, left behind by CubeMX when USB_HOST was enabled. Nothing in the firmware uses it any more.

## Key Files

| File | Description |
|------|-------------|
| `README` | PlatformIO boilerplate describing the `lib/` convention |

## Subdirectories

| Directory | Purpose |
|-----------|---------|
| `Middlewares/ST/STM32_USB_Host_Library/` | Vendored ST USB Host Core (`Core/Inc`, `Core/Src`) and CDC class (`Class/CDC`) with its license. Third-party code, not individually documented |

## For AI Agents

### Working In This Directory
- Do not edit the vendored files. The project no longer initialises USB (`MX_USB_HOST_Init` and `usb_host.*` do not exist), and the build log reports no library dependencies, so this code is not compiled into the firmware.
- PlatformIO's library finder still lists `lib/Middlewares` as an include path in the generated IDE config. That is harmless.
- It can be deleted once `can_test.ioc` has USB_HOST disabled and regenerated; ask the user first.

### Testing Requirements
- Nothing to test here. Confirm with `~/.platformio/penv/bin/pio run` that the build still reports "No dependencies".

### Common Patterns
- None.

## Dependencies

### Internal
- None. Nothing in `../src/` or `../include/` includes these headers.

### External
- ST USB Host middleware (see the license file in `Middlewares/ST/STM32_USB_Host_Library/`).

<!-- MANUAL: Any manually added notes below this line are preserved on regeneration -->
