<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-10-14 | Updated: 2026-10-14 -->

# .vscode

## Purpose
VSCode editor, IntelliSense and debug configuration written by the PlatformIO extension. Not part of the firmware build.

## Key Files

| File | Description |
|------|-------------|
| `c_cpp_properties.json` | IntelliSense include paths and defines. Auto-generated ("DO NOT MODIFY"; change `platformio.ini` instead) and git-ignored |
| `launch.json` | Three "PIO Debug" configurations (with pre-debug build, skipping it, and without uploading) pointing at `.pio/build/disco_f407vg/firmware.elf`, the SVD `STM32F40x.svd` and the ARM toolchain. Auto-generated, git-ignored, regenerated whenever the active env changes |
| `extensions.json` | Recommends `platformio.platformio-ide`; marks the cpptools extension pack and pioarduino as unwanted |

## Subdirectories
None.

## For AI Agents

### Working In This Directory
- Do not hand-edit `c_cpp_properties.json` or `launch.json`; PlatformIO overwrites them. The absolute `/Users/<name>/...` paths inside are machine-specific.
- Editor diagnostics from other tools (for example clang saying `main.h` is missing) do not reflect the real build. Use `~/.platformio/penv/bin/pio run`.

### Testing Requirements
- None. Debugging needs a connected ST-Link (the DISC1 on-board debugger); without one the debug configs cannot start.

### Common Patterns
- None.

## Dependencies

### External
- PlatformIO IDE extension, ST-Link debug probe, `toolchain-gccarmnoneeabi`.

<!-- MANUAL: Any manually added notes below this line are preserved on regeneration -->
