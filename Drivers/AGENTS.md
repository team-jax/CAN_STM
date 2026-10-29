<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-10-14 | Updated: 2026-10-14 -->

# Drivers

## Purpose
Copy of ST's CMSIS and STM32F4 HAL driver that CubeMX placed here (`ProjectManager.LibraryCopy=1` in `can_test.ioc`). The PlatformIO build does **not** use it.

## Key Files

| File | Description |
|------|-------------|
| `CMSIS/LICENSE.txt`, `STM32F4xx_HAL_Driver/LICENSE.txt` | ST licenses for the vendored code |

## Subdirectories

| Directory | Purpose |
|-----------|---------|
| `CMSIS/` | `Include/` (Cortex-M core headers) and `Device/ST/STM32F4xx/Include/` (`stm32f407xx.h`, `system_stm32f4xx.h`, ...) |
| `STM32F4xx_HAL_Driver/` | HAL and LL driver `Inc/` and `Src/` for the modules CubeMX selected |

Third-party vendored code, so the subdirectories have no `AGENTS.md` of their own.

## For AI Agents

### Working In This Directory
- Do not edit anything here, and do not point the build at it. Compilation uses `framework-stm32cubef4` from `~/.platformio/packages/`: that package supplies every include path, the HAL sources, the startup file and the linker script. PlatformIO's generated include list contains no path under this directory.
- The versions differ: this copy has HAL driver 1.8.5 (and `can_test.ioc` records `FW_F4 V1.28.3`), while the build uses HAL driver 1.8.3 from `framework-stm32cubef4` 1.28.1. Behaviour that depends on a HAL detail must be checked against the framework copy, not this one.

### Testing Requirements
- None. A normal `~/.platformio/penv/bin/pio run` is unaffected by this directory.

### Common Patterns
- None.

## Dependencies

### Internal
- None.

### External
- STMicroelectronics STM32CubeF4 firmware package (CMSIS and HAL driver).

<!-- MANUAL: Any manually added notes below this line are preserved on regeneration -->
