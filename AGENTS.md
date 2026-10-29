<!-- Generated: 2026-10-14 | Updated: 2026-10-14 -->

# cancan

## Purpose
Bare-metal STM32F407 firmware (HAL, no RTOS) for the Team JAX single-arm pick & place robot. ROS on a PC sends a CAN frame (standard ID 7) carrying `motor_id` and an angle; the firmware turns it into the PWM pulse of one of 10 hobby servos (TIM2/TIM3/TIM4, 50 Hz). Target board is the STM32F407G-DISC1, built with PlatformIO (`framework = stm32cube`). Project docs are written in Korean. The directory is not a git repository.

## Key Files

| File | Description |
|------|-------------|
| `CLAUDE.md` | Commands, current state, architecture summary. Read this first |
| `Stm32servosepc.md` | Authoritative firmware spec: CAN frame contract, angle→PWM formula, structure rules, staged hardware verification (§8), open questions |
| `Stm32pinmap.md` | Clock, CAN, servo PWM and ADC pin map; pins that must not be used |
| `REDAME.md` | Project README (the filename typo is real; its links point at the two spec files above) |
| `platformio.ini` | Single env `disco_f407vg` (platform `ststm32`, framework `stm32cube`) |
| `can_test.ioc` | CubeMX project the generated code comes from. Still enables I2S3, USB_OTG_FS and USB_HOST, which were removed from the code by hand |
| `Projects.code-workspace` | VSCode multi-root workspace; not part of the build |
| `.gitignore` | Ignores `.pio/` and the auto-generated `.vscode` files |

## Subdirectories

| Directory | Purpose |
|-----------|---------|
| `src/` | All firmware C sources; servo/CAN logic is in `main.c` (see `src/AGENTS.md`) |
| `include/` | Headers and the HAL module configuration (see `include/AGENTS.md`) |
| `lib/` | PlatformIO private libs; only an unused vendored USB Host stack (see `lib/AGENTS.md`) |
| `Drivers/` | Vendored ST CMSIS/HAL copy that the build does **not** use (see `Drivers/AGENTS.md`) |
| `test/` | Empty PlatformIO test dir (see `test/AGENTS.md`) |
| `.vscode/` | Auto-generated IDE/debug config (see `.vscode/AGENTS.md`) |

Not documented on purpose: `.pio/` (build output) and `.omc/` (oh-my-claudecode tool state). The inner folders of `Drivers/` and `lib/Middlewares/` are third-party vendored code, covered by their parent file.

## For AI Agents

### Working In This Directory
- Read `CLAUDE.md`, then the relevant section of `Stm32servosepc.md` / `Stm32pinmap.md`, before changing behaviour.
- A change to any `motor_id`, pin, timer channel, clock or conversion formula must be made in code, code comments, `can_test.ioc` and both spec files **together**; the specs say so explicitly.
- Hand-written code goes only inside `USER CODE BEGIN/END` blocks. Everything else is CubeMX output that regeneration overwrites.
- `pio` is not on `PATH`; use `~/.platformio/penv/bin/pio`. Changing build flags (for example `PLATFORMIO_BUILD_FLAGS`) or running `-t idedata` afterwards wipes `.pio/build/`; run a plain `pio run` to restore it.
- Do not edit `Drivers/`, `lib/Middlewares/` or the auto-generated `.vscode/*.json`.
- IDE clang diagnostics such as "`main.h` file not found" or "unknown type name" come from a missing include-path setup, not from the build. Trust `pio run`.

### Testing Requirements
- `~/.platformio/penv/bin/pio run` must succeed. It currently builds clean, including with `-Wall -Wextra`.
- There are no unit tests. The servo/CAN logic can be checked on the host by extracting the `PTD`, `PD`, `PV`, `PFP` and `0` USER CODE blocks of `src/main.c` and compiling them against a small stub of the HAL types and `__HAL_TIM_SET_COMPARE`. That is how the spec §8 validation table was checked (not stored in the repo).
- Hardware: boot, clock tree, CAN1 setup and the timers were verified over SWD on a DISC1 (see `CLAUDE.md`). Servo waveforms and CAN frame reception are **not yet verified**; follow the staged procedure in `Stm32servosepc.md` §8 (scope the PWM, then `cansend can0 007#030384` from a Linux `can-utils` host). OpenOCD from `~/.platformio/packages/tool-openocd` can read registers while the board runs.

### Common Patterns
- The CAN RX interrupt only validates and stores (`target_x10[]`, `dirty[]`); the main loop applies CCR. No `HAL_Delay` in the main loop.
- Hand-written code uses tabs and Korean `//` comments; generated code uses two spaces.
- HAL failures call `Error_Handler()`, which disables interrupts and spins.

## Dependencies

### External
- PlatformIO platform `ststm32` 19.6.0, `framework-stm32cubef4` 1.28.1 (HAL driver 1.8.3), `toolchain-gccarmnoneeabi` 7.2.1.
- Hardware: STM32F407VGT6 with 8 MHz HSE, SN65HVD230 CAN transceiver (1 Mbps), 10 servos.

<!-- MANUAL: Any manually added notes below this line are preserved on regeneration -->
