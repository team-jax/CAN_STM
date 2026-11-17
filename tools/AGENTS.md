<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-11-10 | Updated: 2026-11-10 -->

# tools

## Purpose
Host-side helper scripts. They run on the developer's PC, not on the STM32, and are not part of the PlatformIO build.

## Key Files

| File | Description |
|------|-------------|
| `calibrate.py` | Interactive per-servo pulse measurement. Starts OpenOCD on a free telnet port, writes the CCR register of the selected servo over SWD (CCR value = pulse width in µs), records the 0° and 180° ends, then rewrites `servo_us_at_0[]` / `servo_us_at_180[]` in `src/main.c` and can run `pio run -t upload`. `--check` only reads (firmware state, CCR values) and never writes a servo |

## Subdirectories
None.

## For AI Agents

### Working In This Directory
- The script reads everything it needs from `src/main.c` by regex: `SERVO_COUNT`, `SERVO_MIN_US`/`SERVO_MAX_US`, the `servo[]` table (motor_id → timer/channel, pin names from the line comments) and the two calibration tables. If you rename or reformat those in `main.c`, update the parsers in `calibrate.py` and re-run the checks below.
- CCR addresses are derived (`TIM_BASE + CCR_OFF`), not hard-coded per motor. The offsets match `TIM_TypeDef` in `stm32f407xx.h` (checked with `_Static_assert`/`offsetof`).
- OpenOCD facts verified on a real board (0.12.0 xPack): the telnet server echoes each command, prefixes output with a NUL byte and ends with `\r> `; `mww` to an unmapped address returns **no** error text, so `write32` reads the value back and compares. `gdb_port` and `tcl_port` are disabled so it does not clash with a PIO debug session.
- Safety rules the script follows and that must be kept: it never writes a servo at startup (only on a jog key), clamps to the `main.c` hard window, restores the start value of every touched servo on exit, and refuses to run when the board's TIM PSC/ARR are not 83/19999 (`--force` overrides).
- Never run the interactive mode, or write to a servo CCR address, against a user's board without them knowing: servos physically move. `--check` is safe.

### Testing Requirements
- `python3 -m py_compile tools/calibrate.py`.
- `tools/calibrate.py --check` with a board attached must print the 10 motors, CCR addresses and "펌웨어 상태 OK".
- Logic (key handling, clamping, restore, report, patch) was tested by importing the module and replacing the OpenOCD object with a dict-backed fake exposing `read32` / `write32`; the firmware side by extracting the `USER CODE` blocks of `main.c` and sweeping every (motor, angle) on the host. Neither script is stored in the repo.

### Common Patterns
- Korean user-facing messages, standard library only, POSIX terminal (`termios`).

## Dependencies

### Internal
- `../src/main.c` (parsed and patched), `../platformio.ini` (for `pio run -t upload`).

### External
- PlatformIO's `tool-openocd` (`~/.platformio/packages/tool-openocd`, falls back to `openocd` on PATH), an ST-Link, Python 3.9+.

<!-- MANUAL: Any manually added notes below this line are preserved on regeneration -->
