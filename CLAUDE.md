# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with this repository.

## What this is

STM32F407 firmware (bare-metal, HAL, no RTOS) for a single-arm pick & place robot ("Team JAX"). ROS on a PC sends `motor_id` + angle over CAN; the firmware drives 10 hobby servos via PWM. The project docs are written in Korean.

## Commands

`pio` is not on `PATH`; use `~/.platformio/penv/bin/pio` (or the PlatformIO VSCode toolbar).

```bash
pio run                      # build (env: disco_f407vg)
pio run -t upload            # flash
pio run -t clean
pio device monitor
tools/calibrate.py            # per-servo pulse-range measurement over SWD (interactive; patches main.c)
tools/calibrate.py --check    # read-only: board/firmware sanity check, never moves a servo
```

There are no tests (`test/` is empty) and no linter configured. Verification is on hardware, following the staged procedure in section 8 of `Stm32servosepc.md` (scope the PWM, then `cansend can0 005#030384` from a Linux `can-utils` host).

## Current state

- The firmware is implemented and **builds** (env `disco_f407vg`, ~8.7 KB flash) and was flashed to a DISC1 on 2026-10-15. Checked over SWD: `SystemCoreClock` = 168 MHz, HAL tick = 1 ms, CAN1 started with no errors and BTR for 1 Mbps, TIM2/3/4 at 50 Hz with all 10 channels enabled, green LED LD4 lit at boot. **Not yet checked on hardware:** servo waveforms and CAN frame reception (`Stm32servosepc.md` §8 steps 1–6). The servo logic (frame validation, `target_x10[]`/`dirty[]`, `servo[]` table, CCR update loop) lives in the `USER CODE` blocks of `src/main.c`.
- Per-servo calibration: `servo_us_at_0[]` / `servo_us_at_180[]` in `src/main.c` (defaults reproduce the old fixed 500..2500 mapping exactly) are filled in by `tools/calibrate.py`, which writes CCR registers over SWD/OpenOCD and patches those two lines. Verified: all 18,010 (motor, angle) points through the real CAN callback on the host, tool logic with a fake SWD memory, and on a real DISC1 `--check`, an SRAM write/read round trip and a key-driven session that sends no pulses. **Not yet done:** an actual measurement run with servos attached. The tool assumes the running firmware has PSC 83 / ARR 19999 and refuses to run otherwise (`--force` overrides); it also assumes ROS is not sending CAN commands, since the firmware would overwrite CCR.
- `can_test.ioc` (CubeMX, board STM32F407G-DISC1) is in the repo root. Its clock, CAN, TIM2/3/4 and servo-pin values match `src/main.c` (cross-checked), **but it still has I2S3, USB_OTG_FS and USB_HOST enabled**. Those three were removed **by hand** from `main.c`, `stm32f4xx_hal_msp.c`, `stm32f4xx_it.[ch]` and `stm32f4xx_hal_conf.h`. Disable them in CubeMX before regenerating, or `usb_host.h` / `hhcd_USB_OTG_FS` (which do not exist here) break the build again.
- Pressure sensors (added 2026-11-10, **builds; not checked on hardware**): `Fsr_Update()` in the main loop reads the 6 FSR channels (`Stm32pinmap.md` §4) every 100 ms by software-triggered single conversions. It switches the channel with `HAL_ADC_ConfigChannel` before each one (144 cycles, polled, about 50 µs in total, no DMA). It stores the 12-bit values in `volatile fsr_raw[]` and sends one CAN frame: StdId `CAN_FSR_STDID` = 6, DLC 6, `byte[k] = fsr_raw[k] >> 4`. **ID 6 is a placeholder that has not been agreed with the ROS side.** The `.ioc` still has only one ADC channel (IN11, now 144 cycles); the other five are configured at runtime. FSR dividers must be fed from 3V, not 5V.
- If `Can_Set()` fails (`HAL_CAN_Start` times out after 10 ms when the bus is not seen idle, e.g. no transceiver wired), `Error_Handler()` halts with interrupts disabled, so the main loop never runs. PWM is already started at CCR=0, so servos stay idle, but a PWM-only bench test (spec §8 steps 1–2) needs the CAN transceiver connected or `Can_Set()` temporarily skipped.
- `platformio.ini` was switched from `black_f407ve` to `disco_f407vg` to match the docs and pin map (STM32F407G-DISC1). PlatformIO regenerates `.vscode/launch.json` for the active env.
- Doc filenames contain typos (`REDAME.md`, `Stm32servosepc.md`); the README links now point at the real names. Renaming them is left to the user.

## Architecture

The authoritative design is in `Stm32servosepc.md` (frame contract, conversion, structure, validation) and `Stm32pinmap.md` (clocks, pins, forbidden pins). Read them before changing behaviour. Their rule: **code, comments, `.ioc` and these docs must be changed together** whenever an ID, pin or formula changes.

Intended data flow:

```
CAN RX0 interrupt → HAL_CAN_RxFifo0MsgPendingCallback → validate → target_x10[id] = …; dirty[id] = 1
main loop         → for each dirty channel: __HAL_TIM_SET_COMPARE(CCR)
                  → every 100 ms (HAL_GetTick): poll 6 ADC channels → CAN TX StdId 6, DLC 6
```

Constraints from the spec that aren't obvious from the code:

- The callback must never write CCR; it only stores values. Arrays shared with the main loop are `volatile`. No `HAL_Delay` / blocking waits in the main loop.
- Boot: all 10 channels get `HAL_TIM_PWM_Start` with CCR=0 (no pulse), so servos stay still until the first command.
- Frame: standard 11-bit ID **5**, data frame, DLC ≥ 3, bytes `[motor_id][angle_x10 H][angle_x10 L]`, angle = value/10 °. Clamp >1800 to 1800 and negative (as int16) to 0. Silently drop anything else (wrong ID/IDE/RTR, DLC<3, `motor_id > 9`). Last write wins; no queue. The RX filter is accept-all into FIFO0; the ID check happens in the callback.
- Angle → pulse: `CCR(µs) = at0 + angle_x10 * (at180 - at0) / 1800` with per-servo `servo_us_at_0[]` / `servo_us_at_180[]` (default 500 / 2500; `at0 > at180` reverses a servo), clamped to the hard window `SERVO_MIN_US`..`SERVO_MAX_US` (500..2500). Timer tick = 1 µs, so CCR is the pulse width.
- Clocks: HSE 8 MHz → PLL → 168 MHz; APB1 /4 so timer clock is 84 MHz → PSC 83, ARR 19999 on TIM2/3/4. Do not copy PSC values from the sibling `UART_STM` project (HSI 16 MHz, PSC 15). CAN timing (Prescaler 3, BS1 11, BS2 2) assumes PCLK1 = 42 MHz; changing clocks means recomputing both.
- `include/stm32f4xx_hal_conf.h` must keep `HSE_VALUE = 8000000U`, **and** `platformio.ini` must keep `board_build.stm32cube.custom_config_header = yes`. Without it the HAL library is compiled against the framework's own `hal_conf.h` (`HSE_VALUE` 25 MHz), so `SystemCoreClock` reads 525 MHz and `HAL_Delay` runs 3.125x slow, even though the project header says 8 MHz (found on hardware, 2026-10-15). The actual PLL registers are unaffected.

`motor_id` → timer channel (index into the `servo[]` table; B-plan pin assignment, confirmed 2026-10-14):

| id | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
|---|---|---|---|---|---|---|---|---|---|---|
| name | F1_1 | F1_2 | F2_1 | F2_2 | F3_1 | F3_2 | F_ROT | W_X | W_YAW | W_PITCH |
| pin | PA6 | PA7 | PB6 | PB9 | PA0 | PB7 | PB8 | PB0 | PB1 | PA1 |
| ch | TIM3_1 | TIM3_2 | TIM4_1 | TIM4_4 | TIM2_1 | TIM4_2 | TIM4_3 | TIM3_3 | TIM3_4 | TIM2_2 |

PA0, PA6, PA7, PB6, PB9 are shared with DISC1 on-board parts (button, MEMS, audio codec); `Stm32pinmap.md` §5 lists pins that must not be used at all.

## Code layout / CubeMX workflow

- Code was generated by CubeMX (CubeIDE project `can_ioc`) and ported to PlatformIO: `Core/Src/*` → `src/`, `Core/Inc/*` → `include/`. Re-generating overwrites files, so back up or avoid overwriting `main.c`, and put hand-written code **only inside `USER CODE BEGIN/END` blocks**.
- `src/stm32f4xx_hal_msp.c` holds the pin/AF/NVIC setup for CAN1 and TIM2/3/4 (this is where pin↔channel mapping is verifiable); `src/stm32f4xx_it.c` holds IRQ handlers.
- The HAL driver is compiled from the PlatformIO `stm32cube` framework package (`FrameworkHALDriver`), not from the top-level `Drivers/` directory, which is a vendored copy the build does not use. `include/stm32f4xx_hal_conf.h` is what selects enabled HAL modules.
- Out of scope for now: FreeRTOS, gestures.
