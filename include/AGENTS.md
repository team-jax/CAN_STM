<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-10-14 | Updated: 2026-10-14 -->

# include

## Purpose
Project headers: the CubeMX-generated `main.h` and interrupt header, plus the HAL module configuration that decides which HAL drivers are compiled and what `HSE_VALUE` is.

## Key Files

| File | Description |
|------|-------------|
| `main.h` | Includes `stm32f4xx_hal.h`; declares `Error_Handler` and `HAL_TIM_MspPostInit`; defines the board pin labels (`LD3_Pin`, `CS_I2C_SPI_Pin`, ...). Labels for I2S3 and USB (`I2S3_*`, `OTG_FS_*`, `VBUS_FS_*`) are still defined but no longer used by any initialised peripheral |
| `stm32f4xx_hal_conf.h` | HAL module selection and clock constants. Enabled: ADC, CAN, TIM, GPIO, EXTI, DMA, RCC, FLASH, PWR, CORTEX. `HAL_I2S_MODULE_ENABLED` and `HAL_HCD_MODULE_ENABLED` were commented out by hand. `HSE_VALUE` is `8000000U` |
| `stm32f4xx_it.h` | Prototypes of the handlers in `src/stm32f4xx_it.c` (`CAN1_RX0_IRQHandler` and the Cortex-M exceptions) |
| `README` | PlatformIO boilerplate |

## Subdirectories
None.

## For AI Agents

### Working In This Directory
- This `stm32f4xx_hal_conf.h` only reaches the HAL library because `platformio.ini` sets `board_build.stm32cube.custom_config_header = yes` (it makes the builder drop the framework's own copy). Without that, `src/` sees this file but the HAL library is built with the framework default (`HSE_VALUE` 25 MHz), so `SystemCoreClock` and `HAL_Delay` are wrong. Keep `HSE_VALUE` at `8000000U`; the CAN timing and the PWM prescaler (PSC 83) assume 168 MHz SYSCLK, 84 MHz timer clock and 42 MHz PCLK1.
- Enabling or disabling a HAL module here must match the peripherals initialised in `src/main.c` and `can_test.ioc`; a module whose handle type is used but not enabled fails to compile.
- Add hand-written declarations only inside `USER CODE` blocks (`main.h` has `Includes`, `ET`, `EC`, `EM`, `EFP`, `Private defines`).

### Testing Requirements
- Build with `~/.platformio/penv/bin/pio run`.

### Common Patterns
- Pin labels follow CubeMX naming: `<Label>_Pin` / `<Label>_GPIO_Port`.

## Dependencies

### Internal
- Consumed by every file in `../src/`.

### External
- `stm32f4xx_hal.h` and the CMSIS device headers from `framework-stm32cubef4`.

<!-- MANUAL: Any manually added notes below this line are preserved on regeneration -->
