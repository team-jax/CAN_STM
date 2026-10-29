<!-- Parent: ../AGENTS.md -->
<!-- Generated: 2026-10-14 | Updated: 2026-10-14 -->

# test

## Purpose
PlatformIO's test directory. It holds only the stock `README`; the project has no tests, and `platformio.ini` defines no test or `native` environment.

## Key Files

| File | Description |
|------|-------------|
| `README` | PlatformIO boilerplate about the Test Runner |

## Subdirectories
None.

## For AI Agents

### Working In This Directory
- Verification today is: a clean `pio run`, a throwaway host-side check of the `main.c` logic (see the root `AGENTS.md`), and the manual hardware procedure in `Stm32servosepc.md` §8.
- If you add automated tests, a `[env:native]` host environment is the practical route: the servo logic only needs `TIM_HandleTypeDef`, the `CAN_*` types and a few HAL calls stubbed. It needs `platformio.ini` changes, so confirm with the user first.
- Do not add on-target tests that move servos without telling the user.

### Testing Requirements
- Not applicable yet.

### Common Patterns
- None.

## Dependencies

### Internal
- None.

### External
- None.

<!-- MANUAL: Any manually added notes below this line are preserved on regeneration -->
