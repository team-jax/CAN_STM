# Team JAX — STM32 서보 제어 펌웨어

단일 팔 Pick & Place 로봇의 STM32F407 펌웨어. ROS(PC)가 CAN으로 `motor_id`와 각도를 보내면 해당 서보 PWM을 그 각도로 움직인다.

## 하는 일

- CAN으로 `motor_id`(0~9)와 각도를 받아 서보 10개를 구동한다.
- 하지 않는 일: ADC 압력센서(2026-10-29 이후), FreeRTOS, 제스처, 서보별 보정.

## 하드웨어

| 항목 | 값 |
|---|---|
| 보드 | STM32F407G-DISC1 (STM32F407VGT6) |
| 클럭 | HSE 8MHz + PLL → 168MHz, 타이머 클럭 84MHz |
| CAN | CAN1, PD0(RX) / PD1(TX), SN65HVD230, 1Mbps |
| 서보 | PWM 10개, 50Hz (TIM2/3/4, PSC 83, ARR 19999) |

## 빌드

PlatformIO (VSCode).

```ini
[env:disco_f407vg]
platform = ststm32
board = disco_f407vg
framework = stm32cube
```

- CubeMX로 생성한 `Core/Src/*`는 `src/`, `Core/Inc/*`는 `include/`에 둔다.
- I2S3, USB_OTG_FS, USB_HOST는 CubeMX에서 끄고 생성한다.
- 재생성할 때는 `main.c`를 덮어쓰지 않거나 백업한다. 직접 쓴 코드는 `USER CODE` 블록 안에만 둔다.
- `stm32f4xx_hal_conf.h`의 `HSE_VALUE`가 `8000000U`인지 확인한다. `platformio.ini`의 `board_build.stm32cube.custom_config_header = yes`가 있어야 HAL 라이브러리도 이 파일을 쓴다 (없으면 프레임워크 기본값 25MHz로 컴파일되어 `HAL_Delay`가 3배 이상 느려진다).
- 빌드 확인됨 (2026-10-14, `pio run` 성공). 2026-10-15 DISC1에 업로드해 SWD로 확인: `SystemCoreClock` 168MHz, HAL 틱 1ms, CAN1 1Mbps 정상 시작(오류 없음), TIM2/3/4 50Hz·10채널 동작, 부팅 시 초록 LED(LD4) 켜짐. 서보 파형과 CAN 수신은 아직 미확인이다.

## 실행

```bash
pio run                # 빌드
pio run -t upload      # 보드에 업로드 (DISC1을 USB로 연결)
pio run -t clean       # 빌드 결과 삭제
```

- `pio`가 PATH에 없으면 `~/.platformio/penv/bin/pio`로 실행한다. VSCode에서는 아래 PlatformIO 툴바의 Build / Upload를 눌러도 된다.
- 업로드 후 보드가 부팅하면 초록 LED(LD4)가 켜진다 (클럭 초기화 통과 표시). 서보는 첫 CAN 명령이 올 때까지 움직이지 않는다. 명령 보내는 법은 [동작 확인](#동작-확인)을 본다.

## CAN 프로토콜 (ROS → STM)

| 항목 | 값 |
|---|---|
| ID | 표준 11비트, StdId = 7, 데이터 프레임 |
| DLC | 3 이상 |
| 데이터 | `[motor_id][angle_H][angle_L]`, 각도 = `((H<<8)\|L) / 10.0` 도 |
| 범위 | 0~1800 (0.0°~180.0°), 넘으면 180°로 자름, 음수는 0° |

규칙 위반 프레임(다른 ID, DLC<3, motor_id>9)은 무시한다.
각도 → PWM: `CCR(µs) = 500 + angle_x10 × 2000 / 1800` (0°=500, 90°=1500, 180°=2500).

## motor_id 배정

| id | 이름 | 핀 | 타이머 채널 |
|---|---|---|---|
| 0 | F1_1 | PA6 | TIM3_CH1 |
| 1 | F1_2 | PA7 | TIM3_CH2 |
| 2 | F2_1 | PB6 | TIM4_CH1 |
| 3 | F2_2 | PB9 | TIM4_CH4 |
| 4 | F3_1 | PA0 | TIM2_CH1 |
| 5 | F3_2 | PB7 | TIM4_CH2 |
| 6 | F_ROT | PB8 | TIM4_CH3 |
| 7 | W_X (MG996R) | PB0 | TIM3_CH3 |
| 8 | W_YAW | PB1 | TIM3_CH4 |
| 9 | W_PITCH | PA1 | TIM2_CH2 |

PA0, PA6, PA7, PB6, PB9는 DISC1 온보드 부품(USER 버튼, MEMS, 오디오 코덱)과 핀을 공유한다. 실물 파형으로 확인한다.

## 동작 확인

리눅스에서 `can-utils`로 직접 보낸다. motor_id 3을 90.0°로:

```bash
sudo ip link set can0 up type can bitrate 1000000   # 처음 한 번 (1Mbps)
cansend can0 007#030384
```

단계별 검증과 오류 입력 표는 [`Stm32servosepc.md`](Stm32servosepc.md) 8장을 따른다.

## 구조

```
CAN RX 인터럽트 → 검증 → target_x10[id] 저장, dirty[id]=1
main 루프       → dirty인 채널만 CCR 갱신 (HAL_Delay 금지)
```

부팅 직후에는 CCR=0이라 서보가 움직이지 않고, 첫 명령이 오면 움직이기 시작한다.

## 문서

- [`Stm32servosepc.md`](Stm32servosepc.md): 프레임 계약, 변환식, 구조, 검증 절차, 미결 사항
- [`Stm32pinmap.md`](Stm32pinmap.md): 클럭, CAN, PWM, ADC 핀 맵과 쓰면 안 되는 핀

## 상태

- 완료: 핀 배치 확정(B안), 클럭·PSC·ARR 확정, 서보 제어 코드 초안 (CubeIDE 프로젝트 `can_ioc` 기준, 빌드·실물 검증 전)
- 완료: PlatformIO 프로젝트로 이전, 서보 제어 코드 구현·빌드 (`src/main.c`의 USER CODE 블록). I2S3·USB_HOST는 코드에서 제거함
- 진행: 실물 검증 (`Stm32servosepc.md` 8장 순서대로)
- 미결: ROS 쪽 프레임과의 일치 확인, 각도 부호, 부팅 홈 위치, 서보 보정, 종단저항 위치
- 연기: ADC 압력센서 6채널 (2026-10-29 이후)