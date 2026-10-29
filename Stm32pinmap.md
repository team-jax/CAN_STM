# STM32F407G-DISC1 핀 맵 (Team JAX 단일 팔)

> 기준: 2026-10-13. CubeMX/IOC 설정과 PCB 스키매틱을 이 표에 맞춘다.
> 핀 번호나 설정을 바꾸면 이 파일, IOC, 스키매틱, 펌웨어 주석을 같이 고친다.
> 보드 위에 자체 PCB를 얹는 구조다. 서보 커넥터는 J1~J10(motor_id 0~9)이고, 압력센서 커넥터 J11~J16은 스키매틱에 아직 없다.

## 1. 클럭 전제

- 최종 펌웨어는 `team-jax/CAN_IOC`(STM32CubeIDE + CubeMX, RTOS 없음)를 기반으로 한다. [확정, 2026-10-13]
- 클럭: HSE 8MHz + PLL (PLLM 8, PLLN 336, PLLP /2) → SYSCLK/HCLK **168MHz**, APB1 /4 (PCLK1 42MHz, **타이머 클럭 84MHz**), APB2 /2 (84MHz). 근거: CAN_IOC `.ioc`의 RCC 값(`APB1TimFreq_Value=84000000`).
- 클럭을 바꾸면 CAN 비트 타이밍과 PWM Prescaler를 다시 계산해야 한다. 최종 펌웨어에서는 HSE + PLL을 유지한다. CAN에는 크리스털 클럭이 안정적이기 때문이다 (HSI 정확도는 데이터시트로 확인하지 못했다).
- 참고: `team-jax/UART_STM`(PlatformIO, 시험용)은 HSI 16MHz, PLL 없음, APB1 /1이라 PSC 15가 맞았다. 두 프로젝트의 PSC 값을 서로 복사하면 안 된다.

### 1-1. 타이머와 펄스 (서보 PWM 공통)

근거: `UART_STM`의 `main.c`(HSI 16MHz)에서 PSC 15가 50Hz임을 코드로 추적했고, `CAN_IOC`는 `.ioc`의 타이머 클럭 84MHz로 다시 계산했다.

| 항목 | CAN_IOC에 적용할 값 |
|---|---|
| 타이머 입력 클럭 | 84MHz (APB1 /4이므로 타이머 클럭 = PCLK1 × 2) |
| Prescaler (PSC) | **83** (TIM2, TIM3, TIM4 모두) |
| Counter Period (ARR) | **19999** (TIM4도 65535에서 19999로 변경) |
| 카운터 모드 / ClockDivision | Up / DIV1 |
| AutoReloadPreload | Disable (실행 중 ARR을 바꾸지 않아서 영향 없음) |

- 1틱 = (PSC+1) / 84MHz = 84 / 84MHz = **1µs**, 주기 = (ARR+1) × 1µs = **20ms (50Hz)**.
- 생성된 코드의 이전 값(PSC 15·0·15, TIM4 ARR 65535)은 각각 262.5Hz, 4200Hz, 약 80Hz라서 서보에 쓰면 안 된다.
- CCR 값이 곧 펄스폭(µs)이다. 듀티는 500 → 2.5%, 1500 → 7.5%, 2500 → 12.5%.
- 변환식: `CCR = 500 + (angle / 180) × 2000`, angle은 0~180으로 자른다. 예: 0° → 500, 90° → 1500, 180° → 2500.
- ⚠ "0~180°"는 펄스 500~2500µs의 이름일 뿐 서보의 실제 회전각이 아닐 수 있다. 판매처 페이지는 MG90S를 1~2ms·180°(±15°) ([ProtoSupplies](https://protosupplies.com/product/servo-motor-micro-mg90s/))와 500~2500µs ([KPower](https://en.kpower.com/blog/3511.html))로, MG996R을 1~2ms ([Components101](https://components101.com/node/467))로 다르게 적는다. 제조사 데이터시트는 확인하지 못했다.
- 그래서 서보마다 실물로 보정한다. 끝값(180° 근처, 2498~2500µs)에서 서보가 떨리거나 뜨거워지면 기계적 끝단에 걸린 것이니 값을 줄인다.

## 2. CAN1 (확정)

| 핀 | 기능 |
|---|---|
| PD0 | CAN1_RX |
| PD1 | CAN1_TX |

- 속도 1Mbps: Prescaler 3, BS1 11, BS2 2, SJW 1 (PCLK1 42MHz, 1 TQ = 71.4ns, 14 TQ, 샘플 포인트 85.7%). 근거: CAN_IOC `.ioc`가 계산한 `CalculateBaudRate=1000000`.
- CubeMX의 Bit Rate 칸이 1000000으로 나오는지 확인한다.
- 수신 인터럽트(`CAN1_RX0_IRQn`)는 CAN_IOC에서 이미 켜져 있다.
- 트랜시버는 SN65HVD230. 버스 양 끝 종단저항 120Ω의 위치는 미확인.

## 3. 서보 PWM 10개 (J1~J10)

B안(손가락을 ⚠ 핀에 몰기) [확정, 2026-10-14]. 핀과 타이머 채널은 A안과 같고 motor_id 배정만 바뀌었다.

| motor_id | 이름 | 핀 | 타이머 채널 | 주의 |
|---|---|---|---|---|
| 0 | F1_1 | PA6 | TIM3_CH1 | ⚠ 온보드 MEMS(SPI1_MISO)와 공유 |
| 1 | F1_2 | PA7 | TIM3_CH2 | ⚠ 온보드 MEMS(SPI1_MOSI)와 공유 |
| 2 | F2_1 | PB6 | TIM4_CH1 | ⚠ 오디오 코덱 I2C1 SCL과 공유 |
| 3 | F2_2 | PB9 | TIM4_CH4 | ⚠ 오디오 코덱 I2C1 SDA와 공유 |
| 4 | F3_1 | PA0 | TIM2_CH1 | ⚠ USER 버튼(B1)과 공유 |
| 5 | F3_2 | PB7 | TIM4_CH2 | |
| 6 | F_ROT | PB8 | TIM4_CH3 | |
| 7 | W_X (MG996R) | PB0 | TIM3_CH3 | |
| 8 | W_YAW (DM-CLS200MD) | PB1 | TIM3_CH4 | |
| 9 | W_PITCH | PA1 | TIM2_CH2 | |

- TIM2, TIM3, TIM4 모두 `PWM Generation CHx`, 클럭 소스 Internal Clock.
- 50Hz: Prescaler = 83, Counter Period = 19999 (타이머 클럭 84MHz 기준, 1µs 한 칸).
- 펄스(CCR) 500~2500 = 0~180°. DM-CLS 서보는 6~7.4V, 500~2500µs.
- 커넥터 번호(J1~J10)와 motor_id의 대응은 B안 기준으로 스키매틱에서 다시 확인한다.

## 4. 압력센서 ADC 6개 (ADC1) — 10월 29일 이후로 연기 [확정]

| 센서 | 핀 | 채널 |
|---|---|---|
| 1 | PC1 | ADC1_IN11 |
| 2 | PC2 | ADC1_IN12 |
| 3 | PC4 | ADC1_IN14 |
| 4 | PC5 | ADC1_IN15 |
| 5 | PA2 | ADC1_IN2 |
| 6 | PA3 | ADC1_IN3 |

- 설정: Scan Conversion Enabled, Continuous Enabled, Number of Conversion 6, DMA Circular.
- Sampling Time은 84~144 cycles. FSR 같은 고저항 센서는 길게 줘야 값이 안정된다.
- 센서와 손가락의 대응, 부착 위치는 미확정.

## 5. 쓰면 안 되는 핀 (DISC1 온보드 부품이 이미 사용)

| 핀 | 온보드 용도 |
|---|---|
| PC0 | USB 전원스위치 enable |
| PC3 | 온보드 마이크 데이터 |
| PA5 | MEMS(SPI1_SCK) |
| PD12~PD15 | 사용자 LED 4개 |
| PC7, PC10, PC12, PA4 | 오디오 I2S |
| PD4, PB10 | 오디오 코덱 |
| PD5 | USB 과전류 LED |
| PA9~PA12 | USB |
| PA13, PA14, PB3 | SWD 디버거 |

## 6. 미결 사항

1. **PA0를 그대로 쓸지, 버튼 없는 핀으로 옮길지.** 옮기면 스키매틱 J9 배선과 motor_id 8 한 줄만 바뀐다.
2. **온보드 부품과 같이 물리는 핀 5개(PA0, PA6, PA7, PB6, PB9).** 서보 신호가 부품 쪽으로도 흐르므로 PCB 발주 전에 실물로 파형과 서보 동작을 확인한다.
3. 타이머 채널과 핀의 대응(예: PB6이 TIM4_CH1인지)은 CubeMX에서 그 핀이 후보로 뜨는지 확인한다. 데이터시트 표는 아직 열어 보지 못했다.
4. 압력센서 커넥터 J11~J16을 스키매틱에 추가한다.