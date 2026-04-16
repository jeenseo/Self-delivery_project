# Self-delivery_project

---

## Architecture

- STM32F103RB: 순수 모터 슬레이브 (CAN Rx → Motor_Drive)
- Raspberry Pi: 자율주행 로직 전담 (CAN Tx → STM32 제어)
- 통신: CAN 500 kbps, ID `0x123`, Big-Endian `int16_t × 2`

---

## CAN Frame Format (Pi → STM32)

| Byte | Field | Type | Range |
|------|-------|------|-------|
| 0~1 | left_speed | int16_t (Big-Endian) | -9999 ~ +9999 |
| 2~3 | right_speed | int16_t (Big-Endian) | -9999 ~ +9999 |
| 4~7 | reserved | — | 0x00 |

- 양수 = 전진, 음수 = 후진, 0 = 정지

---

## STM32 Guide

- **IDE**: STM32CubeIDE
- **Build**: 🔨 Hammer 아이콘 클릭 (또는 `Project → Build All`)
- **Flash**: ▶ Run / Debug 아이콘 클릭 (ST-LINK 연결 필요)
- **주의**: `HAL_CAN_Start()`가 `main.c`에 있으므로, 플래시 후 수동 리셋(NRST 버튼) 필요할 수 있음

### 주요 파일

- `Core/Src/motor.c` — `Motor_Drive(left, right)`, htim2/htim1 PWM, GPIOC 방향핀
- `Core/Src/can.c` — CAN Rx 콜백 → `Motor_Drive()` 직접 호출
- `Core/Src/main.c` — 초기화 + CAN 슬레이브 루프

---

## Raspberry Pi Guide

### CAN 인터페이스 설정
```bash
sudo ip link set can0 up type can bitrate 500000
```

### Python 환경 설정
```bash
cd raspberry_pi_node
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### 실행
```bash
python3 can_bridge.py
```

### 주요 API (`can_bridge.py`)

- `init_bus()` — SocketCAN 버스 초기화
- `send_motor_command(bus, left_speed, right_speed)` — CAN 프레임 전송
- `motor_stop(bus)` — 정지 명령 (0, 0)

---

## Physical Wiring

| Pi 측 | STM32 측 |
|-------|---------|
| CAN_H | CAN_H (PA12 → TJA1050) |
| CAN_L | CAN_L (PA11 → TJA1050) |
| GND | GND (공통 접지 필수) |

- CAN 트랜시버(TJA1050 등) 양 끝단에 **120Ω 종단 저항** 필요
- Pi의 MCP2515 또는 PiCAN2 모듈 사용 권장

---

## Directory Structure

```
Self-delivery_project/
├── Core/
│   ├── Inc/          ← can.h, motor.h, gpio.h, tim.h, usart.h, main.h
│   ├── Src/          ← can.c, motor.c, main.c, gpio.c, tim.c, usart.c
│   └── Startup/
├── Drivers/          ← STM32 HAL (수정 금지)
├── raspberry_pi_node/
│   ├── can_bridge.py
│   ├── requirements.txt
│   └── .gitignore
├── .clineignore      ← Drivers/, Debug/ 인덱싱 제외
└── README.md
```
