# Day 5-6 | ArduPilot SITL 설치 및 pymavlink 연동

> **학습 목표**: 가상 드론과 실제 MAVLink 통신 체험  
> **소요 시간**: 일 2~3시간 (총 2일)  
> **선수 조건**: Day 3-4 pymavlink 기초 실습 완료

---

## 학습 흐름 개요

```
ArduPilot 소스 빌드
        │
        ▼
SITL 실행 (가상 드론 부팅)
        │
        ▼
pymavlink로 UDP 접속
        │
        ▼
ARM → TAKEOFF → 상태 모니터링 → RTL
```

---

## 디렉토리 구조

```
day5-6/
├── README.md
├── 01_heartbeat.py          # HEARTBEAT 수신 확인
├── 02_arm_takeoff_rtl.py    # ARM → TAKEOFF → RTL 명령
└── 03_telemetry_monitor.py  # 드론 상태 실시간 출력
```

---

## Step 1 | ArduPilot 소스 클론 및 빌드

```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
```

### 의존성 설치

```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile          # PATH 갱신 (파이썬 venv 포함)
```

> ⚠️ 설치 후 재로그인 또는 `. ~/.profile` 실행 필수. 건너뛰면 `sim_vehicle.py` 명령을 못 찾음.

### SITL 빌드

```bash
./waf configure --board sitl
./waf copter
```

> ⏱ 첫 빌드는 10~20분 소요. `ccache` 설치 후 재빌드는 1~2분으로 단축됨.

---

## Step 2 | SITL 실행

```bash
cd ~/ardupilot

# GUI 없는 환경 (WSL2 권장)
Tools/autotest/sim_vehicle.py -v ArduCopter --out udp:127.0.0.1:14551

# GUI 있는 환경 (X11 사용 가능할 때)
Tools/autotest/sim_vehicle.py -v ArduCopter --console --map --out udp:127.0.0.1:14551
```

| 옵션 | 설명 |
|---|---|
| `-v ArduCopter` | 멀티콥터(쿼드) 펌웨어 선택 |
| `--console` | MAVProxy 상태 콘솔 창 (배터리·모드·GPS 표시) |
| `--map` | 지도 위에 드론 위치 표시 |
| `--out udp:127.0.0.1:14551` | pymavlink용 추가 출력 포트 개방 |

### 정상 부팅 확인 로그

```
Detected vehicle 1:1 on link 0
AP: EKF3 IMU0 origin set
AP: Field Elevation Set: 584m
Flight battery 100 percent
```

> `EKF3 origin set` 이 찍히면 GPS Fix 완료 — pymavlink 접속 준비 완료.

### ⚠️ 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| `Unable to access the X Display` | WSL2에 X 서버 없음 | `--console --map` 제거 후 재실행 (통신에는 무관) |
| `wait_heartbeat()`에서 멈춤 | 포트 충돌 또는 SITL 미기동 | `--out` 포트 확인, SITL 재시작 |
| ARM 거부 (ACK result != 0) | GPS Fix 실패, Pre-arm check 통과 못함 | SITL 로그 확인, 테스트용으로 `param set ARMING_CHECK 0` |

---

## Step 3 | pymavlink 설치

```bash
pip install pymavlink
```

---

## Step 4 | 실습 코드

### 01_heartbeat.py — 접속 및 HEARTBEAT 수신

```python
from pymavlink import mavutil

master = mavutil.mavlink_connection('udp:127.0.0.1:14551')

print("HEARTBEAT 대기 중...")
master.wait_heartbeat()
print(f"연결됨! system_id={master.target_system}, "
      f"component_id={master.target_component}")
```

**확인 포인트**
- `system_id=1` → SITL 드론
- `component_id=1` → Autopilot (Flight Controller)

---

### 02_arm_takeoff_rtl.py — 비행 명령 순서 전송

```python
import time
from pymavlink import mavutil

master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
master.wait_heartbeat()
print("연결 완료")


def set_mode(mode):
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )


def wait_ack(cmd_name=""):
    msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    print(f"[ACK] {cmd_name}: {msg}")
    return msg


# 1) GUIDED 모드 전환 (ARM/TAKEOFF는 GUIDED에서만 가능)
set_mode("GUIDED")
time.sleep(1)

# 2) ARM
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0       # param1=1: arm
)
wait_ack("ARM")

# 3) TAKEOFF (목표 고도 10m)
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0, 10      # param7=altitude(m)
)
wait_ack("TAKEOFF")

# 4) 목표 고도(9m 이상)까지 대기
print("이륙 중... 고도 모니터링")
while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    alt = msg.relative_alt / 1000.0       # mm → m
    print(f"현재 상대고도: {alt:.1f} m")
    if alt >= 9.0:
        print("목표 고도 도달")
        break

time.sleep(5)

# 5) RTL (Return to Launch)
set_mode("RTL")
print("RTL 모드 전환, 귀환 시작")

# 6) 착륙 완료(DISARM)까지 모니터링
while True:
    hb = master.recv_match(type='HEARTBEAT', blocking=True)
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    mode = mavutil.mode_string_v10(hb)

    pos = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    alt = pos.relative_alt / 1000.0 if pos else -1

    print(f"모드: {mode:10s} | 고도: {alt:.1f}m | ARMED: {armed}")

    if not armed:
        print("착륙 완료 및 DISARM 확인")
        break
```

---

### 03_telemetry_monitor.py — 드론 상태 실시간 출력

```python
from pymavlink import mavutil

master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
master.wait_heartbeat()
print("텔레메트리 모니터링 시작\n")

while True:
    msg = master.recv_match(blocking=True)
    if msg is None:
        continue

    t = msg.get_type()

    if t == 'GLOBAL_POSITION_INT':
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.relative_alt / 1000.0
        print(f"[위치] lat={lat:.6f}  lon={lon:.6f}  고도={alt:.1f}m")

    elif t == 'SYS_STATUS':
        print(f"[배터리] 잔량={msg.battery_remaining}%")

    elif t == 'HEARTBEAT':
        mode  = mavutil.mode_string_v10(msg)
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"[상태]   모드={mode}  ARMED={armed}")
```

---

## Step 5 | 실습 실행 순서

터미널 3개를 동시에 열어 실행하면 "명령 → 상태 변화" 흐름을 실시간으로 관찰할 수 있습니다.

```
터미널 1  │  {ardupilot 경로}/Tools/autotest/sim_vehicle.py -v ArduCopter --out udp:127.0.0.1:14551
터미널 2  │  python3 03_telemetry_monitor.py   ← 모니터링 먼저 켜기
터미널 3  │  python3 02_arm_takeoff_rtl.py     ← 명령 전송
```

### 예상 상태 변화 흐름

```
[상태] 모드=STABILIZE   ARMED=False   ← 초기 상태
[상태] 모드=GUIDED      ARMED=False   ← set_mode("GUIDED")
[상태] 모드=GUIDED      ARMED=True    ← ARM 명령
[위치] 고도=0.0m
[위치] 고도=3.5m                      ← TAKEOFF 상승 중
[위치] 고도=7.8m
[위치] 고도=10.0m                     ← 목표 고도 도달
[상태] 모드=RTL         ARMED=True    ← RTL 전환
[위치] 고도=15.0m                     ← RTL_ALT(15m)까지 상승
[위치] 고도=9.2m                      ← 이륙 지점으로 복귀 하강 중
[위치] 고도=0.1m
[상태] 모드=RTL         ARMED=False   ← 착륙 후 자동 DISARM
착륙 완료 및 DISARM 확인
```

---

## 핵심 개념 정리

### COMMAND_ACK result 코드

| result 값 | 상수명 | 의미 |
|---|---|---|
| 0 | MAV_RESULT_ACCEPTED | 명령 수락 |
| 1 | MAV_RESULT_TEMPORARILY_REJECTED | 일시 거부 (Pre-arm 미통과 등) |
| 2 | MAV_RESULT_DENIED | 거부 (현재 모드에서 불가) |
| 4 | MAV_RESULT_FAILED | 실행 실패 |

### RTL 동작 시퀀스

```
RTL 명령 수신
    │
    ▼
현재 고도 < RTL_ALT(기본 15m)?
    ├─ Yes → RTL_ALT까지 상승
    └─ No  → 현재 고도 유지
    │
    ▼
이륙 지점(Launch Point) 상공으로 수평 이동
    │
    ▼
수직 하강 및 착륙
    │
    ▼
자동 DISARM
```

### RTL 관련 파라미터

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `RTL_ALT` | 1500 cm (15m) | 귀환 비행 최소 고도 |
| `RTL_ALT_FINAL` | 0 cm | 착륙 전 호버링 고도 (0=바로 착륙) |
| `RTL_LOIT_TIME` | 5000 ms | 홈 상공 도착 후 호버링 시간 |
| `RTL_SPEED` | 0 | 귀환 수평 속도 (0=기본값 사용) |

MAVProxy 콘솔에서 확인:
```
MAV> param show RTL_ALT
```

---

## 체크포인트

```
□ SITL 정상 부팅 확인 (EKF origin set 로그)
□ pymavlink UDP 접속 및 HEARTBEAT 수신 성공
□ ARM 명령 전송 → ACK result=0 확인
□ TAKEOFF 후 고도 상승 수치 실시간 확인
□ RTL 전환 → 자동 착륙 → DISARM 흐름 관찰
□ MAVLink 명령 → 드론 상태 변화 흐름 설명 가능
```

---

## 다음 단계 (Day 7)

- ROS2 Humble 설치 및 talker/listener 통신 확인
- Day 5-6에서 pymavlink로 다룬 드론 상태 데이터를  
  ROS2 Topic으로 발행하는 구조로 전환 예정 (Week 3, MAVROS)
