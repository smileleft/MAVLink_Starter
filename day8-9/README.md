# Day 8-9 | MAVLink 미션(Mission) 프로토콜 이해

> 학습 목표: Fleet/Mission Planning의 기반 프로토콜 파악
> 소요 시간: 일 2시간 (총 2일)

---

## 0. 사전 조건

Day 5-6에서 구성한 ArduPilot SITL 환경이 필요합니다.

```bash
# ArduPilot 클론 경로에서 SITL 실행 확인
cd ~/ardupilot
sim_vehicle.py -v ArduCopter --console --map
```

`pymavlink`가 설치되어 있어야 합니다.

```bash
pip install pymavlink
```

---

## 1일차 (Day 8): 프로토콜 이론 학습

### 1-1. 읽을 문서

| 문서 | 링크 |
|---|---|
| Mission Protocol | https://mavlink.io/en/services/mission.html |
| File Transfer Protocol (FTP) | https://mavlink.io/en/services/ftp.html |

### 1-2. 핵심 메시지 정리

| 메시지 | 방향 | 역할 |
|---|---|---|
| `MISSION_COUNT` | GCS → Vehicle | "웨이포인트가 N개 있다"고 개수 통보 |
| `MISSION_REQUEST_INT` | Vehicle → GCS | "n번째 아이템을 보내달라"고 요청 |
| `MISSION_ITEM_INT` | GCS → Vehicle | 실제 웨이포인트 좌표/명령 데이터 |
| `MISSION_ACK` | 양방향 | 전송 완료/에러 확인 응답 |
| `MISSION_CURRENT` | Vehicle → GCS | 현재 비행 중인 웨이포인트 인덱스 |
| `MISSION_ITEM_REACHED` | Vehicle → GCS | 특정 웨이포인트 도달 알림 |

### 1-3. 미션 업로드 핸드셰이크 시퀀스

```
GCS                                Vehicle (드론)
 |                                     |
 |----- MISSION_COUNT (count=3) ----->|   "웨이포인트 3개 보낼 예정"
 |                                     |
 |<---- MISSION_REQUEST_INT (seq=0) --|   "0번 줘"
 |----- MISSION_ITEM_INT (seq=0) ----->|
 |                                     |
 |<---- MISSION_REQUEST_INT (seq=1) --|   "1번 줘"
 |----- MISSION_ITEM_INT (seq=1) ----->|
 |                                     |
 |<---- MISSION_REQUEST_INT (seq=2) --|   "2번 줘"
 |----- MISSION_ITEM_INT (seq=2) ----->|
 |                                     |
 |<---- MISSION_ACK (accepted) -------|   "전부 잘 받았다"
```

핵심 포인트:
- **Pull 방식**: GCS가 일방적으로 밀어넣는 게 아니라, Vehicle이 하나씩 요청(`MISSION_REQUEST_INT`)하면 GCS가 응답하는 구조입니다.
- 순서가 어긋나거나 타임아웃되면 `MISSION_ACK`에 에러 코드가 담겨 재전송이 필요합니다.
- `seq=0`은 보통 홈 포지션(현재 위치)이고, 실제 웨이포인트는 `seq=1`부터 시작하는 경우가 많습니다(펌웨어에 따라 다름).

### 1-4. 웨이포인트 좌표계 (WGS84)

- MAVLink의 위경도는 **WGS84** 좌표계를 사용합니다 (GPS가 쓰는 표준 좌표계).
- `MISSION_ITEM_INT`에서 위경도는 정수(`int32`)로 전송되며 실제 값에 **1e7을 곱한 값**입니다.
  - 예: 위도 `37.5665`도 → 전송 값 `375665000`
- 고도(`z`)는 `frame` 필드에 따라 의미가 달라집니다.
  - `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT` : 이륙 지점 기준 상대 고도 (실습에서 주로 사용)
  - `MAV_FRAME_GLOBAL_INT` : 해수면 기준 절대 고도(AMSL)

### 체크포인트 (Day 8)

- [ ] 미션 업로드 핸드셰이크 시퀀스를 그림 없이 말로 설명할 수 있다
- [ ] 위경도가 정수로 전송되는 이유와 변환 배율(1e7)을 안다
- [ ] `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`와 `MAV_FRAME_GLOBAL_INT`의 차이를 안다

---

## 2일차 (Day 9): pymavlink 실습 — 웨이포인트 업로드 + AUTO 비행

### 2-1. SITL 실행

```bash
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
```

`--map` 옵션을 켜두면 지도에서 실제 비행 경로를 시각적으로 확인할 수 있습니다.
SITL 기본 접속 주소는 `udp:127.0.0.1:14550` 입니다 (MAVProxy가 자동으로 열어줌).

### 2-2. 실습 스크립트 작성

`upload_mission.py` 파일을 작성합니다.

```python
"""
Day 8-9 실습: pymavlink로 3개 웨이포인트 미션을 업로드하고
AUTO 모드로 자동비행을 실행하는 스크립트

전제:
- ArduPilot SITL이 udp:127.0.0.1:14550 으로 떠 있어야 함
"""

import time
from pymavlink import mavutil

# -----------------------------
# 1. 연결
# -----------------------------
print("[1] SITL 연결 시도...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print(f"연결 완료 (system={master.target_system}, component={master.target_component})")


# -----------------------------
# 2. 미션 아이템 정의 (홈 기준 상대 좌표계 사용)
# -----------------------------
# 실습용 홈 위치 근처 임의 좌표 (SITL 기본 스폰 위치: CMAC 활주로 근방)
# 실제 값은 SITL 콘솔에 뜨는 홈 좌표를 참고해 조정 가능
HOME_LAT = -35.363262
HOME_LON = 149.165237

waypoints = [
    # (lat, lon, alt)
    (HOME_LAT + 0.0005, HOME_LON,           10),  # WP1
    (HOME_LAT + 0.0005, HOME_LON + 0.0005,  15),  # WP2
    (HOME_LAT,          HOME_LON + 0.0005,  10),  # WP3
]


def make_mission_item(seq, lat, lon, alt):
    """MISSION_ITEM_INT 메시지 생성 헬퍼"""
    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT

    return master.mav.mission_item_int_encode(
        master.target_system,
        master.target_component,
        seq,
        frame,
        command,
        0,          # current (0: 일반 웨이포인트)
        1,          # autocontinue
        0, 0, 0, 0,             # param1~4 (accept radius 등, 기본값 0)
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    )


# -----------------------------
# 3. 미션 업로드 (MISSION_COUNT → 요청에 응답)
# -----------------------------
def upload_mission():
    total = len(waypoints)
    print(f"[2] MISSION_COUNT 전송 (총 {total}개)...")

    master.mav.mission_count_send(
        master.target_system,
        master.target_component,
        total,
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    )

    uploaded = 0
    while uploaded < total:
        msg = master.recv_match(
            type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'],
            blocking=True,
            timeout=5
        )
        if msg is None:
            print("타임아웃: MISSION_REQUEST를 받지 못했습니다.")
            break

        seq = msg.seq
        lat, lon, alt = waypoints[seq]
        print(f"  -> 요청 받음: seq={seq}, 전송: ({lat}, {lon}, {alt}m)")

        item = make_mission_item(seq, lat, lon, alt)
        master.mav.send(item)
        uploaded += 1

    # 최종 ACK 확인
    ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print("[3] MISSION_ACK: 업로드 성공")
        return True
    else:
        print(f"[3] MISSION_ACK: 실패 또는 응답 없음 ({ack})")
        return False


# -----------------------------
# 4. ARM + 이륙 (AUTO 모드 진입 전 준비)
# -----------------------------
def arm_and_takeoff(target_altitude=10):
    print("[4] GUIDED 모드로 전환 후 ARM...")
    master.set_mode('GUIDED')

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("  -> ARM 완료")

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, target_altitude
    )
    print(f"  -> TAKEOFF 명령 전송 (목표 고도 {target_altitude}m)")

    # 목표 고도 근접까지 대기
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        alt = msg.relative_alt / 1000.0  # mm -> m
        print(f"     현재 고도: {alt:.1f} m")
        if alt >= target_altitude * 0.95:
            print("  -> 목표 고도 도달")
            break


# -----------------------------
# 5. AUTO 모드 전환 (미션 자동 실행)
# -----------------------------
def start_auto_mission():
    print("[5] AUTO 모드 전환...")
    master.set_mode('AUTO')
    print("  -> AUTO 모드 진입, 미션 자동 비행 시작")


# -----------------------------
# 6. 미션 진행 상황 모니터링
# -----------------------------
def monitor_mission(total_waypoints):
    print("[6] 미션 진행 상황 모니터링 (Ctrl+C로 중단)")
    reached = set()
    try:
        while len(reached) < total_waypoints:
            msg = master.recv_match(
                type=['MISSION_CURRENT', 'MISSION_ITEM_REACHED'],
                blocking=True,
                timeout=10
            )
            if msg is None:
                continue

            if msg.get_type() == 'MISSION_ITEM_REACHED':
                print(f"  -> 웨이포인트 도달: seq={msg.seq}")
                reached.add(msg.seq)
            elif msg.get_type() == 'MISSION_CURRENT':
                print(f"  -> 현재 진행 중인 웨이포인트: seq={msg.seq}")
    except KeyboardInterrupt:
        print("모니터링 중단됨 (사용자 인터럽트)")


# -----------------------------
# 실행 순서
# -----------------------------
if __name__ == '__main__':
    if upload_mission():
        arm_and_takeoff(target_altitude=10)
        time.sleep(2)
        start_auto_mission()
        monitor_mission(total_waypoints=len(waypoints))
    else:
        print("미션 업로드 실패로 비행을 진행하지 않습니다.")
```

### 2-3. 실행

```bash
python3 upload_mission.py
```

### 2-4. 기대 동작

1. SITL과 연결되고 하트비트를 확인
2. `MISSION_COUNT` → `MISSION_REQUEST_INT` → `MISSION_ITEM_INT` 핸드셰이크가 콘솔에 순서대로 출력됨
3. `MISSION_ACK` 수신 후 업로드 성공 메시지 출력
4. GUIDED 모드로 ARM 후 10m 고도까지 이륙
5. AUTO 모드로 전환되며 SITL `--map` 창에서 드론이 3개 웨이포인트를 순서대로 비행하는 것을 확인
6. 콘솔에 `MISSION_ITEM_REACHED` 로그가 웨이포인트 도달마다 출력됨

### 2-5. 미션 업로드 검증만 별도로 하고 싶다면

QGroundControl 등 다른 GCS 없이도, MAVProxy 콘솔에서 직접 확인 가능합니다.

```bash
# SITL 콘솔(MAVProxy)에서
wp list
```

업로드한 3개의 웨이포인트가 목록에 뜨는지 확인합니다.

---

## 체크포인트 (Day 9)

- [ ] 미션 업로드 핸드셰이크 시퀀스를 코드 실행 로그로 직접 확인했다
- [ ] 웨이포인트 좌표계(WGS84) 변환(위경도 × 1e7)을 코드에서 직접 다뤘다
- [ ] AUTO 모드 진입 후 드론이 3개 웨이포인트를 순서대로 통과하는 것을 지도에서 확인했다
- [ ] `MISSION_ITEM_REACHED` 메시지로 웨이포인트 도달을 감지할 수 있다

---

## 자주 발생하는 문제 (Troubleshooting)

| 증상 | 원인 / 해결 |
|---|---|
| `wait_heartbeat()`에서 멈춤 | SITL이 아직 부팅 중이거나 포트 불일치. SITL 콘솔에 `APM: EKF3 IMU0 origin set` 등의 로그가 뜬 후 실행 |
| `MISSION_REQUEST`만 오고 `MISSION_REQUEST_INT`는 안 옴 | 펌웨어/버전에 따라 구버전 프로토콜(`MISSION_REQUEST`)을 쓰는 경우가 있음 → 코드에서 둘 다 받도록 이미 처리되어 있음 |
| ARM이 거부됨 (`PreArm: ...`) | SITL 콘솔 에러 확인. GPS lock 대기 중이거나 배터리/센서 체크 실패 → 몇 초 더 대기 후 재시도 |
| AUTO 모드 진입 후 바로 비행 안 함 | 미션 업로드 후 시작 웨이포인트 인덱스가 지정 안 된 경우 → `wp set 1` (MAVProxy 콘솔)로 시작 지점 지정 |
| 웨이포인트가 이상한 위치에 찍힘 | HOME 좌표를 실제 SITL 스폰 위치와 다르게 하드코딩한 경우 → SITL 콘솔 시작 로그의 홈 좌표로 `HOME_LAT`, `HOME_LON` 교체 |
