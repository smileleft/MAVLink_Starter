"""
Day 8-9 실습: pymavlink로 3개 웨이포인트 미션을 업로드하고
AUTO 모드로 자동비행을 실행하는 스크립트

전제:
- ArduPilot SITL이 udp:127.0.0.1:14550 으로 떠 있어야 함

핵심 포인트:
- ArduPilot은 업로드된 미션의 seq=0을 "홈 포지션"으로 취급한다.
  따라서 seq=0에는 반드시 현재 위치를 기반으로 한 홈 아이템을 넣고,
  실제 웨이포인트는 seq=1부터 시작해야 한다.
"""

import time
from pymavlink import mavutil

# -----------------------------
# 1. 연결
# -----------------------------
print("[1] SITL 연결 시도...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()

master.target_component = 1
print(f"연결 완료 (system={master.target_system}, component={master.target_component})")

print("[1-1] 기존 버퍼 메시지 비우는 중...")
flushed = 0
while True:
    msg = master.recv_match(blocking=False)
    if msg is None:
        break
    flushed += 1
print(f"  -> {flushed}개 stale 메시지 제거 완료")

print("[1-2] GPS 준비 대기 중...")
gps_msg = master.recv_match(
    type='GPS_RAW_INT',
    condition='GPS_RAW_INT.fix_type >= 3',
    blocking=True,
    timeout=30
)

if gps_msg is None:
    raise RuntimeError("GPS 3D Fix를 얻지 못했습니다. SITL 상태를 확인하세요.")

print("  -> GPS 준비 완료")


# -----------------------------
# 1-3. 현재 위치(=홈 포지션) 조회
# -----------------------------
print("[1-3] 현재 위치(홈 포지션) 조회 중...")
pos_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=10)
if pos_msg is None:
    raise RuntimeError("GLOBAL_POSITION_INT를 받지 못했습니다. SITL 상태를 확인하세요.")

HOME_LAT = pos_msg.lat / 1e7
HOME_LON = pos_msg.lon / 1e7
HOME_ALT_AMSL = pos_msg.alt / 1000.0  # mm -> m, 절대고도(AMSL)

print(
    f"  -> 홈 포지션: lat={HOME_LAT}, "
    f"lon={HOME_LON}, alt(AMSL)={HOME_ALT_AMSL}m"
)


# -----------------------------
# 2. 미션 아이템 정의
# -----------------------------
# seq=0 : 홈 포지션 (ArduPilot 관례상 반드시 필요, 절대고도 프레임 사용)
# seq=1~3 : 실제 웨이포인트 (상대고도 프레임 사용)
nav_waypoints = [
    (HOME_LAT + 0.0005, HOME_LON,            10),  # WP1 -> seq=1
    (HOME_LAT + 0.0005, HOME_LON + 0.0005,   15),  # WP2 -> seq=2
    (HOME_LAT,          HOME_LON + 0.0005,    10),  # WP3 -> seq=3
]

# 전체 미션 아이템: (lat, lon, alt, frame, command, is_home)
mission_items = [
    (
        HOME_LAT,
        HOME_LON,
        HOME_ALT_AMSL,
        mavutil.mavlink.MAV_FRAME_GLOBAL,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        True
    ),  # seq=0: 홈
]

for lat, lon, alt in nav_waypoints:
    mission_items.append((
        lat,
        lon,
        alt,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        False
    ))

TOTAL_ITEMS = len(mission_items)  # 4 (홈 1 + 웨이포인트 3)


def make_mission_item_int(seq, lat, lon, alt, frame, command):
    return master.mav.mission_item_int_encode(
        master.target_system,
        master.target_component,
        seq,
        frame,
        command,
        0,  # current
        1,  # autocontinue
        0,  # param1: hold time
        2,  # param2: acceptance radius
        0,  # param3: pass radius
        0,  # param4: yaw
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    )


REQUEST_TYPES = ['MISSION_REQUEST_INT', 'MISSION_REQUEST']
ALL_TYPES = REQUEST_TYPES + ['MISSION_ACK']


def get_next_action(timeout=5):
    """
    Mission Protocol 메시지를 수신 순서대로 하나씩 처리한다.
    """
    return master.recv_match(
        type=ALL_TYPES,
        blocking=True,
        timeout=timeout
    )


# -----------------------------
# 3. 미션 업로드
# -----------------------------
def upload_mission():
    print(f"[2] MISSION_COUNT 전송 (총 {TOTAL_ITEMS}개, 홈 포함)...")

    master.mav.mission_count_send(
        master.target_system,
        master.target_component,
        TOTAL_ITEMS,
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    )

    while True:
        msg = get_next_action(timeout=5)

        if msg is None:
            print("타임아웃: 응답을 받지 못했습니다.")
            return False

        mtype = msg.get_type()

        if mtype in REQUEST_TYPES:
            seq = msg.seq

            if seq >= TOTAL_ITEMS:
                print(f"잘못된 seq 요청: {seq}")
                continue

            lat, lon, alt, frame, command, is_home = mission_items[seq]

            label = "홈" if is_home else "웨이포인트"

            print(
                f"  -> 요청 받음: seq={seq} "
                f"({label}, {mtype}), "
                f"전송: ({lat:.6f}, {lon:.6f}, {alt}m)"
            )

            # MISSION_REQUEST와 MISSION_REQUEST_INT 모두
            # MISSION_ITEM_INT로 응답
            item = make_mission_item_int(
                seq,
                lat,
                lon,
                alt,
                frame,
                command
            )

            master.mav.send(item)

        elif mtype == 'MISSION_ACK':
            result = msg.type

            try:
                result_name = (
                    mavutil.mavlink.enums['MAV_MISSION_RESULT'][result].name
                )
            except (KeyError, AttributeError):
                result_name = 'UNKNOWN'

            if result == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print("[3] MISSION_ACK: 업로드 성공")
                return True

            print(
                f"[3] MISSION_ACK: 실패 "
                f"(에러 코드: {result}, {result_name})"
            )

            return False


# -----------------------------
# 4. ARM + 이륙
# -----------------------------
def arm_and_takeoff(target_altitude=10):
    print("[4] GUIDED 모드로 전환 후 ARM...")
    master.set_mode('GUIDED')

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )

    master.motors_armed_wait()
    print("  -> ARM 완료")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,
        target_altitude
    )

    print(f"  -> TAKEOFF 명령 전송 (목표 고도 {target_altitude}m)")

    while True:
        msg = master.recv_match(
            type='GLOBAL_POSITION_INT',
            blocking=True
        )

        alt = msg.relative_alt / 1000.0

        print(f"     현재 고도: {alt:.1f} m")

        if alt >= target_altitude * 0.95:
            print("  -> 목표 고도 도달")
            break


# -----------------------------
# 5. AUTO 모드 전환
# -----------------------------
def start_auto_mission():
    print("[5] AUTO 모드 전환...")

    master.set_mode('AUTO')

    print("  -> AUTO 모드 진입, 미션 자동 비행 시작")


# -----------------------------
# 6. 미션 진행 상황 모니터링
# -----------------------------
def monitor_mission(total_nav_waypoints):
    # 홈(seq=0)을 제외한 실제 웨이포인트(seq=1~3) 도달만 카운트
    print("[6] 미션 진행 상황 모니터링 (Ctrl+C로 중단)")

    reached = set()

    try:
        while len(reached) < total_nav_waypoints:
            msg = master.recv_match(
                type=['MISSION_CURRENT', 'MISSION_ITEM_REACHED'],
                blocking=True,
                timeout=10
            )

            if msg is None:
                continue

            if msg.get_type() == 'MISSION_ITEM_REACHED':
                print(f"  -> 웨이포인트 도달: seq={msg.seq}")

                if msg.seq >= 1:  # 홈(0)은 제외
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

        monitor_mission(
            total_nav_waypoints=len(nav_waypoints)
        )

    else:
        print("미션 업로드 실패로 비행을 진행하지 않습니다.")
