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
master.recv_match(
    type='GPS_RAW_INT',
    condition='GPS_RAW_INT.fix_type >= 3',
    blocking=True,
    timeout=30
)
print("  -> GPS 준비 완료")


# -----------------------------
# 2. 미션 아이템 정의
# -----------------------------
HOME_LAT = -35.363262
HOME_LON = 149.165237

waypoints = [
    (HOME_LAT + 0.0005, HOME_LON,           10),  # WP1
    (HOME_LAT + 0.0005, HOME_LON + 0.0005,  15),  # WP2
    (HOME_LAT,          HOME_LON + 0.0005,  10),  # WP3
]


def make_mission_item_int(seq, lat, lon, alt):
    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    return master.mav.mission_item_int_encode(
        master.target_system, master.target_component,
        seq, frame, command,
        0, 1,
        0, 2, 0, 0,
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    )


def make_mission_item_float(seq, lat, lon, alt):
    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    return master.mav.mission_item_encode(
        master.target_system, master.target_component,
        seq, frame, command,
        0, 1,
        0, 2, 0, 0,
        lat,
        lon,
        alt,
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    )


REQUEST_TYPES = ['MISSION_REQUEST_INT', 'MISSION_REQUEST']
ALL_TYPES = REQUEST_TYPES + ['MISSION_ACK']


def get_next_action(timeout=5):
    """
    미션 업로드 중 다음에 처리할 메시지 하나를 결정한다.

    규칙:
    1) 버퍼에 MISSION_ACK가 하나라도 있으면 무조건 그것을 우선 반환한다.
       ACK는 "이 거래는 끝났다"는 신호이므로, 그 이후에 뒤섞여 도착한
       낡은 재요청(MISSION_REQUEST)으로 절대 덮어써서는 안 된다.
       (이 부분이 이전 시도들에서 반복적으로 실패한 근본 원인이었다)
    2) ACK가 없다면, 같은 배치 안에 쌓인 요청들 중 seq가 가장 높은 것,
       즉 가장 앞서 나간 요청만 채택하고 낡은 저 seq 중복 요청은 버린다.
    """
    first = master.recv_match(type=ALL_TYPES, blocking=True, timeout=timeout)
    if first is None:
        return None

    batch = [first]
    while True:
        more = master.recv_match(type=ALL_TYPES, blocking=False)
        if more is None:
            break
        batch.append(more)

    acks = [m for m in batch if m.get_type() == 'MISSION_ACK']
    if acks:
        if len(batch) > 1:
            print(f"    [정보] 배치 내 MISSION_ACK 발견 → 세션 종료 신호 우선 처리 "
                  f"(폐기된 나머지: {[ (m.get_type(), getattr(m,'seq','-')) for m in batch if m is not acks[-1] ]})")
        return acks[-1]

    requests = sorted(batch, key=lambda m: m.seq)
    chosen = requests[-1]
    if len(batch) > 1:
        print(f"    [정보] 낡은 중복 요청 {len(batch)-1}개 폐기, "
              f"최신 {chosen.get_type()}(seq={chosen.seq}) 사용")
    return chosen


# -----------------------------
# 3. 미션 업로드
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

    while True:
        msg = get_next_action(timeout=5)
        if msg is None:
            print("타임아웃: 응답을 받지 못했습니다.")
            return False

        mtype = msg.get_type()
        print(f" [DEBUG] from system={msg.get_srcSystem()}, "
              f"component={msg.get_srcComponent()}, type={mtype}")

        if mtype in REQUEST_TYPES:
            seq = msg.seq
            if seq >= total:
                continue

            lat, lon, alt = waypoints[seq]
            print(f"  -> 요청 받음: seq={seq} ({mtype}), 전송: ({lat}, {lon}, {alt}m)")

            if mtype == 'MISSION_REQUEST_INT':
                item = make_mission_item_int(seq, lat, lon, alt)
            else:
                item = make_mission_item_float(seq, lat, lon, alt)

            master.mav.send(item)

        elif mtype == 'MISSION_ACK':
            if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print("[3] MISSION_ACK: 업로드 성공")
                return True
            else:
                print(f"[3] MISSION_ACK: 실패 (에러 코드: {msg.type})")
                return False


# -----------------------------
# 4. ARM + 이륙
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

    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
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
