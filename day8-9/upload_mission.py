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
