#!/usr/bin/env python3
"""
실습 1: HEARTBEAT 메시지 수신 후 파싱 출력
목표: MAVLink 패킷 구조(sys_id, comp_id, 페이로드)를 직접 확인
"""
from pymavlink import mavutil

# ── 연결 ──────────────────────────────────────────────
conn = mavutil.mavlink_connection(
    'udpout:127.0.0.1:14550',
    source_system=255,    # GCS sys_id
    source_component=190  # Mission Planner comp_id
)

# 드론이 우리 주소를 알 수 있도록 먼저 HEARTBEAT 전송
print("GCS → 드론: HEARTBEAT 전송 중...")
conn.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_GCS,
    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
    0, 0,
    mavutil.mavlink.MAV_STATE_ACTIVE
)

# 드론의 HEARTBEAT 대기
print("드론 → GCS: HEARTBEAT 수신 대기 중...")
msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
if msg is None:
    print("오류: fake_drone.py가 실행 중인지 확인하세요.")
    exit(1)

conn.target_system = msg.get_srcSystem()
conn.target_component = msg.get_srcComponent()
print(f"연결 성공! (드론 sys={conn.target_system}, comp={conn.target_component})\n")

# ── HEARTBEAT 5개 파싱 ─────────────────────────────────
type_map = {
    0: 'GENERIC', 1: 'FIXED_WING', 2: 'QUADROTOR',
    6: 'GCS', 10: 'GROUND_ROVER'
}
autopilot_map = {
    0: 'GENERIC', 3: 'ARDUPILOTMEGA', 8: 'INVALID'
}
status_map = {
    0: 'UNINIT', 1: 'BOOT', 2: 'CALIBRATING',
    3: 'STANDBY', 4: 'ACTIVE', 5: 'CRITICAL'
}

print("=" * 50)
print("HEARTBEAT 수신 시작 (5회)")
print("=" * 50)

for i in range(1, 6):
    msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
    if msg is None:
        print(f"[{i}번] 타임아웃")
        continue

    print(f"\n[{i}번째 HEARTBEAT]")
    print(f"  ┌ sys_id      : {msg.get_srcSystem()}")
    print(f"  ├ comp_id     : {msg.get_srcComponent()}")
    print(f"  ├ type        : {msg.type} "
          f"({type_map.get(msg.type, '?')})")
    print(f"  ├ autopilot   : {msg.autopilot} "
          f"({autopilot_map.get(msg.autopilot, '?')})")
    print(f"  ├ base_mode   : {msg.base_mode} (비트 플래그)")
    print(f"  └ sys_status  : {msg.system_status} "
          f"({status_map.get(msg.system_status, '?')})")

print("\n실습 1 완료!")
