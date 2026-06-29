#!/usr/bin/env python3
"""
실습 2: COMMAND_LONG(MAV_CMD_NAV_TAKEOFF) 전송 및 ACK 수신
목표: GCS → 드론 명령 전송 흐름 + 핸드셰이크 이해
"""
from pymavlink import mavutil

# ── 연결 ──────────────────────────────────────────────
conn = mavutil.mavlink_connection(
    'udpout:127.0.0.1:14550',
    source_system=255,
    source_component=190
)

print("GCS → 드론: HEARTBEAT 전송 중...")
conn.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_GCS,
    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
    0, 0,
    mavutil.mavlink.MAV_STATE_ACTIVE
)

msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
if msg is None:
    print("오류: fake_drone.py가 실행 중인지 확인하세요.")
    exit(1)

conn.target_system = msg.get_srcSystem()
conn.target_component = msg.get_srcComponent()
print(f"연결 성공! (sys={conn.target_system})\n")

# ── COMMAND_LONG 전송 ──────────────────────────────────
TARGET_ALTITUDE = 10.0  # 목표 고도 (m)

print("=" * 50)
print(f"TAKEOFF 명령 전송 (목표고도: {TARGET_ALTITUDE}m)")
print("=" * 50)

print("\n[송신] COMMAND_LONG")
print(f"  target_system    : {conn.target_system}")
print(f"  target_component : {conn.target_component}")
print(f"  command          : MAV_CMD_NAV_TAKEOFF (22)")
print(f"  param7 (고도)    : {TARGET_ALTITUDE}m")

conn.mav.command_long_send(
    conn.target_system,                        # target_system
    conn.target_component,                     # target_component
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,       # command = 22
    0,                                         # confirmation
    0,                                         # param1: pitch (미사용)
    0,                                         # param2: 미사용
    0,                                         # param3: 미사용
    float('nan'),                              # param4: yaw (nan = 현재방향 유지)
    0,                                         # param5: latitude (0 = 현재위치)
    0,                                         # param6: longitude (0 = 현재위치)
    TARGET_ALTITUDE                            # param7: 목표고도 (m)
)

# ── COMMAND_ACK 수신 ───────────────────────────────────
print("\nACK 대기 중...")
ack = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)

if ack is None:
    print("오류: ACK 타임아웃")
    exit(1)

result_map = {
    0: 'ACCEPTED (수락)',
    1: 'TEMPORARILY_REJECTED (일시 거부)',
    2: 'DENIED (거부)',
    3: 'UNSUPPORTED (미지원)',
    4: 'FAILED (실패)',
    5: 'IN_PROGRESS (처리 중)',
}

print("\n[수신] COMMAND_ACK")
print(f"  command : {ack.command} (22 = MAV_CMD_NAV_TAKEOFF)")
print(f"  result  : {ack.result} → {result_map.get(ack.result, '?')}")

if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
    print("\n이륙 명령 수락됨! 드론이 상승 중입니다.")
    print("(fake_drone.py 터미널에서 고도 변화 확인 가능)")

print("\n실습 2 완료!")
