#!/usr/bin/env python3
"""
실습 3: GLOBAL_POSITION_INT에서 위경도/고도 추출
목표: MAVLink 정수 인코딩(×1e7) 이해 + 실시간 텔레메트리 수신
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

# ── GLOBAL_POSITION_INT 수신 ───────────────────────────
print("=" * 50)
print("위치 데이터 수신 (5회)")
print("MAVLink는 위경도를 정수(× 1e7)로 전송")
print("=" * 50)

for i in range(1, 6):
    msg = conn.recv_match(
        type='GLOBAL_POSITION_INT',
        blocking=True,
        timeout=3
    )
    if msg is None:
        print(f"[{i}번] 타임아웃")
        continue

    # ── 단위 변환 ──────────────────────────────────────
    # MAVLink raw 값 (정수)
    lat_raw = msg.lat          # 예: 374314940
    lon_raw = msg.lon          # 예: 1270016540
    alt_raw = msg.alt          # 예: 5000 (mm)
    rel_alt_raw = msg.relative_alt  # mm

    # 실제 값으로 변환
    lat = lat_raw / 1e7        # 37.4314940°
    lon = lon_raw / 1e7        # 127.0016540°
    alt_m = alt_raw / 1000.0   # mm → m
    rel_alt_m = rel_alt_raw / 1000.0

    vx = msg.vx / 100.0        # cm/s → m/s
    vy = msg.vy / 100.0
    vz = msg.vz / 100.0

    print(f"\n[{i}번째 위치]")
    print(f"  ┌ lat (raw → 변환)  : {lat_raw} → {lat:.7f}°")
    print(f"  ├ lon (raw → 변환)  : {lon_raw} → {lon:.7f}°")
    print(f"  ├ alt (raw → 변환)  : {alt_raw}mm → {alt_m:.2f}m (해발)")
    print(f"  ├ relative_alt      : {rel_alt_raw}mm → {rel_alt_m:.2f}m (지면기준)")
    print(f"  └ 속도 vx,vy,vz     : {vx:.1f}, {vy:.1f}, {vz:.1f} m/s")

print("\n실습 3 완료!")
