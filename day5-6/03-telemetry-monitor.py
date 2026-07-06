# 03_telemetry_monitor.py
from pymavlink import mavutil

master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
master.wait_heartbeat()

while True:
    msg = master.recv_match(blocking=True)
    if msg is None:
        continue

    if msg.get_type() == 'GLOBAL_POSITION_INT':
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.relative_alt / 1000.0
        print(f"위치: lat={lat:.6f}, lon={lon:.6f}, 상대고도={alt:.1f}m")

    elif msg.get_type() == 'SYS_STATUS':
        battery = msg.battery_remaining
        print(f"배터리 잔량: {battery}%")

    elif msg.get_type() == 'HEARTBEAT':
        mode = mavutil.mode_string_v10(msg)
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"모드: {mode}, ARMED: {armed}")
