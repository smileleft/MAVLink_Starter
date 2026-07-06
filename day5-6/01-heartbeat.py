# 01_heartbeat.py
from pymavlink import mavutil

# SITL의 MAVProxy가 14550번으로 GCS용 포트를 열어둠
# 이미 그 포트를 MAVProxy/지도가 쓰고 있다면 14551 등 다른 출력 포트를 sim_vehicle.py에 --out udp:127.0.0.1:14551 로 추가해야 함
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

print("HEARTBEAT 대기 중...")
master.wait_heartbeat()
print(f"연결됨! system_id={master.target_system}, component_id={master.target_component}")
