# 02_arm_takeoff_rtl.py
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

# 1) GUIDED 모드로 전환 (ARM/TAKEOFF는 보통 GUIDED에서)
set_mode("GUIDED")
time.sleep(1)

# 2) ARM
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0   # param1=1: arm
)
wait_ack("ARM")

# 3) TAKEOFF (목표 고도 10m)
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0, 10   # param7=altitude
)
wait_ack("TAKEOFF")

# 4) 고도 모니터링 (목표 고도 근접까지 대기)
print("이륙 중... 고도 모니터링")
while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    alt = msg.relative_alt / 1000.0  # mm -> m
    print(f"현재 상대고도: {alt:.1f} m")
    if alt >= 9.0:
        print("목표 고도 도달")
        break

time.sleep(5)  # 잠시 공중에 머무름

# 5) RTL (Return to Launch)
set_mode("RTL")
print("RTL 모드 전환, 귀환 시작")
