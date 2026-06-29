#!/usr/bin/env python3
"""
가상 드론 시뮬레이터
- UDP 14550 포트에서 대기
- HEARTBEAT / GLOBAL_POSITION_INT 1초마다 전송
- COMMAND_LONG 수신 시 ACK 응답
"""
import time
import threading
from pymavlink import mavutil

def main():
    print("[드론] UDP 14550 포트 바인딩 중...")
    conn = mavutil.mavlink_connection(
        'udpin:0.0.0.0:14550',
        source_system=1,       # 드론 sys_id = 1
        source_component=1     # 비행 컨트롤러 comp_id = 1
    )

    print("[드론] GCS 접속 대기 중... (ex1/ex2/ex3 중 하나를 실행하세요)")

    # GCS로부터 첫 메시지 수신 → 응답 주소 학습
    conn.recv_match(blocking=True, timeout=60)
    print("[드론] GCS 감지! 텔레메트리 전송 시작")

    state = {
        'alt_mm': 0,        # 현재 고도 (mm 단위)
        'flying': False,    # 비행 여부
    }

    def telemetry_loop():
        """1초마다 HEARTBEAT + GLOBAL_POSITION_INT 전송"""
        lat_e7 = 374314940   # 37.4314940° (위도 × 1e7)
        lon_e7 = 1270016540  # 127.0016540° (경도 × 1e7)

        while True:
            time_boot_ms = int(time.time() * 1000) & 0xFFFFFFFF

            # HEARTBEAT 전송
            conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_QUADROTOR,
                mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                mavutil.mavlink.MAV_MODE_GUIDED_ARMED,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE
            )

            # GLOBAL_POSITION_INT 전송
            conn.mav.global_position_int_send(
                time_boot_ms,
                lat_e7,
                lon_e7,
                state['alt_mm'],   # 해발고도 (mm)
                state['alt_mm'],   # 상대고도 (mm)
                0, 0, 0,           # vx, vy, vz (cm/s)
                0                  # 방위각 (cdeg)
            )

            # 비행 중이면 고도 상승 시뮬레이션 (0.5m/s)
            if state['flying'] and state['alt_mm'] < 50_000:
                state['alt_mm'] += 500
                print(f"[드론] 현재 고도: {state['alt_mm'] / 1000:.1f}m")

            time.sleep(1)

    # 텔레메트리 전송 스레드 시작
    t = threading.Thread(target=telemetry_loop, daemon=True)
    t.start()

    # 명령 수신 루프
    print("[드론] 명령 대기 중...\n")
    while True:
        msg = conn.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue

        msg_type = msg.get_type()

        if msg_type == 'COMMAND_LONG':
            cmd = msg.command
            print(f"\n[드론] ▼ COMMAND_LONG 수신")
            print(f"       cmd    : {cmd}")
            print(f"       param7 : {msg.param7} (보통 목표고도)")

            # COMMAND_ACK 전송
            conn.mav.command_ack_send(
                cmd,
                mavutil.mavlink.MAV_RESULT_ACCEPTED
            )
            print(f"[드론] ▲ COMMAND_ACK 전송 (ACCEPTED)")

            if cmd == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                print(f"[드론] → 이륙 시작! 목표고도 {msg.param7}m")
                state['flying'] = True

            elif cmd == mavutil.mavlink.MAV_CMD_NAV_LAND:
                print(f"[드론] → 착지 시작!")
                state['flying'] = False
                state['alt_mm'] = 0

if __name__ == '__main__':
    main()
