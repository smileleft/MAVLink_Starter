# Day 3-4 Example

## exercise file

```bash
cd ~/mavlink_study

# make 4 files below
~/mavlink_study/
├── venv/
├── fake_drone.py      ← fake drone (always need to execute first)
├── ex1_heartbeat.py   ← example 1
├── ex2_takeoff.py     ← example 2
└── ex3_position.py    ← example 3
```

## how to execute

```bash
source {your venv directory}/bin/activate

# execute fake drone
python3 fake drone.py

# 예상 출력
[드론] UDP 14550 포트 바인딩 중...
[드론] GCS 접속 대기 중... (ex1/ex2/ex3 중 하나를 실행하세요)

# ex1 (다른 터미널에서 실행)
source {your venv directory}/bin/activate
python3 ex1_heartbeat.py

# 예상 출력
# 예상 출력
GCS → 드론: HEARTBEAT 전송 중...
드론 → GCS: HEARTBEAT 수신 대기 중...
연결 성공! (드론 sys=1, comp=1)

==================================================
HEARTBEAT 수신 시작 (5회)
==================================================

[1번째 HEARTBEAT]
  ┌ sys_id      : 1
  ├ comp_id     : 1
  ├ type        : 2 (QUADROTOR)
  ├ autopilot   : 3 (ARDUPILOTMEGA)
  ├ base_mode   : 217 (비트 플래그)
  └ sys_status  : 4 (ACTIVE)

# ex2 (다른 터미널에서 실행)
source {your venv directory}/bin/activate
python3 ex2_takeoff.py

# 예상 출력 (터미널 2)
연결 성공! (sys=1)

[송신] COMMAND_LONG
  target_system    : 1
  target_component : 1
  command          : MAV_CMD_NAV_TAKEOFF (22)
  param7 (고도)    : 10.0m

[수신] COMMAND_ACK
  command : 22 (22 = MAV_CMD_NAV_TAKEOFF)
  result  : 0 → ACCEPTED (수락)

이륙 명령 수락됨!

# 동시에 터미널 1(fake_drone)에 출력됨:
[드론] ▼ COMMAND_LONG 수신
       cmd    : 22
[드론] → 이륙 시작! 목표고도 10.0m
[드론] 현재 고도: 0.5m
[드론] 현재 고도: 1.0m

# ex3 - 드론이 상승 중인 상태에서 실행하면 고도 변화를 실시간으로 확인할 수 있슴
python3 ex3_position.py

# 예상 출력
연결 성공! (sys=1)

==================================================
위치 데이터 수신 (5회)
MAVLink는 위경도를 정수(× 1e7)로 전송
==================================================

[1번째 위치]
  ┌ lat (raw → 변환)  : 374314940 → 37.4314940°
  ├ lon (raw → 변환)  : 1270016540 → 127.0016540°
  ├ alt (raw → 변환)  : 3000mm → 3.00m (해발)
  ├ relative_alt      : 3000mm → 3.00m (지면기준)
  └ 속도 vx,vy,vz     : 0.0, 0.0, 0.0 m/s

[2번째 위치]
  ├ alt (raw → 변환)  : 3500mm → 3.50m (해발)
...


```
