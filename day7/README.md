# Day 7 | ROS2 Humble 설치 + 환경 검증

> 소요 시간: 약 2시간
> 목표: ROS2 Humble을 설치하고 talker/listener 예제로 통신을 확인한다.

---

## 0. 사전 준비 (5분)

로케일이 깨져 있으면 설치 중 인코딩 문제가 생길 수 있으니 먼저 확인합니다.

```bash
locale  # UTF-8인지 확인
```

UTF-8이 아니라면 아래로 설정합니다.

```bash
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

---

## 1. 저장소 등록 및 설치 (20~30분)

```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu \
$(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop -y
```

`ros-humble-desktop`은 용량이 커서(약 2~3GB) 다운로드에 시간이 걸립니다.

나중에 커스텀 패키지 빌드(Day 12~13)에 필요한 개발 도구도 미리 설치해둡니다.

```bash
sudo apt install ros-dev-tools -y
```

---

## 2. 환경 변수 설정 (`~/.bashrc`)

> ⚠️ 원본 자료의 `source /etc/profile.d/ros2.sh`는 존재하지 않는 경로입니다.
> ROS2는 `/opt/ros/humble/setup.bash`를 source 해야 합니다.

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

새 터미널을 열 때마다 자동으로 ROS2 환경이 로드됩니다.

---

## 3. 설치 검증: talker / listener

**터미널 1**

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

**터미널 2** (새 터미널 창)

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

### 정상 동작 예시

- talker 창:
  ```
  [INFO] [talker]: Publishing: 'Hello World: 1'
  [INFO] [talker]: Publishing: 'Hello World: 2'
  ```
- listener 창:
  ```
  [INFO] [listener]: I heard: [Hello World: 1]
  [INFO] [listener]: I heard: [Hello World: 2]
  ```

이 흐름은 Publisher(talker)와 Subscriber(listener)가 **DDS(Data Distribution Service)** 를 통해 통신하는 것으로, RabbitMQ의 producer → 큐 → consumer 구조와 동일한 패턴입니다.

---

## 4. 부가 검증 (선택, 있으면 좋음)

**터미널 3**

```bash
ros2 topic list
# /chatter 가 보여야 함

ros2 topic echo /chatter
# 메시지가 실시간으로 찍히는지 확인

ros2 topic info /chatter
# publisher 1개, subscriber 1개 확인
```

---

## 5. 체크포인트

- [ ] talker/listener 통신 확인
- [ ] `~/.bashrc`에 source 추가 완료

### 최종 확인 명령어

```bash
# 1. bashrc에 추가됐는지 확인
grep "ros/humble" ~/.bashrc

# 2. 새 터미널에서 source 없이 바로 되는지 확인
ros2 --version
```

두 명령이 정상 출력되면 Day 7 완료입니다.

---

## 자주 발생하는 문제 (Troubleshooting)

| 증상 | 원인 / 해결 |
|---|---|
| `ros2: command not found` | `.bashrc`에 source 줄 누락 또는 새 터미널 미실행 → `source ~/.bashrc` 재실행 |
| talker는 도는데 listener가 응답 없음 | WSL2 환경에서 네트워크 이슈일 수 있음 → `echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc` 후 재시도 |
| GUI 관련 패키지 필요 여부 | rviz2 등은 Day 7에서 불필요, Gazebo 사용하는 Week 3부터 WSLg 설정 필요 |
