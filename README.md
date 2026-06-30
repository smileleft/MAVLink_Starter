# MAVLink_Starter
repository for MAVLink starter

## Install Env for MAVLink exercise

```bash
# 패키지 업데이트
sudo apt update && sudo apt install -y python3-pip python3-venv

# 작업 디렉토리 생성
mkdir ~/mavlink_study && cd ~/mavlink_study

# 가상환경 생성 및 활성화
python3 -m venv {your venv name}
source venv/bin/activate

# pymavlink 설치
pip install pymavlink

# 설치 확인
python3 -c "from pymavlink import mavutil; print('pymavlink 설치 성공')"
```
