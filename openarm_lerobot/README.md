# OpenArm LeRobot 데이터 수집

OpenArm 양팔 로봇의 텔레옵 데이터를 LeRobot 데이터셋 형식으로 수집.

---

## 초기 환경 설정 (새 PC)

### 0. 전제 조건

- **Ubuntu 24.04 Noble**
- **ROS2 Jazzy** 설치 완료

ROS2 Jazzy 설치:
```bash
# ROS2 Jazzy 공식 가이드 참고: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
sudo apt install ros-jazzy-desktop
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### 1. 시스템 패키지 설치 (apt)

```bash
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-moveit-servo \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-gripper-controllers \
  ros-jazzy-effort-controllers \
  ros-jazzy-position-controllers \
  ros-jazzy-hardware-interface \
  ros-jazzy-control-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-trajectory-msgs \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-rviz2 \
  ros-jazzy-rclcpp-components \
  can-utils
```

---

### 2. openarm CAN 라이브러리 설치

Damiao 모터 CAN 통신용 라이브러리 (openarm PPA):
```bash
sudo apt install -y software-properties-common
sudo add-apt-repository -y ppa:openarm/main
sudo apt update
sudo apt install -y libopenarm-can-dev openarm-can-utils
```

---

### 3. LeRobot 소스 설치

```bash
git clone https://github.com/xhaka3456/lerobot.git ~/lerobot
cd ~/lerobot
pip install -e ".[feetech,intelrealsense]"
```

---

### 4. openarm workspace 클론 및 빌드

```bash
git clone <openarm repo URL> ~/openarm
cd ~/openarm
```

`src/moveit_servo`는 기본적으로 소스 빌드됩니다. apt 버전을 사용하려면 빌드 전에 아래를 실행:
```bash
touch ~/openarm/src/moveit_servo/COLCON_IGNORE
```

빌드:
```bash
colcon build
echo "source ~/openarm/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### 5. openarm_lerobot 소스 설치

```bash
cd ~/openarm/src/openarm_lerobot
pip install -e .
```

설치 후 `openarm-collect`, `openarm-infer` 명령어 사용 가능.

---

### 6. HuggingFace 로그인 (데이터셋 업로드 시)

```bash
pip install huggingface_hub
huggingface-cli login
```

---

## 설정 파일 수정

`config/collect_data.yaml` 편집:

```yaml
dataset:
  repo_id: "your-hf-id/dataset-name"    # HuggingFace repo ID
  single_task: "작업 설명"
  root: "~/openarm/src/openarm_lerobot/datasets/your_dataset"
  fps: 30
  episode_time_s: 60          # 에피소드당 최대 녹화 시간 (초)
  num_episodes: 50            # 총 에피소드 수
  video: true
  push_to_hub: false          # 수집 완료 후 HF 업로드 여부

resume: false                 # false: 새 데이터셋, true: 기존에 이어서 녹화
```

카메라, 토픽 설정도 환경에 맞게 수정.

## 실행

### 터미널 1 — 로봇 bringup + MoveIt

```bash
ros2 launch openarm_bringup bimanual_bringup.launch.py
```

### 터미널 2 — 텔레옵 노드

```bash
source ~/openarm/install/setup.bash
ros2 launch openarm_quest_teleop quest_teleop_bimanual.launch.py
```

### 터미널 3 — 데이터 수집

```bash
# 방법 1: CLI 명령어
openarm-collect --config ~/openarm/src/openarm_lerobot/config/collect_data.yaml

# 방법 2: 모듈 직접 실행
cd ~/openarm/src/openarm_lerobot
python -m openarm_lerobot.collect_data --config config/collect_data.yaml
```

## 에피소드 제어

| 입력 | 동작 |
|------|------|
| Quest X 버튼 | 현재 에피소드 종료 → 저장 → 다음 에피소드 |
| 키보드 → (오른쪽 화살표) | 위와 동일 |
| 키보드 ← (왼쪽 화살표) | 현재 에피소드 버리고 다시 녹화 |
| 키보드 Esc | 전체 녹화 중단 → finalize |

## 토픽 검증

데이터 수집 전 토픽이 정상 publish 되는지 확인:

```bash
# 새 토픽 목록
ros2 topic list | grep -E "eef|target_joint"

# 개별 토픽 확인
ros2 topic echo /left_eef_pose --once
ros2 topic echo /left_eef_delta --once
ros2 topic echo /left_target_joint_positions --once
```

## 수집되는 데이터 구조

### Observations
| Feature | Shape | 내용 |
|---------|-------|------|
| `observation.state` | (16,) | joint positions (left 7 + right 7 + grippers 2) |
| `observation.velocity` | (16,) | joint velocities |
| `observation.effort` | (16,) | joint efforts |
| `observation.left_eef_pose` | (7,) | 왼팔 EE 절대 pose (xyz + quaternion) |
| `observation.right_eef_pose` | (7,) | 오른팔 EE 절대 pose |
| `observation.images.*` | (H,W,3) | 카메라 영상 |

### Actions
| Feature | Shape | 내용 |
|---------|-------|------|
| `action` | (16,) | **기본 action** (= joint_positions). ACT 등 바로 학습 가능 |
| `action.joint_positions` | (16,) | Servo IK 명령값 (후처리용 보존) |
| `action.left_eef_delta` | (7,) | 왼팔 world frame delta (dxyz + dquaternion) |
| `action.right_eef_delta` | (7,) | 오른팔 world frame delta |
| `action.left_gripper` | (1,) | 왼쪽 gripper trigger |
| `action.right_gripper` | (1,) | 오른쪽 gripper trigger |

## 학습

수집 직후 바로 ACT 등 LeRobot 정책 학습 가능 (`action` = joint_positions 16차원).
EEF delta 등 다른 action 표현으로 실험하려면 후처리 필요 → `POSTPROCESS_SPEC.md` 참고.

## 데이터셋 파일 구조

```
datasets/your_dataset/
├── meta/
│   ├── info.json          # feature 정의, fps 등
│   ├── stats.json         # 정규화 통계
│   ├── tasks.parquet
│   └── episodes/
├── data/
│   └── chunk-000/
│       └── file-000.parquet
└── videos/
    └── left_wrist/
        └── chunk-000/
            └── file-000.mp4
```

## 주의사항

- feature 구조가 변경되면 반드시 `resume: false`로 새 데이터셋 생성
- `resume: true`는 동일 feature 구조일 때만 사용
- `push_to_hub: true` 시 HuggingFace 로그인 필요 (`huggingface-cli login`)
