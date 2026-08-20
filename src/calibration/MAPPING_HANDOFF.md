# Mapping 작업 인계 요약

마지막 확인: 2026-08-20 (Asia/Seoul)

## 목표

사용자 담당 영역은 `calibration`이다. 캘리브레이션 구현에 앞서 수동주행으로 원본
센서 데이터를 기록하고, LiDAR 점유지도와 RDDF 중심경로를 만드는 기반을 준비한다.

지도 수집 단계에는 Controller, Path Planning 및 장애물 회피를 연결하지 않는다.
시뮬레이션에서 먼저 기준 맵과 RDDF를 만들 수 있지만, 실차와 동일하다고 가정하지
않고 실차 수동주행 bag으로 회전·이동·스케일 차이를 검증한다.

## Git 상태

- 저장소: `Team-Stier/AMET2026_coex_stier`
- 로컬 경로: `/home/stier/physicar/physicar_ws`
- 현재 브랜치: `feature/Paik-map`
- 기준 커밋: `c4639c4 Document package structure and node mappings`
- `feature/Paik-map`은 로컬에서 새로 만든 브랜치이며 아직 upstream이 없다.
- 커밋, push 및 PR 생성은 하지 않았다.
- 사용자 변경사항을 삭제하거나 기존 브랜치 이름을 변경하지 않는다.

현재 변경 상태:

```text
 M .gitignore
?? src/calibration/MAPPING.md
?? src/calibration/MAPPING_HANDOFF.md
?? src/calibration/tools/
```

## 이번에 추가한 내용

### `.gitignore`

다음 생성물을 Git에서 제외했다.

```text
/records/
/maps/
```

원본 rosbag은 재처리 기준이므로 삭제하거나 덮어쓰지 않는다.

### `src/calibration/tools/check_mapping_topics.sh`

ROS 2 Jazzy 환경을 불러오고 맵 수집에 필요한 토픽 이름과 타입을 검사한다.

필수 토픽:

- `/scan` (`sensor_msgs/msg/LaserScan`)
- `/odom` (`nav_msgs/msg/Odometry`)
- `/camera/image_raw/compressed` (`sensor_msgs/msg/CompressedImage`)

선택 기록 토픽:

- `/scan_filtered`
- `/imu`
- `/odom/laser`
- `/clock`
- `/camera/pan`
- `/tf`
- `/tf_static`

필수 토픽이 없으면 종료 코드 2로 기록을 차단한다. 검사 후 TF와 토픽 주기 확인 명령을
안내한다.

### `src/calibration/tools/record_mapping_bag.sh`

필수 토픽 검사를 통과한 뒤 다음 위치에 timestamp 기반 rosbag을 기록한다.

```text
records/mapping/<YYYYMMDD_HHMMSS>_<label>/
```

기본 실행:

```bash
cd /home/physicar/physicar_ws
./src/calibration/tools/record_mapping_bag.sh smoke
```

`ROS_SETUP`과 `RECORDS_ROOT` 환경변수로 ROS setup 및 출력 위치를 바꿀 수 있다.

### `src/calibration/MAPPING.md`

다음 전체 절차를 문서화했다.

1. ROS 2 장비와 토픽 확인
2. 짧은 수동주행 시험 bag 기록
3. `ros2 bag info`, replay 및 RViz 검사
4. 실제 frame ID를 확인한 후 `slam_toolbox` 설정
5. 점유지도와 별도 RDDF 생성
6. 실차 bag으로 Calibration 연결 조건 검증

## 검증 결과

확인된 것:

- 두 스크립트의 `bash -n` 문법 검사 통과
- mock ROS 2 토픽 목록으로 필수/선택 토픽 판별 흐름 확인
- 필수 토픽이 없을 때 실패하는 경로 확인
- `git diff --check` 통과

확인하지 못한 것:

- 실제 PhysiCar/시뮬레이터 rosbag 기록
- 실제 `/odom` → `base_link` TF
- 각 토픽의 발행률과 timestamp 동기화
- `slam_toolbox` 설치 및 지도 생성
- loop closure 품질
- 시뮬레이션과 실차 맵의 좌표 정합
- RDDF 생성과 `/odom/calibride` 보정

현재 개발 PC는 `ROS_DISTRO=noetic`이며 `ros2`, `colcon`,
`/opt/ros/jazzy/setup.bash`가 없다. 따라서 실제 ROS 2 실행 성공으로 보고하면 안 된다.

## 확보된 시뮬레이터 런타임 증거

2026-08-20에 ROS 2 시뮬레이터 출력으로 다음을 확인했다.

- `/scan` publisher: `ros_gz_bridge`, type `sensor_msgs/msg/LaserScan`, reliable QoS
- `/scan_filtered`: 실제 토픽 존재
- `/odom` publisher: `ekf_filter_node`, type `nav_msgs/msg/Odometry`, reliable QoS
- `/odom/laser`: 별도 laser odometry 토픽 존재
- `/camera/image_raw/compressed` publisher: `image_republisher`, reliable QoS
- `/clock`: 시뮬레이션 시간 토픽 존재
- `/odom` 메시지: `frame_id=odom`, `child_frame_id=base_footprint`
- 동적 TF: `odom → base_footprint`, 약 30 Hz
- 고정 TF: `base_footprint → base_link → lidar_link/imu_link`
- 카메라 TF: `base_link → camera_pan_link → camera_tilt_link → camera_link → camera_optical_frame`
- pan/tilt와 차륜 TF는 약 20 Hz로 갱신됨

따라서 시뮬레이터 SLAM의 현재 후보 설정은 다음과 같다.

```text
map_frame: map
odom_frame: odom
base_frame: base_footprint
scan_topic: /scan_filtered  # 추가 확인 후 확정
use_sim_time: true
```

아직 `/scan`과 `/scan_filtered`의 `header.frame_id`, `/scan_filtered` publisher/QoS,
토픽별 실제 발행률은 확보하지 못했다.

후속 출력으로 다음이 추가 확인됐다.

- `/scan`과 `/scan_filtered`의 frame은 모두 `lidar_link`
- 두 scan은 약 ±π의 360도 범위, 0.1–16.0 m 범위를 사용
- `scan_filter`: `/scan` → `/scan_filtered`, `range_margin=0.1`, simulation time 사용
- `/odom/laser` publisher는 `laser_odom`
- `/odom/laser`도 `odom → base_footprint` 계약을 사용
- 확인된 `/odom/laser` covariance는 전부 0이므로 신뢰도 수치로 사용하지 않음
- `slam_toolbox`의 async/sync mapping 실행 파일 설치 확인

이 증거를 기준으로 `config/slam_toolbox_sim.yaml`, `tools/start_sim_mapping.sh`,
`tools/save_sim_map.sh`를 추가했다. 초기 `scan_topic`은 `/scan_filtered`로 확정했다.

사용자가 `/sim/api/route` 경로 JSON도 제공했다. 669개 waypoint, 약 32.259 m 길이,
평균 약 0.0483 m 간격의 완전 폐곡선이며 반시계 방향이다. 다만 경로는 world 좌표로
보이고 `/odom`은 원점 부근에서 시작하므로, 동일 timestamp의 world pose와 `/odom`
pose로 `world → odom/map` 변환을 구하기 전에는 RDDF와 SLAM 지도를 직접 겹치지 않는다.

## 다음 화면에서 바로 할 작업

ROS 2 Jazzy가 실행되는 시뮬레이터 또는 PhysiCar 장비에서 다음을 수행한다.

```bash
cd /home/physicar/physicar_ws

./src/calibration/tools/check_mapping_topics.sh

ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic echo /scan --once --field header
ros2 topic echo /scan_filtered --once --field header
ros2 topic info /scan_filtered --verbose
ros2 topic hz /scan
ros2 topic hz /scan_filtered
ros2 topic hz /odom
ros2 topic hz /odom/laser
ros2 topic hz /camera/image_raw/compressed

./src/calibration/tools/record_mapping_bag.sh smoke
```

기록 후:

```bash
ros2 bag info records/mapping/<bag-directory>
ros2 bag play records/mapping/<bag-directory> --clock
```

RViz에서 `/scan`, `/odom`, 카메라 영상 및 TF를 확인한다. 실제 frame ID와 bag 경로,
`ros2 bag info` 결과를 확보한 다음에만 `slam_toolbox` 파라미터 파일을 작성한다.

## 다음 작업에서 지켜야 할 판단 기준

- `/camera/pan`은 실제 서보 위치 피드백이 아니라 목표 명령값이다.
- 첫 맵 기록에서는 카메라를 정면에 고정한다.
- 점유지도와 RDDF는 같은 결과물이 아니므로 분리한다.
- 시뮬레이터 ground-truth pose와 센서 drift가 포함된 `/odom`을 구분한다.
- 일반 사진은 차선 검출 개발에는 사용할 수 있지만, pose와 timestamp가 없으면 odometry
  보정의 정량 평가 기준으로 사용하지 않는다.
- 실제 frame ID, 토픽 타입 및 주기를 코드나 문서만 보고 추측하지 않는다.
- 원본 bag과 사용자 파일을 보존한다.

## 다음 Codex 요청 예시

```text
src/calibration/MAPPING_HANDOFF.md를 먼저 읽고 이어서 진행해.
ROS 2 Jazzy 장비에서 얻은 check_mapping_topics 출력, TF 출력과 smoke bag 정보를
기준으로 slam_toolbox 설정 및 첫 점유지도 생성 절차를 구현해줘.
```
