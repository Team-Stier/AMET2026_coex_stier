# Mapping for calibration

이 문서는 실제 차량을 수동으로 조종해 캘리브레이션 기준 데이터를 수집하는 절차를
정의한다. Controller, Path Planning 및 장애물 회피 기능은 지도 수집에 사용하지 않는다.

## 결과물

지도 수집은 서로 다른 두 결과물을 만든다.

1. LiDAR SLAM 점유지도: `map.yaml`, `map.pgm`
2. 차선과 odometry를 정합하기 위한 주행 중심선: RDDF 좌표

원본 bag은 재처리의 기준이므로 수정하지 않는다. 생성한 지도와 RDDF는 원본 bag과
분리한다.

```text
records/mapping/<timestamp>_<label>/  # 원본 rosbag, Git 제외
maps/occupancy/                       # SLAM 지도, Git 제외
maps/rddf/                            # 검토 전 생성 RDDF, Git 제외
```

## 1. 장비와 토픽 확인

이 저장소의 실행 대상은 ROS 2 Jazzy다. 다음 명령은 ROS 2가 설치된 PhysiCar 장비 또는
동일 인터페이스의 시뮬레이터에서 실행한다.

```bash
cd /home/physicar/physicar_ws
./src/calibration/tools/check_mapping_topics.sh
```

필수 토픽은 `/scan`, `/odom`, `/camera/image_raw/compressed`다. 시뮬레이터에서는
`/clock`을 반드시 함께 기록한다. `/imu`, `/scan_filtered`, `/odom/laser`, `/tf`,
`/tf_static`, `/camera/pan`도 함께 기록해 EKF odometry와 laser odometry를 나중에
비교할 수 있게 한다.

`/camera/pan`은 실제 서보 위치 피드백이 아니라 명령값이다. 첫 데이터 수집에서는
카메라를 정면에 고정하고 pan 명령값과 실제 장착 방향을 별도로 기록한다.

## 2. 짧은 시험 기록

본 주행 전에 직선과 한 개의 코너가 포함된 짧은 bag을 만든다.

```bash
./src/calibration/tools/record_mapping_bag.sh smoke
```

스크립트는 필수 토픽의 이름과 타입을 검사한 뒤 `records/mapping/` 아래에 rosbag을
생성한다. 기록 중에는 다음 원칙을 지킨다.

- 낮고 일정한 속도로 수동 운전한다.
- 급조향, 바퀴 미끄러짐, 차량을 들어 옮기는 동작을 피한다.
- 카메라 pan을 고정한다.
- 움직이는 사람과 물체가 적을 때 기록한다.
- 출발 지점을 다시 통과해 loop closure에 필요한 중첩 구간을 만든다.
- 종료할 때 `Ctrl+C`는 한 번 누르고 bag metadata 기록이 끝날 때까지 기다린다.

## 3. 시험 bag 검사

```bash
ros2 bag info records/mapping/<bag-directory>
ros2 bag play records/mapping/<bag-directory> --clock
```

RViz에서 LaserScan, odometry 이동, TF 연결 및 카메라 프레임을 확인한다. 현재 확인된
시뮬레이터 TF의 이동 기준은 `odom → base_footprint`이며 `base_footprint → base_link`는
고정 TF다. `/scan` 메시지의 `header.frame_id`까지 확인한 뒤 SLAM 설정을 확정한다.

## 4. SLAM과 RDDF 생성

시험 bag 검증 후 ROS 2 Jazzy용 `slam_toolbox`를 사용해 2D 점유지도를 만든다. SLAM
파라미터에는 시험 bag에서 확인한 `map_frame`, `odom_frame`, `base_frame`,
`scan_topic`을 사용한다. 현재 시뮬레이터 확정값은 `map`, `odom`, `base_footprint`,
`/scan_filtered`다. rosbag을 재생해 SLAM을 실행할 때는 `/clock`을 재생하고
`use_sim_time:=true`를 사용한다. 전체
코스를 최소 두 번 겹쳐 주행하고 시작 구간에서 loop closure가 형성되는지 확인한 후
지도를 저장한다.

```bash
./src/calibration/tools/start_sim_mapping.sh
```

RViz의 fixed frame은 `map`으로 설정한다. 지도 품질을 확인한 뒤 다른 터미널에서 결과를
저장한다.

```bash
./src/calibration/tools/save_sim_map.sh smoke
```

점유지도뿐 아니라 `slam_toolbox` pose graph 서비스가 사용 가능하면 재편집 가능한
pose graph도 같은 출력 디렉터리에 저장한다.

### 시뮬레이터 웨이포인트 자동 수집

시뮬레이터에서는 `control`에 보관된 검증 경로를 저속으로 두 바퀴 추종할 수 있다.
SLAM과 `record_mapping_bag.sh`를 먼저 시작한 뒤 별도 터미널에서 실행한다. 이 도구는
SIM ground-truth pose를 사용하므로 실차에는 사용하지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0

python3 src/control/tools/sim_waypoint_mapping_drive.py \
  --laps 2 \
  --target-speed-m-s 0.55 \
  --maximum-path-error-m 0.20
```

주행이 끝나면 `save_sim_map.sh`로 지도와 pose graph를 저장한 다음 rosbag과 SLAM을
각각 한 번의 `Ctrl+C`로 종료한다. 종료 시 차량은 속도·조향 0 명령을 반복 수신한다.

## 5. 노란선·흰선 Lane Map 누적

전체 RGB 지면 텍스처는 누적하지 않는다. 카메라 영상에서 검출한 노란선과 흰선 픽셀만
투영한다. 일반 bag은 영상 timestamp의 SLAM TF를 보간한다. 시뮬레이터에서는
`/mapping/sim_pose`를 함께 기록하고 `--pose-source sim_ground_truth`를 지정해
odometry 방향 오류와 loop closure 변화가 차선 layer에 누적되지 않게 한다. 카메라의
ground projection은 오차가 커지는 원거리를 제외하고 기본 0.15–1.8 m만 사용한다. TF
지지 샘플이 허용 시간보다 멀면 해당 프레임은 버리고, 여러 관측의 hit/view 비율로
잡음을 제거한다.

```bash
python3 src/calibration/tools/build_camera_lane_map.py \
  records/mapping/<bag-directory> \
  maps/occupancy/<map-directory>/map.yaml \
  --pose-source sim_ground_truth \
  --max-tf-age-ms 75
```

기본 출력 디렉터리는 `camera_lane_accumulation/`이며 다음 파일을 만든다.

- `lane_layer.png`: 투명 배경의 노란선·흰선 측정 layer
- `lane_overlay.png`: 원본 LiDAR 지도 위 확인용 overlay
- `lane_votes.npz`: threshold를 다시 조정할 수 있는 hit/view 누적값
- `lane_map_metadata.json`: 입력 bag, TF age, 해상도와 검출 통계

이 layer는 occupancy map이 아니므로 Nav2에는 원본 `map.yaml`과 `map.pgm`을 계속
사용한다. 669개 waypoint는 안정적인 수집 주행에만 사용할 수 있으며, Lane Map의
위치나 모양을 결정하는 입력으로 사용하지 않는다.

점유지도는 주행 중심선을 직접 제공하지 않는다. 저장된 지도 위에서 진행 방향에 맞게
중심점을 선택하고, 일정 간격으로 재샘플링한 별도 RDDF를 만든다. RDDF 좌표계와 SLAM
`map` 좌표계의 관계, 점 간격, 시작점 및 폐곡선 연결 여부를 metadata에 기록한다.

## 6. Calibration 연결 조건

다음 조건이 충족된 후에만 실제 Lane Map 기반 odometry 보정을 구현한다.

- 카메라 intrinsic과 왜곡 계수가 확보되어 있다.
- 카메라와 `base_link` 사이의 extrinsic 또는 측정 가능한 장착값이 있다.
- 이미지, odometry 및 camera pan의 timestamp 관계가 확인되어 있다.
- 노란선·흰선 누적 결과에서 실제 중앙선이 추출되고 `map` 좌표계에서 검증되어 있다.
- 시뮬레이션 RDDF와 실차 지도 사이의 회전, 이동 및 스케일 차이가 측정되어 있다.

시뮬레이션 결과는 초기 기준으로 사용할 수 있지만 실차와 동일하다고 가정하지 않는다.
최종 `/odom/calibride` 평가는 실차 수동주행 bag에서 보정 전후 횡방향 오차와 heading
오차를 비교해 수행한다.
