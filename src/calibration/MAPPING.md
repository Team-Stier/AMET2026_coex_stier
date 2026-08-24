# Mapping for calibration

이 문서는 실제 차량을 수동으로 조종해 캘리브레이션 기준 데이터를 수집하는 절차를
정의한다. Controller, Path Planning 및 장애물 회피 기능은 지도 수집에 사용하지 않는다.

## 결과물

지도 수집은 서로 다른 두 결과물을 만든다.

1. LiDAR SLAM 점유지도: `map.yaml`, `map.pgm`
2. 경로 계획에 사용할 주행 중심선: RDDF 좌표

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

필수 토픽은 `/scan`, `/scan_filtered`, `/odom`이다. 시뮬레이터에서는 `/clock`을
반드시 함께 기록한다. `/imu`, `/odom/laser`, `/tf`, `/tf_static`도 함께 기록해 EKF
odometry와 laser odometry를 나중에 비교할 수 있게 한다.

## 2. 짧은 시험 기록

본 주행 전에 직선과 한 개의 코너가 포함된 짧은 bag을 만든다.

```bash
./src/calibration/tools/record_mapping_bag.sh smoke
```

스크립트는 필수 토픽의 이름과 타입을 검사한 뒤 `records/mapping/` 아래에 rosbag을
생성한다. 기록 중에는 다음 원칙을 지킨다.

- 낮고 일정한 속도로 수동 운전한다.
- 급조향, 바퀴 미끄러짐, 차량을 들어 옮기는 동작을 피한다.
- 움직이는 사람과 물체가 적을 때 기록한다.
- 출발 지점을 다시 통과해 loop closure에 필요한 중첩 구간을 만든다.
- 종료할 때 `Ctrl+C`는 한 번 누르고 bag metadata 기록이 끝날 때까지 기다린다.

## 3. 시험 bag 검사

```bash
ros2 bag info records/mapping/<bag-directory>
ros2 bag play records/mapping/<bag-directory> --clock
```

RViz에서 LaserScan, odometry 이동 및 TF 연결을 확인한다. 현재 확인된
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

위치가 바뀔 수 있는 시뮬레이터 라바콘을 정적 지도에서 제외할 때는 차량을 먼저
초기화한 다음, SLAM을 시작하기 전에 다음 명령을 실행한다.

```bash
./src/calibration/tools/remove_sim_mapping_cones.sh
```

제거 후 시뮬레이터를 초기화하거나 재시작하면 라바콘이 다시 나타날 수 있다. 제거가
끝난 뒤에는 다시 초기화하지 않고 바로 SLAM과 bag 기록을 시작한다. 실차에서는 이
스크립트를 사용하지 않으며, 라바콘을 실시간 obstacle/semantic layer로 처리한다.

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

점유지도는 주행 중심선을 직접 제공하지 않는다. 저장된 지도 위에서 진행 방향에 맞게
중심점을 선택하고, 일정 간격으로 재샘플링한 별도 RDDF를 만든다. RDDF 좌표계와 SLAM
`map` 좌표계의 관계, 점 간격, 시작점 및 폐곡선 연결 여부를 metadata에 기록한다.

## 5. Calibration 연결 조건

다음 조건이 충족된 후에만 LiDAR 펜스 odometry 보정을 사용한다.

- `map` 기준 네 펜스 선분의 실제 좌표가 측정되어 있다.
- `base_footprint`와 LiDAR 사이의 extrinsic이 확인되어 있다.
- `/scan_filtered`와 `/odom`의 timestamp 및 frame ID가 검증되어 있다.
- 같은 고정 시작 pose를 사용하거나 별도의 초기 정렬 수단이 있다.
- 주행 중 최소 3개 펜스를 동시에 관측할 수 있다.

시뮬레이션 결과는 초기 기준으로 사용할 수 있지만 실차와 동일하다고 가정하지 않는다.
최종 `/odom/calibride` 평가는 실차 수동주행 bag에서 보정 전후 위치와 heading 오차를
비교해 수행한다.
