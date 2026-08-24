# PhysiCar simulator runtime evidence

이 문서는 2026-08-20에 사용자가 제공한 ROS 2 CLI 출력에서 직접 확인한 내용만 기록한다.
README의 설계값과 실제 runtime evidence를 구분하기 위한 자료다.

## Mapping 관련 토픽

| 토픽 | 타입 | 확인된 publisher |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | `ros_gz_bridge` |
| `/scan_filtered` | `sensor_msgs/msg/LaserScan` | `scan_filter`로 추론됨 |
| `/odom` | `nav_msgs/msg/Odometry` | `ekf_filter_node` |
| `/odom/laser` | `nav_msgs/msg/Odometry` | `laser_odom` |
| `/imu` | `sensor_msgs/msg/Imu` | 미확인 |
| `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | `image_republisher` |
| `/camera/pan` | `std_msgs/msg/Float64` | 미확인 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 미확인 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 복수 TF broadcaster |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 복수 TF broadcaster |

`/scan`, `/odom`, `/odom/laser`, `/camera/image_raw/compressed` publisher는 reliable QoS다. 센서 소비
노드는 이 publisher들과 호환되는 QoS를 사용해야 한다.

`/scan`과 `/scan_filtered`에서 공통으로 확인된 LaserScan 계약은 다음과 같다.

```text
frame_id: lidar_link
angle_min: 약 -π
angle_max: 약 +π
angle_increment: 약 0.008739 rad
range_min: 0.1 m
range_max: 16.0 m
```

`scan_filter`는 `/scan`을 받아 `/scan_filtered`를 발행하며 `range_margin=0.1`과
`use_sim_time=true`를 사용한다. 따라서 초기 SLAM raster 범위는 0.2–15.9 m로 맞춘다.

## Odometry

`/odom`은 `ekf_filter_node`가 발행한다. 확인된 한 메시지는 다음 frame 계약을 가진다.

```text
header.frame_id: odom
child_frame_id: base_footprint
```

따라서 `/odom`을 Gazebo ground-truth pose라고 단정하면 안 된다. `/odom/laser`는
`laser_odom`이 발행하며 동일하게 `odom → base_footprint` 계약을 사용한다. 다만 확인된
메시지의 pose와 twist covariance가 모두 0이므로 이를 실제 불확실성 0으로 해석하면
안 된다. 두 odometry를 함께 기록해 EKF 출력과 laser odometry의 차이를 비교한다.

## TF tree

확인된 주요 체인은 다음과 같다.

```text
odom
└── base_footprint               # dynamic, 약 30 Hz
    └── base_link                # static
        ├── lidar_link           # static
        ├── imu_link             # static
        └── camera_pan_link      # dynamic, 약 20 Hz
            └── camera_tilt_link # dynamic, 약 20 Hz
                └── camera_link  # static
                    └── camera_optical_frame # static
```

## SLAM 후보 계약

현재 증거로 정할 수 있는 후보는 다음과 같다.

```yaml
use_sim_time: true
map_frame: map
odom_frame: odom
base_frame: base_footprint
scan_topic: /scan_filtered
```

초기 지도 제작에는 차량 센서 스택이 제공하는 range-margin 필터를 거친
`/scan_filtered`를 사용한다. 원시 `/scan`도 같은 bag에 기록하므로 지도 누락이나 왜곡이
보이면 동일 bag을 `/scan`으로 재처리해 비교한다.

## 추가로 필요한 출력

```bash
ros2 topic echo /scan --once --field header
ros2 topic echo /scan_filtered --once --field header
ros2 topic info /scan_filtered --verbose
ros2 topic info /odom/laser --verbose
ros2 topic echo /odom/laser --once
ros2 topic hz /scan
ros2 topic hz /scan_filtered
ros2 topic hz /odom
ros2 topic hz /odom/laser
ros2 topic hz /camera/image_raw/compressed
```

## 제공된 시뮬레이터 경로 데이터

사용자가 제공한 `/sim/api/route` JSON은 파싱 가능한 정상 데이터다.

| 항목 | 확인값 |
|---|---:|
| waypoint 수 | 669개 |
| 총 경로 길이 | 약 32.259 m |
| 평균 점 간격 | 약 0.0483 m |
| 최대 점 간격 | 약 0.0499 m |
| 시작-끝 간격 | 0 m, 완전 폐곡선 |
| 진행 방향 | 반시계 방향(CCW) |
| x 범위 | 1.381–10.833 m |
| y 범위 | 0.668–5.749 m |
| 경로 외접 크기 | 약 9.451 × 5.081 m |

metadata에는 `sim_only=true`, `sample_spacing_m=0.05`, 목표 및 측정 최대 곡률
`1.8 1/m`, 최소 경계 여유 `0.364 m`가 기록되어 있다. 이 수치는 제공 JSON에서 확인한
시뮬레이션 경로 생성 결과이며 실차 검증값으로 사용하지 않는다.

경로 첫 점은 약 `(10.809, 2.440)`이지만 확인된 `/odom` pose는 원점 부근이다. 따라서
경로 좌표를 SLAM `map` 또는 `odom` 좌표라고 단정하지 않는다. 동일 timestamp에서
시뮬레이터 world pose와 `/odom` pose를 함께 확보해 `world → odom/map` 2D rigid
transform `(tx, ty, yaw)`을 계산한 후 RDDF를 변환해야 한다.
