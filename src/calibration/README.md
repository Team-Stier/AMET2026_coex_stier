# Calibration package

현재 활성 보정 경로는 2D LiDAR 외곽 펜스 전용이다. 시뮬레이터의 네 외곽선
`x=0`, `x=12`, `y=0`, `y=7`을 사용하며 기본 실행과 YAML 모두
`correction_source=fence`다. 펜스 모드에서는 카메라 영상과 `/camera/pan`을
구독하지 않고 중앙 노란선을 검출하거나 보정 입력으로 사용하지 않는다.

## LiDAR 펜스 보정

```text
/scan_filtered + scan timestamp의 base_footprint ← lidar_link TF
  → 유효 range를 base_footprint 평면 점으로 변환
  → 현재 보정 pose로 map/odom 좌표에 투영
  → 네 유한 선분에서 0.25m 이내인 점만 연결
  → Huber point-to-line SE(2) 최적화
  → 최소 3개 펜스와 80개 점을 확인
  → EKF에서 x/y/yaw 측정 갱신
  → rate limit을 거친 /odom/calibride
```

라바콘처럼 펜스에서 떨어진 내부 물체는 대응점에서 제외한다. 한쪽 벽만 보이는
경우처럼 pose를 충분히 관측할 수 없으면 보정하지 않고 기존 odometry 예측을
유지한다. 한 번의 측정에서 `0.35m` 또는 `0.15rad`를 넘는 보정도 거부한다.

펜스 좌표는 `reference_frame` 기준이다. 기본 실행은 첫 `/odom` pose를 고정 시작
pose `map (1.4m, 3.4m, -pi/2rad)`에 맞춰 `map→odom`을 한 번 계산하고 static TF로
발행한다. 매번 같은 시작 위치와 방향으로 출발한다는 조건이 깨지면 이 값도 함께
바꿔야 한다. 시뮬레이터 ground truth는 검증에만 사용하며 fence matcher가 직접
구독하지 않는다.

## 비활성 차선 구현

기존 실험 결과와 사용자 변경사항을 보존하기 위해 차선 관련 모듈은 삭제하지 않았다.
하지만 현재 기본 실행과 배포 설정에서는 해당 경로를 사용하지 않는다.

## 처리 흐름

```text
/camera/image_raw/compressed
  + image timestamp의 base_footprint → camera_optical_frame TF
  → 차량 지면 좌표 BEV
  → 좁은 HSV 범위로 노란선 mask
  → 최소 면적 필터 + skeleton 특징점
  → /calibration/detected_centerline
  + sim_lane_map_world_color.png의 전체 중앙선 특징점·접선
  → 관측점마다 반경 0.20m의 국소 1차 직선 피팅
  → 맵 접선과 방향 차이가 0.44rad(약 25도) 이내인 점만 선택
  → 반복 point-to-line 기하 매칭
  → /odom/calibride
```

차량 좌표는 전방 `+x`, 왼쪽 `+y`다. 출력 `nav_msgs/Path`의 frame은
`base_footprint`이며 이미지와 동일한 timestamp를 사용한다.

## 실행

기본값은 코드에 포함되어 있어 기존 실행 계약을 유지한다.

```bash
ros2 run calibration calibration_node
```

명시적인 시뮬레이터 설정을 사용할 때:

```bash
ros2 run calibration calibration_node --ros-args \
  --params-file install/calibration/share/calibration/config/lane_detection.yaml
```

외곽 펜스와 네 경로를 비교하려면 다음 launch 하나를 사용한다.

```bash
ros2 launch calibration sim_localization_rviz.launch.py
```

RViz 표시는 회색 펜스, 빨강 `/odom`, 주황 `/odom/laser`, 청록 fence corrected,
초록 simulator truth다. 각 현재 pose 라벨에는 truth 기준 거리·yaw 오차가 표시된다.
중앙선 이미지나 카메라 검출 결과는 표시하지 않는다. bridge도 각 odometry의 첫
pose를 동일한 고정 시작 pose에 독립 정렬하며 simulator truth는 비교에만 사용한다.
누적 Path는 1초마다 재발행되어 rosbag 주행이 끝난 뒤에도 RViz에 남는다.

## 출력

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/calibration/detected_centerline` | `nav_msgs/msg/Path` | 차량 좌표의 검출 중앙선 |
| `/calibration/debug/bev/compressed` | `sensor_msgs/msg/CompressedImage` | 차량 지면 BEV |
| `/calibration/debug/lane_mask/compressed` | `sensor_msgs/msg/CompressedImage` | 노란선 binary mask |
| `/calibration/debug/lane_overlay/compressed` | `sensor_msgs/msg/CompressedImage` | skeleton 특징점 overlay |
| `/odom/calibride` | `nav_msgs/msg/Odometry` | 보정 또는 안전 fallback odometry |
| `/calibration/rviz/raw_path` | `nav_msgs/msg/Path` | 고정 시작 정렬 `/odom` 경로 |
| `/calibration/rviz/laser_path` | `nav_msgs/msg/Path` | 고정 시작 정렬 `/odom/laser` 경로 |
| `/calibration/rviz/corrected_path` | `nav_msgs/msg/Path` | fence corrected 경로 |
| `/calibration/rviz/truth_path` | `nav_msgs/msg/Path` | 비교 전용 simulator truth 경로 |
| `/calibration/rviz/fence` | `visualization_msgs/msg/Marker` | `12m x 7m` 기준 펜스 |

검출 confidence가 `minimum_confidence`보다 낮으면 centerline Path를 발행하지 않는다.
디버그 영상은 `publish_debug_images=false`로 끌 수 있다.

1.5m BEV에서 실측한 차선 범위인 `H=15~31, S=30~220, V=30~220`을 사용한다.
라바콘의 고채도 중심은 제외되지만 보간된 가장자리는 일부 포함될 수 있으며, 별도의
주변 영역 삭제는 하지 않는다.

## 카메라 모델 주의사항

현재 시뮬레이터 제공값은 해상도 `480×360`, horizontal FOV `1.7453 rad`, distortion
`[-0.045, -0.0001, -0.0003, -0.0001, 0.001]`다. `/camera_info`가 없으므로 square
pixel과 영상 중앙 principal point를 가정해 FOV에서 `fx`, `fy`를 계산한다.

이 가정은 시뮬레이터 BEV 초기 검증에는 사용할 수 있지만 실차 정밀 보정의 근거가
될 수 없다. 실차에서는 체커보드로 구한 camera matrix와 distortion을 사용하도록
`CameraModel` 입력을 교체해야 한다.

카메라 pan 명령 토픽은 실제 각도 피드백으로 사용하지 않는다. 모든 투영은 이미지
timestamp의 `base_footprint → camera_optical_frame` TF를 사용한다.

## Lane Map 연결

컬러 맵에서 HSV로 주황색 중앙선만 선택하고 skeleton 특징점과 국소 접선을 미리
계산한다. 카메라 관측점도 전체를 하나의 직선으로 피팅하지 않고 각 점 주변
`0.20m` 구간을 독립적인 1차 직선으로 피팅한다. 최근접 맵 점까지의 거리뿐 아니라
두 국소 접선의 방향 차이가 `0.44rad` 이내인 대응점만 사용해 횡오차와 yaw를
제한적으로 보정한다. 따라서 급커브와 꺾인 구간을 하나의 직선이나 포물선으로
뭉개지 않는다.

맵 좌표는 `sim_world` 기준 오른쪽 `+x`, 위쪽 `+y`다. 설정의 `reference_frame`은
`map`이므로 실행 중 timestamp에 맞는 `odom ← map` TF가 반드시 있어야 한다.
이 TF는 SLAM 또는 별도의 초기 정렬 노드가 제공해야 한다. 첫 유효 보정 전에는 원본
`/odom`을 발행한다. 첫 유효 보정 이후에는 보정 pose EKF가 odom의 상대 이동량으로
상태를 예측하고 차선 매칭 pose로 상태를 갱신한다. 출력 pose는 EKF 추정값까지 위치
`0.08m/s`, yaw `0.08rad/s` 이하로 접근하므로 새 측정 순간에 순간이동하지 않는다.
차선 측정이 끊겨도 마지막 보정 상태에서 odom 이동량만 계속 적용하며, 시뮬 시간이
되감길 때만 초기화한다.
초기 EKF 공분산은 첫 `/odom.pose.covariance`의 x·y·yaw 블록에서 가져오고, 이후
process noise는 `/odom.twist.covariance`를 `dt²`로 적분한 값과 YAML noise floor를
합쳐 계산한다. `/odom/calibride.pose.covariance`에는 EKF posterior와 rate-limit으로
아직 적용되지 않은 보정 잔여량을 함께 반영한다.

기존 순서형 CSV는 `reference_lane_map_file`을 비웠을 때만 fallback으로 사용할 수
있다.

## 검증 순서

1. 정지 상태에서 BEV 도로 경계가 실제와 맞는지 확인한다.
2. 직선에서 노란 mask와 skeleton 특징점을 확인한다.
3. 좌우 코너에서 `Path`의 곡률과 방향을 확인한다.
4. TF 누락·오래된 timestamp에서는 Path가 발행되지 않는지 확인한다.
5. rosbag 전체에서 confidence, 횡위치 및 heading 연속성을 기록한다.
6. 원본과 `/odom/calibride`의 횡오차, yaw, 불연속 및 fallback 전환을 비교한다.
