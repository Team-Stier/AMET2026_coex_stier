# Calibration package

카메라 중앙 노란선의 BEV 특징점 검출과 시뮬레이터 컬러 Lane Map 기준
횡방향·yaw odometry 보정을 제공한다. 기준 맵은
`docs/sim_lane_map_world_color.png`이며 해상도는 `0.01 m/px`다.

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

전체 컬러 맵 위에서 시뮬 위치와 보정 결과를 비교하려면 다음 launch 하나를 사용한다.

```bash
ros2 launch calibration sim_localization_rviz.launch.py
```

RViz 표시는 빨강 `RAW`, 청록 `CORRECTED`, 초록 `TRUTH`이며 각 라벨에 실제 위치
기준 거리·yaw 오차가 표시된다. 자홍색 점은 현재 카메라에서 검출한 중앙선이다.
지도 배경은 `0.02m` 간격으로 표시하지만 노란 중앙선은 누락되지 않도록 원본 지도
해상도인 `0.01m` 간격의 모든 픽셀을 별도 레이어로 표시한다.
브리지는 시작 순간의 시뮬 실제 pose만 이용해 고정 `map→odom`을 초기화한다. 이후
실제 pose는 비교 표시에만 사용하며 보정 계산에는 입력하지 않는다.

## 출력

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/calibration/detected_centerline` | `nav_msgs/msg/Path` | 차량 좌표의 검출 중앙선 |
| `/calibration/debug/bev/compressed` | `sensor_msgs/msg/CompressedImage` | 차량 지면 BEV |
| `/calibration/debug/lane_mask/compressed` | `sensor_msgs/msg/CompressedImage` | 노란선 binary mask |
| `/calibration/debug/lane_overlay/compressed` | `sensor_msgs/msg/CompressedImage` | skeleton 특징점 overlay |
| `/odom/calibride` | `nav_msgs/msg/Odometry` | 보정 또는 안전 fallback odometry |

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
