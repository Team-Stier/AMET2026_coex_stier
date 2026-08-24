# Calibration package

카메라 중앙 노란선의 BEV 검출과 Lane Map 기준 횡방향·yaw odometry 보정을
제공한다. 현재는 실제 Lane Map이 없으므로 기본 설정에서 보정이 비활성화되며,
`/odom/calibride`에는 입력 `/odom`이 변경 없이 발행된다.

## 처리 흐름

```text
/camera/image_raw/compressed
  + image timestamp의 base_footprint → camera_optical_frame TF
  → 차량 지면 좌표 BEV
  → HSV 노란선 mask
  → 강건한 2차 곡선 y(x) 피팅
  → /calibration/detected_centerline
  + 실제 Lane Map centerline CSV
  → point-to-line 횡오차/yaw 추정
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

## 출력

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/calibration/detected_centerline` | `nav_msgs/msg/Path` | 차량 좌표의 검출 중앙선 |
| `/calibration/debug/bev/compressed` | `sensor_msgs/msg/CompressedImage` | 차량 지면 BEV |
| `/calibration/debug/lane_mask/compressed` | `sensor_msgs/msg/CompressedImage` | 노란선 binary mask |
| `/calibration/debug/lane_overlay/compressed` | `sensor_msgs/msg/CompressedImage` | 피팅 곡선 overlay |
| `/odom/calibride` | `nav_msgs/msg/Odometry` | 보정 또는 안전 fallback odometry |

피팅 confidence가 `minimum_confidence`보다 낮으면 centerline Path를 발행하지 않는다.
디버그 영상은 `publish_debug_images=false`로 끌 수 있다.

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

이전에 제공된 669개 waypoint는 주행경로이지 실제 중앙선이 아니므로 보정 입력으로
사용하지 않는다. 여러 번의 mapping으로 실제 중앙선이 생성된 뒤, `map` 좌표의 순서가
있는 점들을 다음 CSV 형식으로 저장한다.

```csv
x,y
1.25,0.84
1.30,0.86
1.35,0.89
```

폐곡선은 마지막 점 다음에 첫 점이 오도록 첫 점을 파일 끝에 한 번 더 기록한다.
실행할 때 다음 파라미터를 지정한다.

```bash
ros2 run calibration calibration_node --ros-args \
  --params-file install/calibration/share/calibration/config/lane_detection.yaml \
  -p reference_centerline_file:=/absolute/path/centerline.csv \
  -p reference_frame:=map
```

노드는 timestamp에 맞는 `odom ← map` TF로 기준선을 변환한 뒤 보정한다. 전후 방향
위치는 원본 odometry에 맡기고 차량 횡방향과 yaw만 제한적으로 변경한다. 기준선 파일,
TF 또는 최근의 신뢰도 높은 차선 검출 중 하나라도 없으면 원본 `/odom`을 그대로
발행한다.

## 검증 순서

1. 정지 상태에서 BEV 도로 경계가 실제와 맞는지 확인한다.
2. 직선에서 노란 mask와 피팅 곡선을 확인한다.
3. 좌우 코너에서 `Path`의 곡률과 방향을 확인한다.
4. TF 누락·오래된 timestamp에서는 Path가 발행되지 않는지 확인한다.
5. rosbag 전체에서 confidence, 횡위치 및 heading 연속성을 기록한다.
6. 원본과 `/odom/calibride`의 횡오차, yaw, 불연속 및 fallback 전환을 비교한다.
