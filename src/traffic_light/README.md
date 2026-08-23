# traffic_light

카메라 영상에서 신호등을 검출하고 주행 허용 여부를 ROS 2 토픽으로 발행하는
패키지입니다. Raspberry Pi 5 배포를 위해 YOLO26n 모델을 NCNN 형식으로
실행합니다.

## 동작

| 구분 | 이름 | 타입 |
| --- | --- | --- |
| 입력 | `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` |
| 출력 | `/gosign` | `std_msgs/msg/Bool` |
| 선택 출력 | `/traffic_light/debug/compressed` | `sensor_msgs/msg/CompressedImage` |

판정 규칙은 다음과 같습니다.

- 빨강 또는 노랑이 검출되면 즉시 `false`를 발행합니다.
- 초록만 연속 3프레임 검출되면 `true`를 발행합니다.
- 신호등을 찾지 못하거나 여러 색 중 정지 신호가 포함되면 `false`입니다.
- 주행 허용 상태에서 카메라 입력이 1초 이상 끊기면 `false`를 발행합니다.

## 모델

기본 배포 모델은 다음 디렉터리에 있습니다.

```text
models/traffic_light_yolo26n-3/deploy/best_ncnn_model
```

클래스 ID는 `0: red`, `1: yellow`, `2: green`입니다. 노드 시작 시 모델의
클래스 구성이 다르면 실행을 중단합니다.

## 빌드 및 테스트

워크스페이스 루트에서 실행합니다.

```bash
cd /home/physicar/physicar_ws
source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-select traffic_light \
  --symlink-install

source install/setup.bash

colcon test \
  --packages-select traffic_light \
  --event-handlers console_direct+

colcon test-result \
  --test-result-base build/traffic_light \
  --verbose
```

정상 결과는 `0 errors`, `0 failures`입니다. Matplotlib의 `Axes3D` 경고는
신호등 추론 기능과 관계없습니다.

Python 실행 의존성이 준비됐는지는 다음 명령으로 확인할 수 있습니다.

```bash
python3 -c "import cv2, ncnn, numpy, ultralytics; print('dependencies OK')"
```

## 카메라 입력 확인

시뮬레이터와 카메라 노드를 먼저 실행한 뒤 확인합니다.

```bash
ros2 topic info /camera/image_raw/compressed
ros2 topic hz /camera/image_raw/compressed
```

`ros2 topic hz`는 `Ctrl+C`로 종료합니다.

## 안전한 실행

처음 검증할 때는 실제 제어 노드가 구독하는 `/gosign` 대신 테스트 토픽으로
remap합니다.

```bash
ros2 run traffic_light traffic_light_node \
  --ros-args \
  -r /gosign:=/traffic_light_test/gosign
```

다른 터미널에서 결과를 확인합니다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/physicar/physicar_ws/install/setup.bash

ros2 topic echo \
  /traffic_light_test/gosign \
  std_msgs/msg/Bool
```

## 추론 결과 시각화

시각화는 기본적으로 꺼져 있습니다. 켜면 신호등 박스, 클래스, confidence가
그려진 JPEG 이미지를 `/traffic_light/debug/compressed`로 발행합니다.

```bash
ros2 run traffic_light traffic_light_node \
  --ros-args \
  -p visualizer_enabled:=true \
  -r /gosign:=/traffic_light_test/gosign
```

Ubuntu 24.04와 ROS 2 Jazzy 환경에서는 다른 터미널에서 다음 명령으로 Image
View 플러그인을 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/physicar/physicar_ws/install/setup.bash

ros2 run rqt_image_view rqt_image_view
```

창의 토픽 목록에서 `/traffic_light/debug/compressed`를 선택합니다. 목록에
토픽이 없으면 다음 항목을 확인합니다.

- `traffic_light_node`가 계속 실행 중인지
- 실행 옵션에 `-p visualizer_enabled:=true`가 있는지
- `/camera/image_raw/compressed`에 이미지가 들어오는지
- 두 터미널의 ROS 환경과 `ROS_DOMAIN_ID`가 같은지

시각화에는 박스 그리기와 JPEG 인코딩 비용이 추가됩니다. 실제 Raspberry Pi 5
주행에서는 검증이 끝난 뒤 시각화를 끄는 것을 권장합니다.

## 실제 `/gosign` 발행

안전한 테스트가 끝난 뒤 remap 없이 실행하면 `control` 패키지가 구독하는
`/gosign`으로 결과가 발행됩니다.

```bash
ros2 run traffic_light traffic_light_node
```

이 명령은 차량을 움직이게 할 수 있으므로 바퀴를 띄우거나 주행 공간을 확보한
상태에서 실행합니다.

## 파라미터

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `model_path` | 설치된 NCNN 모델 경로 | 사용할 모델 디렉터리 |
| `confidence` | `0.5` | 검출 confidence 기준값 |
| `image_size` | `640` | YOLO 입력 이미지 크기 |
| `green_confirm_frames` | `3` | 주행 허용에 필요한 연속 초록 프레임 수 |
| `image_timeout_seconds` | `1.0` | 카메라 입력 중단 판정 시간 |
| `visualizer_enabled` | `false` | 디버그 이미지 발행 여부 |
