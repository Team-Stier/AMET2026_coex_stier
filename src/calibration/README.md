# Calibration package

이 패키지는 2D LiDAR로 외곽 펜스를 관측해 `/odom`의 평면 위치와 yaw를 보정한다.
카메라, 중앙 노란선, 차선 지도는 사용하지 않는다.

## 입출력

```text
/odom + /scan_filtered + base_footprint <- lidar_link TF
  -> 유효 scan을 base_footprint 평면 점으로 변환
  -> map의 네 유한 펜스 선분과 point-to-line 정합
  -> x/y/yaw fence 측정 생성
  -> EKF 측정 갱신과 출력 rate limit
  -> /odom/calibride
```

| 토픽 | 타입 | 역할 |
|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | 보정 전 fused odometry |
| `/scan_filtered` | `sensor_msgs/msg/LaserScan` | 외곽 펜스 관측 |
| `/odom/calibride` | `nav_msgs/msg/Odometry` | 펜스 보정 결과 |

라바콘처럼 펜스에서 떨어진 내부 물체는 대응점에서 제외한다. 기본값은 네 선분 중
최소 3개, 전체 80점, 선분별 10점 이상이 연결될 때만 측정을 채택한다. 한 번의
정합에서 위치 `0.35 m`, yaw `0.15 rad`, RMS `0.08 m`를 넘으면 보정하지 않는다.

## 고정 시작 pose

매 주행은 다음 `map` pose에서 시작한다고 가정한다.

```text
x = 1.4 m
y = 3.4 m
yaw = -pi/2 rad
```

첫 `/odom` pose와 고정 시작 pose로 `map -> odom`을 계산해 static TF로 발행한다.
시작 위치나 방향이 바뀌면 `config/fence_localization.yaml`의 값을 먼저 수정해야 한다.
simulator truth는 보정 입력이 아니며 성능 평가와 RViz 비교에만 사용한다.

## 기준 펜스

기본 펜스는 `map` 기준 `x=0`, `x=12`, `y=0`, `y=7`인 `12 m x 7 m` 사각형이다.
실제 설치 치수가 달라지면 네 경계를 설정에서 바꿔야 한다.

## 실행

기본 파라미터로 실행:

```bash
ros2 run calibration calibration_node
```

설정 파일을 명시해 실행:

```bash
ros2 run calibration calibration_node --ros-args \
  --params-file install/calibration/share/calibration/config/fence_localization.yaml
```

RViz 비교:

```bash
ros2 launch calibration sim_localization_rviz.launch.py
```

RViz 색상은 회색 펜스, 빨강 `/odom`, 주황 `/odom/laser`, 청록
`/odom/calibride`, 초록 simulator truth다. 누적 Path는 1초마다 다시 발행되므로
rosbag 재생이 끝난 뒤에도 화면에 남는다.

## 분석

`tools/analyze_fence_localization.py`는 MCAP 전체를 순차 처리해 위치/yaw 오차 CSV,
요약 JSON과 그래프를 만든다. 검증된 예시는
`docs/fence_localization_analysis/strict_two_lap_full_20260823_152110/`에 있다.
