# Calibration package

2D LiDAR로 맵 외곽의 네 펜스를 관측해 `/odom`의 평면 위치와 yaw를 보정한다.
카메라, 중앙 노란선, 차선 지도는 사용하지 않는다.

## 먼저 보는 핵심

```text
/odom + /scan_filtered + base_footprint <- lidar_link TF
  -> 네 펜스 선분과 LiDAR point-to-line 정합
  -> EKF와 출력 변화율 제한
  -> /odom/calibride
```

| 구분 | 현재 계약 |
|---|---|
| 입력 | `/odom` (`nav_msgs/msg/Odometry`) |
| 입력 | `/scan_filtered` (`sensor_msgs/msg/LaserScan`) |
| 출력 | `/odom/calibride` (`nav_msgs/msg/Odometry`) |
| 기준 펜스 | `map` 기준 `x=0`, `x=12`, `y=0`, `y=7`인 `12 m x 7 m` 사각형 |
| 고정 시작 pose | `map (x=1.4 m, y=3.4 m, yaw=-pi/2 rad)` |
| 보정 채택 조건 | 최소 80점, 3개 펜스, 펜스별 10점, RMS 0.08m 이하 |
| 비교 전용 | `/odom/laser`, `/sim/ground_truth/tf` |

`/scan_filtered`가 없거나 정합 품질이 기준에 미달하면 새 펜스 측정을 EKF에 넣지
않는다. 첫 유효 보정 전에는 `/odom/calibride`가 `/odom`을 그대로 따르고, 보정 후에는
마지막 보정 상태를 유지하면서 `/odom`의 이동량으로 계속 예측한다. simulator truth는
보정 입력으로 사용하지 않는다.

기본값으로 실행:

```bash
ros2 run calibration calibration_node
```

YAML 설정으로 실행:

```bash
ros2 run calibration calibration_node --ros-args \
  --params-file install/calibration/share/calibration/config/fence_localization.yaml
```

저장소 루트의 `run.sh`는 현재 `ros2 run calibration calibration_node`를 실행하므로
코드 기본값을 사용한다. `config/fence_localization.yaml`을 수정했다면 위처럼
`--params-file`을 명시하거나 `run.sh`의 Calibration 실행 줄을 함께 연결해야 한다.

## 상세 동작

### 좌표계와 TF

- 차량 좌표는 진행 방향 `+x`, 왼쪽 `+y`다.
- scan의 `header.frame_id`가 `base_footprint`가 아니면 scan timestamp의
  `base_footprint <- scan_frame` TF로 LiDAR 점을 변환한다.
- 첫 `/odom` pose와 고정 시작 pose로 static `map -> odom`을 한 번 계산해 발행한다.
- 기준 펜스는 이 변환을 통해 `odom` frame으로 옮긴 뒤 정합한다.
- `/odom/calibride`의 `header.frame_id`와 `child_frame_id`는 입력 `/odom`을 유지한다.

다른 SLAM 또는 localization 노드가 `map -> odom`을 이미 발행한다면 TF authority가
충돌하지 않도록 한 노드만 해당 TF를 소유해야 한다. 시작 위치나 방향이 달라지는
환경에서는 고정 시작 pose를 실제 값으로 먼저 수정해야 한다.

### LiDAR 전처리

`calibration_node`는 `/scan_filtered`를 sensor-data QoS로 구독한다. 각 scan에서는
다음 순서로 점을 만든다.

1. `scan_stride=2` 간격으로 range를 선택한다.
2. NaN, Inf와 `range_min <= range < range_max`를 벗어난 값을 제거한다.
3. 각도와 거리로 2D Cartesian 점을 계산한다.
4. 필요한 경우 timestamp가 일치하는 TF로 `base_footprint` 좌표로 변환한다.
5. 최소 점 개수보다 적거나 TF 변환에 실패한 scan은 사용하지 않는다.

### 네 펜스 정합

기준 펜스는 무한 직선이 아니라 끝점이 있는 네 선분이다. 현재 pose로 LiDAR 점을
`odom` frame에 놓고, 각 점을 가장 가까운 유한 선분에 대응시켜 x, y, yaw를 반복
최소제곱으로 보정한다.

- 선분 끝점 바깥 `0.10 m`까지만 대응을 허용한다.
- 펜스까지 수직 거리가 `0.25 m`를 넘는 점은 제외한다.
- 라바콘처럼 펜스에서 떨어진 내부 물체는 이 거리 게이트에서 제외된다.
- Huber 가중치로 큰 잔차의 영향을 줄이고 펜스별 점 개수를 균형 있게 반영한다.
- 최소 80점, 최소 3개 펜스, 각 펜스 최소 10점이 있어야 pose를 관측 가능하다고 본다.
- 한 번의 정합 결과가 위치 `0.35 m`, yaw `0.15 rad`를 넘으면 폐기한다.
- 최종 point-to-line RMS가 `0.08 m`를 넘으면 EKF 측정으로 채택하지 않는다.

현재 로직은 펜스와 `0.25 m` 안쪽에 붙은 장애물까지 완벽하게 분류하는 semantic
필터가 아니다. 이런 물체가 많으면 대응점과 RMS를 RViz 및 rosbag으로 확인해야 한다.

### EKF와 출력

EKF 상태는 `[x, y, yaw]`다.

- 예측: 연속된 `/odom` pose의 차량 기준 이동량과 pose/twist covariance를 사용한다.
- 보정: 펜스 정합으로 얻은 absolute pose와 RMS, match 수를 측정 잡음에 반영한다.
- 출력 제한: 위치는 기본 `0.08 m/s`, yaw는 `0.08 rad/s` 이내로 보정값을 적용한다.
- covariance: x, y, yaw posterior covariance와 아직 출력에 반영되지 않은 보정 지연을
  `/odom/calibride.pose.covariance`에 기록한다.
- 시간이 뒤로 이동하면 rosbag 반복 재생으로 판단해 EKF, scan 처리 상태와 고정 TF
  초기화 상태를 리셋한다.

### 주요 파라미터

설정 파일은 `config/fence_localization.yaml`이다.

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `maximum_scan_age_sec` | `0.15` | odom 대비 사용할 수 있는 scan 최대 지연 |
| `fence_maximum_match_distance_m` | `0.25` | 점과 펜스의 최대 대응 거리 |
| `fence_minimum_matches` | `80` | 전체 최소 대응점 수 |
| `fence_minimum_segments` | `3` | 관측되어야 하는 최소 펜스 수 |
| `fence_minimum_matches_per_segment` | `10` | 각 펜스의 최소 대응점 수 |
| `fence_maximum_position_correction_m` | `0.35` | scan 한 장의 최대 위치 보정량 |
| `fence_maximum_yaw_correction_rad` | `0.15` | scan 한 장의 최대 yaw 보정량 |
| `fence_maximum_rms_error_m` | `0.08` | 채택 가능한 최대 정합 RMS |
| `maximum_correction_position_rate_m_s` | `0.08` | 출력 위치 보정 변화율 제한 |
| `maximum_correction_yaw_rate_rad_s` | `0.08` | 출력 yaw 보정 변화율 제한 |

실제 펜스 치수나 시작 pose가 달라지면 다음 값을 함께 갱신한다.

```yaml
fence_minimum_x_m: 0.0
fence_maximum_x_m: 12.0
fence_minimum_y_m: 0.0
fence_maximum_y_m: 7.0
fixed_start_map_x_m: 1.4
fixed_start_map_y_m: 3.4
fixed_start_map_yaw_rad: -1.5707963267948966
```

## RViz에서 비교

다음 launch는 Calibration Node, 비교용 bridge와 RViz를 함께 실행한다.

```bash
ros2 launch calibration sim_localization_rviz.launch.py
```

| 색상 | 데이터 |
|---|---|
| 회색 | `12 m x 7 m` 기준 펜스 |
| 빨강 | `/odom` |
| 주황 | `/odom/laser` |
| 청록 | `/odom/calibride` |
| 초록 | simulator truth |

누적 Path는 1초마다 다시 발행되므로 rosbag 재생이 끝난 뒤에도 화면에 남는다. truth와
보정 결과가 거의 같으면 청록과 초록 경로가 겹쳐 보일 수 있다.

현재 RViz bridge는 펜스와 고정 시작 pose의 기본값을 코드에 별도로 가지고 있다.
`fence_localization.yaml`의 치수를 변경했다면
`calibration_fence_rviz_bridge`에도 같은 파라미터를 전달해야 화면과 실제 정합 기준이
일치한다.

## rosbag 분석

`tools/analyze_fence_localization.py`는 MCAP 전체를 순차 처리해 위치/yaw 오차 CSV,
요약 JSON과 그래프를 만든다.

```bash
python3 src/calibration/tools/analyze_fence_localization.py \
  <bag_directory> <output_directory>
```

필요 토픽과 기록 방법은 `docs/CALIBRATION_DIAGNOSTIC_BAG.md`에 정리되어 있다.
검증된 예시는
`docs/fence_localization_analysis/strict_two_lap_full_20260823_152110/`에 있다.

분석 도구의 현재 기준 펜스와 시작 pose도 `12 m x 7 m`, `(1.4, 3.4, -pi/2)`로
고정되어 있다. 설정을 바꾼 실험에서는 분석 도구 값도 동일하게 맞춘 뒤 결과를
비교해야 한다.

## 현재 검증 범위와 제한

- 합성 단위 테스트로 네 펜스 pose 복원, 내부 장애물 제외, 관측 불충분 거부와 EKF
  동작을 확인했다.
- ROS 2 Jazzy Docker에서 패키지 build/test와 저장된 simulator MCAP 재생을 확인했다.
- 기록된 2회전 예제에서는 simulator truth를 입력으로 사용하지 않고 성능 비교에만
  사용했다.
- `12 m x 7 m` 펜스와 고정 시작 pose는 현재 simulator 실험 조건이다. 실제 설치
  오차와 실차 LiDAR extrinsic은 별도로 측정해야 한다.
- `/scan_filtered` publisher와 `base_footprint <- lidar_link` TF는 외부 센서/bringup이
  제공해야 하며, 이 패키지가 생성하지 않는다.
