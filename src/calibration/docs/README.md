# Calibration Lane Map reference

이 디렉터리는 PhysiCar 시뮬레이터의 중앙선 로컬리제이션 기준 지도와 좌표 보정
근거를 보관한다. 오래된 SLAM 시각화와 중간 이미지는 제거했다.

## 문서와 이미지

- `mapping_reference.md`: bag, 카메라, 지도, TF, 경로 및 좌표변환 근거
- `mapping_reference.yaml`: 프로그램과 후속 분석에서 읽을 수 있는 구조화된 기준정보
- `sim_lane_map_world_color.png`: 중앙선 매칭에 사용하는 최종 metric 컬러 지도
- `Screenshot from 2026-08-21 16-38-50.png`: 최종 지도의 재생성용 원본 캡처
- `sim_lane_map_calibration.json`: homography, 축, 해상도와 정합 오차
- `sim_lane_map_cone_reference.json`: 캡처 당시 cone의 `sim_world` 좌표
- `sim_lane_map_calibration.md`: 생성 방법과 TF 사용 원칙
- `ekf_waypoint_error_over_time.png`: 전체 localization bag의 시간별 waypoint 경로 오차
- `ekf_waypoint_error_over_time.json`: RAW/EKF 오차 통계와 평가 표본 수
- `calibration_stage_analysis/ANALYSIS.md`: rosbag 기반 검출부터 EKF까지 단계별 진단 결과
- `calibration_stage_analysis/*.csv`: 프레임·매칭·pose 단위 원시 진단 데이터
- `calibration_stage_analysis/01_*.png` ~ `08_*.png`: 단계별 시각화
- `calibration_stage_analysis/initial_lateral_only_experiment/`: 초기 60초 yaw-off 분리 실험
- `fence_localization_analysis/strict_two_lap_full_20260823_152110/ANALYSIS.md`:
  고정 시작 pose와 네 직선 LiDAR 펜스를 사용한 전체 bag 성능 그래프
- `fence_localization_analysis/strict_two_lap_full_20260823_152110/05_rviz_replay_final.png`:
  주행 종료 뒤 실제 RViz에 남은 truth, `/odom`, `/odom/laser`, fence corrected 경로
- `fence_localization_analysis/strict_two_lap_full_20260823_152110/RVIZ_REPLAY.md`:
  RViz 재생 조건, 색상표와 종료 뒤 Path 포인트 수

`sim_lane_map_world_color.png`는 카메라 중앙선 매칭용이다. Nav2 또는 LiDAR SLAM의
occupancy map으로 사용하지 않는다.

## 원본 보존 원칙

- rosbag은 수정하거나 덮어쓰지 않는다.
- waypoint 계산은 이미지 픽셀이 아니라 원본 `/sim/api/route` 좌표를 사용한다.
- 최종 컬러 지도에서 HSV로 노란/주황 중앙선만 추출해 매칭한다.
- world, map, odom 좌표를 동일하다고 가정하지 않는다.

## EKF waypoint 경로 오차

`ekf_waypoint_error_over_time.png`는 669개 waypoint 폐곡선을 기준으로 동일 timestamp의
원본 `/odom`과 EKF `/odom/calibride` 위치에서 가장 가까운 경로 선분까지의 거리를
비교한다. waypoint는 명령 기준 경로이며 실제 차량 ground-truth pose는 아니므로,
이 결과는 절대 위치 정확도가 아니라 기준 경로에 대한 횡방향 정합 성능을 뜻한다.

전체 bag의 13,418쌍에서 RAW RMSE는 `0.134m`, EKF RMSE는 `0.124m`로 약 `7.3%`
감소했다. P95는 `0.310m → 0.269m`, 최대값은 `0.503m → 0.446m`로 감소했다.
다만 시작 구간에는 EKF 오차가 RAW보다 큰 구간이 있으므로 초기 공분산과 측정 잡음은
추가 튜닝 대상이다.

## RViz 보정 비교

```bash
ros2 launch calibration sim_localization_rviz.launch.py
```

전체 컬러 맵 위에 원본 odom(빨강), 보정 odom(청록), 시뮬 실제 위치(초록)와 각각의
주행 경로가 표시된다. 차량 라벨의 `m / deg` 값으로 보정 전후 오차를 직접 비교한다.
- 지도 위 경로는 저장된 변환 근거와 함께 사용한다.

## 최종 이미지 재생성

`src/calibration`에서 다음 명령을 실행한다.

```bash
python3 tools/rectify_sim_screenshot_map.py \
  'docs/Screenshot from 2026-08-21 16-38-50.png' \
  --cone-reference docs/sim_lane_map_cone_reference.json \
  --output-dir docs \
  --resolution-m 0.01
```

도구는 검증용 이미지도 생성하므로 재생성 후 최종 컬러 지도와 원본을 제외한 중간
이미지는 다시 정리한다.
