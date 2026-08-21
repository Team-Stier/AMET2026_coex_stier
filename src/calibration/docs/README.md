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

`sim_lane_map_world_color.png`는 카메라 중앙선 매칭용이다. Nav2 또는 LiDAR SLAM의
occupancy map으로 사용하지 않는다.

## 원본 보존 원칙

- rosbag은 수정하거나 덮어쓰지 않는다.
- waypoint 계산은 이미지 픽셀이 아니라 원본 `/sim/api/route` 좌표를 사용한다.
- 최종 컬러 지도에서 HSV로 노란/주황 중앙선만 추출해 매칭한다.
- world, map, odom 좌표를 동일하다고 가정하지 않는다.

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
