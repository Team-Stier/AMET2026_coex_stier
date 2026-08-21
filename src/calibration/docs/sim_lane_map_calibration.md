# 시뮬 캡처 Lane Map 보정 기준

## 결론

- 기준 좌표계: `sim_world`
- 월드 범위: `x=0–12 m`, `y=0–7 m`
- 최종 raster: `1200×700 px`
- 해상도: `0.01 m/px` (100 px/m)
- raster 방향: 오른쪽 `+x`, 위쪽 `+y`
- TF scale: `1.0`

ROS TF의 translation은 이미 SI metre 단위이므로 TF 값에 영상 scale을 곱하지 않는다.
캡처의 원근 왜곡만 homography로 제거하고 metric raster로 다시 샘플링한다.

## 확인한 TF

실행 중인 시뮬레이터에서 다음 체인을 확인했다.

```text
odom → base_footprint
     → base_link
     → camera_pan_link
     → camera_tilt_link
     → camera_link
     → camera_optical_frame
```

`base_footprint → base_link`의 z translation은 `0.0375 m`이며,
`camera_link → camera_optical_frame`의 optical-axis quaternion도 정상적으로 발행된다.
현재 SLAM이 실행 중이지 않아 `map → odom`은 없었다. 이 Lane Map을 `map` frame으로
사용하려면 `map=sim_world`로 두고, `map → odom`은 같은 timestamp의 차량 world pose와
`odom → base_footprint`로 구한 rigid transform을 사용해야 한다.

## 캡처 정합

원본 `Screenshot from 2026-08-21 16-38-50.png`는 `616×1020 px`이었다. 캡처 당시
6개 cone의 live `sim_world` 위치와 영상에서 검출한 cone 접지점을 사용해 projective
homography를 구했다. 최종 생성 시 재투영 RMSE는 `0.561 px`, 최대 오차는
`0.886 px`였다.

캡처 후 시뮬레이터가 movable cone을 초기 위치로 reset했기 때문에 당시 위치는
`sim_lane_map_cone_reference.json`에 별도로 보존했다. 이 캡처를 다시 처리할 때는 반드시
해당 파일을 사용한다.

## 생성 명령

원본 캡처를 `docs`에 다시 넣은 뒤 `src/calibration`에서 실행한다.

```bash
python3 tools/rectify_sim_screenshot_map.py \
  'docs/Screenshot from 2026-08-21 16-38-50.png' \
  --cone-reference docs/sim_lane_map_cone_reference.json \
  --output-dir docs \
  --resolution-m 0.01
```

도구의 전체 생성물:

- `sim_lane_map_world_color.png`: 노란색/흰색을 보존한 metric image
- `sim_lane_map_world_binary.png`: 두 선을 모두 흰색으로 만든 binary image
- `sim_lane_map.pgm`, `sim_lane_map.yaml`: map_server 형식 (`negate: 1`)
- `sim_lane_map_calibration_debug.png`: cone correspondence 검증 이미지
- `sim_lane_map_calibration.json`: homography와 오차 metadata

현재 중앙선 로컬리제이션 입력은 `sim_lane_map_world_color.png` 하나로 확정했다.
`sim_lane_map_world_binary.png`, PGM/YAML, 원근 보정 전 중간본과 디버그 이미지는
검증 완료 후 제거했다. 재생성이 필요할 때 위 명령으로 다시 만들 수 있다.

이 결과는 차선 기준 visual map이다. LiDAR obstacle을 표현하는 SLAM occupancy map과는
역할이 다르므로 기존 LiDAR `map.pgm`을 대체한다고 가정하면 안 된다.
