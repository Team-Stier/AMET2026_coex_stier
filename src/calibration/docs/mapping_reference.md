# PhysiCar 지도 및 캘리브레이션 기준정보

## 결론

현재 생성물은 세 역할로 분리한다.

| 생성물 | 역할 |
|---|---|
| `map.pgm`, `map.yaml`, pose graph | SLAM localization 및 Nav2 원본 |
| `/sim/api/route` waypoint | Path Planning과 RDDF 계산 원본 |
| 융합 및 경로 PNG | 사람이 확인하는 시각화와 캘리브레이션 디버그 |

컬러 PNG를 Nav2 occupancy map으로 사용하지 않는다. 프로그램은 PNG의 파란 픽셀을
다시 읽어 경로를 복원하지 않고 원본 waypoint와 좌표변환을 사용한다.

## 원본 데이터

- rosbag: `20260820_062525_conefree_auto_control_slam`
- bag 형식: MCAP, 약 143 MiB, 185.39초, 75,558 messages
- 점유지도: `20260820_062838_conefree_auto_control_slam/map.pgm`
- 저장 pose graph: `posegraph.data`, `posegraph.posegraph`
- 경로 API: `http://localhost/sim/api/route`
- simulator world: `custom_e09090b056ef1f90f845419690065271`
- world bounds: `x=0–12 m`, `y=0–7 m`

## 지도 계약

```yaml
image: map.pgm
mode: trinary
resolution: 0.050
origin: [-3.878, -1.081, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

지도 크기는 `237×279 px`이며 총 66,123 cell 중 47,845 cell을 관측했다. coverage는
약 `72.36%`다. 점유지도에는 외곽 벽이 주로 나타나며 바닥 색상과 차선은 LiDAR
occupancy obstacle이 아니므로 직접 표시되지 않는다.

## 카메라와 융합 기준

- 토픽: `/camera/image_raw/compressed`
- 해상도: `480×360`
- 수평 FOV: `1.7453 rad`
- OpenCV distortion `[k1,k2,p1,p2,k3]`:
  `[-0.045, -0.0001, -0.0003, -0.0001, 0.001]`
- ground projection: `0.15–3.0 m`
- 처리 간격: 매 5번째 frame, pixel stride 3
- 처리 frame: 536 / bag camera messages 2,677
- 투영 ground pixels: 4,728,519
- TF nearest-sample 최대 age: 160 ms

사용한 TF chain:

```text
map → odom → base_footprint → base_link
    → camera_pan_link → camera_tilt_link
    → camera_link → camera_optical_frame
```

## 저장 지도와 경로

저장 직전 bag에서 복구한 최종 `map → odom`은 다음과 같다.

```text
translation = (-0.632, 0.147, 0.0) m
yaw = -0.395 rad = -22.638°
```

경로는 669개 waypoint로 구성된 약 `32.259 m` 폐곡선이며 반시계 방향이다. 평균
간격은 약 `0.0483 m`, 최대 간격은 약 `0.0499 m`다. 최소 경계 여유는 약
`0.364 m`다.

경로 생성 metadata의 평균 centerline deviation은 약 `0.0238 m`, 최대값은 약
`0.2581 m`다. 따라서 RDDF를 실제로 칠해진 Lane Map과 완전히 동일하다고 간주하지
않는다.

## 경로 오버레이

world bounds와 SLAM 외곽 벽을 정합해 만든 초기 시각화 변환은 다음과 같다.

```text
world → map translation = (2.372, -1.081) m
world → map yaw = 1.088459 rad = 62.364°
raster fit scale = 0.997088
```

이 변환으로 669개 경로점 모두가 `237×279 px` 이미지 내부에 들어간다. raster scale은
벽 두께와 픽셀 반올림을 포함한 시각화 값이다. 물리 좌표계의 강체변환을 정의할 때는
scale을 1로 고정하고 translation과 yaw를 사용한다.

## Calibration 사용 원칙

권장 localization 보정 흐름은 다음과 같다.

```text
SLAM map pose
  + map 좌표 Lane Map
  + 카메라에서 검출한 중앙차선
  → 횡방향 오차와 yaw 오차
  → /odom/calibride
```

RDDF는 주행경로이고 Lane Map은 실제로 보이는 차선 기준이다. 두 데이터를 분리한다.
카메라 차선만으로 관측하기 어려운 전후 위치는 SLAM에 맡기고, 횡방향과 yaw를 차선으로
미세보정한다. confidence가 낮거나 timestamp에 맞는 TF가 없으면 SLAM pose를 그대로
사용한다.
