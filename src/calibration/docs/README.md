# Calibration mapping reference

이 디렉터리는 PhysiCar 시뮬레이터에서 생성한 지도와 캘리브레이션 기준정보를 보관한다.
Nav2가 읽는 원본 `map.pgm`과 `map.yaml`을 수정하지 않고, 사람이 확인하는 융합 지도와
경로 오버레이를 문서 자산으로 별도 관리한다.

## 문서와 이미지

- `mapping_reference.md`: bag, 카메라, 지도, TF, 경로 및 좌표변환 근거
- `mapping_reference.yaml`: 프로그램과 후속 분석에서 읽을 수 있는 구조화된 기준정보
- `map.png`: LiDAR 지도에 카메라 지면 투영을 합성한 참조 이미지
- `map_with_route.png`: 점유지도 위에 RDDF를 표시한 검증 이미지

이미지는 시각화 및 검증용이다. Nav2 또는 SLAM 입력은 시뮬레이터에서 생성한 원본
`map.pgm`과 `map.yaml`을 계속 사용한다. 색상이 포함된 PNG를 occupancy map 대신
사용하지 않는다.

## 원본 보존 원칙

- rosbag은 수정하거나 덮어쓰지 않는다.
- waypoint 계산은 이미지 픽셀이 아니라 원본 `/sim/api/route` 좌표를 사용한다.
- `map.png`는 navigation map이 아니라 관찰용 reference layer다.
- world, map, odom 좌표를 동일하다고 가정하지 않는다.
- 지도 위 경로는 저장된 변환 근거와 함께 사용한다.

## 이미지 갱신

시뮬레이터에서 생성한 원본 융합 PNG를 검증한 뒤 `map.png`로 복사한다.

```bash
./src/calibration/tools/update_docs_map.sh
```

다른 원본 파일을 사용할 때는 첫 번째 인자로 명시한다.

```bash
./src/calibration/tools/update_docs_map.sh /absolute/path/to/source.png
```

스크립트는 원본이 존재하고 비어 있지 않은 PNG인지 확인한 후 임시 파일을 거쳐
`docs/map.png`를 교체한다. 검증에 실패하면 기존 정상 이미지를 변경하지 않는다.
