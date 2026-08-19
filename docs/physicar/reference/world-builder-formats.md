# World Builder 파일 포맷 참고

- 원문: <https://physicar.ai/ko/learn/reference/world-builder-formats/>
- 확인일: 2026-08-19

World Builder의 Publish는 월드 번들을 CDN에 올리고 32자리 16진수 월드 ID를 만든다.
Print는 실물 트랙용 고해상도 탑다운 JPEG를 생성한다. 게시된 월드는 불변이며 새로
Publish하면 새 ID가 만들어진다.

## 평가 번들

평가를 활성화한 월드는 다음 두 파일을 포함한다.

- `evaluations/custom_<월드ID>.json`: 버전, 설명, 점수 방향, 제한 시간, 로봇,
  학생 코드 실행 명령
- `evaluations/custom_<월드ID>.js`: `initialize`와 `evaluate`를 구현하는 채점 코드

채점 코드는 상태를 읽고 물체·신호등·오버레이를 조작한 뒤 `sim.result()`와
`sim.finish()`로 결과를 확정한다. 제한 시간과 로봇 세대가 평가 설정과 맞아야 한다.

## 오브젝트 계약

배치한 오브젝트는 SDF 인라인 모델로 내보낸다. 모델 이름은 중복 없이
`A-Za-z0-9_`를 사용하고, 도구는 모델명보다 link 마커로 종류를 판별한다.

| 종류 | 주요 link 마커 |
|---|---|
| Box, Cylinder, Sphere, Traffic Cone | `object` |
| Traffic Light | `light` |
| 둘레 펜스 | `wall` |

오브젝트는 DeepRacer 충돌 판정과 보상 함수의 `objects` 목록에 참여한다. 바닥 데칼은
별도 모델이 아니라 트랙 시각 mesh에 레이어 순서대로 bake된다.

## Waypoints

Waypoints 파일은 `(N, 6)` NumPy 배열이다.

```text
[center_x, center_y, inner_x, inner_y, outer_x, outer_y]
```

보상 함수의 `waypoints` 입력은 첫 두 열인 중앙선 좌표를 사용한다.
