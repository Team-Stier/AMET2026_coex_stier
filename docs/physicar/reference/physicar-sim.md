# PhysiCar Sim 참고

- 원문: <https://physicar.ai/ko/learn/reference/physicar-sim/>
- 확인일: 2026-08-19

PhysiCar Sim은 Gazebo Harmonic 기반 환경이다. 로봇 자체는 실물과 같은 ROS 스택을
사용하지만 `/sim/api/`는 월드와 평가를 제어하기 위한 시뮬레이터 전용 HTTP API이며
실물 키트에는 존재하지 않는다.

기본 URL은 `http://localhost/sim/api`이다.

## 주요 API 그룹

- 상태·시간: `GET /status`, `GET /clock`
- 차량 포즈: `GET|POST /pose`
- 월드: `GET /world`, `/route`, `/bounds`, `/objects`
- 물체 이동: `POST /models/<name>/pose`
- 신호등: `GET /traffic_lights`, `POST /traffic_lights/<name>`
- 빠른 초기화: `POST /reset`
- 월드와 odometry 스택 재시작: `POST /respawn`
- 화면: `GET|POST /brightness`, `GET|POST /overlay`
- 모니터링: `GET /state`, `GET /events`(SSE)
- 평가: `GET /evaluation`, `POST /evaluation/run`, `POST /evaluation/stop`
- 월드 관리: `GET /worlds`, `POST /switch`, `GET /worldpub`,
  `POST /worlds/install`

차량을 `/pose`로 순간 이동하면 LiDAR+IMU odometry가 이동을 인지하지 못해 `/odom`에
오프셋이 남을 수 있다. 깨끗한 odometry가 필요하면 더 무겁지만 관련 스택까지
재시작하는 `/respawn`을 사용한다. 일반 학습 에피소드 사이에는 월드를 다시 읽지 않는
`/reset`이 기본 선택이다.

신호등의 green→red 전환은 3초간 yellow 상태를 거치며 그동안 추가 명령은 거부될 수
있다. 실시간 변경 감시는 `/events`의 이름 있는 SSE 이벤트를 사용한다.

