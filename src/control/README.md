# Control module

`control`은 ROS와 독립적인 Pure Pursuit/곡률 적응형 제어 코어와 PhysiCar SIM 전용
지도 수집 드라이버를 제공한다. 운영용 `control_node`의 `/path` 계약과 지도 수집용
SIM 드라이버는 분리한다.

## SIM 웨이포인트 지도 수집

검증된 폐곡선은 `config/sim_mapping_waypoints.json`에 저장되어 있다. 다음 드라이버는
SIM ground-truth pose를 읽어 경로를 따라가며 `/speed`, `/steering`, `/camera/pan`만
제어하고, 동일 pose를 `/mapping/sim_pose`로 발행한다. SLAM과 rosbag 기록은
`calibration` 도구로 별도 실행한다.

```bash
cd /home/physicar/physicar_ws/AMET2026_coex_stier
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0

python3 src/control/tools/sim_waypoint_mapping_drive.py \
  --laps 2 \
  --target-speed-m-s 0.55 \
  --maximum-path-error-m 0.20
```

드라이버는 시작 위치와 진행 방향, 차량 명령 subscriber, `/odom` freshness를 먼저
검사한다. 주행 중 경로 오차가 한계를 넘거나 pose/odometry가 끊기면 즉시 중단하며,
정상 종료와 예외 종료 모두 속도와 조향 0 명령을 반복 발행한다. 이 도구는 metadata의
simulator world가 일치할 때만 사용하며 실차에는 사용하지 않는다.

## 제어 코어

`ControllerCore`는 Pure Pursuit 조향, 곡률 기반 lookahead 및 속도 제한, 선택적 종방향
PID를 하나의 ROS-independent 인터페이스로 묶는다. 현재 지도 수집 설정은 카메라를
정면에 고정하고 곡선에서 약 0.50 m/s로 감속한다.
