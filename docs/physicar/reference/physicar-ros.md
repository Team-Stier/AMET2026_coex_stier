# PhysiCar ROS 참고

- 원문: <https://physicar.ai/ko/learn/reference/physicar-ros/>
- 확인일: 2026-08-19
- 배포판: ROS 2 Jazzy

PhysiCar는 시뮬레이터와 실물 키트에서 같은 `physicar-ros` 인터페이스를 사용한다.

## 센서 토픽

| 토픽 | 타입 | 계약 |
|---|---|---|
| `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | JPEG 카메라 영상 |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | 1 Hz, `percentage`는 0~1 |
| `/imu` | `sensor_msgs/msg/Imu` | 50 Hz |
| `/odom` | `nav_msgs/msg/Odometry` | LiDAR와 IMU 융합 odometry |
| `/scan` | `sensor_msgs/msg/LaserScan` | 원시 LiDAR scan |
| `/scan_filtered` | `sensor_msgs/msg/LaserScan` | 필터링된 LiDAR scan |

`/scan`과 `/scan_filtered`는 best-effort 센서 QoS로 발행되므로 구독자는
`qos_profile_sensor_data`를 사용해야 한다.

## 제어 토픽

| 토픽 | 타입 | 계약 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `linear.x` 속도(m/s), `angular.z` 회전 속도(rad/s) |
| `/speed` | `std_msgs/msg/Float64` | 속도(m/s) |
| `/steering` | `std_msgs/msg/Float64` | 조향각(rad), 양수는 좌회전, 최대 약 ±0.35 rad |
| `/camera/pan` | `std_msgs/msg/Float64` | 팬 명령(rad), 양수는 왼쪽, 최대 약 ±0.52 rad |
| `/camera/tilt` | `std_msgs/msg/Float64` | 틸트 명령(rad), 양수는 위, 최대 약 ±0.52 rad |

속도 명령은 약 1초 동안 갱신되지 않으면 안전 워치독에 의해 만료된다. 지속 주행에는
주기적인 발행이 필요하다. 드라이버가 퍼블리셔를 발견하기 전에 보낸 초기 메시지는
유실될 수 있으므로 구독자 연결을 확인한 뒤 발행한다.

공식 ROS 토픽 표에는 카메라 팬의 실제 위치를 발행하는 별도 상태 토픽이 없다.
`/camera/pan`은 목표 각도 명령 계약이며 실제 서보 피드백으로 간주하면 안 된다.

## 기타 로컬 인터페이스

로봇 스택은 `http://localhost`의 Web API, `/physicar-ext/`의 로컬 tool server,
포트 5000의 MyApp 연결도 제공한다. 시뮬레이터 전용 월드 제어는
[PhysiCar Sim](physicar-sim.md)의 `/sim/api/` 계약을 사용한다.

