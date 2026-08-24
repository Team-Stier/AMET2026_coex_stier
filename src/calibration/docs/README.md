# LiDAR fence localization evidence

이 디렉터리는 네 직선 외곽 펜스를 사용한 위치 보정의 정량 결과와 RViz 증거를
보관한다. 중앙선·카메라 기반 보정 자료는 제거했으며 삭제 전 상태는 Git commit
`47eb46c`에서 복구할 수 있다.

## 성능 분석

- `fence_localization_analysis/strict_two_lap_full_20260823_152110/ANALYSIS.md`:
  고정 시작 pose와 `12 m x 7 m` 펜스를 사용한 전체 bag 성능표
- `01_trajectory_comparison.png`: truth, `/odom`, `/odom/laser`, fence corrected 궤적
- `02_position_error_over_time.png`: 시간별 위치 오차
- `03_yaw_error_over_time.png`: 시간별 yaw 오차
- `04_rviz_style_final_paths.png`: RViz 색상과 동일한 정적 비교 그림
- `05_rviz_replay_final.png`: 실제 ROS 2 Jazzy RViz에서 재생 종료 후 캡처한 화면
- `RVIZ_REPLAY.md`: 재생 환경, 색상표와 종료 뒤 Path 포인트 수
- `summary.json`, `pose_errors.csv`, `laser_pose_errors.csv`: 수치 원본

## 해석 주의사항

- simulator truth는 성능 계산에만 사용했으며 fence matcher 입력에는 사용하지 않았다.
- 고정 시작 pose는 `map (1.4 m, 3.4 m, -pi/2 rad)`이다.
- 현재 결과는 동일 시작 위치와 동일 펜스 치수 조건에 한정된다.
- 다른 코스나 실제 설치에서는 펜스 좌표와 시작 pose를 다시 측정해야 한다.
