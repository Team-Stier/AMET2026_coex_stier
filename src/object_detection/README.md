# Object detection

`/scan`의 2D LiDAR 점을 전방 관심 영역으로 제한한 뒤 DBSCAN으로 클러스터링한다.
크기가 지나치게 큰 클러스터는 벽으로 간주해 제거하고, 가까운 클러스터부터 최대 20개의
클러스터에 모든 점을 포함하는 최소 크기의 원을 계산한다. 최소 포함 원의 중심점은
`/object_info`로 발행한다. 반지름은 RViz 시각화에만 사용하며, 반지름이나 피팅 오차를
이용한 추가 장애물 판정은 하지 않는다.

## 실행

```bash
colcon build --packages-select interfaces object_detection
source install/setup.bash
ros2 run object_detection object_detection_node
```

주요 기본 파라미터는 다음과 같다.

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `minimum_range_m` | `0.15` | 사용할 최소 LiDAR 거리 |
| `maximum_range_m` | `4.0` | 사용할 최대 LiDAR 거리 |
| `minimum_forward_x_m` | `0.0` | 전방 관심 영역의 최소 x |
| `maximum_absolute_y_m` | `1.5` | 좌우 관심 영역 `|y|` 한계 |
| `dbscan_epsilon_m` | `0.17` | DBSCAN 이웃 거리 |
| `dbscan_minimum_samples` | `3` | DBSCAN core point 최소 이웃 수 |
| `minimum_cluster_points` | `3` | 최종 클러스터가 가져야 할 최소 점 개수 |
| `maximum_cluster_extent_m` | `0.8` | 이보다 큰 클러스터 제거 |
| `maximum_objects` | `20` | 메시지에 넣을 최대 장애물 수 |
| `publish_markers` | `true` | RViz MarkerArray 발행 여부 |
| `marker_topic` | `/object_detection/markers` | 클러스터·중심·ROI 마커 토픽 |
| `fitted_circle_marker_topic` | `/object_detection/fitted_circles` | 최소 포함 원 마커 토픽 |
| `marker_roi_line_width_m` | `0.01` | RViz ROI 경계선 두께 |
| `marker_circle_line_width_m` | `0.02` | RViz 피팅 원 선 두께 |

예를 들어 시각화를 끄고 실행하려면 다음 명령을 사용한다.

```bash
ros2 run object_detection object_detection_node --ros-args \
  -p publish_markers:=false
```

## RViz 확인

노드는 기존 클러스터 시각화와 피팅 원 시각화를 서로 다른
`visualization_msgs/msg/MarkerArray` 토픽으로 발행한다.

- `/object_detection/markers`: 클러스터 점, 최소 포함 원 중심, 청록색 ROI 경계
- `/object_detection/fitted_circles`: 클러스터마다 다른 색의 최소 포함 원

1. RViz의 Fixed Frame을 `lidar_link`로 설정한다.
2. `Add`에서 `MarkerArray`를 두 개 추가한다.
3. 각각의 Topic을 `/object_detection/markers`,
   `/object_detection/fitted_circles`로 설정한다.

결과 메시지는 다음 명령으로 함께 확인할 수 있다.

```bash
ros2 topic echo /object_info
```

RViz 시각화는 개발과 파라미터 조정용이다. Raspberry Pi 5 최종 주행에서는 처리량을
측정한 뒤 필요하지 않으면 `publish_markers:=false`로 비활성화한다.
