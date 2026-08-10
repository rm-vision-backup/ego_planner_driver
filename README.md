# ego_planner_driver

机载 ego-swarm 风格带时轨迹生成与互惠机间避碰驱动。

- 抽取 ego-planner-swarm 内核的 bspline 优化与轨迹生成，纯数值无 ROS 依赖（`EgoPlannerCore`）。
- 保留 jerk 平滑、动态可行性、reciprocal swarm 避碰三项代价；排除感知/静态障碍外挂。
- 通过 ROS 节点订阅本机目标/邻机轨迹意图，发布 setpoint 与本机轨迹意图。
- 该包是独立 Git 仓库，不 push、不配置 remote。

## 接口与命名空间

节点在 `<namespace>/`（如 `/UAV1/`）公开以下接口，Python `EgoSwarmDriver` 与 topology bridge 直接使用：

| Topic | 类型 | 方向 |
|---|---|---|
| `exec_state` | `std_msgs/String`（latched） | 出：`IDLE/EXECUTING/COMPLETED/HOLD/POSE_STALE/EGO_PLAN_FAILED/EGO_EXEC_TIMEOUT` |
| `setpoint` | `geometry_msgs/PoseStamped` | 出：30 Hz OFFBOARD 兼容位置 setpoint |
| `trajectory_intent` | `swarm_uav_interfaces/UavTrajectoryIntent` | 出：本机带时轨迹 |
| `goal` | `geometry_msgs/PointStamped` | 入：单点目标 |
| `waypoints` | `geometry_msgs/PolygonStamped` | 入：航线（P1 leader FOLLOW_ROUTE） |
| `local_pose` | `geometry_msgs/PoseStamped` | 入：本机位姿 |
| `neighbor_intent` | `swarm_uav_interfaces/UavTrajectoryIntent` | 入：邻机轨迹 |
| `hold` | `std_msgs/Empty` | 入：HOLD |

身份参数（`uav_id`/`exec_target`）位于公开命名空间 `<namespace>/`；规划调参位于私有 `<node_name>/`。

## 安全与验证

- 默认不 arm、不切 OFFBOARD；HOLD 始终在最新 fresh local pose 处保持，无 fresh pose 时停止发布 setpoint（fail-safe）。
- 自动化：`catkin_test_results --all build` 全工作空间 74 tests 全绿（含本包 10 个 gtest）。
- 单 Master 冒烟（2026-08-10）：`IDLE → EXECUTING → COMPLETED → HOLD` 生命周期 PASS，详见 `tcp_to_ros/docs/tcp_to_ros_测试记录.md`。
