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
| `goal_yaw` | `std_msgs/Float64` | 入：冻结的集群参考航向 |
| `waypoints` | `geometry_msgs/PolygonStamped` | 入：航线（P1 leader FOLLOW_ROUTE） |
| `local_pose` | `geometry_msgs/PoseStamped` | 入：本机位姿 |
| `neighbor_intent` | `swarm_uav_interfaces/UavTrajectoryIntent` | 入：邻机轨迹 |
| `hold` | `std_msgs/Empty` | 入：HOLD |
| `direct_control_active` | `std_msgs/Bool` | 入：垂直轨迹/PI 直接控制仲裁 |

身份参数（`uav_id`/`exec_target`）位于公开命名空间 `<namespace>/`；规划调参位于私有 `<node_name>/`。

## 安全与验证

- C++ 节点不 arm、不切 OFFBOARD；旧 arm+OFFBOARD 自动 TAKEOFF 默认关闭。Python executor driver 在直接垂直/PI 控制期间通过仲裁暂停 EGO setpoint。
- `swarm_clearance=0.5` 表示每机安全半径：互惠检查使用两倍值，对应 1.0 m 水平中心距；原有 2:1 椭球对应 2.0 m 垂直中心距。规划结果在发布前执行硬 clearance 校验。
- 自动化：2026-08-13 全工作空间 133 tests 全绿（含本包 30 个 gtest 计数）。
- 单 Master 冒烟（2026-08-10）：`IDLE → EXECUTING → COMPLETED → HOLD` 生命周期 PASS，详见 `tcp_to_ros/docs/tcp_to_ros_测试记录.md`。

## 环境变量（ROS 日志重定向）

为避免 ROS 运行日志散落在 `~/.ros/log`，本工作空间统一将运行日志重定向到 `.ros_home/`：

```bash
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
mkdir -p "$ROS_LOG_DIR"
```

手动启动前请先在终端执行以上命令，确保当前工作空间所有 ROS 运行日志均写入 `.ros_home/log`。
