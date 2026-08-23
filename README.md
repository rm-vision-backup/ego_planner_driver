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
| `exec_state` | `std_msgs/String`（latched） | 出：`HOLD`（初始）/`TAKEOFF`/`EXECUTING`/`COMPLETED`/`POSE_STALE`/`EGO_PLAN_FAILED`/`EGO_EXEC_TIMEOUT`（`IDLE` 常量保留但不再作为初始态） |
| `setpoint/ego` | `mavros_msgs/PositionTarget` | 出：30 Hz 候选命令（`FRAME_LOCAL_NED`，业务字段 ROS ENU；`setpoint_relay` 独占转发 `/mavros/setpoint_raw/local`） |
| `trajectory_intent` | `swarm_uav_interfaces/UavTrajectoryIntent` | 出：本机带时轨迹 |
| `goal` | `geometry_msgs/PointStamped` | 入：单点目标 |
| `goal_yaw` | `std_msgs/Float64` | 入：冻结的集群参考航向 |
| `waypoints` | `geometry_msgs/PolygonStamped` | 入：航线（P1 leader FOLLOW_ROUTE） |
| `local_pose` | `geometry_msgs/PoseStamped` | 入：本机位姿 |
| `neighbor_intent` | `swarm_uav_interfaces/UavTrajectoryIntent` | 入：邻机轨迹 |
| `hold` | `std_msgs/Empty` | 入：HOLD |
| `direct_control_active` | `std_msgs/Bool` | 入：follower PI 直接控制仲裁（true 时本节点不发布候选） |

## PositionTarget 契约

所有候选统一为 `mavros_msgs/PositionTarget`：

| 模式 | `coordinate_frame` | 启用字段 | `type_mask` |
|---|---|---|---:|
| EGO 轨迹 | `FRAME_LOCAL_NED` | position、velocity、acceleration、yaw | `IGNORE_YAW_RATE` = **2048** |
| HOLD/TAKEOFF/steady/COMPLETED | `FRAME_LOCAL_NED` | position、yaw | `IGNORE_VX..Z\|IGNORE_AFX..Z\|IGNORE_YAW_RATE` = **2552** |

- P/V/A/yaw 来自同一轨迹同一采样时刻（`TimedTrajectory::samplePVA`），不跨帧差分；
- 业务字段按 ROS ENU 填写，MAVROS setpoint_raw 插件完成 ENU→NED，禁止业务层重复换轴；
- 轨迹耗尽后先切固定 position/yaw HOLD 再做 arrival 状态转换，末段非零导数不泄漏；
- `direct_control_active=true` 时本节点停止发布候选，MAVROS 所有权由 relay 保证；
- `replan_period_s` 默认 1.0 s（常规滚动重规划阈值，等价于降低轨迹陈旧风险；**不等价**原版
  20 Hz 独立安全检查、碰撞预测或紧急停止，本实现不包含这些机制）。

身份参数（`uav_id`/`exec_target`）位于公开命名空间 `<namespace>/`；规划调参位于私有 `<node_name>/`。

## 安全与验证

- C++ 节点不 arm、不切 OFFBOARD，也不因 arm+OFFBOARD 自动起飞。除 FOLLOW_ROUTE follower 的 PI 位置跟随外，轨迹均由 Ego 规划；PI 直接控制期间通过仲裁暂停 EGO setpoint。
- `swarm_clearance=0.5` 表示每机安全半径：互惠检查使用两倍值，对应 1.0 m 水平中心距；原有 2:1 椭球对应 2.0 m 垂直中心距。规划结果在发布前执行硬 clearance 校验。
- 自动化：2026-08-13 全工作空间 133 tests 全绿（含本包 30 个 gtest 计数）。
- 单 Master 冒烟（2026-08-10）：`IDLE → EXECUTING → COMPLETED → HOLD` 生命周期 PASS；2026-08-12 起初始态改为 `HOLD`，arm+OFFBOARD 后进入 `TAKEOFF` 软起飞。详见 `tcp_to_ros/docs/test_record_26081402.md`。

## 环境变量（ROS 日志重定向）

为避免 ROS 运行日志散落在 `~/.ros/log`，本工作空间统一将运行日志重定向到 `.ros_home/`：

```bash
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
mkdir -p "$ROS_LOG_DIR"
```

手动启动前请先在终端执行以上命令，确保当前工作空间所有 ROS 运行日志均写入 `.ros_home/log`。
