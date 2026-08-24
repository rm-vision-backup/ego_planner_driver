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
- `replan_period_s` 默认 1.0 s；另有 25 Hz `SafetyPredictor` 独立监督。阶段 A 默认 `shadow`，
  只计算同一绝对时间轴的 CPA/TTC、动态 warning/emergency 门槛、ID right-of-way 并异步记录，
  **不改变**状态、轨迹、intent 或 setpoint；受约束制动与主动让路尚未接通。

## 预测安全监督（26082500 阶段 A）

- `SafetyPredictor` 对本机执行轨迹和邻机 intent 取共同绝对时间窗，计算相对状态、closing speed、
  CPA、首次越过 1m 的 TTC、数据年龄及动态两级距离门槛；预测窗不足显式输出
  `INSUFFICIENT_DATA`。
- 数字身份取末尾数字；低 ID=`STAND_ON`、高 ID=`YIELD`，非法或重复数字=`UNKNOWN`。
- `SafetyLogger` 使用有界队列和单写线程。默认覆盖写
  `/home/ub20tg/catkin_swarm6-2/runtime_logs/ego_planner/UAVn-ego-planner.log`；安全 timer
  不做同步磁盘 IO。目录和队列容量可由 launch 参数覆盖。
- `safety_supervisor_mode=off` 完全关闭；`shadow` 仅诊断。阶段 A 若误配 `active`，节点告警并
  降级 `shadow`，禁止伪装成已启用制动闭环。

## min-snap 统一单段重构（26082415）

- 上层 `EgoSwarmDriver` 把分层关键点（start + 层点 + 弓字转折 + 目标）按最大 **2m** 间距密化后
  发布 `/waypoints`；**2m 密化点只用于窗口推进（`advanceConsumed`）/末点参考，不进 B-spline**。
- `planOnce` 用原版 `one_segment_traj_gen`（单段五次多项式闭式解，min-snap）生成初始轨迹：
  起点=旧轨迹 P/V/A（滚动连续性），end_vel=末点切向 × min(`cruise_velocity`, `max_vel`)，
  end_acc=0，仅最终段 end_vel=0；段时长 T 用梯形速度估计（v0 投影，加速-匀速-减速）。
- 多项式按 **0.4m 弧长等距**采样为 point_set → `parameterizeToBspline` → L-BFGS（jerk +
  feasibility 软惩罚）→ 采样发布 PositionTarget P/V/A。起点/末端控制点分别在优化前/后精确锚定。
- 默认停用 `enable_rebound`（rebound 硬推）与 `enable_time_reparam`（时间重参数化）；机间安全由
  执行层运行时距离门禁兜底（`EgoSwarmDriver._distance_safe`，水平 <1.0m 且垂直 <2.0m → 本机 HOLD）。
- `UavTrajectoryIntent.stamp` 是轨迹实际激活 epoch，`t` 严格递增且相对该 epoch（`t[0]=0`、
  `t.back()=duration`）；生产者填写 `frame_id=map`。接收端拒绝协议、frame、epoch、序列、长度、
  单调性或有限值不合法的 intent。规划完成后按候选实际激活时刻重新推进邻机时间轴，最多重试 3 次，
  并在提交锁内使用最新 intent 再做一次 1m hard-clearance 校验，防止规划耗时与并发更新批准陈旧候选。
- 多段 `minSnapTraj` 为备用方案（单段无法处理时的修改方案），本计划不实现。

身份参数（`uav_id`/`exec_target`）位于公开命名空间 `<namespace>/`；规划调参位于私有 `<node_name>/`。

## 安全与验证

- C++ 节点不 arm、不切 OFFBOARD，也不因 arm+OFFBOARD 自动起飞。除 FOLLOW_ROUTE follower 的 PI 位置跟随外，轨迹均由 Ego 规划；PI 直接控制期间通过仲裁暂停 EGO setpoint。
- `swarm_clearance=0.5` 表示每机安全半径：互惠检查使用两倍值，对应 1.0 m 水平中心距；原有 2:1 椭球对应 2.0 m 垂直中心距。规划结果在发布前执行硬 clearance 校验。rebound 硬推默认关闭，机间安全由执行层运行时距离门禁（`EgoSwarmDriver._distance_safe`）兜底：邻机水平 <1.0m 且垂直 <2.0m 时本机 HOLD 暂停。
- rebound 手工启用时会对硬推后的轨迹强制时间重参数化，再按新时间轴重做 clearance；理想匀速正交交叉可在速度 ≤1.5m/s、加速度 ≤`max_acc*1.05` 下到达。但由同一 min-snap 核心生成的真实 peer nominal intent 仍可能使让路方规划失败，多轮互惠收敛尚未通过，因此禁止仅翻转默认开关。
- 自动化：2026-08-25 阶段 A `ego_planner_driver` 96 tests、0 failure（原 78 全回归 + 新 18），
  覆盖 CPA/TTC、绝对时间同步、数据年龄、动态门槛、身份/角色、episode 迟滞、异步日志和 shadow
  行为不变。双节点动力学、SITL 与完整链路尚未执行。
- 单 Master 冒烟（2026-08-10）：`IDLE → EXECUTING → COMPLETED → HOLD` 生命周期 PASS；2026-08-12 起初始态改为 `HOLD`，arm+OFFBOARD 后进入 `TAKEOFF` 软起飞。详见 `tcp_to_ros/docs/test_record_26081402.md`。

## 环境变量（ROS 日志重定向）

为避免 ROS 运行日志散落在 `~/.ros/log`，本工作空间统一将运行日志重定向到 `.ros_home/`：

```bash
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
mkdir -p "$ROS_LOG_DIR"
```

手动启动前请先在终端执行以上命令，确保当前工作空间所有 ROS 运行日志均写入 `.ros_home/log`。
