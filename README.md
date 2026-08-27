# ego_planner_driver

机载 ego-swarm 风格带时轨迹生成与互惠机间避碰驱动。

- 抽取 ego-planner-swarm 内核的 bspline 优化与轨迹生成，纯数值无 ROS 依赖（`EgoPlannerCore`）。
- 保留 jerk 平滑、动态可行性、reciprocal swarm 避碰三项代价；排除感知/静态障碍外挂。
- 通过 ROS 节点订阅本机目标/邻机轨迹意图，发布 setpoint 与本机轨迹意图。
- 该包是独立 Git 仓库，不 push、不配置 remote。

> 本文档是本包**状态机 + replan 策略的单一权威**。实现计划的演进记录见
> `src/tcp_to_ros/plans/README.md`「当前状态」；测试证据见 `src/tcp_to_ros/test_records/`。

## 接口与命名空间

节点在 `<namespace>/`（如 `/UAV1/`）公开以下接口，Python `EgoSwarmDriver` 与 topology bridge 直接使用：

| Topic | 类型 | 方向 |
|---|---|---|
| `exec_state` | `std_msgs/String`（latched） | 出：8 态（见下方状态机） |
| `setpoint/ego` | `mavros_msgs/PositionTarget` | 出：30 Hz 候选命令（`FRAME_LOCAL_NED`，业务字段 ROS ENU；`setpoint_relay` 独占转发 `/mavros/setpoint_raw/local`） |
| `trajectory_intent` | `swarm_uav_interfaces/UavTrajectoryIntent` | 出：本机带时轨迹（生成即发） |
| `goal` | `geometry_msgs/PointStamped` | 入：单点目标 |
| `goal_yaw` | `std_msgs/Float64` | 入：冻结的集群参考航向 |
| `waypoints` | `geometry_msgs/PolygonStamped` | 入：密化后的关键点序列 |
| `local_pose` | `geometry_msgs/PoseStamped` | 入：本机位姿 |
| `neighbor_intent` | `swarm_uav_interfaces/UavTrajectoryIntent` | 入：邻机轨迹 |
| `hold` | `std_msgs/Empty` | 入：HOLD（高优先级，清空规划请求） |
| `direct_control_active` | `std_msgs/Bool` | 入：follower PI 直接控制仲裁（true 时本节点不发布候选） |
| `gps_bias` | `std_msgs/Float64` | 入：本机 GPS 时钟偏置 |

## PositionTarget 契约

所有候选统一为 `mavros_msgs/PositionTarget`：

| 模式 | `coordinate_frame` | 启用字段 | `type_mask` |
|---|---|---|---|
| EGO 轨迹 | `FRAME_LOCAL_NED` | position、velocity、acceleration、yaw | `IGNORE_YAW_RATE` = **2048** |
| HOLD/TAKEOFF/steady/COMPLETED | `FRAME_LOCAL_NED` | position、yaw | `IGNORE_VX..Z\|IGNORE_AFX..Z\|IGNORE_YAW_RATE` = **2552** |

- P/V/A/yaw 来自同一轨迹同一采样时刻（`TimedTrajectory::samplePVA`），不跨帧差分；
- 业务字段按 ROS ENU 填写，MAVROS setpoint_raw 插件完成 ENU→NED，禁止业务层重复换轴；
- 轨迹耗尽后先切固定 position/yaw HOLD 再做 arrival 状态转换，末段非零导数不泄漏；
- `direct_control_active=true` 时本节点停止发布候选，MAVROS 所有权由 relay 保证；
- `setpoint_relay`（swarm_uav_executor）是 `/mavros/setpoint_raw/local` 唯一发布者，本节点不直接发 MAVROS setpoint。

## 状态机（8 态）

```
HOLD(初始) ──arm+OFFBOARD──► TAKEOFF ──爬升到 takeoff_height_m(5m)──► HOLD
HOLD ──goal/waypoints───────► EXECUTING
EXECUTING ──分支A 推进───► EXECUTING（距当前目标<0.5m 切下一关键点，置位唤醒 replan）
EXECUTING ──分支B 到达───► COMPLETED（距最终关键点<0.2m 稳定 1s）──► HOLD
EXECUTING ──执行超时───────► EGO_EXEC_TIMEOUT ──(executor 收口)──► HOLD
EXECUTING ──replan 全失败≥10s──► EGO_PLAN_FAILED ──(executor 收口)──► HOLD
EXECUTING ──EMERGENCY+replan失败+ttc<1s──► EMERGENCY_BRAKE ──轨迹耗尽──► BRAKE_HOLD（锁存，不自动恢复）
EXECUTING ──EMERGENCY+replan失败+ttc≥1s──► EGO_PLAN_FAILED（立即）──► HOLD
```

- 状态集：`HOLD / TAKEOFF / EXECUTING / COMPLETED / EGO_PLAN_FAILED /
  EGO_EXEC_TIMEOUT / EMERGENCY_BRAKE / BRAKE_HOLD`（8 个；`POSE_STALE` 与 `NEGOTIATING` 已删除）。
- `HOLD` 锁存进入时位置/yaw 为 setpoint，不随 pose 漂移（删 POSE_STALE 后无 pose 过期态；
  pose 过期期间不发布无效 setpoint，pose 恢复后自愈）。
- `TAKEOFF`：arm+OFFBOARD 沿后从当前 z 以 `takeoff_ascend_speed` 软上升到 `takeoff_height_m`，
  完成即锁存 HOLD。此状态是本机"软起飞"来源；executor 不再执行按需 arm（见 swarm_uav_executor README）。

## plan-exec 执行循环（publishSetpoint 每 tick）

任务航点 = 上层 `EgoSwarmDriver` 按 **3m 间距**密化后的关键点序列（`waypoint_densify_spacing=3.0`）。
C++ 侧 `waypoint_consumed_` 指向当前目标（初始 = 相邻下一个 keypoints(n+1)，替换 horizon 窗口滚动）。
每 tick 三分支，全部只置位：

| 分支 | 条件 | 动作 |
|---|---|---|
| A 推进 | 距当前目标 `< reach_thresh_m_`(0.5m) | 切相邻下一关键点；过近(<0.5m)丢弃、最远 `max_advance_dist_m_`(3.5m) → 置 `planning_requested_` + 唤醒 |
| B 到达 | 距最终关键点 `< arrival_reach_thresh_m_`(0.2m) 且稳定 `arrival_stable_s`(1.0s) | `COMPLETED` → 冻结终点点位为 HOLD（0.2m 为 SITL 稳态偏差 0.13–0.17m 复盘放宽） |
| C 定时 | `elapsed ≥ min(replan_period_s_=1.0, duration×0.85)` | 置 `planning_requested_`；距最终目标 `≤ no_replan_thresh_m_`(1.0m) 时停发，让末段自然走完触发 B |

## 单一 replan 入口（方案 A）

`replanLoop` 线程（条件变量，`replan_period_s_` 超时兜底）是**唯一轨迹提交者**。
所有触发只置位/唤醒，不在回调里直接改轨迹：

| 触发源 | 频率 | 动作 |
|---|---|---|
| 定时 replan（分支 C） | 1s | `planning_requested_=true` |
| 周期碰撞检查 `collisionCheckTick` | 10Hz | `current_` vs fresh 邻机同刻距离 < `1.2×swarm_clearance×2`(1.2m) → `planning_requested_=true` |
| 推进（分支 A） | 每 tick | 切关键点时 `planning_requested_=true` |
| 监督 EMERGENCY（方案 Y） | 25Hz | 只置 `emergency_pending_`（记录 primary peer + ttc）+ 唤醒 |

`replanLoop` 处理：HOLD → TAKEOFF/直接控制跳过 → SEQUENTIAL_START 首轮排序
（leader(id≤1)/已收前驱 intent/超时 `first_plan_wait_timeout_s`(10s) 才放行首轮）→
事件置位或定时超时执行一轮 `replanOnce`。

## replanOnce（唯一提交者，按优先级）

```
① EMERGENCY 且常规 replan 失败：
     ttc < emergency_replan_time_s(1.0s) → 受约束制动（EMERGENCY_BRAKE → BRAKE_HOLD，方案 Y）
     ttc ≥ 1.0s                          → EGO_PLAN_FAILED（立即收口）
② 常规 replan（planOnce，激活时刻对齐 ≤3 次重试）：
     checkClearance（固定 swarm_clearance×2=1m 硬门，规划/激活/提交三处）→ 提交
③ 常规规划"成功但裕度不足"：min_clear < 1.2m → tryYieldReplan 生成候选替换（失败保持原轨迹）
④ 常规 replan 失败：tryYieldReplan 候选（right-of-way）→ 成功提交
⑤ 软着陆（26082700）：全失败不立即 PLAN_FAILED——保持当前轨迹执行（末端自然悬停），
   由 1s 定时/冲突/推进事件继续重试；连续失败 ≥ plan_fail_timeout_s(10s) 才 EGO_PLAN_FAILED
   （EMERGENCY 分支保持立即收口）
```

提交段安全校验：提交锁内用最新邻机 intent 复验 1m 硬门；`safety_generation_` 防晚到结果覆盖
制动/候选。常规 replan/候选/制动提交均清空 `plan_fail_since_`（退出软着陆计时）。

## 让路候选（replan 失败分支，SLOW/LEFT/RIGHT + right-of-way）

- **right-of-way**：`parseNumericId` 数字比较。低 ID `STAND_ON` 只生成 SLOW 变速候选
  （`slow_only`，不横向偏移）；高 ID `YIELD` 生成 SLOW/LEFT/RIGHT 全候选。episode 内锁存方向与
  primary peer 轨迹版本（防重复提交）。
- **SLOW**：几何不变，时间轴缩放 `{0.5, 0.8, 1.2}`（含加速档；0.5/0.8 减速等待、1.2 加速先过）。
  邻机近似悬停（末速 < `hovering_velocity_mps`=0.2）跳过 SLOW。速度上限 `yield_max_velocity_mps`(2.5)。
- **LEFT/RIGHT**：横向 plateau 偏移 `{1.3, 1.8, 2.2}` m × 时间缩放 `{0.7, 0.5}`；方向取远离
  peer 时空重叠时段的法向分量，平台覆盖重叠全程 + C2 过渡。
- **验证门槛**：全时域 vs 全部 fresh 邻机同刻 `min_clear ≥ 1.2×clearance`；
  `cost = |ΔT|/T_nominal + lateral/5.0`（dev 权重 26082700 由 /2.0 下调），选 cost 最小者提交。
- 候选**只**在 `replanOnce()` 内生成/提交（10Hz 独立提交已删除，提交唯一化）。

## 机间避碰（规划层）

- `calcSwarmCost`：固定 `swarm_clearance×2`=1m 软惩罚（26082602 解耦 data_age/动态门槛）。
  软惩罚椭球为**对称 1m 球**（`kSwarmA=kSwarmB=1.0`，水平=垂直=1m），与原版 `a=2.0,b=1.0`
  椭圆不同——当前按 CONTEXT「机间最小距离」规约（水平 1m / 垂直 2m 由执行层门禁实施）。
- `checkClearance`：提交前固定 1m 硬门（对全部采样点）。
- 预测能力来自**同刻采样 + GPS bias 时间戳修正**（`T_own = T_other - bias_other + bias_own`），
  不来自 intent 年龄。intent 生成即发（提交时发布，`stamp`=激活 epoch，`traj_id` 递增），
  接收窗 `[now-30s, now+30s]`（`kNeighborStaleS`）。
- 10Hz `collisionCheckTick` 检测冲突只置 `planning_requested_`（提交统一走 replan 线程）。

## 监督层 SafetyPredictor（25Hz，方案 Y）

- 对本机执行轨迹和邻机 intent 取共同绝对时间窗，计算 CPA/TTC/min_d。EMERGENCY 判定以
  **固定物理 1m**（`protected_distance_m`）或 `ttc ≤ emergency_ttc_s`(1.5s) 为准；动态门槛
  （`dynamicMargin`：closing×reaction + closing²/2decel）只作 WARNING 缓冲，不进入 EMERGENCY。
  `data_age` 仅诊断（reaction 不含 age，防发布频率降低后门槛放大误触发）。
- `level==EMERGENCY`：只置 `emergency_pending_` + 唤醒（方案 Y），制动由 replan 线程在
  "常规 replan 失败且 ttc < `emergency_replan_time_s`(1.0s)" 时提交，监督回调不改轨迹。
- `level==WARNING`：仅写日志（交规划层软惩罚绕开）。
- `safety_supervisor_mode`：`off`（关闭）/ `shadow`（仅诊断日志，不改行为）/ `active`（接制动兜底）。
  launch 默认 `shadow`；`startup_ego_sitl.sh` 机载层统一以 `active` 启动。
- `SafetyLogger` 有界队列 + 单写线程，覆盖写
  `/home/ub20tg/catkin_swarm6-2/runtime_logs/ego_planner/UAVn-ego-planner.log`。

## 轨迹生成（EgoPlannerCore::planOnce）

- 起点 = 旧轨迹同刻 P/V/A/yaw（滚动连续，`sampleCurrentState`），首次 = 实际位置 + 零导数；
  起点导数按范数限幅（`clampStartDerivatives`）。
- 终点 = 当前任务航点（相邻关键点或前方推进目标）；`end_vel` = 末点切向 ×
  `min(cruise_velocity, max_vel)`，仅最终段 `end_vel=0`；段时长 T 用梯形速度估计。
- min-snap 单段五次多项式（`one_segment_traj_gen`）→ **0.4m 弧长**等距采样 point_set →
  `parameterizeToBspline` → L-BFGS（jerk + feasibility 软惩罚）→ 按 0.1s 步长采样 TimedTrajectory。
- 起点/终点控制点在优化前/后精确锚定（保证严格过关键点与指定边界导数）。
- `enable_rebound` / `enable_time_reparam` 默认 false（机间安全由执行层运行时距离门禁兜底）。
- intent 时间契约：`stamp`=实际激活 epoch、`t[0]=0`、`t.back()=duration`、`frame_id=map`；
  规划完成后按实际激活时刻重新推进邻机时间轴，最多重试 3 次。

## 参数（launch：`ego_planner_driver.launch` / `uav_executor_ego.launch`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `ts` / `max_vel` / `max_acc` | 0.4 / 1.0 / 6.0 | B-spline 节距 / 速度 / 加速度上限 |
| `swarm_clearance` | 0.5 | 每机安全半径，互惠使用 ×2 = 1.0m |
| `planning_range` | 30.0 | 邻机轨迹纳入避碰的范围 |
| `cruise_velocity` / `acceleration_limit` | 1.0 / 2.0 | 段末切线巡航速度 / 梯形时间加速度 |
| `enable_rebound` / `enable_time_reparam` | false / false | 硬推 / 时间重参数化（默认关） |
| `replan_period_s` | 1.0 | 定时 replan 周期 |
| `reach_thresh_m` / `max_advance_dist_m` / `arrival_reach_thresh_m` | 0.5 / 3.5 / 0.2 | 推进 / 推进上限 / 到达阈值 |
| `no_replan_thresh_m` | 1.0 | 距最终目标该值内停止定时 replan |
| `activation_replan_attempts` | 3 | 激活时刻对齐重试次数 |
| `setpoint_rate_hz` / `local_pose_timeout_s` / `arrival_stable_s` | 30 / 1.0 / 1.0 | setpoint 频率 / pose 超时 / 到达稳定 |
| `exec_timeout_margin_s` | 5.0 | 轨迹耗尽后收口余量 |
| `takeoff_height_m` / `takeoff_ascend_speed` | 5.0 / 1.0 | 软起飞高度 / 上升速度 |
| `collision_check_rate_hz` | 10.0 | 周期碰撞检查频率 |
| `bias_valid_s` | 120.0 | GPS bias 新鲜窗口 |
| `safety_supervisor_mode` / `safety_supervisor_rate_hz` | shadow / 25.0 | 监督模式 / 频率 |
| `protected_distance_m` | 1.0 | EMERGENCY 物理距离 |
| `emergency_ttc_s` / `emergency_replan_time_s` | 1.5 / 1.0 | EMERGENCY ttc / 制动前重规划窗口 |
| `emergency_decel_mps2` / `emergency_max_velocity_mps` / `emergency_jerk_mps3` | 2.0 / 1.5 / 8.0 | 制动动力学 |
| `safety_prediction_horizon_s` / `safety_sample_period_s` | 5.0 / 0.05 | 监督预测窗 / 采样步长 |
| `first_plan_wait_timeout_s` | 10.0 | SEQUENTIAL_START 首轮等待前驱超时 |
| `enable_yield_candidates` | true | 让路候选开关 |
| `yield_clearance_factor` / `yield_lateral_max_m` / `yield_slow_min_scale` | 1.2 / 2.2 / 0.3 | 候选门槛 / 横向上限 / SLOW 最低档 |
| `yield_max_velocity_mps` | 2.5 | 候选/加速档速度上限（独立于制动 1.5） |
| `diagnostic_log_dir` / `diagnostic_log_queue_size` | 见 launch / 2048 | 监督日志目录 / 队列 |

身份参数（`uav_id`/`exec_target`）位于公开命名空间 `<namespace>/`；规划调参位于私有 `<node_name>/`。

## 安全与验证

- C++ 节点不 arm、不切 OFFBOARD，也不因 arm+OFFBOARD 自动起飞之外的动作改变飞行。
- 规划结果提交前硬 clearance 校验；监督 EMERGENCY 与执行层运行时距离门禁
  （`EgoSwarmDriver._distance_safe`，水平 <1.0m 且垂直 <2.0m → 本机 HOLD）兜底。
- 全量自动测试 0 failure（含单 Master 冒烟生命周期、状态机、轨迹、候选、监督、制动）。

## 环境变量（ROS 日志重定向）

为避免 ROS 运行日志散落在 `~/.ros/log`，本工作空间统一将运行日志重定向到 `.ros_home/`：

```bash
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
mkdir -p "$ROS_LOG_DIR"
```

手动启动前请先在终端执行以上命令，确保当前工作空间所有 ROS 运行日志均写入 `.ros_home/log`。
