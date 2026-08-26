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
| `exec_state` | `std_msgs/String`（latched） | 出：`HOLD`（初始）/`TAKEOFF`/`EXECUTING`/`COMPLETED`/`EGO_PLAN_FAILED`/`EGO_EXEC_TIMEOUT`/`EMERGENCY_BRAKE`/`BRAKE_HOLD`（26082621 删 `POSE_STALE`，HOLD 锁存位置） |
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
- 避碰门槛解耦（26082602）：规划层 `calcSwarmCost` 回**固定 clearance**（`swarm_clearance×2=1m`），
  不再消费 `data_age`/动态安全距离；`intent_publish_rate_hz` 周期 5Hz 刷新已删除，intent
  **生成即发**（replan/制动提交时发布，带 `stamp`+`traj_id`）；新增 **10Hz 周期碰撞检查**
  `collisionCheckTick`（本机 `current_` vs fresh 邻机轨迹 <1m → 请求 replan），接收邻机 intent
  的事件驱动 replan 已删除（作用重复，备用记录见计划 §5）。监督层 `SafetyPredictor` 动态
  EMERGENCY 门槛、GPS bias 修正与旧 1m 门禁保留。
- **plan-exec 循环重构（26082621）**：上层 `EgoSwarmDriver` 把分层关键点按 **3m 间距**密化后
  发布 `/waypoints`；C++ 任务航点 = **相邻下一个 keypoints(n+1)**（替换 horizon 窗口滚动）。
  执行循环每 tick 三分支：①推进（距当前目标 <0.5m → 切下一关键点，<0.5m 丢弃、3.5m 上限）、
  ②到达（距最终关键点 <0.2m → COMPLETED；26082621 复盘由 0.1m 放宽——PX4 SITL 稳态偏差
  实测 0.13-0.17m，0.1m 太紧导致末段 HOLD）、③定时 replan（elapsed ≥1.0s → 置位唤醒）。
- **单一 replan 入口（26082621 方案 A）**：1s 定时 / 10Hz 冲突 / 推进 / EMERGENCY 事件**全部
  只置位**（`planning_requested_`/`emergency_pending_`）并唤醒 replanLoop（条件变量，1s 超时
  兜底）；replanLoop → `replanOnce()` 是**唯一轨迹提交者**（常规 replan / 候选 / 制动）。
- **EMERGENCY 制动走 replan（26082621 方案 Y）**：监督层检测 EMERGENCY 只置
  `emergency_pending_`（记录 primary peer + ttc）；replan 线程在"常规 replan 失败且
  ttc < `emergency_replan_time_s`(1.0s)"时提交受约束制动（EMERGENCY_BRAKE → BRAKE_HOLD），
  否则收口 PLAN_FAILED。监督回调不再直接改轨迹。
- **机间让路候选（26082621 迭代，replan 失败分支 + 裕度不足分支）**：right-of-way
  （低 ID STAND_ON 只生成 **SLOW 变速**候选、高 ID YIELD 生成 SLOW/LEFT/RIGHT）+ **SLOW
  {0.5, 0.8, 1.2}（含加速）** / LEFT-RIGHT lateral {1.3, 1.8, 2.2}×scale {0.7,0.5}，
  min_clear ≥1.2×clearance，cost=ΔT/T（绝对值）+dev/2.0；常规规划**成功但裕度 <1.2m**
  也生成候选替换（B）。候选**只**在 `replanOnce()` 内提交（10Hz 独立提交已删除）。

## 预测安全监督（26082500 阶段 A）

- `SafetyPredictor` 对本机执行轨迹和邻机 intent 取共同绝对时间窗，计算相对状态、closing speed、
  CPA、首次越过 1m 的 TTC 及动态两级距离门槛（反应时间**不含 data_age**——邻机轨迹激活时刻已由
  GPS bias 修正到本机时间轴，同刻采样即预测；age 仅诊断，防止发布频率降低后门槛被放大误触发）；
  预测窗不足显式输出 `INSUFFICIENT_DATA`。
- EMERGENCY 处理走 replan 线程（26082621 方案 Y）：监督回调只置 `emergency_pending_` 唤醒
  replanLoop；replan 线程先尝试常规重规划（成功则继续执行），失败且预测越界时间
  `ttc < emergency_replan_time_s`(1.0s) 才提交受约束制动，否则收口 PLAN_FAILED。
  数字身份取末尾数字；低 ID=`STAND_ON`、高 ID=`YIELD`，非法或重复数字=`UNKNOWN`。
- `SafetyLogger` 使用有界队列和单写线程。默认覆盖写
  `/home/ub20tg/catkin_swarm6-2/runtime_logs/ego_planner/UAVn-ego-planner.log`；安全 timer
  不做同步磁盘 IO。目录和队列容量可由 launch 参数覆盖。
- `safety_supervisor_mode=off` 完全关闭；`shadow` 仅诊断。阶段 A 若误配 `active`，节点告警并
  降级 `shadow`，禁止伪装成已启用制动闭环。

### 受约束紧急制动（阶段 B）

- 显式配置 `safety_supervisor_mode=active` 后，任一 UAV 进入 `EMERGENCY` 即从当前执行轨迹同刻
  P/V/A 生成五次多项式停车轨迹；时间伸长直到速度、加速度、jerk 门禁通过。
- 紧急提交递增 safety generation，常规规划快照提交时校验 generation，晚到结果不能覆盖制动。
- 对外状态 `EMERGENCY_BRAKE` 表示正在制动；轨迹结束锁存真实制动终点并发布 `BRAKE_HOLD`，
  executor 将后者收口为失败。制动 intent 继续沿现有 active 语义广播。
- 生成器异常时立即中止 nominal 并在当前样本位置 fallback HOLD，同时高优先告警；该降级只防止继续
  nominal，不替代正常受约束制动验收。生产 launch 仍默认 `shadow`，SITL 标定前不默认 active。

### Intent 协商屏障（阶段 C）

- `UavTrajectoryIntent` 协议 2.0 增加 `TENTATIVE/ACTIVE/BRAKING` phase。首个 nominal 候选只广播
  tentative，不写 `current_`；生产配置最多等待 1s，期间不发布轨迹 setpoint。
- 新 peer intent 到达会丢弃 tentative 并按最新 inbox 重算；超时后不等待从未出现的邻机，commit-lock
  hard-clearance 通过才激活。协议 1.0 数据仅按旧 ACTIVE 语义接收。
- ROS1 `.msg` 变更会改变 MD5；部署必须同批重建 interfaces、planner、executor 和 bridge，不能混跑
  新旧二进制。bridge 仍只做 ROS 原生序列化透传，不理解 phase 业务。

### 确定性让路候选（阶段 D）

- active WARNING 时低 ID `STAND_ON` 保持 nominal；高 ID `YIELD` 按固定顺序构造 LEFT、RIGHT、
  SLOW、FAST。UNKNOWN 不猜责任，转制动；EMERGENCY 对所有角色直接制动。
- LEFT/RIGHT 使用 C2 横向 bell，候选起终点 P/V/A 连续；四类均经过 finite、速度、加速度、jerk
  和全 peer clearance validator。评分固定为 clearance、到达时间增量、横移、枚举顺序。
- episode 内候选只提交一次并锁存方向，避免 25Hz 重复切轨迹；失败转阶段 B 制动。`enable_rebound`
  仍为 false。生产默认 shadow，因此 SITL 验收前不会自动执行候选。

## min-snap 统一单段重构（26082415）

- 上层 `EgoSwarmDriver` 把分层关键点（start + 层点 + 弓字转折 + 目标）按最大 **3m** 间距密化后
  发布 `/waypoints`；**3m 密化关键点只用于相邻目标推进（plan-exec 分支 A）/末点参考，不进
  B-spline**。任务航点 = 相邻下一个 keypoints(n+1)。
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
- 单 Master 冒烟（2026-08-10）：`IDLE → EXECUTING → COMPLETED → HOLD` 生命周期 PASS；2026-08-12 起初始态改为 `HOLD`，arm+OFFBOARD 后进入 `TAKEOFF` 软起飞。详见 `tcp_to_ros/test_records/test_record_26081402.md`。

## 环境变量（ROS 日志重定向）

为避免 ROS 运行日志散落在 `~/.ros/log`，本工作空间统一将运行日志重定向到 `.ros_home/`：

```bash
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
mkdir -p "$ROS_LOG_DIR"
```

手动启动前请先在终端执行以上命令，确保当前工作空间所有 ROS 运行日志均写入 `.ros_home/log`。
