# Offboard 新状态机设计（2026-09-02）

本文定义 `swarm_uav_executor`、`ego_planner_driver` 与 `tcp_to_ros` 的职责边界、状态机动作和任务分配规则。本文严格区分 runtime 内部任务模型、TCP/ROS 任务协议和机载内部安全动作。当前 `tcp_to_ros` parser 支持的 Group A `command` 值包括 `MOVE_TO`、`FOLLOW_ROUTE`、`FAULT_EXIT`、`HOVER`、`HOLD`；上游 runtime 的 `hold-a-end` 计划实际使用普通命令 `HOVER`，不能把计划名称 `hold-a-end` 或普通 `HOVER` 与安全保持 `HOLD` 混为一谈。

本文的关键边界是：多机 `MOVE_TO` 任务的执行对象分配和发布前冲突协调由 `tcp_to_ros` 完成；`ego_planner_driver` 只负责本机任务的连续执行、单层 nominal 轨迹更新和运行时安全收口。

---

## 0. 适用范围与设计边界

### 0.1 字段和命名约定

文档中出现的字段必须按所在层次解释，不能跨层替换：

| 层次 | 本次任务/计划标识 | 单机身份字段 | 命令字段 | 说明 |
|---|---|---|---|---|
| runtime 内部 reviewed plan | `mission_id`、`plan_id` | `robot_id` | `command_type` | `plan_id` 是内部计划引用；`robot_id`/`command_type` 不直接等于 TCP/ROS 字段 |
| TCP 整组协议 | `mission_id`、`command_id` | `assignments[].uav_id` | `command` | `command_id` 是本次整组命令实例的唯一标识 |
| `ExecuteGroupTask` Action | `mission_id`、`command_id` | `TaskAssignment.uav_id` | `command` | Action 使用 `assignments` 数组，不使用 `robot_assignments` |
| 单机 `UavTask` Service / `UavTaskState` | `mission_id`、`command_id` | `uav_id` + `exec_target` | `command` | Service 下发时二者都是执行层 UAVn；`UavTaskState` 也报告执行层 UAVn；GCS 再按活动任务映射还原为任务层 Axx |

因此：外部 TCP/ROS 文档使用 `command` 和 `uav_id`；只有 runtime 内部 reviewed plan/`CommandEnvelope` 使用 `command_type` 和 `robot_id`。不能用 `command` 代替 `command_id` 去重，也不能用内部 `plan_id` 代替线上 `command_id`。当前 Group A 的 `hold-a-end` 是 runtime 内部计划名称，其线上 `command` 为 `HOVER`，不是 `HOLD`。

安全保持还要再区分：普通任务 `HOVER` 完成后进入 `COMPLETED_HOLD`；本文内部 `HOLD` 表示安全保持请求并进入 `EMERGENCY_HOLD`；runtime abort fallback 的内部命令名 `SAFE_HOLD` 不作为 TCP/ROS 普通任务命令使用。

### 0.2 文档阅读规则

本文使用以下标记避免把设计目标误读为当前实现：

```text
源码事实
  = 已存在的字段、状态、函数或当前控制流；使用现有名称。

目标设计
  = 本文要求后续实现的职责或事务；使用“目标逻辑名称”并明确它不是现有 API。

迁移约束
  = 当前实现与目标设计之间必须补上的适配、测试或状态映射。
```

### 0.3 本文解决的问题

```text
任务发布前：
  tcp_to_ros 在当前 MOVE_TO 任务集合内选择 Axx -> UAVn 的执行分配，
  验证该分配是否可行，并在无解时直接向上游返回任务失败。

任务发布后：
  ego_planner_driver 保证本机 setpoint 连续、轨迹更新串行、
  过期规划结果不提交，并在实际安全条件失效时进入安全保持。
```

状态机的实施价值不是在机载侧重新求解全局多机冲突，而是保证已发布任务能够以可控、可验证的方式执行。

### 0.4 不属于机载状态机的职责

`ego_planner_driver` 不负责：

- 重新分配 Axx 与 UAVn；
- 修改或恢复 `tcp_to_ros` 的运行时映射；
- 通过 UAV 序号决定 STAND_ON/YIELD；
- 在 nominal 规划失败后执行第二层 SLOW/LEFT/RIGHT 轨迹变形；
- 保证每次 `replanOnce()` 都能产生新轨迹；
- 用局部重规划替代任务发布前的全局冲突协调。

### 0.5 当前设计与实现的关系

以下内容是目标设计约束，不表示当前源码已经全部实现。特别是：

- 当前 `tcp_to_ros` 的身份配置仍是静态基线；
- 当前 `t_off` 实现的 `_schedule_applies()` 同时覆盖 `MOVE_TO` 和 `FAULT_EXIT`，这与本文“只有 `MOVE_TO` 进入新分配逻辑”的目标不同，后续实现必须显式收紧，不能把现状误写成已完成设计；
- 当前 `ego_planner_driver` 仍存在第二层候选相关代码，目标设计不再使用该路径；
- 第 6 章规定的运行时当前映射、MOVE_TO 分配和失败反馈需要后续在 `tcp_to_ros` 中实现。

---

## 1. 系统分层与身份边界

### 1.1 总体职责

```text
runtime
  │ 内部 reviewed plan 使用 plan_id、robot_assignments、robot_id、command_type
  │ 对外发送整组 TCP command 时转换为 mission_id/command_id/command/assignments
  ▼
tcp_to_ros / GCS
  │ 接收 TCP/ROS 层的 Axx 任务（assignments[].uav_id）
  │ 维护当前运行时映射，仅对 command == MOVE_TO 做任务分配
  │ 验证参与集合、路径冲突、状态和发布前预算
  │ 将任务转换为对应 UAVn 的 executor 请求
  ▼
swarm_uav_executor
  │ 按单机请求中的 uav_id + exec_target 接收、保存、启动和监控任务
  │ 向 driver 提供 waypoints、leader/offset 或安全保持输入
  ▼
ego_planner_driver
  │ 只执行本机已经分配的任务
  │ 维护内部状态机、setpoint、nominal 轨迹和安全动作
  ▼
PX4
  │ 接收 offboard setpoint 并执行飞行控制
```

### 1.2 两个身份命名空间

```text
Axx / task_uav_id
  = 任务层身份，由 runtime、TCP assignments[].uav_id 和 GCS GroupState 使用。
  = `completed_uavs`、`failed_uavs` 等上游结果中的 ID。

UAVn / exec_uav_id
  = 执行层身份，由 executor 绑定的实际 UAV 执行对象使用。
  = 下发 `UavTask` 时写入 `uav_id`，机载 `UavTaskState` 也报告该身份。

exec_target
  = 执行目标/路由字段，与执行层 UAVn 当前通常取相同字符串，
    但它是独立字段，不能和 `uav_id` 在接口定义上合并。
```

`tcp_to_ros` 是两套身份之间的唯一转换边界。executor 和 driver 不需要知道全局 Axx 分配过程，只处理收到的 UAVn 执行请求。

### 1.3 配置映射与当前运行时映射

```text
config map
  = 配置文件提供的初始 Axx -> UAVn 基线。

current map
  = tcp_to_ros 当前任务系统实际使用的运行时映射。
```

`current map`、`assignment snapshot` 和 `candidate version` 是本文目标设计中的概念名，不是当前源码已经存在的同名变量或类型。当前 backend 只有静态 `IdentityMap`、活动任务的 `GroupState.exec_by_uav` 和 Action goal 等隐式状态；后续实现必须明确把这些目标概念落到具体对象，并保持以下语义：当前 command 的执行快照不能被下一条 MOVE_TO 覆盖，MOVE_TO 分配失败不能部分更新运行时映射。当前 backend 还通过单活动任务限制拒绝并发 group command；在该限制取消前，“下一条 MOVE_TO 不覆盖正在执行快照”实际表现为新任务被拒绝，而不是两个 command 并存。

### 1.4 状态机骨架

```text
程序启动
    │
    ▼
INIT
    │ arm + OFFBOARD
    ▼
TAKEOFF
    │ 达到起飞切换条件
    ▼
COMPLETED_HOLD
    │
    ├─ MOVE_TO / FAULT_EXIT
    │       └─ target_pose 或 waypoints → EGO_EXECUTING
    │
    ├─ FOLLOW_ROUTE leader
    │       └─ waypoints → EGO_EXECUTING
    │
    ├─ FOLLOW_ROUTE follower
    │       └─ leader_id + formation_offset → PI_EXECUTING
    │
    ├─ HOVER ───────────────────────► COMPLETED_HOLD
    │
    └─ HOLD / 安全事件 ─────────────► EMERGENCY_HOLD
```

---

## 2. executor 任务接入与反馈

### 2.1 统一任务流程

所有普通单机 command 使用同一基本生命周期：

```text
runtime 任务计划
    → tcp_to_ros 批次校验与（仅 MOVE_TO 的目标）任务分配
    → executor 接收单机 `UavTask` 请求并校验执行层 `uav_id`/`exec_target`
    → executor 保存任务，返回 ACCEPTED
    → tcp_to_ros 通过 `UavTaskControl.operation = START` 启动已接受任务
    → executor 启动对应动作线程
    → executor 向 driver 提供 command 输入
    → driver 自行完成内部状态转换
    → executor 汇总运动结果
    → tcp_to_ros 汇总批次结果并反馈 runtime
```

`ACCEPTED` 表示任务已校验和保存，不表示已经开始运动；`START` 只启动已接受的任务，不重新分配任务，也不重新解析另一份映射。

### 2.2 command 与 driver 输入

| command | executor 提供的主要输入 | driver 状态/动作 | 映射规则 |
|---|---|---|---|
| `MOVE_TO` | 单个 `target_pose`（当前协议语义）；executor 可在内部转换为单点目标或兼容的 waypoints | `EGO_EXECUTING` | 允许按第 6 章尝试更新 `current map` |
| `FAULT_EXIT` | 每架任务独立的 `target_pose` 或 `waypoints` | `EGO_EXECUTING` | 沿用当前运行时映射，不进入新的 MOVE_TO 分配 |
| `FOLLOW_ROUTE` leader | `waypoints` | `EGO_EXECUTING` | 沿用当前运行时映射 |
| `FOLLOW_ROUTE` follower | `leader_id`、`formation_follow`、`formation_offset` | `PI_EXECUTING` | 沿用当前运行时映射和编队关系 |
| `HOVER` | 仅参与 UAV 的 `uav_id`；不携带目标坐标 | `COMPLETED_HOLD` | 沿用当前运行时映射 |
| `HOLD` | 仅参与 UAV 的 `uav_id`；安全保持输入 | `EMERGENCY_HOLD` | 沿用当前运行时映射 |

任务集合外的 UAV 不因当前 `MOVE_TO` 分配而被改写。例如本批只允许 `A01–A12` 与 `UAV1–UAV12` 参与时，`A13–A15` 与 `UAV13–UAV15` 保持原有当前映射。

### 2.3 executor 与 driver 的边界

```text
executor 接收单机 `UavTask`
    ├─ 校验任务、command 和执行层 `uav_id`/`exec_target`
    ├─ 保存当前 command 的执行快照
    ├─ 接收 `UavTaskControl` 的 `START`/`ABORT`
    ├─ 启动或终止对应动作
    └─ 提供 driver 所需输入

executor 不直接写入 driver 的内部状态。

driver 根据收到的有效输入自行转换：
    → target_pose/waypoints → EGO_EXECUTING
    → leader_id + formation_follow/formation_offset → PI_EXECUTING
    HOVER                 → COMPLETED_HOLD
    HOLD/SAFE_HOLD/安全事件 → EMERGENCY_HOLD
```

### 2.4 反馈分层

```text
ego_planner_driver
    → executor 内部运动结果
    → UavTaskState：ACCEPTED / COMPLETED / FAILED
    → tcp_to_ros Action 与 GroupTaskState
    → TCP runtime：ACCEPTED / COMPLETED / FAILED
```

`detail_stage`、`error_code` 和 `message` 用于补充诊断，不直接等同于 driver 内部的 `INIT`、`TAKEOFF`、`EGO_EXECUTING`、`PI_EXECUTING`、`COMPLETED_HOLD` 或 `EMERGENCY_HOLD`。

字段的身份语义随接口变化：TCP/Action goal 的 `assignments[].uav_id` 和 backend `GroupState.uav_ids` 是任务层 Axx；下发到 `UavTask` 的 `uav_id`、`exec_target` 以及机载 `UavTaskState.uav_id` 是执行层 UAVn；GCS 在吸收机载状态时通过活动任务的映射还原为 Axx。机载状态按每个任务层 Axx 的 `status_seq` 去重；TCP final-response cache 当前实际按 `command_id` 去重，不应把两种去重机制写成同一个键。

`UavTaskState.status` 的主状态使用 `ACCEPTED`、`COMPLETED`、`FAILED`；runtime 内部可能使用 `SUCCESS`/`FAILURE`，由 GCS/TCP 边界归一化为线上 `COMPLETED`/`FAILED`。Action 取消路径还可能返回 `CANCELLED`，不得把所有 Action `final_status` 简化成只有三态。`GroupTaskState` 只有 `mission_id`、`command_id`、`command`、状态集合和 `detail`，没有 `exec_target`、`status_seq` 或独立 `error_code`。

---

## 3. ego_planner_driver 状态机

本章使用两套名称：源码事实使用当前 C++/executor 状态名；目标设计使用逻辑状态名。目标设计名不是现有 ROS `/exec_state` 的枚举值，实施时需要显式建立映射，不能直接假定源码已有同名状态。

| 目标逻辑名称 | 当前源码对应 | 说明 |
|---|---|---|
| `INIT` | `IDLE`/启动前置逻辑 | 当前 driver 没有 `INIT` 状态常量；这里仅表示初始化和等待起飞条件 |
| `TAKEOFF` | `TAKEOFF` | 当前源码已有起飞状态 |
| `EGO_EXECUTING` | `EXECUTING` | 目标设计名称；当前普通 waypoint/目标执行发布 `EXECUTING` |
| `PI_EXECUTING` | follower loop | 当前 follower 由 `FOLLOW_ROUTE + formation_follow` 选择，不发布同名 `/exec_state` |
| `COMPLETED_HOLD` | `COMPLETED` 后的 `HOLD` | 当前完成终态和保持动作是两个状态/动作，不存在同名状态 |
| `EMERGENCY_HOLD` | `EMERGENCY_BRAKE` → `BRAKE_HOLD` | 当前先执行制动轨迹，再进入锁存保持；目标名称是统一逻辑抽象 |
| 规划失败/超时 | `EGO_PLAN_FAILED`、`EGO_EXEC_TIMEOUT` | 当前已有的失败终态，目标设计不能遗漏 |

后续状态图描述目标逻辑行为；涉及当前源码字段时，使用上表中的实际名称。`PlanResult::SUCCESS` 只表示 planner 内部计算结果，不等同于 driver `COMPLETED`、executor `COMPLETED` 或 TCP `COMPLETED`。

### 3.1 目标状态范围与源码映射

```text
目标逻辑阶段：
  INIT
  TAKEOFF
  EGO_EXECUTING
  PI_EXECUTING
  COMPLETED_HOLD
  EMERGENCY_HOLD

当前源码还包含：
  IDLE
  EXECUTING
  COMPLETED
  HOLD
  EGO_PLAN_FAILED
  EGO_EXEC_TIMEOUT
  EMERGENCY_BRAKE
  BRAKE_HOLD
```

任务发布前的 assignment 由 `tcp_to_ros` 冻结；driver 状态机不修改 Axx/UAVn 分配。`current map` 和 `assignment snapshot` 仍是第 6 章规定的目标设计对象，不是当前 driver 或 backend 已有字段。

### 3.2 目标状态转换

```text
INIT（当前实现对应 IDLE/启动前置逻辑）
  → arm + OFFBOARD
TAKEOFF
  → 达到起飞切换条件
COMPLETED_HOLD（当前实现对应 COMPLETED 后的 HOLD）
  → MOVE_TO/FAULT_EXIT/FOLLOW_ROUTE leader：EGO_EXECUTING
  → FOLLOW_ROUTE follower：PI_EXECUTING
  → HOVER：保持 COMPLETED_HOLD
  → 安全 HOLD：EMERGENCY_HOLD
EGO_EXECUTING（当前实现对应 EXECUTING）
  → end_pt 到达且稳定：COMPLETED_HOLD
  → BRAKE_REQUEST：EMERGENCY_HOLD
PI_EXECUTING（当前实现为 follower loop）
  → leader 完成、本机到达 offset 且保持满足条件：COMPLETED_HOLD
  → 安全事件：EMERGENCY_HOLD
```

实际实现迁移时，必须同时定义目标逻辑状态与当前 `/exec_state`、executor `UavTaskState.status` 的映射，不得把目标名称直接写入现有协议而不更新接口。

### 3.3 COMPLETED_HOLD（目标逻辑状态）

```text
COMPLETED_HOLD
    ├─ 锁定当前位置和 yaw 的保持 setpoint
    ├─ 持续发布保持控制
    ├─ 等待新的 command
    └─ 收到安全事件后进入 EMERGENCY_HOLD
```

该状态允许后续普通任务进入，但任务的执行身份来自 `tcp_to_ros` 当前运行时映射，不重新恢复 config map。

### 3.4 EGO_EXECUTING（目标逻辑状态）

`EGO_EXECUTING` 的跨层输入必须按 command 区分：

```text
MOVE_TO
    → TCP/Action assignment.target_pose
    → executor/driver 可转换为本机内部目标或 waypoints_

FAULT_EXIT / FOLLOW_ROUTE leader
    → waypoints

FOLLOW_ROUTE follower
    → 顶层 leader_id + assignment.formation_follow/formation_offset
    → 不依赖 leader/follower 的 waypoints 混用
```

其中 `waypoints` 是 TCP/ROS/executor 的接口字段；`keypoint` 是本文对 driver 内部航点推进语义的称呼；C++ 实际存储仍是 `waypoints_` 和 `waypoint_consumed_`。`end_pt` 仅是 planner/driver 内部对最终目标的简称，实际通常来自 `target_pose` 或 `waypoints_.back()`，不是 ROS/TCP 字段。

状态内只负责：

- 维持当前轨迹或首次规划前的 HOLD setpoint；
- 推进未完成 waypoint 和判断最终目标到达；
- 通过单一轨迹更新执行器调用 `replanOnce()`；
- 检查本机轨迹、邻机状态和实际安全距离；
- 在完成、规划失败或安全条件失效时执行规定的动作。

`EGO_EXECUTING` 不负责全局任务分配和第二层主动避让。

### 3.5 PI_EXECUTING（目标逻辑状态）

```text
PI_EXECUTING
    ├─ 读取执行层 leader UAVn 的运动/完成信息
    ├─ 计算 leader + offset 跟随目标
    ├─ 发布 PI 控制 setpoint
    └─ leader 完成且本机达到 offset 后进入 COMPLETED_HOLD
```

跨层字段必须区分：TCP/Action 的 `leader_id` 仍是任务层 Axx；GCS 下发 `UavTask` 前将其转换为执行层 leader UAVn；`formation_follow` 和 `formation_offset` 位于 assignment。当前 driver/executor 用 follower loop 表达这一行为，不发布 `PI_EXECUTING` 这个状态字符串。

leader/follower 关系属于当前任务的执行快照，不因新的 MOVE_TO 分配而在执行中改变。

### 3.6 EMERGENCY_HOLD（目标逻辑状态）

目标逻辑上的安全收口为：

```text
BRAKE_REQUEST
    → EMERGENCY_HOLD
```

当前 C++ driver 的实际路径是：

```text
emergency_pending_ / 安全条件
    → EMERGENCY_BRAKE
    → 制动轨迹耗尽
    → BRAKE_HOLD
```

因此 `EMERGENCY_HOLD` 不是当前 `/exec_state` 的字面量。后续实现若保留该目标名称，必须将它映射到 `EMERGENCY_BRAKE` 与 `BRAKE_HOLD` 的完整过程，而不能在进入 `EMERGENCY_BRAKE` 时直接宣称已经进入最终保持状态。

最终保持阶段的目标行为为：

```text
BRAKE_HOLD / EMERGENCY_HOLD
    ├─ 锁存安全位置或悬停 setpoint
    ├─ 停止正常 EGO 自主导航和 PI 跟随
    ├─ 拒绝普通 command、START 和自动 replan 退出
    └─ 等待遥控器切换 onboard 后人工接管
```

组级安全联动仍由 `tcp_to_ros` 按现有规则处理。

---

## 4. EGO_EXECUTING 内部动作分支

`EGO_EXECUTING` 由三个职责分支共同维持：

```text
                         EGO_EXECUTING
                                │
          ┌─────────────────────┼─────────────────────┐
          ▼                     ▼                     ▼
 A. 轨迹执行分支       B. 请求仲裁/轨迹更新       C. 条件触发分支
 publish setpoint      合并请求并执行一次更新      只检测并产生请求
```

### 4.1 A. 轨迹执行分支

```text
publishSetpoint() 周期运行
    ├─ current_ 存在
    │    ├─ 按执行时间采样轨迹
    │    ├─ 发布 PX4 setpoint
    │    └─ 提供 tracker 所需位姿和执行时间
    └─ current_ 不存在
         └─ 持续发布进入 EGO_EXECUTING 前锁存的 HOLD setpoint
```

A 分支只保证 setpoint 连续，不产生请求、不仲裁请求，也不直接提交新轨迹。

### 4.2 B. 请求仲裁与轨迹更新分支

```text
replanTimer ─────────────► REPLAN_REQUEST ─┐
waypointsTracker ────────► REPLAN_REQUEST  ├─► 请求仲裁器
waypointsTracker ────────► COMPLETE_REQUEST│
collisionChecker ────────► REPLAN_REQUEST  │
safety supervisor ────────► BRAKE_REQUEST ──┘

请求仲裁：BRAKE > COMPLETE > REPLAN
    BRAKE    → EMERGENCY_HOLD
    COMPLETE → COMPLETED_HOLD
    REPLAN   → REPLAN 合并槽 → 单次 replanOnce(snapshot)
```

`BRAKE` 和 `COMPLETE` 不调用 `replanOnce()`。只有获准的 `REPLAN_REQUEST` 才进入第 5 章定义的轨迹更新执行器。

`collisionChecker`、`REPLAN_REQUEST`、`COMPLETE_REQUEST`、`BRAKE_REQUEST` 和“单一轨迹更新执行器”是本文的目标逻辑名称，不是当前源码中同名的类、枚举或消息。当前实现中较接近的入口分别是 `collisionCheckTick()`/安全 supervisor、`planning_requested_`、到达检查、`emergency_pending_` 和 `replanLoop()`。后续实现可以沿用这些目标职责，但必须在代码中明确对应关系，不能把本文名称当成已经存在的 API。

### 4.3 C. 条件触发分支

C 分支只产生请求，不修改 `current_`，不直接提交轨迹。

#### 4.3.1 replanTimer

```text
current_ 不存在
    → 产生 REPLAN_REQUEST
    → 用于建立首条轨迹

current_ 存在且 trajectory_age >= 1s
    → 产生 REPLAN_REQUEST
    → 用于更新执行中轨迹

pending 或 replan_running
    → 抑制重复请求
```

首次轨迹和执行中更新共用同一个 `replanOnce()` 输入/输出契约。首条轨迹成功提交前，A 分支继续发布锁存 HOLD setpoint。

#### 4.3.2 waypointsTracker

```text
当前 waypoint 到达
    → 推进执行索引
    → REPLAN_REQUEST

最终 end_pt 到达且满足稳定条件
    → COMPLETE_REQUEST
```

辅助前视目标的计算不等于 waypoint 已消费；只有真实 odometry 满足到达阈值才推进执行索引。

#### 4.3.3 collisionChecker

```text
收到合法的新邻机 trajectory_intent 或有效规划输入变化
    ├─ 当前轨迹仍满足安全约束
    │    └─ 不产生请求或按需要产生 REPLAN_REQUEST
    └─ 预测/实际安全边界即将违反或已经违反
         └─ BRAKE_REQUEST
```

`collisionChecker` 不再负责选择 STAND_ON/YIELD，也不触发机载第二层主动避让。发布前的主要多机冲突由第 6 章的 `tcp_to_ros` 任务分配处理；运行时检查只负责发现新的有效输入和安全失效。

#### 4.3.4 Safety supervisor

```text
读取本机和邻机 fresh pose
    → 直接检查当前中心距离
    → 触发紧急距离门禁时产生 BRAKE_REQUEST
```

目标设计中安全 supervisor 负责最高优先级安全请求；当前源码对应 `SafetyPredictor`、`runSafetySupervisor()`/`safetySupervisorTick()` 和 `emergency_pending_`，不调用 `replanOnce()`，不直接修改 `current_`，不等待普通规划重试。

### 4.4 请求优先级

```text
BRAKE_REQUEST > COMPLETE_REQUEST > REPLAN_REQUEST
```

```text
存在 BRAKE_REQUEST
    → 直接进入 EMERGENCY_HOLD

否则存在 COMPLETE_REQUEST
    → 进入 COMPLETED_HOLD

否则存在 REPLAN_REQUEST
    → 进入 REPLAN 合并槽和轨迹更新执行器

没有请求
    → 保持 EGO_EXECUTING
```

该优先级防止完成事件掩盖安全事件，也防止已经到达最终目标后继续普通 replan。

---

## 5. 单层轨迹更新动作分支设计

本章定义 B 分支的最小可实施结构。轨迹更新的目标是生成并提交本机 nominal 轨迹，不承担任务分配或全局多机冲突求解。

### 5.0 已确定设计边界

```text
1. MOVE_TO 的主要多机冲突在 tcp_to_ros 发布前处理。
2. ego_planner_driver 不修改 Axx/UAVn assignment。
3. replanOnce() 单次、有界，不保证每次成功。
4. 规划失败不强行提交退化或不安全轨迹。
5. 有效 current_ 保留并继续执行；没有 current_ 时保持 HOLD。
6. BRAKE/COMPLETE 由请求仲裁器处理，不进入 replanOnce()。
7. REPLAN 请求合并，不建立 FIFO。
8. 同一时刻只执行一个 replanOnce()。
9. 输入版本变化时，过期候选只能丢弃，不能提交。
10. 轨迹更新失败不直接承担状态机终态转换。
```

### 5.1 请求合并、退避和版本检查

请求先经过仲裁器；只有获准的 `REPLAN_REQUEST` 进入合并槽。

```text
REPLAN_REQUEST
    → pending = true
    → 记录会影响规划有效性的 input_version
    → 重复请求合并，不建立 FIFO
```

最小运行状态：

```cpp
struct ReplanSlot {
  bool pending{false};
  uint64_t input_version{0};
};

bool replan_running{false};
Time retry_not_before;
```

执行条件：

```text
pending == true
replan_running == false
且：
  相同输入的 retry_not_before 已到达，或
  input_version 相对上次失败发生变化
```

执行器执行一次：

```text
设置 replan_running = true
    → 复制最新 pose、目标、路径和邻机安全快照
    → 记录 started_input_version
    → 调用 replanOnce(snapshot)
```

只有版本一致才允许提交：

```text
候选成功
    → started_input_version == input_version
        → 提交 current_
        → 更新激活时间
        → 发布 trajectory_intent
        → 清除本次 pending 和失败退避

    → started_input_version != input_version
        → 丢弃候选
        → 不更新激活时间
        → 不发布 intent
        → 保留新 pending
        → 返回轨迹更新执行器重新调度
```

规划失败时：

```text
保留有效 current_ 或 HOLD setpoint
    → 设置 retry_not_before
    → replan_running = false
    → 等待执行器再次调度
```

`input_version` 只在会影响候选有效性的语义变化时递增，例如路径替换、当前 waypoint 推进、有效邻机安全输入变化或任务替换；不应因每次普通 odometry 更新而递增。

`input_version`、`started_input_version`、`ReplanSlot`、`retry_not_before` 和“轨迹更新执行器”是本章的目标设计对象；当前 `replanOnce()` 仍声明为 `bool replanOnce(bool timed_trigger = false)`，planner 内部实际返回 `PlanResult::SUCCESS`、`DEGENERATE_SEGMENT`、`RETRYABLE_FAILURE` 或 `RESOURCE_LIMIT`。实施时需要增加明确的适配层，不得把本文事务结果名称误认为当前 C++ 返回值。

### 5.2 单层 nominal `replanOnce()`

```text
replanOnce(snapshot)
        │
        ▼
选择规划起点
    ├─ current_ 在当前时刻可采样
    │    → 使用旧轨迹 P/V/A
    └─ 没有可用 current_
         → 使用实际 pose、零或安全初始 V/A
        │
        ▼
选择当前未完成 keypoint
        │
        ▼
沿当前 route segment 选择最多约 5 m 的局部目标
        │
        ├─ 下一 keypoint 距离 > 5 m
        │    → 当前点沿该 segment 方向前视约 5 m
        └─ 下一 keypoint 距离 <= 5 m
             → 目标为下一 keypoint
        │
        ▼
判断最终段
    ├─ 非最终段：按当前 segment 方向设置目标速度
    └─ 最终段：目标为 end_pt，末速度为 0
        │
        ▼
core_->planOnce(...)
        │
        ▼
finite / 资源 / 动力学 / clearance 检查
        │
        ├─ 失败 → 返回 NO_SOLUTION 或 RESOURCE_ERROR
        └─ 成功 → 返回 nominal 候选轨迹
```

5 m 是本次规划的辅助局部目标，不是任务完成目标，也不直接消费 waypoint。只有真实 odometry 到达当前 waypoint 的约 `0.5 m` 阈值后，`waypointsTracker` 才推进执行索引。局部目标不得跨越当前未完成的 route keypoint；`end_pt` 始终是唯一最终目标。

如果当前 `max_advance_dist_m` 与 5 m 目标选择语义冲突，应拆分为明确的 `local_target_lookahead_m` 与 `waypoint_reach_thresh_m`，或删除不再需要的旧参数，不能让两个参数同时隐式控制目标选择和执行索引。

本流程不包含：

```text
STAND_ON / YIELD
SLOW / LEFT / RIGHT
随机避让点
移动 keypoint 后的第二层候选搜索
已生成 TimedTrajectory 的直接横向或时间变形
```

### 5.3 更新结果与失败行为

轨迹更新执行器应能够区分至少以下结果：

```text
UPDATED
    候选通过检查且 input_version 一致，已提交 current_。

KEEP_CURRENT
    本次没有新轨迹，但已有 current_ 仍有效，继续执行并退避。

HOLD
    没有 current_ 或无法安全继续，保持锁存 HOLD setpoint。

STALE
    候选计算完成但 input_version 已变化，丢弃并保留 pending。

NO_SOLUTION
    nominal 规划不可行，未提交新轨迹。

RESOURCE_ERROR
    规划资源、输入或运行预算不足，未提交新轨迹。
```

这些结果描述轨迹更新事务，不直接决定状态机终态：

```text
轨迹失败且 current_ 安全
    → KEEP_CURRENT + retry_not_before

轨迹失败且没有 current_
    → HOLD + retry_not_before

实际安全边界失效
    → BRAKE_REQUEST → EMERGENCY_HOLD

BRAKE/COMPLETE 已被仲裁接受
    → 丢弃普通 replan 结果，不提交到新状态
```

---

## 6. tcp_to_ros 的 MOVE_TO 任务分配与失败反馈

### 6.1 适用范围与职责

**当前实现事实：** `tcp_to_ros` 目前通过静态 `IdentityMap` 建立 `GroupState.exec_by_uav`，尚未实现 `current map`、MOVE_TO 候选分配或映射版本。以下 `current map` 和候选流程是目标设计，不能当作当前代码已经提供的能力。

```text
runtime
    → 只发布任务层 Axx，不关心 UAVn 的具体分配

tcp_to_ros
    → 目标设计：维护 current map
    → 仅对 command == MOVE_TO 的当前任务集合尝试任务分配
    → 读取一致的 UAV 状态和路径输入
    → 验证分配可行性
    → 成功后发布对应 UAVn 任务
    → 无有效分配时在 PREPARE 前向 runtime 返回任务失败

swarm_uav_executor / ego_planner_driver
    → 只执行已下发的 UAVn 任务
    → 不修改任务分配
    → 不知道全局 Axx 重排过程
```

例如，`prepare` 若只包含 `A01–A12`，分配算法只允许在 `A01–A12` 与 `UAV1–UAV12` 的闭集内工作；`A13–A15` 和 `UAV13–UAV15` 不参与，也不被改写。

新的 MOVE_TO 分配用于替代目标设计中的 MOVE_TO 发布前 `t_off` 启动时间协调；但当前源码仍会在 backend 中按 `SCHEDULE_*` 规则为 `MOVE_TO` 和 `FAULT_EXIT` 计算 `schedule_tau`，并在全部 PREPARE accepted 后按独立 worker 延迟 START。迁移完成前，不能把 `t_off` 说成已删除；也不能让动态 MOVE_TO 分配和旧的 `schedule_tau` 同时承担同一批次的冲突协调。非 `MOVE_TO` 不进入新的分配算法；`FAULT_EXIT` 当前仍可能进入旧排程，是否移除必须单独实施和测试。

### 6.2 current map 的持久运行时语义

配置映射只提供初始值；任务发布后由 `tcp_to_ros` 维护 current map：

```text
启动/初始化
    → 从 config map 建立 current map

MOVE_TO
    → 基于 current map 和本批参与集合生成候选
    → 候选完整有效后原子更新 current map

非 MOVE_TO
    → 直接沿用 current map
    → 不重新加载 config map
    → 不恢复此前 MOVE_TO 已成功产生的映射
```

必须遵守：

- 不修改、覆盖或写回 `uav_identity_map.yaml`；
- 不把 config map 当作每条 command 的恢复目标；
- MOVE_TO 失败时保持旧 current map 不变；
- MOVE_TO 候选验证成功并提交 current map 后，即使后续 PREPARE/START 失败，也不得恢复 config map 或提交前的旧映射；
- MOVE_TO 成功后，新的 current map 持续用于后续 command；
- 正在执行的 command 使用其开始时冻结的 assignment snapshot；
- 新 MOVE_TO 不得覆盖正在执行 snapshot；
- current map 的更新只能整批原子提交，不能部分更新后继续尝试。

### 6.3 MOVE_TO 分配范围和合法性

分配算法只在当前 MOVE_TO 的闭集参与者内运行：

```text
本次任务 assignments
    → 任务参与集合 A_set
    → 对应执行集合 U_set
    → 只允许 A_set ↔ U_set 内的一一重新分配
```

至少检查：

- 每个任务层 Axx 只出现一次；
- 每个执行层 UAVn 只绑定一个 Axx；
- 候选不引入当前任务集合外的 UAV；
- 所有执行目标来自已知、启用且健康的注册信息；
- `exec_target`、Service、状态话题和 pose 来源相互一致；
- 当前 UAV 实际位置到 MOVE_TO 路径的接入段也纳入冲突检查；
- 路径、能力、状态、超时和计算资源满足本批任务要求；
- 候选版本、参与集合和 assignment snapshot 一起冻结。

分配不是修改实体 UAV 的身份。`UAVn` 到实际主机、IP、能力和通信对象的物理注册仍保持稳定。

### 6.4 MOVE_TO 分配与发布事务

```text
接收 MOVE_TO
    │
    ▼
读取 current map 和一致的 pose/状态/路径快照
    │
    ▼
在当前参与闭集内生成 Axx -> UAVn 候选
    │
    ▼
检查一一映射、接入段、主路径、能力、状态和预算
    │
    ├─ 无有效候选
    │    → 第 6.5 节的原子失败
    │
    └─ 候选有效
         → 原子更新 current map
         → 冻结本 command 的 assignment snapshot
         → 仅使用该 snapshot 进入 PREPARE
         → 全部 PREPARE accepted 后并发 START
         → 执行期间不改变该 snapshot
```

候选生成和检查必须使用一致输入快照。不能在候选计算期间混用不同时间的 UAV pose、任务路径或映射版本。

### 6.5 无有效分配时的原子失败

以下任一情况都必须在 PREPARE 前失败：

```text
候选为空
路径或接入段冲突不可行
pose 缺失、过期或无法转换
UAV 能力/状态不匹配
参与集合或一一映射非法
计算资源上限或 timeout 预算不足
```

失败动作固定为：

```text
丢弃候选
    → current map 保持旧值
    → 不发送任何 PREPARE
    → 不发送任何 START
    → 不产生部分成功
    → 直接向 runtime 返回 FAILED
```

该语义与当前 `t_off` 排程前置失败一致：尚未向机载发布任务，因此不能把分配失败伪装成某架 UAV 的执行失败，也不需要对未启动任务发送 HOLD。

### 6.6 MOVE_TO 的时间和发布约束

**当前源码事实：** `gcs_a_backend_node.py` 将排程结果存为 `schedule_tau`；`t_off` 是设计/日志术语，不是 `ExecuteGroupTask`、`UavTask` 或 `TaskAssignment` 的字段。当前流程为：

```text
计算 schedule_tau
    → 全部 PREPARE
    → PREPARE 全部 accepted
    → 每架按 t0 + schedule_tau[Axx] 独立 worker 调用 START
```

**目标设计约束：** MOVE_TO 动态分配验证成功后，不再让旧 `schedule_tau` 与新分配同时承担同一批次的主要冲突协调。迁移期间若仍保留旧排程，必须明确它是兼容路径，并把等待时间纳入 timeout 预算；不能把当前实现误写成“已并发无偏移 START”。

不论采用哪条迁移路径，都必须保证：

```text
有效分配/兼容排程成功并冻结执行快照
    → 全部 PREPARE
    → 全部 PREPARE accepted
    → 按明确的 START 规则发布
```

无有效分配或排程失败时，不得继续发送任务。

### 6.7 失败反馈契约

第 6.5 节是目标设计的反馈契约；当前代码的实际状态要单独说明：TCP 解析拒绝路径可以直接发布 `ros_status = REJECTED`，但 backend 内部的排程失败是在 Action goal 已发送后通过 `set_aborted()` 返回，当前 TCP 收口通常会发布 `ros_status = FAILED`。后续实现若要把 MOVE_TO 分配失败统一标为 `REJECTED`，必须显式增加该错误路径，不能仅修改本文文字。

目标设计的分配或发布前验证失败反馈为：

```text
Action result
    final_status = FAILED
    failed_uavs = 原任务层 Axx，非空、无重复
    error_code = 明确的 mapping/pose/infeasible/timeout 错误码
    message = 简短原因

GroupTaskState
    tcp_status = FAILED
    ros_status = REJECTED（仅在实现显式区分前置拒绝时）
    failed_uavs = 同一批 Axx
    detail = 诊断摘要

TCP runtime
    status = FAILED
    failed_uavs = 只包含本次 assignments 的 Axx
```

当前 `GroupTaskState` 没有独立的 `error_code` 字段；映射失败、排程失败和执行失败的细节需要放入 `detail`，或通过 Action/TCP 的 `error_code` 传递。

`REJECTED` 不是当前 backend 统一的 group 状态：TCP 解析拒绝路径可以使用它，但 Action/backend 的前置排程失败通常以 `final_status = FAILED` 收口并发布 `ros_status = FAILED`。如果目标设计要求 `ros_status = REJECTED`，必须新增并测试显式前置拒绝路径。

`FAILED` 是当前 TCP/ROS 任务协议的失败状态；`FAILURE` 仅在 runtime 或自然语言语境中表示内部失败概念，不作为本文新增的主协议枚举。

当前 `t_off` 失败码实际是 `SCHEDULE_POSE_UNAVAILABLE`、`SCHEDULE_GEOMETRY_UNSUITABLE`、`SCHEDULE_PAIR_SAFE_SET_EMPTY`、`SCHEDULE_INFEASIBLE`、`SCHEDULE_LIMIT_REACHED`、`SCHEDULE_TIMEOUT_INSUFFICIENT`，并且旧 scheduler 同时服务 `MOVE_TO`/`FAULT_EXIT`。第 6 章后续若增加 `MOVE_TO_ASSIGNMENT_*` 错误码，必须同时修改 parser/backend/TCP 映射和测试；本节列出的名称是目标设计候选，不是当前已有常量。

详细候选、冲突区间、计算耗时、映射版本和预测距离写入日志或测试记录，不改变线上主状态协议。

### 6.8 执行阶段失败与前置分配失败的区别

```text
前置分配失败
    → 尚未 PREPARE
    → 直接 FAILED
    → current map 不变
    → 不需要向未启动 UAV 发送 HOLD

已 PREPARE/START 后执行失败
    → 接收 executor 的 FAILED 或安全状态
    → GCS 汇总 failed_uavs
    → 按现有规则对其他已启动成员执行安全 HOLD
    → 向 runtime 返回整组 FAILED
```

`EGO_PLAN_FAILED`、`EMERGENCY_HOLD`、链路丢失和执行超时属于执行阶段失败，不能与发布前分配失败混淆。

### 6.9 非 MOVE_TO 的映射连续性

必须保证以下行为：

```text
MOVE_TO 成功更新 current map
    → 后续 FOLLOW_ROUTE / FAULT_EXIT / HOVER / HOLD
      继续使用 current map
    → 不重新加载 config map
    → 不发生 Axx/UAVn 映射突变

MOVE_TO 分配失败
    → current map 保持失败前的值
    → 后续 command 继续使用失败前的 current map
```

非 `MOVE_TO` 不应因为新 MOVE_TO 的候选算法而引入新的任务分配、回滚或跨集合改写。`FOLLOW_ROUTE` 的 leader/follower 关系尤其不能被隐式替换。

### 6.10 最低回归测试要求

后续实现至少需要验证：

- `MOVE_TO` 成功后 current map 持续影响后续非 `MOVE_TO`；
- `MOVE_TO` 失败时 current map 完全不变；
- `A01–A12` 的分配不影响 `A13–A15/UAV13–UAV15`；
- 无有效分配时不产生任何 PREPARE/START；
- `failed_uavs` 使用任务层 Axx，且只来自原 assignments；
- 正在执行的 assignment snapshot 不被下一条 MOVE_TO 覆盖；
- `FOLLOW_ROUTE` 的 leader/follower 不被 MOVE_TO 候选污染；
- status sequence 去重、终态重发、组级 FAILED/HOLD 收口继续有效；
- 日志能够区分 mapping failure、schedule failure 和 executor/driver execution failure。

---

## 7. 实施顺序

```text
1. 先在 tcp_to_ros 实现 current map、MOVE_TO 闭集分配和原子失败反馈。
2. 再收紧 MOVE_TO 的 t_off/旧排程入口，避免同一批任务同时使用两套协调机制。
3. 将 ego_planner_driver 的 replanOnce() 收敛为单层 nominal 更新。
4. 删除或停用第二层 SLOW/LEFT/RIGHT 候选路径。
5. 按第 4、5 章补齐状态机请求合并、版本检查和失败结果语义。
6. 完成跨层回归测试后，再更新运行记录和部署参数。
```

本设计的最终职责划分为：

```text
任务发布前：tcp_to_ros 选择并验证 MOVE_TO 的执行分配。
任务发布后：executor 按冻结 snapshot 管理单机任务。
本机执行期间：ego_planner_driver 负责 nominal 轨迹、setpoint 连续性和安全退出。
```
