# 当前源码结构与运行流程（2026-09-06）

本文用于说明当前 `/src/` 内各功能包从任务接入到 PX4 setpoint 输出的实际结构和运行流程，服务于后续判断 `tcp_to_ros`、`swarm_uav_executor`、`ego_planner_driver` 及相关桥接节点哪些结构或运行逻辑需要修改。

本文是**当前源码结构说明**，不是新的状态机设计方案。它与
`notes/offboard_new_state_machine_260902.md` 的关系是：

- 本文记录当前代码已经存在的节点、字段、线程、状态和调用顺序；
- `offboard_new_state_machine_260902.md` 中的目标状态、`ReplanSlot`、`input_version`、动态 `current map` 等，如果当前源码没有对应实现，本文只作为“目标/差异”引用，不当作当前能力；
- 当前源码、launch、配置和接口发生变化后，应以源码重新核对本文。

---

## 0. 文档用途、范围与阅读规则

### 0.1 覆盖范围

本文覆盖以下层次：

```text
runtime / TCP 对端
        │
        ▼
tcp_to_ros
        │  ROS Action、Service、Topic、跨 Master bridge
        ▼
swarm_uav_executor
        │  单机任务生命周期与 MotionDriver 适配
        ▼
EgoSwarmDriver（Python）
        │  goal/waypoints/follower PI/状态等待
        ▼
ego_planner_driver（C++）
        │  nominal planner、replan、安全监督、EGO setpoint candidate
        ▼
setpoint_relay + pose_relay + MAVROS
        │
        ▼
PX4
```

同时纳入当前运行不可省略的支撑包：

- `swarm_uav_interfaces`：Service、Message 和 Action 中使用的接口定义；
- `swarm_topology_bridge`：多 Master 之间的 ROS Topic/Service 透明代理；
- `swarm_simulation`：SITL/多机启动与仿真拓扑，不负责任务语义；
- MAVROS/PX4：本机状态、位姿、offboard setpoint 和飞控执行边界。

### 0.2 事实等级

本文使用三种标记：

```text
[源码事实]
    可以直接在当前文件、函数、接口或 launch 中找到。

[运行推导]
    由多个源码文件连接得到的调用/消息关系；不是单个字段的直接声明。

[待确认]
    当前日志、rosbag、跨 Master 时序或硬件行为不足以完全证明，
    需要通过测试、录包或运行时观测确认。
```

如果本文的文字解释与源码发生冲突，以当前源码为准，并应将冲突记录为文档问题，而不是默认源码已经符合本文。

### 0.3 不应混用的概念

以下名称属于不同层次：

| 名称 | 所属层次 | 当前含义 |
|---|---|---|
| `A01`、`A02` | runtime/TCP 任务层 | `assignments[].uav_id` 中的任务身份 |
| `UAV1`、`UAV2` | executor/driver/ROS 执行层 | 实际单机执行身份 |
| `exec_target` | executor/bridge 路由层 | 当前请求应发送到哪个执行节点，通常为 `UAVn` |
| `mission_id` | 任务实例层 | 任务/批次所属的 mission 标识 |
| `command_id` | 线上命令实例层 | 一次整组命令的唯一标识 |
| `command` | TCP/ROS 任务层 | `MOVE_TO`、`FOLLOW_ROUTE`、`FAULT_EXIT`、`HOVER`、`HOLD` 等 |
| `status` | 协议/任务层 | `ACCEPTED`、`COMPLETED`、`FAILED` 等任务状态 |
| `PlanResult::SUCCESS` | C++ planner 层 | 一次轨迹计算结果，不等于任务完成 |
| `exec_state` | ego driver 层 | 当前 C++ driver 的运行状态字符串 |
| `schedule_tau` / `t_off` | `tcp_to_ros` backend 内部 | 任务启动时间偏移，不是 planner clearance |
```

特别注意：

```text
PlanResult::SUCCESS
    ≠ ego_planner_driver 的 COMPLETED
    ≠ swarm_uav_executor 的 COMPLETED
    ≠ ExecuteGroupTask 的 COMPLETED
    ≠ TCP runtime 的 COMPLETED
```

### 0.4 当前代码与旧状态机文档的关系

当前 `/exec_state` 由 C++ driver 发布的实际状态包括：

```text
IDLE
HOLD
TAKEOFF
EXECUTING
COMPLETED
EGO_PLAN_FAILED
EGO_EXEC_TIMEOUT
EMERGENCY_BRAKE
BRAKE_HOLD
```

旧文档中的以下名称是逻辑设计名或描述性阶段，不能直接当作当前 `/exec_state` 字符串：

```text
INIT
EGO_EXECUTING
PI_EXECUTING
COMPLETED_HOLD
EMERGENCY_HOLD
```

当前映射只能描述为：

```text
INIT              → 当前启动/IDLE 前置过程，不是实际状态字符串
EGO_EXECUTING     → 当前 EXECUTING 的逻辑描述
PI_EXECUTING      → executor 内 follower loop，不由 C++ exec_state 发布
COMPLETED_HOLD    → 当前 COMPLETED 后的 steady/HOLD 行为描述
EMERGENCY_HOLD    → 当前 EMERGENCY_BRAKE → BRAKE_HOLD 的目标抽象
```

---

## 1. 包、节点、Master 与身份边界

### 1.1 当前包职责

```text
┌────────────────────────────────────────────────────────────────────────┐
│ runtime / TCP 对端                                                     │
│  发送 JSON line COMMAND，接收 STATUS                                   │
└──────────────────────────────┬─────────────────────────────────────────┘
                               │ TCP
                               ▼
┌────────────────────────────────────────────────────────────────────────┐
│ tcp_to_ros                                                              │
│  tcp_server_node.py                                                     │
│    JSON 解析、命令去重、活动命令限制、TCP STATUS                       │
│                                                                          │
│  gcs_a_backend_node.py                                                  │
│    ExecuteGroupTask Action server、Axx→UAVn 映射、排程、PREPARE/START   │
│    单机状态聚合、GroupTaskState、EmergencyHold                         │
└──────────────┬─────────────────────────────┬───────────────────────────┘
               │ 本地 ROS Action             │ 跨 Master ROS Service/Topic
               ▼                             ▼
┌──────────────────────────────┐   ┌─────────────────────────────────────┐
│ ExecuteGroupTask Action      │   │ swarm_topology_bridge                 │
│ /group_a/execute_task        │   │ Topic PUB/SUB + Service DEALER/ROUTER │
└──────────────┬───────────────┘   └──────────────────┬──────────────────┘
               │                                      │
               ▼                                      ▼
┌────────────────────────────────────────────────────────────────────────┐
│ 每架 UAV 的 swarm_uav_executor                                         │
│  uav_executor_node.py                                                   │
│    UavTask / UavTaskControl / UavHold Service                           │
│    TaskStore、StateReporter、UavTaskExecutor                            │
│                                                                          │
│  EgoSwarmDriver                                                         │
│    C++ driver 适配、任务分层、FOLLOW_ROUTE follower PI、结果等待        │
└──────────────┬──────────────────────────────┬─────────────────────────┘
               │ ROS task input               │ candidate / state / pose
               ▼                              ▼
┌──────────────────────────────┐   ┌─────────────────────────────────────┐
│ ego_planner_driver           │   │ pose_relay / setpoint_relay           │
│ C++ planner + safety         │   │ 位姿中继、候选 setpoint 仲裁           │
└──────────────┬───────────────┘   └──────────────────┬──────────────────┘
               │ /setpoint/ego                         │ /mavros/setpoint_raw/local
               └───────────────────────────────────────┘
                                      │
                                      ▼
                               ┌────────────┐
                               │ MAVROS/PX4 │
                               └────────────┘
```

### 1.2 启动形态

当前 EGO 单机 launch 为：

`src/swarm_uav_executor/launch/uav_executor_ego.launch`

它同时启动：

1. `ego_planner_driver_node`；
2. `pose_relay_node.py`；
3. `setpoint_relay_node.py`；
4. `uav_executor_node.py`，driver 设置为 `ego_swarm`。

GCS 侧 launch 为：

`src/tcp_to_ros/launch/gcs_a_backend.launch`

它启动：

1. `gcs_a_backend_node.py`；
2. `gcs_a_calibration_node.py`。

TCP 接入节点通常由对应的 TCP launch 启动，入口是：

`src/tcp_to_ros/scripts/tcp_server_node.py`

### 1.3 Master 与命名空间

当前 EGO launch 的设计前提是：每架 UAV 的本机 Master 内，MAVROS、executor、relay 和 C++ driver 使用同一组本地接口；机间消息通过 `swarm_topology_bridge` 转发。

```text
本机 Master：
  /mavros/*
  /local_pose
  /exec_state
  /goal
  /waypoints
  /setpoint/ego
  /setpoint/follower
  /mavros/setpoint_raw/local

跨机转发后的远端消息：
  /UAVn/trajectory_intent
  /UAVn/mavros/local_position/odom
  /UAVn/uav_task_state
  /UAVn/mavros/state
  /UAVn/local_pose
```

当前 C++ driver 的公共接口使用 `ros::NodeHandle nh_` 发布/订阅，算法参数使用私有参数 `~`。`uav_id` 和 `exec_target` 放在公共参数空间，供 Python driver 和拓扑发现。

### 1.4 身份流转

当前 backend 从 `uav_identity_map.yaml` 加载任务身份到执行目标的映射：

```text
TCP assignments[].uav_id = A01
        │
        │ IdentityMap.resolve()
        ▼
GroupState.exec_by_uav[A01] = UAV1
        │
        ├─ Service 路径：/UAV1/uav_task
        ├─ Service 路径：/UAV1/uav_task_control
        └─ Topic 路径：/UAV1/uav_task_state
```

下发到单机 executor 后，请求内的 `uav_id` 和 `exec_target` 都是执行层 `UAV1`。收到单机状态后，backend 根据 `GroupState.exec_by_uav` 将执行层身份映射回任务层 A01，再进行整组聚合。

当前代码存在的是静态 `IdentityMap` 和每个活动任务的 `exec_by_uav`，不是文档目标中的动态 `current_map`、assignment version 或 MOVE_TO 闭集动态重排。

---

## 2. 接口合同与数据流

### 2.1 TCP JSON 命令

`tcp_to_ros/scripts/tcp_to_ros/protocol.py` 当前支持：

```text
MOVE_TO
FOLLOW_ROUTE
FAULT_EXIT
HOVER
HOLD
```

命令基本字段：

```json
{
  "version": "1.0",
  "type": "COMMAND",
  "mission_id": "...",
  "group_id": "GroupA",
  "command_id": "...",
  "command": "MOVE_TO",
  "timeout_s": 150,
  "assignments": []
}
```

命令输入语义：

| command | 每个 assignment 的当前输入 |
|---|---|
| `MOVE_TO` | `uav_id` + `target_pose` |
| `FOLLOW_ROUTE` leader | `uav_id` + `waypoints`；顶层 `leader_id` 必须属于 assignments |
| `FOLLOW_ROUTE` follower | `uav_id` + `formation_follow=true` + `formation_offset`；顶层 `leader_id` 指向 leader |
| `FAULT_EXIT` | `uav_id` + `waypoints` |
| `HOVER` | 参与 UAV；不依赖普通 waypoints |
| `HOLD` | 参与 UAV + `reason`，是安全/中止语义，不等于普通 HOVER |

`hold-a-end` 是 runtime 内部计划名称时，线上 command 仍可能是普通 `HOVER`；不能根据计划名把它写成 TCP `HOLD`。

### 2.2 Action 接口

`src/tcp_to_ros/action/ExecuteGroupTask.action`：

```text
Goal:
  mission_id
  group_id
  command_id
  command
  timeout_s
  leader_id
  TaskAssignment[] assignments

Result:
  final_status
  completed_uavs[]
  failed_uavs[]
  error_code
  message

Feedback:
  phase
  active_uavs[]
  detail
```

`tcp_server_node.py` 是 TCP 与 Action 的边界：

```text
TCP JSON line
    → parse_json_line()
    → ParsedCommand
    → goal_from_command()
    → ExecuteGroupTask Action goal
```

Action 完成后，再转换为 TCP `STATUS`：

```text
Action COMPLETED / FAILED / CANCELLED
    → build_completed() / build_failed()
    → TCP JSON STATUS
```

### 2.3 单机 Service 和状态 Topic

`src/swarm_uav_interfaces` 定义：

```text
UavTask.srv
  request: protocol_version, mission_id, group_id, command_id,
           uav_id, exec_target, command, timeout_s, leader_id, assignment
  response: accepted, status, error_code, message

UavTaskControl.srv
  request: protocol_version, operation, mission_id, command_id,
           uav_id, exec_target, reason
  response: accepted, status, error_code, message

UavHold.srv
  request: protocol_version, mission_id, command_id,
           uav_id, exec_target, reason
  response: accepted, error_code, message

UavTaskState.msg
  protocol_version, mission_id, command_id,
  uav_id, exec_target, command, status,
  status_seq, detail_stage, error_code, message, reported_at
```

当前 EGO executor 的 launch 设置：

```text
require_explicit_start = true
supported_commands = MOVE_TO, FOLLOW_ROUTE, FAULT_EXIT, HOVER
```

因此 `HOLD` 是 executor 的独立 Service，不是 EGO launch 中普通任务 command 的一个执行 driver 分支。

### 2.4 轨迹 intent 接口

`swarm_uav_interfaces/msg/UavTrajectoryIntent.msg` 当前字段：

```text
protocol_version
phase                 # UNKNOWN / TENTATIVE / ACTIVE / BRAKING
uav_id
exec_target
frame_id
traj_id
stamp
float64[] t           # 相对 stamp，严格递增，t[0] == 0
geometry_msgs/Pose[] sampled_traj
clearance
```

它不是任务状态，也不是 task result。它用于机间交换本机未来轨迹意图，供邻机 collision checker 和 SafetyPredictor 使用。

### 2.5 状态层次

```text
C++ planner:
  PlanResult::SUCCESS / DEGENERATE_SEGMENT /
  RETRYABLE_FAILURE / RESOURCE_LIMIT

Ego driver:
  /exec_state = EXECUTING / COMPLETED / EGO_PLAN_FAILED / ...

Executor:
  UavTaskState.status = ACCEPTED / COMPLETED / FAILED

GCS Action:
  final_status = COMPLETED / FAILED / CANCELLED

TCP runtime:
  STATUS.status = ACCEPTED / COMPLETED / FAILED / ...
```

各层有独立生命周期，不能把一层的成功直接替换为另一层的成功。

---

## 3. `tcp_to_ros` 当前运行流程

### 3.1 TCP server：连接、解析与去重

`tcp_server_node.py` 当前流程：

```text
监听 host:port
    │
    ▼
接收 newline-delimited JSON
    │
    ├─ JSON/协议错误
    │     ├─ 可提取 command_id/assignments
    │     ├─ 回复 TCP FAILED
    │     └─ 发布 GroupTaskState(REJECTED/FAILED 诊断)
    │
    └─ parse_json_line()
          │
          ├─ CommandCache 已有最终响应 → 原样重发
          │
          ├─ command == HOLD → EmergencyHold 路径
          │
          ├─ 当前有其他活动整组命令
          │     └─ COMMAND_ALREADY_ACTIVE
          │
          └─ ActiveTaskRegistry.start()
                │
                ▼
          ActionClient.send_goal()
```

当前 TCP server 的去重 key 主要使用 `command_id`；backend/单机 executor 的任务 key 则包含 `(mission_id, command_id, uav_id)`。这两个去重层不能混写。

### 3.2 GCS backend 初始化

`gcs_a_backend_node.py` 初始化：

1. 读取 Action、EmergencyHold、确认重试、排程和链路健康参数；
2. 加载 `IdentityMap`；
3. 加载 formation offset、参与者集合、leader 和 mission anchor；
4. 创建 `UavLinkMonitor`；
5. 订阅每个执行目标的：
   - `/{exec_target}/uav_task_state`；
   - `/{exec_target}/mavros/state`；
   - `/{exec_target}/mavros/local_position/pose`；
6. 建立 `ExecuteGroupTask` Action server；
7. 建立整组 `EmergencyHold` service；
8. 启动 link health 线程。

### 3.3 Action goal 到整组任务

当前 backend 的主流程：

```text
ExecuteGroupTask goal
    │
    ├─ assignments 非空检查
    ├─ calibration_ready 检查
    ├─ FOLLOW_ROUTE formation/leader 检查
    ├─ IdentityMap.resolve(Axx)
    ├─ 当前是否已有活动 GroupState
    ├─ 建立 GroupState
    ├─ 参与 UAV link health 检查
    │
    ▼
可选 schedule_tau / t_off 计算
    │
    ▼
并发 PREPARE：/{exec_target}/uav_task
    │
    ├─ 全部 accepted=true → group 可继续
    ├─ 任一拒绝/超时 → ABORT 已接受者，整组 FAILED
    └─ 不发送 START
    │
    ▼
并发 START：/{exec_target}/uav_task_control
    │
    ├─ MOVE_TO/FAULT_EXIT 且排程启用：每架 worker 等待 t0+schedule_tau
    ├─ 非排程路径：普通并发 START
    └─ 任一 START 失败 → ABORT/安全 HOLD/整组 FAILED
    │
    ▼
发布 Group Action ACCEPTED
    │
    ▼
等待每架 UavTaskState terminal
    │
    ├─ 全部 COMPLETED → Action SUCCEEDED / TCP COMPLETED
    ├─ 任一 FAILED → group HOLD fan-out / Action ABORTED / TCP FAILED
    ├─ timeout → group HOLD / Action FAILED
    └─ cancel → group HOLD / Action PREEMPTED / TCP CANCELLED
```

### 3.4 当前 `schedule_tau` / `t_off`

当前 `_schedule_applies()` 对以下 command 启用排程：

```text
MOVE_TO   + move_to_schedule_enabled
FAULT_EXIT + fault_exit_schedule_enabled
```

排程输入是：

```text
当前 bridge 转发的各执行 UAV 水平 pose
        +
MOVE_TO target_pose 或 FAULT_EXIT 最后 waypoint
        +
ScheduleParams
```

当前默认参数包括：

```text
horizontal_speed_mps = 0.7
schedule_geometric_distance_m = 1.0
schedule_safety_margin_m = 1.0
```

因此预测安全距离为：

```text
1.0 m 几何距离 + 1.0 m safety margin = 2.0 m
```

`schedule_tau` 只表示各架 START 的时间偏移：

```text
全部 PREPARE accepted
    → 建立 t0
    → 每架独立 worker 等待 t0 + schedule_tau[Axx]
    → 调用该 UAV 的 UavTaskControl START
```

必须区分：

```text
schedule_safety_margin_m
    = 排程阶段的启动时间协调参数

planner swarm_clearance × 2
    = ego_planner_core 轨迹检查门槛，当前默认约 1.0m

yield_clearance_factor × swarm_clearance × 2
    = 当前 driver 候选让路门槛

SafetyPredictor protected/TTC/dynamic margin
    = 运行时预测监督参数

executor min_center_distance_m
    = 实际 fresh 邻机 odom 的 3D 中心距门禁
```

它们不是同一个 `margin`，修改其中一个不会自动修改其他链路。

### 3.5 backend 反馈和失败收口

| 失败位置 | 是否已发送 PREPARE | 是否发送 START | 当前收口 |
|---|---:|---:|---|
| TCP JSON/字段解析 | 否 | 否 | TCP FAILED，通常带 REJECTED 诊断 |
| calibration 未准备 | 否 | 否 | Action FAILED |
| identity/formation 不合法 | 否 | 否 | Action FAILED |
| pose 缺失/过期 | 否 | 否 | schedule 前 FAILED |
| schedule 无解/超时预算不足 | 否 | 否 | schedule-specific FAILED |
| PREPARE 任一拒绝/超时 | 部分请求可能已到达 | 否 | ABORT 已接受者，整组 FAILED |
| START 任一失败 | 是 | 部分可能已 START | ABORT/HOLD，整组 FAILED |
| 运行时 UAV FAILED | 是 | 是 | group HOLD，其余参与者收口，FAILED |
| group timeout | 是 | 是 | group HOLD，FAILED/TIMEOUT |
| Action cancel | 是 | 是 | group HOLD，Action CANCELLED |
| 全部 UAV COMPLETED | 是 | 是 | Action COMPLETED |

当前 backend 对同一时间只允许一个活动 GroupState。文档目标中的“下一条 MOVE_TO 不覆盖正在执行 snapshot”在当前代码中主要表现为新整组命令被 `COMMAND_ALREADY_ACTIVE` 拒绝，而不是多个命令并行持有不同 assignment snapshot。

### 3.6 EmergencyHold fan-out

backend 还提供整组 `EmergencyHold` service。它不依赖普通 command 的 PREPARE/START 流程，而是将 HOLD 请求扇出到参与 UAV，等待各机 HOLD 结果，并在必要时收口整组状态。

---

## 4. `swarm_uav_executor` 当前运行流程

### 4.1 节点启动

`uav_executor_node.py` 的当前启动顺序：

```text
rospy.init_node("uav_executor")
    │
    ├─ load_config_from_ros_params()
    ├─ 读取 uav_id / exec_target / service_namespace
    ├─ build_driver(driver)
    │     ├─ mock
    │     ├─ mavros_position
    │     └─ ego_swarm
    ├─ 创建 TaskStore
    ├─ 创建 StateReporter
    ├─ 创建 UavTaskExecutor
    ├─ 注册 UavTask service
    ├─ 注册 UavTaskControl service
    ├─ 注册 UavHold service
    └─ rospy.spin()
```

EGO launch 显式配置：

```text
uav_id = UAVn
exec_target = UAVn
driver = ego_swarm
require_explicit_start = true
```

### 4.2 PREPARE：`handle_task()`

当前 `UavTaskExecutor.handle_task()` 的实际顺序：

```text
UavTask request
    │
    ├─ validate_task_request()
    │     ├─ protocol_version == 1.0
    │     ├─ identity 匹配本机
    │     ├─ GroupA
    │     ├─ command 支持
    │     ├─ timeout 有限且 > 0
    │     └─ assignment 与 command 合法
    │
    ├─ TaskKey = (mission_id, command_id, uav_id)
    ├─ request fingerprint
    │
    ├─ TaskStore 已有同 key？
    │     ├─ fingerprint 相同 → 幂等返回现有状态
    │     └─ fingerprint 不同 → DUPLICATE_CONFLICT
    │
    ├─ shutdown/HOLD/active task 检查
    ├─ driver.health()
    ├─ build_goal(request)
    ├─ driver.prepare(goal)
    ├─ TaskStore.register(TaskRecord)
    ├─ StateReporter.publish(ACCEPTED)
    │
    └─ require_explicit_start？
          ├─ false → 立即启动任务线程
          └─ true  → 等待 UavTaskControl START
```

当前 EGO 任务的 PREPARE 已经会检查 pose、MAVROS 连接/arm/OFFBOARD，以及 MOVE_TO 所需的 global origin confirmed（由 `EgoSwarmDriver.prepare()` 完成）。

### 4.3 START：`handle_task_control()`

```text
UavTaskControl(operation=START)
    │
    ├─ protocol/identity 校验
    ├─ 查找已 PREPARE 的 TaskKey
    ├─ 拒绝不存在、已 started 或 terminal 的 record
    ├─ 拒绝 active/HOLD/busy 冲突
    │
    ├─ TaskStore.mark_started()
    ├─ StateReporter.publish(ACCEPTED)
    ├─ 创建 cancel_event
    ├─ 创建 daemon thread
    └─ thread.start(_run_task)

Service callback 立即返回：accepted=true, status=ACCEPTED
```

任务 deadline 使用：

```text
prepared_at + timeout_s
```

所以从 PREPARE 到真正 START 的等待时间已经计入单机任务 deadline。

### 4.4 任务执行和安全收口

```text
_run_task()
    │
    ├─ driver.start_move_to(goal, cancel_event, deadline)
    │
    ├─ MotionResult.success == true
    │     ├─ 检查 record 仍未 terminal
    │     ├─ 检查没有被 HOLD cancel
    │     └─ publish_transition(COMPLETED)
    │
    └─ MotionResult.success == false
          │
          ├─ 如果 cancel_event 已设置
          │     └─ 由 HOLD 线程拥有最终收口
          │
          ├─ 否则获得 _hold_in_progress 所有权
          ├─ driver.hold(HoldGoal(error))
          ├─ hold 成功 → FAILED / detail_stage=HOLD / 原错误码
          ├─ hold 失败 → FAILED / HOLD_FAILED
          └─ 清理 _cancel / _hold_in_progress
```

### 4.5 ABORT 与 HOLD

ABORT 只适用于：

```text
已 PREPARE
尚未 START
```

START 后收到 ABORT：

```text
TASK_ALREADY_STARTED
“started task requires HOLD, not ABORT”
```

运行中 HOLD：

```text
UavHold
    → 设置 cancel_event
    → _hold_in_progress = true
    → daemon _run_hold()
    → driver.hold()
    → 原任务 FAILED / HOLD / COMMAND_HELD 或 HOLD_FAILED
```

HOLD 是 executor 的安全收口动作，不等于 C++ driver 的某一个单独 `/exec_state` 字符串。

### 4.6 TaskStore 和 StateReporter

`TaskStore` 保存：

```text
TaskKey
fingerprint
exec_target
command
timeout_s
MotionGoal
status/status_seq/detail_stage/error_code/message
prepared_at/updated_at/started
```

当前 terminal 状态集合主要是：

```text
ACCEPTED
COMPLETED
FAILED
```

`status_seq` 在 `TaskStore.update()` 时递增；`mark_started()` 只修改 started，不单独递增序列。

`StateReporter`：

- 发布 PREPARE/START/transition 状态；
- 对 terminal 状态按配置重复发布；
- 通过 `threading.Timer` 补发；
- backend 依靠 `status_seq` 去重和聚合。

---

## 5. `EgoSwarmDriver` 到 C++ driver 的任务适配

### 5.1 连接的 topic

`EgoSwarmDriver` 当前主要连接：

```text
订阅：
  /exec_state
  /local_pose
  /mavros/state（可选）
  /gp_origin_confirmed
  /<leader>/mavros/local_position/odom（follower）
  邻机 odom（距离门禁）

发布：
  /goal
  /goal_yaw
  /waypoints
  /hold
  /setpoint/follower
  /direct_control_active
  /neighbor_intent（转发输入）
```

C++ driver 自己发布：

```text
/setpoint/ego
/trajectory_intent
/exec_state
```

### 5.2 非 follower 任务：分层和航点密化

`EgoSwarmDriver._plan_horizontal()` 对以下任务进入 C++ EGO planner：

```text
MOVE_TO
FAULT_EXIT
FOLLOW_ROUTE leader
HOVER（按当前 goal/waypoint 语义进入相同等待链）
```

当前动作高度分层：

```text
MOVE_TO       → layer_z = 15m
FOLLOW_ROUTE  → layer_z = 12m
FAULT_EXIT    → layer_z = 8m
```

FOLLOW_ROUTE leader 的路线会被构造成：

```text
当前位置
  → 当前高度切到任务层
  → route waypoints 的水平段
  → 最终 waypoint 的最终高度
```

之后按 `waypoint_densify_spacing` 密化，默认 EGO 配置为 3m，发布到 C++：

```text
EgoSwarmDriver
    → PolygonStamped /waypoints
    → ego_planner_driver::onWaypoints()
    → waypoint_consumed_ / current target
```

密化航点是 C++ driver 的航点推进参考；C++ planner 内部仍生成自己的轨迹采样和多项式/B-spline 轨迹。

### 5.3 FOLLOW_ROUTE follower：不调用 C++ nominal planner

follower 任务由 `EgoSwarmDriver._follower_loop()` 执行：

```text
leader odom
    + formation_offset
    + own local pose
        │
        ▼
PI loop（10Hz 计算）
    target = leader_pose + offset
    error = target - own_pose
    integral += error * dt
    velocity = P*error + I*integral
    xy/z 限幅
        │
        ▼
PositionTarget /setpoint/follower（30Hz 发布）
        │
        ▼
direct_control_active = true
        │
        ▼
setpoint_relay 选择 follower source
```

follower 失败条件包括：

```text
leader odom 超过约 1s 未更新 → LEADER_LOST
本机 pose 缺失 → POSE_STALE
fresh 邻机 3D 中心距离低于门限 → MIN_DISTANCE_BREACH
deadline 到期 → LOCAL_TIMEOUT
cancel/HOLD → COMMAND_HELD
```

follower 完成需要：

1. 本机接近 `leader_pose + formation_offset`；
2. leader 接近任务终点；
3. 稳定达到 `steady_s`。

finally 中一定发布 `direct_control_active=false`，让 setpoint relay 回到 EGO source。

### 5.4 `prepare()` 与状态等待

`EgoSwarmDriver.prepare()` 是任务真正开始前的健康门：

```text
ego node 已可见
local_pose 新鲜
MAVROS state（若配置）新鲜且 connected
vehicle armed
vehicle mode == OFFBOARD
MOVE_TO 时 origin_confirmed=true
```

它不负责自动 arm 或自动切换 OFFBOARD；这些属于外部起飞/部署流程。

执行期间 `_wait_for_terminal()` 同时观察：

```text
cancel_event
任务 deadline
EGO exec_state
本机 local pose
fresh 邻机 3D 中心距离
```

当前终态映射：

| C++ `exec_state` 或本地条件 | `MotionResult` |
|---|---|
| `COMPLETED` | success |
| `EGO_PLAN_FAILED` | failure / EGO_PLAN_FAILED |
| `EGO_EXEC_TIMEOUT` | failure / EGO_EXEC_TIMEOUT |
| `BRAKE_HOLD` | failure / BRAKE_HOLD |
| `HOLD` | failure / COMMAND_HELD |
| fresh 邻机距离低于门限 | failure / MIN_DISTANCE_BREACH |
| deadline 到期 | failure / LOCAL_TIMEOUT 或 DRIVER_TIMEOUT |
| cancel | failure / COMMAND_HELD |

---

## 6. `ego_planner_driver` 当前状态、线程和回调

### 6.1 当前状态字符串

定义位置：

`src/ego_planner_driver/include/ego_planner_driver/ego_planner_driver_node.h`

当前状态：

```text
IDLE
EXECUTING
COMPLETED
HOLD
EGO_PLAN_FAILED
EGO_EXEC_TIMEOUT
TAKEOFF
EMERGENCY_BRAKE
BRAKE_HOLD
```

初始化时 C++ driver 直接设置为 `HOLD`，并启动 setpoint timer。

### 6.2 当前节点接口

构造函数建立：

```text
发布：
  setpoint_out_topic，EGO launch 中为 /setpoint/ego
  trajectory_intent
  exec_state（latched）

订阅：
  goal
  goal_yaw
  waypoints
  local_pose
  neighbor_intent
  gps_bias
  hold
  /mavros/state
  direct_control_active
```

定时/线程：

```text
publishSetpoint      默认 30Hz
runSafetySupervisor  默认 25Hz（active/shadow 时）
collisionCheckTick   默认 10Hz
replanLoop           独立 C++ thread
```

### 6.3 当前实际控制关系

```text
                    ┌─────────────────────┐
/goal  ────────────►│                     │
/waypoints ────────►│                     │
/local_pose ───────►│  mutex_ 共享状态     │
/neighbor_intent ──►│  current_ / goal    │
/hold ─────────────►│  inbox_ / flags     │
/direct_control ───►│                     │
                    └─────────┬───────────┘
                              │
          ┌───────────────────┼────────────────────┐
          │                   │                    │
          ▼                   ▼                    ▼
   publishSetpoint      safety supervisor     collision check
       30Hz                   25Hz                  10Hz
          │                   │                    │
          │                   └─ emergency_pending_│
          │                                        │
          └─ planning_requested_ ◄────────────────┘
                              │
                              ▼
                        replanLoop
                              │
                              ▼
                        replanOnce()
                              │
              ┌───────────────┼─────────────────────┐
              ▼               ▼                     ▼
          current_       trajectory_intent      exec_state
          替换/保持        发布                   发布
```

当前设计中，安全回调和 collision callback 不直接替换 `current_`；正常轨迹、候选轨迹和制动轨迹主要由 replan 线程提交。这是当前代码已经具备的单一提交者特征。

### 6.4 与旧文档逻辑名的差异

当前 C++ driver 没有名为 `PI_EXECUTING` 的 `/exec_state`。follower PI 是 Python `EgoSwarmDriver` 内部后台循环，并通过 `direct_control_active` 选择 relay source。

当前也没有一个实际发布的 `EMERGENCY_HOLD` 字符串。当前安全链为：

```text
SafetyPredictor EMERGENCY
    → emergency_pending_
    → replanLoop 处理
    → 必要时 EmergencyBrakeGenerator
    → EMERGENCY_BRAKE
    → 制动轨迹耗尽
    → BRAKE_HOLD
```

因此后续修改时必须先决定是：

```text
保持当前 EMERGENCY_BRAKE → BRAKE_HOLD 兼容链
```

还是实施旧文档中的直接 `EMERGENCY_HOLD` 目标策略。二者会产生不同飞行表现，不能只做名称替换。

---

## 7. 当前 `replanOnce()` 和轨迹执行

### 7.1 当前入口门控

`replanOnce(bool timed_trigger)` 的入口依次检查：

```text
hold_requested_        → 处理 HOLD 并返回
 direct_control_active_ → follower 接管时跳过
TAKEOFF                → 跳过
无 trigger             → 跳过
无 pending_goal_       → 跳过
steady_target_ 锁存    → 通常跳过
local_pose 过期        → 跳过本轮
```

当前 trigger 来源被压缩成：

```text
planning_requested_
emergency_pending_
或 replanLoop 的 timed_trigger
```

没有独立的 `BRAKE_REQUEST`、`COMPLETE_REQUEST`、`REPLAN_REQUEST` 对象。

### 7.2 快照和目标选择

进入规划后，当前代码：

1. 清除 `planning_requested_`；
2. 读取 fresh local pose；
3. 先进行 `handleArrivalBeforeSnapshotLocked()`；
4. 从当前轨迹同刻采样起点 P/V/A/yaw；
5. 首次规划没有 current_ 时使用 live pose + 零速度/加速度；
6. 根据 `waypoint_consumed_` 选择当前未完成航点；
7. 标记是否为最终段；
8. 取得 fresh 邻机 intent 快照；
9. 解锁后调用规划核心。

### 7.3 activation retry 与 nominal planning

当前 `replanOnce()` 内部最多进行 `activation_replan_attempts_` 次尝试，默认 3 次：

```text
抓取当前 fresh 邻机 intent
    → 以 assumed_activation 重建相对时间轴
    → core_->planOnce()
    ├─ RESOURCE_LIMIT
    │     → 立即 EGO_PLAN_FAILED + HOLD target
    ├─ DEGENERATE_SEGMENT
    │     → 保留 current_，按到达/软着陆路径处理
    ├─ RETRYABLE_FAILURE
    │     → 更新 assumed_activation，继续
    └─ SUCCESS
          → completed_at 重新对齐邻机
          → core_->checkClearance()
          → 通过才 safe_candidate=true
```

`PlanResult::SUCCESS` 仅表示 planner 产生了一个候选轨迹并进入后续 clearance/提交流程。

### 7.4 失败路径和软着陆

nominal 规划没有安全候选时，当前顺序为：

```text
EMERGENCY pending？
    ├─ 尝试 tryEmergencyBrakeLocked()
    ├─ 成功 → EMERGENCY_BRAKE
    └─ 失败 → 捕获 HOLD target + EGO_PLAN_FAILED

普通 nominal 失败
    ├─ tryYieldReplan()
    │     ├─ COMMITTED → 提交候选
    │     ├─ RESOURCE_LIMIT → EGO_PLAN_FAILED
    │     └─ NOT_AVAILABLE → 继续
    │
    └─ soft landing
          ├─ 保留 current_ 或末端 HOLD
          ├─ plan_fail_since_ 记录首次连续失败时刻
          ├─ 周期/事件继续重试
          └─ 超过 plan_fail_timeout_s_ → EGO_PLAN_FAILED
```

当前 `tryYieldReplan()` 仍可能生成：

```text
SLOW
LEFT
RIGHT
```

并使用基于 UAV 数字身份的 STAND_ON/YIELD 角色和候选验证。它不是纯 nominal 单层更新，也不是完整地图/动力学重新优化。

### 7.5 nominal 成功后的提交

候选提交前，当前代码还会：

```text
检查 latest neighbors clearance
检查 hold_requested_
检查 emergency_braking_
检查 safety_generation_
写入 current_
更新时间 plan_start_
清理 arrival/失败/steady 状态
发布 ACTIVE trajectory_intent
设置 EXECUTING
```

当前没有文档目标中的：

```text
ReplanSlot
input_version
started_input_version
replan_running
retry_not_before
```

现有 `safety_generation_` 不是完整规划输入版本；它主要用于安全/制动代际检查，不能覆盖 waypoint、goal、普通 neighbor intent 等所有变化。

### 7.6 setpoint 执行

`publishSetpoint()` 30Hz 采样 `current_`：

```text
current_ 未建立
    → 如果 HOLD 且没有待处理规划，锁存当前 pose HOLD

TAKEOFF
    → 以 HOLD mask 发布当前位置 + 逐渐上升 z
    → 到达 takeoff_height - 0.1m 后进入 HOLD

steady_target_
    → 发布 position/yaw HOLD

current_ 未耗尽
    → 发布 position + velocity + acceleration + yaw

current_ 耗尽
    → 先发布轨迹终点 HOLD
    → 若 emergency_braking_，进入 BRAKE_HOLD
    → 否则执行到达/航点推进/周期 replanning 检查
```

C++ EGO candidate 只发布到 `/setpoint/ego`，不是最终 MAVROS 出口。

---

## 8. 安全、intent、跨机和 PX4 setpoint

### 8.1 邻机 intent 传播

当前 intent 路径可概括为：

```text
UAV1 ego_planner_driver
    → /trajectory_intent
    → swarm_topology_bridge PUB
    → 远端 bridge SUB
    → /UAV1/trajectory_intent
    → 远端 EgoSwarmDriver neighbor_intents
    → /neighbor_intent
    → 远端 ego_planner_driver acceptNeighborIntentLocked()
```

`EgoSwarmDriver._on_neighbor_intent()` 当前主要将收到的 intent 转发到 C++ driver 的 `neighbor_intent` topic。

C++ 接收侧检查：

- protocol version；
- phase/frame；
- uav_id/exec_target；
- finite 数据；
- 时间数组严格递增；
- trajectory 样本数量上限；
- stamp/epoch 新鲜度；
- traj_id 版本单调；
- GPS bias 时间修正。

当前 intent 新消息先进入 inbox，collision check 由 10Hz timer 统一检查；不是每次 intent callback 都立即执行规划。

### 8.2 `swarm_topology_bridge`

`swarm_topology_bridge/scripts/bridge_node.py` 是通用代理，不理解任务业务：

```text
Topic：
  本机 ROS Subscriber
      → ROS serialize
      → ZMQ PUB
      → 远端 ZMQ SUB
      → 反序列化
      → /<source>/<topic> ROS Publisher

Service：
  本机上层 ROS Service call
      → ZMQ DEALER
      → 远端 ROUTER
      → 目标节点本地 ROS Service
      → 序列化响应返回
```

当前 bridge 还负责：

- 根据 topology 配置建立连接；
- 每 Topic 限频；
- 请求 ID；
- service 并发请求锁；
- bridge timeout；
- 迟到 response 丢弃；
- 远端服务名透明代理。

因此跨 Master 的消息丢失/延迟可能同时影响任务 Service、任务状态 Topic、neighbor intent、odom 和 MAVROS 状态；必须按具体接口区分，不能统称为“ROS 断链”。

### 8.3 三种独立的安全/距离门

当前至少有以下三类独立门限：

```text
A. ego_planner_core::checkClearance()
   current trajectory 与 neighbor intent 的规划时域门槛
   当前使用 swarm_clearance × 2

B. collisionCheckTick()/yield candidate gate
   低于候选触发/验证门槛时请求 replan 或生成候选

C. SafetyPredictor
   运行时预测未来距离、closing speed、CPA、TTC、WARNING/EMERGENCY

D. EgoSwarmDriver distance gate
   fresh neighbor odom 的实际三维中心距
   低于 min_center_distance_m → /hold + MIN_DISTANCE_BREACH
```

此外，`setpoint_relay` 还有 candidate freshness/finite/mask 合同；它不是距离门，但会决定最终是否输出 EGO/follower candidate 或固定 HOLD。

### 8.4 pose relay

`pose_relay_node.py` 当前只做：

```text
/mavros/local_position/pose
    → /local_pose
```

它不做坐标规划、不做安全判断、不做状态转换。

### 8.5 setpoint relay：唯一 MAVROS raw setpoint 出口

当前 `setpoint_relay_node.py` 的关系：

```text
/setpoint/ego       PositionTarget
/setpoint/follower  PositionTarget
/direct_control_active Bool
/mavros/local_position/pose PoseStamped
        │
        ▼
SetpointRelay.tick() 30Hz
        │
        ├─ false → 选 ego
        ├─ true  → 选 follower
        ├─ 校验 source mask/frame/finite/freshness
        ├─ 模式切换时 generation 递增并作废旧候选
        ├─ 候选无效/过期 → 捕获 fresh pose 固定 HOLD
        └─ 无安全 HOLD 目标 → 不发布伪造的 (0,0,0)
        │
        ▼
/mavros/setpoint_raw/local
```

当前唯一性是指：在 EGO launch 约束下，`setpoint_relay_node.py` 是最终 `/mavros/setpoint_raw/local` 的唯一业务发布者。EGO C++ 和 follower PI 只发布候选。

候选 mask：

```text
EGO trajectory = 2048
HOLD           = 2552
follower PI    = 2496
```

relay 当前要求：

```text
FRAME_LOCAL_NED
启用字段 finite
候选 freshness <= 0.2s
local pose freshness <= 1.0s（用于捕获 HOLD）
```

业务层位置数据由 EGO/driver 按当前 ROS ENU 语义填写；`PositionTarget.coordinate_frame` 使用 `FRAME_LOCAL_NED`。当前源码注释和工程约定将 MAVROS ENU→NED 转换视为 MAVROS 边界职责。该坐标变换应通过实机/仿真 topic 观测继续验证，不能只凭 topic 名称假设。

### 8.6 到 PX4 的实际出口

```text
普通 EGO：
  C++ current_ sample P/V/A/yaw
      → /setpoint/ego
      → setpoint_relay 选 ego
      → /mavros/setpoint_raw/local
      → MAVROS
      → PX4 offboard controller

FOLLOW_ROUTE follower：
  leader odom + own pose
      → Python PI
      → /setpoint/follower
      → direct_control_active=true
      → setpoint_relay 选 follower
      → /mavros/setpoint_raw/local
      → MAVROS/PX4

候选过期/模式切换/任务 HOLD：
  fresh local pose
      → relay/C++ HOLD candidate
      → 固定 position/yaw
      → /mavros/setpoint_raw/local
      → PX4

EMERGENCY：
  SafetyPredictor
      → C++ emergency_pending_
      → EmergencyBrakeGenerator（当前策略）
      → /setpoint/ego 的 braking candidate
      → relay
      → PX4
      → 轨迹耗尽后 BRAKE_HOLD
```

本文不把 PX4 内部位置环、offboard failsafe 参数或 MAVROS plugin 内部转换实现冒充成 `src/` 中的业务源码；这些属于外部组件，需要通过其版本文档和飞行日志独立核对。

---

## 9. 失败、取消、并发与安全收口矩阵

### 9.1 入口和 backend 层

| 事件 | 当前行为 | 实际影响 |
|---|---|---|
| TCP 格式错误 | TCP FAILED；可解析身份时发布拒绝诊断 | 不进入 Action |
| 重复 command_id | 返回缓存最终响应 | 不重复创建整组任务 |
| 其他 group active | `COMMAND_ALREADY_ACTIVE` | 新任务直接拒绝 |
| calibration 未就绪 | Action ABORTED/FAILED | 不发单机 PREPARE |
| identity/formation 错误 | Action FAILED | 不进入执行 |
| schedule pose 缺失 | schedule 前 FAILED | 不发 PREPARE |
| schedule 无解 | schedule 前 FAILED | 不发 PREPARE/START |
| schedule timeout 不足 | schedule 前 FAILED | 不发 PREPARE/START |
| UAV link LOST | 活动 GroupState 标记失败并 stop | 进入整组失败/HOLD 收口 |

### 9.2 executor 层

| 事件 | 当前行为 |
|---|---|
| protocol/identity/assignment 错误 | `UavTask` 返回 accepted=false/FAILED |
| 相同 TaskKey、相同 fingerprint | 幂等返回当前记录并重发状态 |
| 相同 TaskKey、不同 fingerprint | `DUPLICATE_CONFLICT` |
| active task/HOLD 存在 | `BUSY` |
| driver health 不可用 | PREPARE FAILED |
| driver.prepare 失败 | PREPARE FAILED |
| START 前 ABORT | `FAILED/ABORTED/TASK_ABORTED` |
| START 后 ABORT | 拒绝，提示使用 HOLD |
| 运动失败 | 先 driver.hold，再 `FAILED/HOLD` |
| 运动失败且 HOLD 失败 | `FAILED/HOLD/HOLD_FAILED` |
| 外部 HOLD | cancel 当前动作，driver.hold，任务 `FAILED/HOLD` |
| shutdown | active task 尝试 HOLD，等待线程，关闭 driver |

### 9.3 Python EGO driver 层

| 事件 | 当前 MotionResult/行为 |
|---|---|
| C++ `COMPLETED` | success |
| C++ `EGO_PLAN_FAILED` | failure |
| C++ `EGO_EXEC_TIMEOUT` | failure |
| C++ `BRAKE_HOLD` | failure |
| C++ `HOLD` | `COMMAND_HELD` |
| fresh 邻机 odom 3D 距离过小 | `/hold` + `MIN_DISTANCE_BREACH` |
| leader odom stale | `LEADER_LOST` |
| follower pose stale | `POSE_STALE` |
| 本地 deadline 到期 | `LOCAL_TIMEOUT` |
| executor cancel | `COMMAND_HELD` |

### 9.4 C++ driver 层

| 事件 | 当前行为 |
|---|---|
| local pose stale | 本轮 replan 跳过，等待恢复 |
| goal/waypoint 推进 | `planning_requested_`，唤醒 replanLoop |
| neighbor intent clearance 不足 | collision checker 置 `planning_requested_` |
| planner `RESOURCE_LIMIT` | `EGO_PLAN_FAILED`，捕获 HOLD target |
| nominal retryable failure | activation retry；之后 yield/soft landing |
| 连续普通失败 | 保留 current_/末端，超过 timeout 才 `EGO_PLAN_FAILED` |
| SafetyPredictor EMERGENCY | 设置 `emergency_pending_`，replan 线程处理 |
| Emergency brake 成功 | `EMERGENCY_BRAKE`，发布 braking trajectory |
| braking trajectory 耗尽 | `BRAKE_HOLD` 锁存 |
| 正常到达稳定 | 清 current_，`COMPLETED`，steady target |

### 9.5 relay/PX4 边界

| 事件 | 当前行为 |
|---|---|
| candidate mask/frame 非法 | relay 丢弃 candidate |
| candidate NaN/Inf | relay 丢弃 candidate |
| candidate 过期 | relay 输出固定 HOLD |
| mode 切换 | generation 递增，新 source 候选到达前 HOLD |
| local pose stale 且无已验证 HOLD | 不发布伪造原点 setpoint |
| PX4/MAVROS 状态不满足 | `EgoSwarmDriver.prepare()` 先拒绝任务，driver 不自动 arm/OFFBOARD |

### 9.6 当前并发边界

当前系统同时使用：

```text
backend：RLock + GroupState lock + 多个 service worker thread
executor：RLock + 每任务 daemon thread + HOLD daemon thread
EgoSwarmDriver：RLock + follower daemon task loop
C++ driver：mutex_ + ROS timers + replan thread
relay：ROS callbacks 缓存 + 30Hz tick 统一输出
bridge：ZMQ 接收线程、service dealer/router 线程
```

这些线程模型带来几个需要实际验证的边界：

1. backend 的 PREPARE/START worker 和 group deadline 的交错；
2. executor 任务线程、HOLD 线程和状态补发 timer 的先后；
3. C++ replan 期间 `current_`、neighbor intent 和到达状态的变化；
4. bridge service timeout 与远端迟到响应；
5. relay mode generation 切换期间旧 candidate 是否完全失效；
6. PX4 实际接收频率与 ROS relay 发布频率是否一致。

---

## 10. 用于判断是否修改源码的结构检查清单

### 10.1 当前已经存在、应避免重复重写的结构

```text
[已存在]
- backend → executor 的 PREPARE/START 分离
- 单机 TaskKey + fingerprint 幂等
- executor 运动失败后的 HOLD 收口
- EGO/follower 候选分离
- setpoint relay 唯一最终 MAVROS raw setpoint 出口
- C++ driver 单一 replan 提交线程
- terminal 状态重发和 status_seq
- bridge 的 Topic/Service 代理与 timeout
- C++ 资源参数 fail-fast 检查
```

这些结构的修改应以实际 bug、日志证据或明确接口需求为依据，不能因为目标文档的描述更简洁就直接替换。

### 10.2 当前存在的结构差异，可能影响实际表现

```text
[高影响候选]
- t_off/schedule_tau 与运行时轨迹避碰之间的模型差异
- C++ driver 仍保留 tryYieldReplan() 的 SLOW/LEFT/RIGHT 候选
- SafetyPredictor 预测、planner clearance 和 executor 3D distance gate 不同源
- intent 跨 Master 的 freshness、时间轴和版本连续性
- C++ replan 没有完整 input_version/started_input_version 事务检查
- COMPLETE、EMERGENCY 和 REPLAN 没有统一请求仲裁对象
- follower PI 与 EGO relay source 切换时的 generation/候选间隙
- active group 单任务限制阻止了真正并发 assignment snapshot
```

### 10.3 主要改善可观测性、通常不直接改变飞行策略

```text
[可观测性优先]
- 为每层状态附带 mission_id/command_id/uav_id/exec_target
- 记录 PREPARE、START、driver start、first setpoint、terminal 的统一时间戳
- 记录 intent traj_id、stamp、receive time、data age、common window
- 记录 PlanResult 与最终 exec_state 的映射
- 记录 relay candidate source、generation、freshness、HOLD 原因
- 记录 bridge request id、timeout、late response
- 区分预测 EMERGENCY、实际 3D distance breach、INSUFFICIENT_DATA
```

### 10.4 属于目标设计、当前尚未实现

```text
[不能误认为当前已有]
- tcp_to_ros current_map 持久运行时映射
- MOVE_TO 闭集动态 Axx/UAVn 重分配
- assignment snapshot/version 的显式事务对象
- ReplanSlot / input_version / retry_not_before
- 完整 BRAKE > COMPLETE > REPLAN 请求仲裁器
- 将 replanOnce() 完全收敛为单层 nominal 纯函数式动作
- 删除 driver 第二层 SLOW/LEFT/RIGHT 候选
- 直接 EMERGENCY_HOLD 替代当前 EMERGENCY_BRAKE → BRAKE_HOLD
```

### 10.5 每项修改前应回答的问题

```text
1. 修改的是哪一层的身份、时间、状态或空间坐标？
2. 该值是 Axx、UAVn 还是 exec_target？
3. 该状态是 PlanResult、exec_state、UavTaskState 还是 Action/TCP status？
4. 修改会改变 PREPARE、START、driver 执行还是 PX4 setpoint？
5. 修改会改变哪个 gate：schedule、planner、yield、SafetyPredictor、3D odom 或 relay freshness？
6. 失败时谁拥有 HOLD 的最终收口权？
7. 是否可能由旧线程/旧候选覆盖新输入？
8. 是否有对应的 ULog、task-state、intent 和 bridge 日志验证？
```

### 10.6 修改优先级建议

当前建议按以下顺序调查，而不是直接整体重写状态机：

```text
P0  补齐端到端时间戳和状态关联
P0  核对 intent freshness/版本/跨 Master 传播
P0  区分预测安全事件与实际距离门禁
P1  检查 executor HOLD 所有权和任务失败竞态
P1  为 replan candidate 增加输入有效性/过期诊断
P1  验证 relay source generation 与 PX4 实际接收
P2  再评估 t_off 模型、tryYieldReplan 和 SafetyPredictor 参数
P3  最后决定是否实施动态 current map 或完整新状态机
```

---

## 11. 推荐阅读顺序与证据索引

### 11.1 推荐源码阅读顺序

```text
1. tcp_to_ros/scripts/tcp_server_node.py
2. tcp_to_ros/scripts/tcp_to_ros/protocol.py
3. tcp_to_ros/scripts/tcp_to_ros/bridge.py
4. tcp_to_ros/scripts/gcs_a_backend_node.py
5. tcp_to_ros/scripts/tcp_to_ros/identity.py
6. tcp_to_ros/scripts/tcp_to_ros/time_offset_scheduler.py
7. swarm_uav_interfaces/srv/*.srv
8. swarm_uav_interfaces/msg/*.msg
9. swarm_uav_executor/scripts/uav_executor_node.py
10. swarm_uav_executor/src/swarm_uav_executor/executor.py
11. swarm_uav_executor/src/swarm_uav_executor/validation.py
12. swarm_uav_executor/src/swarm_uav_executor/task_store.py
13. swarm_uav_executor/src/swarm_uav_executor/state_reporter.py
14. swarm_uav_executor/src/swarm_uav_executor/drivers/ego_swarm.py
15. swarm_uav_executor/src/swarm_uav_executor/setpoint_relay.py
16. ego_planner_driver/include/ego_planner_driver/ego_planner_driver_node.h
17. ego_planner_driver/src/ego_planner_driver_node.cpp
18. ego_planner_driver/src/ego_planner_core.cpp
19. ego_planner_driver/src/safety_predictor.cpp
20. swarm_topology_bridge/scripts/bridge_node.py
21. 各包 launch/config
```

### 11.2 关键文件索引

| 主题 | 文件 |
|---|---|
| TCP JSON 协议 | `src/tcp_to_ros/scripts/tcp_to_ros/protocol.py` |
| TCP server | `src/tcp_to_ros/scripts/tcp_server_node.py` |
| GCS Action backend | `src/tcp_to_ros/scripts/gcs_a_backend_node.py` |
| Identity map | `src/tcp_to_ros/scripts/tcp_to_ros/identity.py`、`config/uav_identity_map.yaml` |
| t_off/schedule_tau | `src/tcp_to_ros/scripts/tcp_to_ros/time_offset_scheduler.py`、`gcs_a_backend_node.py` |
| 跨 Master bridge | `src/swarm_topology_bridge/scripts/bridge_node.py` |
| 单机 executor 启动 | `src/swarm_uav_executor/scripts/uav_executor_node.py` |
| executor 生命周期 | `src/swarm_uav_executor/src/swarm_uav_executor/executor.py` |
| 任务验证 | `src/swarm_uav_executor/src/swarm_uav_executor/validation.py` |
| EGO 适配 | `src/swarm_uav_executor/src/swarm_uav_executor/drivers/ego_swarm.py` |
| setpoint 仲裁 | `src/swarm_uav_executor/src/swarm_uav_executor/setpoint_relay.py`、`scripts/setpoint_relay_node.py` |
| 位姿中继 | `src/swarm_uav_executor/scripts/pose_relay_node.py` |
| C++ 状态和接口 | `src/ego_planner_driver/include/ego_planner_driver/ego_planner_driver_node.h` |
| C++ 主控制流 | `src/ego_planner_driver/src/ego_planner_driver_node.cpp` |
| planner core | `src/ego_planner_driver/src/ego_planner_core.cpp` |
| 安全预测 | `src/ego_planner_driver/src/safety_predictor.cpp`、`safety_predictor.h` |
| PX4 仿真启动 | `src/swarm_simulation/launch/multi_uav_ego_15sim.launch` |
| EGO 单机 launch | `src/swarm_uav_executor/launch/uav_executor_ego.launch` |
| GCS backend launch | `src/tcp_to_ros/launch/gcs_a_backend.launch` |

### 11.3 端到端简图

```text
TCP COMMAND
    │
    ▼
parse_json_line / CommandCache / ActiveTaskRegistry
    │
    ▼
ExecuteGroupTask Action
    │
    ▼
GCS backend
  IdentityMap.resolve
  calibration/link/formation checks
  optional schedule_tau
    │
    ├─────────────── PREPARE ───────────────┐
    │                                       ▼
    │                              UavTaskExecutor
    │                                validate
    │                                fingerprint
    │                                driver.prepare
    │                                TaskStore
    │                                       │
    │                              UavTaskState ACCEPTED
    │                                       │
    └─────────────── START ─────────────────┘
                                            ▼
                                      _run_task thread
                                            │
                                            ▼
                                      EgoSwarmDriver
                         ┌──────────────────┴──────────────────┐
                         ▼                                     ▼
                  EGO leader/normal                       follower PI
                         │                                     │
                  /goal,/waypoints                    /setpoint/follower
                         │                                     │
                         ▼                                     ▼
                  ego_planner_driver                   direct_control_active
                         │                                     │
                  current_/replan/safety                       │
                         │                                     │
                         └──────────┬──────────────────────────┘
                                    ▼
                              setpoint_relay
                         candidate/freshness/HOLD
                                    │
                                    ▼
                       /mavros/setpoint_raw/local
                                    │
                                    ▼
                                   PX4
                                    │
                         pose/state/terminal feedback
                                    │
                                    ▼
              UavTaskState → backend GroupState → Action/TCP STATUS
```

本文的核心判断边界是：

```text
任务调度是否成功
    看 tcp_to_ros 的 schedule/prepare/start/aggregation；

单机任务是否执行
    看 executor 的 TaskRecord、MotionDriver 和 HOLD 收口；

轨迹是否更新
    看 ego_planner_driver 的 current_/PlanResult/clearance/replan；

最终飞控是否收到有效 setpoint
    看 setpoint_relay、MAVROS topic 和 PX4 状态；

多机预测与实际是否一致
    必须同时看 intent、SafetyPredictor、odom、relay 和 task-state 时间线。
```
