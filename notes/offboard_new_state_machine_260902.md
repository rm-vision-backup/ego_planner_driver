# Offboard 新状态机设计（2026-09-02）

本文逐步整理 `swarm_uav_executor` 与 `ego_planner_driver` 的新状态机设计。任务名称与固定命令按《实物无人机上下层 TCP/IP 接口对接方案》区分：任务名称表示高层目的，可以任意命名；`MOVE_TO`、`FOLLOW_ROUTE`、`FAULT_EXIT`、`HOVER`、`HOLD` 等是固定的 `command` 字段。

---

## 0. 尚待确认的反馈细节

以下反馈、状态动作和联动细节尚未最终确认，暂不作为已经实现的状态机事实：

- 普通失败和 `EMERGENCY_HOLD` 是否使用不同的 `MotionResult` 语义
  - 规划失败、跟随失败、超时、通信失败是否分别编码
- `replanOnce()` 失败后的 `replanTimer` 重试策略
  - 每隔约 1 秒允许重试
  - 还是每条 `current_` 只尝试一次
- `PI_EXECUTING` 的完成反馈边界
  - leader 的 `SUCCESS` 如何发布和被 follower 识别
  - follower 到达 `leader + offset` 后的悬停保持时长

已经确定的 EGO_EXECUTING 内部结构不再列为待确认事项：

```text
A. 轨迹执行分支：发布当前轨迹 setpoint，并提供 waypointTracker 所需的当前位姿和执行时间
B. 轨迹更新分支：定时检查轨迹年龄，并消费统一的 REPLAN_REQUEST 执行 replanOnce()
C. 条件触发分支：waypointsTracker、collisionChecker、emergencyDog
请求优先级：BRAKE_REQUEST > COMPLETE_REQUEST > REPLAN_REQUEST
```

已经确定的安全动作边界是：

```text
emergencyDog 产生 BRAKE_REQUEST
    → 请求仲裁器接受 brake
    → 直接进入 EMERGENCY_HOLD
```

---
## 1. 系统分层

### 1.1 总体职责关系

```text
┌──────────────────────────────────────────────────────────────────────┐
│                         tcp_to_ros / GCS                             │
│                                                                      │
│  · 接收批次任务、执行批次级 prepare、排程和任务协调                 │
│  · 按 assignments 向各 UAV executor 发送单机任务请求                 │
│  · 汇总各 UAV 的 ACCEPTED / START / SUCCESS / FAILURE 等反馈         │
└──────────────────────────────┬───────────────────────────────────────┘
                               │ bridge 转发任务与状态话题/服务
                               ▼
┌──────────────────────────────────────────────────────────────────────┐
│                      swarm_uav_executor                              │
│                                                                      │
│  任务接入与任务适配层                                                │
│                                                                      │
│  · 接收、校验、保存单机任务并返回 ACCEPTED，等待 GCS 后续发送 START  │
│  · 按固定 command 整理 ego driver 所需信息并启动对应动作线程         │
│  · 监控 ego driver 状态和本机运动结果，向 tcp_to_ros 反馈结果         │
└───────────────┬──────────────────────────────┬───────────────────────┘
                │ 提供不同任务信息             │ 接收 ego 状态/结果
                ▼                              │
┌──────────────────────────────────────────────────────────────────────┐
│                       ego_planner_driver                             │
│                                                                      │
│  飞行控制与内部状态机层                                              │
│                                                                      │
│  · 接收 executor 提供的信息，并根据这些输入自行完成内部状态转换     │
│    · INIT：初始化完成后等待 arm + OFFBOARD（一次性描述阶段）          │
│    · TAKEOFF：arm + OFFBOARD 后执行起飞（一次性描述阶段）            │
│    · EGO_EXECUTING：使用 ego 算法自主导航                            │
│    · PI_EXECUTING：使用 PI 控制跟随 leader                            │
│    · COMPLETED_HOLD：正常完成/悬停后的可退出保持                      │
│    · EMERGENCY_HOLD：安全故障后的锁存保持                            │
│  · 向 executor 提供任务执行所需的状态依据和运动结果                │
└──────────────────────────────┬───────────────────────────────────────┘
                               │ setpoint / position / velocity control
                               ▼
┌──────────────────────────────────────────────────────────────────────┐
│                              PX4                                     │
│                                                                      │
│  · 接收 offboard 控制 setpoint                                       │
│  · 执行位置、速度或相关飞行控制                                      │
│  · 遥控器切换 onboard 后可接管                                        │
└──────────────────────────────────────────────────────────────────────┘
```

### 1.2 核心边界

```text
executor 不直接发送“切换 ego 状态”的专用命令

executor 接收固定 command
    │
    ├─ 处理 command 内容
    ├─ 发布 ego 所需信息
    └─ 启动对应 driver 执行路径
            │
            ▼
ego driver 根据收到的信息
自行进行内部状态转换
```

“使 ego driver 进入某状态”表示：

```text
executor 提供足够且正确的输入
    → ego driver 按自身状态机转换到预期状态
```

这不表示 executor 绕过 ego driver、直接写入或强制设置其内部状态。


### 1.3 当前已确认的状态机骨架

```text
程序启动
    │
    ▼
INIT（一次性描述阶段：初始化并等待 arm + OFFBOARD）
    │
    │ 满足 arm + OFFBOARD
    ▼
TAKEOFF（一次性描述阶段：内部起飞进度达到 takeoff_height - 0.1 m 后切换）
    │
    │ 立即切换
    ▼
COMPLETED_HOLD
    │
    ├─ MOVE_TO / FAULT_EXIT / FOLLOW_ROUTE（leader）
    │       └─ executor 提供 waypoints
    │               ▼
    │         EGO_EXECUTING
    │
    ├─ FOLLOW_ROUTE（follower）
    │       └─ executor 提供 leader + offset
    │               ▼
    │         PI_EXECUTING
    │
    ├─ HOVER ───────────────────────────────────► COMPLETED_HOLD
    │
    └─ HOLD / 安全事件 ─────────────────────────► EMERGENCY_HOLD
```

`EGO_EXECUTING` 内部由三类动作分支共同维持：

```text
A. 轨迹执行分支
   publish current trajectory setpoint
   provide current pose and execution time to waypointTracker

B. 轨迹更新分支
   check latest trajectory age >= 1s
   consume REPLAN_REQUEST and call replanOnce()

C. 条件触发分支
   waypointsTracker → REPLAN_REQUEST / COMPLETE_REQUEST
   collisionChecker → REPLAN_REQUEST
   emergencyDog → BRAKE_REQUEST

请求优先级：BRAKE_REQUEST > COMPLETE_REQUEST > REPLAN_REQUEST
```

```text
EMERGENCY_HOLD
        │
        ├─ 锁存
        ├─ 不允许普通 command 或 START 退出
        ├─ 不允许自动 replan 恢复普通执行
        └─ 仅遥控器切换 onboard 后人工接管
```

该图是当前设计骨架；尚未确定的错误码细分、replan 失败重试策略和 PI_EXECUTING 完成反馈边界仍记录在第 0 章。

---

## 2. swarm_uav_executor 细节

### 2.1 任务名称、固定 command 与 ACCEPTED/START

```text
上层任务名称
    = 任意名称，表示高层目的
    = 例如 prepare、coverage-segment-1、fault-exit、recovery、hold-end

固定 command
    = 协议规定的具体动作
    = MOVE_TO、FOLLOW_ROUTE、FAULT_EXIT、HOVER、HOLD

ACCEPTED
    = executor 已校验并保存该 UAV 的单机任务，等待 START
    = 不表示已经开始运动，也不表示 ego 已进入执行状态

START
    = GCS 要求 executor 启动已经 ACCEPTED 的单机任务
```

### 2.2 ACCEPTED/START 流程

```text
上层任务名称 + 固定 command + assignments
        │
        ▼
tcp_to_ros 执行批次级协调/排程
        │
        │ 向各 UAV executor 发送单机任务请求
        ▼
executor 接收单机任务
        │
        ├─ 校验请求、command、uav_id 和任务参数
        ├─ 检查 executor/driver 是否可用
        ├─ 将请求转换为内部 goal
        ├─ 保存任务记录
        └─ 返回 ACCEPTED
                │
                ▼
        任务已受理，但尚未开始运动
                │
                │ GCS 等待批次协调完成并按计划发送 START
                ▼
executor 收到 START
        │
        ├─ 查找对应 ACCEPTED 任务
        ├─ 检查任务未启动且未终止
        ├─ 启动本机动作线程
        ├─ 向对应 driver 提供 command 对应的输入
        └─ 返回 START accepted / started
```

START 的职责是启动已经 ACCEPTED 的单机任务，而不是重新解析或重新受理任务。

### 2.3 批次任务与单机执行链路

```text
批次任务名称，例如 prepare
        │
        ▼
tcp_to_ros 执行批次 prepare / 排程
        │
        ├─ 单机任务请求 ─► executor-1 ─► ACCEPTED
        ├─ 单机任务请求 ─► executor-2 ─► ACCEPTED
        └─ ...
        │
        ▼
全部或规定参与者 ACCEPTED
        │
        ▼
tcp_to_ros 按批次协调结果发送 START
        │
        ├─ executor-1 ─► 启动对应 command
        ├─ executor-2 ─► 启动对应 command
        └─ ...
        │
        ▼
各 executor 向 ego driver 提供 command 对应的信息
        │
        ▼
ego driver 自行进入预期状态
```

### 2.4 导航命令

适用 command：

```text
MOVE_TO
FAULT_EXIT
FOLLOW_ROUTE（leader）
以及其他“本机根据航点自主运动”的固定 command
```

```text
ACCEPTED/START
        │
        ▼
executor 读取 command 的航点参数
        │
        ├─ 生成 ego 可使用的 waypoints 序列
        ├─ 执行必要的航点处理/密化
        └─ 向 ego driver 发布 waypoints 等信息
                │
                ▼
        ego driver 根据 waypoints 自行转换状态
                │
                ▼
             EGO_EXECUTING
                │
                ▼
        使用 ego 算法执行自主导航
```

示例：

```text
任务名称：prepare       command：MOVE_TO
任务名称：fault-exit   command：FAULT_EXIT
```

二者的任务名称不同，但都可以通过航点输入使 ego driver 进入 `EGO_EXECUTING`。

### 2.5 跟随命令

适用 command：

```text
FOLLOW_ROUTE（follower）
```

```text
ACCEPTED/START
        │
        ▼
executor 读取 follower 参数
        │
        ├─ 提供 leader ID（UAVn）
        ├─ 提供本机相对 leader 的 offset
        └─ 向 ego driver 发布跟随所需信息
                │
                ▼
        ego driver 根据 leader/offset 自行转换状态
                │
                ▼
             PI_EXECUTING
                │
                ▼
        使用 PI 控制跟随 leader
```

```text
EGO_EXECUTING = 以 waypoints 为输入，使用 ego 算法自主导航
PI_EXECUTING  = 以 leader ID 和 offset 为输入，使用 PI 控制跟随
```

### 2.6 悬停命令

适用 command：

```text
HOVER
```

```text
ACCEPTED/START
        │
        ▼
executor 提供 HOVER 所需信息
        │
        ▼
ego driver 执行悬停/保持动作
        │
        ▼
      COMPLETED_HOLD
```

```text
COMPLETED_HOLD
      │
      ├─ 允许新的 MOVE_TO / FAULT_EXIT command
      ├─ 允许新的 FOLLOW_ROUTE command
      ├─ 允许再次执行 HOVER command
      └─ 收到安全 HOLD 或安全事件时转入 EMERGENCY_HOLD
```

### 2.7 安全命令

适用 command：

```text
HOLD
```

```text
收到 HOLD command 或确认的安全事件
        │
        ▼
executor 提供安全保持所需信息
        │
        ▼
ego driver 根据自身状态机转换
        │
        ▼
      EMERGENCY_HOLD
```

```text
COMPLETED_HOLD
    = 正常完成/HOVER 后的保持状态
    = 允许切换到其他普通状态

EMERGENCY_HOLD
    = 安全故障后的锁存保持状态
    = 不允许普通 command、START 或自动 replan 退出
    = 只能通过遥控器切换到 onboard 模式后人工接管
```

```text
EMERGENCY_HOLD
        │
        ├─ 停止正常自主导航
        ├─ 停止 PI 跟随
        ├─ 保持当前安全位置/悬停
        ├─ 拒绝普通 command 的自动启动
        └─ 拒绝自动恢复到 EGO_EXECUTING 或 PI_EXECUTING

        直到：遥控器切换 onboard → 人工接管
```

组级安全联动由 `tcp_to_ros` 负责：任一 UAV 的普通 `FAILURE` 由 `tcp_to_ros` 汇总为批次失败；任一 UAV 报告单机安全事件时，`tcp_to_ros` 立即向全组传播 `HOLD`，其他 UAV 收到组级 `HOLD` 后统一进入 `EMERGENCY_HOLD`。

### 2.8 executor 监控和反馈

```text
ego driver 状态/运动结果
        │
        ▼
executor 监控
        │
        ├─ 正常完成/COMPLETED_HOLD
        │      └─ 向 tcp_to_ros 反馈 SUCCESS
        │             （包括 MOVE_TO、FAULT_EXIT、FOLLOW_ROUTE 和 HOVER）
        │
        ├─ 普通执行失败
        │      └─ 向 tcp_to_ros 反馈 FAILURE
        │
        └─ EMERGENCY_HOLD
               └─ 反馈安全故障/FAILURE，记录锁存状态
```

本机运动结果可抽象为：

```text
MotionResult
    = success / failure
    + error_code
    + message
```

反馈映射规则：

- 对 runtime 要求的任务正常完成，ego 进入 `COMPLETED_HOLD` 后，executor 向 `tcp_to_ros` 反馈 `SUCCESS`。
- 自动 TAKEOFF 完成并切换到 `COMPLETED_HOLD`，不单独向 `tcp_to_ros` 产生任务 `SUCCESS`；只有对应任务被执行并正常完成时，才反馈该任务的 `SUCCESS`。
- `EMERGENCY_HOLD` 统一映射为带安全错误码的 `FAILURE`。
- executor 只报告任务结果，不向 `tcp_to_ros` 持续转发 ego 当前状态。

TCP 接口的线上状态仍使用协议规定的 `ACCEPTED`、`COMPLETED`、`FAILED`；executor 向 `tcp_to_ros` 提供 `SUCCESS`、`FAILURE` 等内部任务结果，`tcp_to_ros` 汇总各 UAV 结果后，按《实物无人机上下层 TCP/IP 接口对接方案》第 4 节向 runtime 返回整组结果。

反馈时机沿用事件驱动语义：executor 在任务受理、启动、状态变化和终态时发布反馈；终态按配置进行有限次数重复发布，不按固定周期持续转发任务状态或 ego 当前状态。

`detail_stage` 继续作为 executor 的任务状态反馈阶段字段，用于补充 `status`、`error_code` 和 `message`，不作为 ego 状态反馈，也不映射 `INIT`、`TAKEOFF`、`EGO_EXECUTING`、`PI_EXECUTING`、`COMPLETED_HOLD` 或 `EMERGENCY_HOLD`。当前任务阶段值的生成规则保持不变。

---

---

## 3. ego_planner_driver 状态机详细描述

### 3.1 状态机总图

```text
程序启动
    │
    ▼
  INIT  （一次性描述阶段：初始化并等待 arm + OFFBOARD）
    │
    │ arm + OFFBOARD
    ▼
 TAKEOFF（一次性描述阶段：内部起飞进度达到 takeoff_height - 0.1 m 后切换）
    │
    │ 立即切换
    ▼
COMPLETED_HOLD
    │
    ├─ MOVE_TO / FAULT_EXIT / FOLLOW_ROUTE leader
    │       └─ executor 提供 waypoints
    │               ▼
    │         EGO_EXECUTING
    │               ├─ 到达 end_pt ─────────────► COMPLETED_HOLD
    │               ├─ pose 距离门禁触发 ───────► EMERGENCY_HOLD
    │               └─ 其他情况 ────────────────► 保持本状态
    │
    ├─ FOLLOW_ROUTE follower
    │       └─ executor 提供 leader ID + offset
    │               ▼
    │         PI_EXECUTING
    │               ├─ leader SUCCESS
    │               │   + 到达 leader + offset
    │               │   + 悬停达到规定时间 ─────► COMPLETED_HOLD
    │               └─ 其他情况 ────────────────► 保持本状态
    │
    ├─ HOVER ───────────────────────────────────► COMPLETED_HOLD
    │
    └─ HOLD / 安全事件 ─────────────────────────► EMERGENCY_HOLD
                                                     │
                                                     ├─ 锁存保持
                                                     ├─ 禁止普通自动退出
                                                     └─ onboard 人工接管
```

本图先给出 ego 状态机的总体结构，后续小节分别说明各状态的进入条件、状态内动作和退出条件。

### 3.2 状态范围和描述性阶段

```text
INIT 和 TAKEOFF 是为了描述启动过程而使用的阶段名称。
它们是一次性的启动过程，不是任务执行期间反复进入的正式长期运行状态。

正式运行状态：
  EGO_EXECUTING
  PI_EXECUTING
  COMPLETED_HOLD
  EMERGENCY_HOLD
```

### 3.3 INIT 阶段

```text
ego_planner_driver 程序启动
        │
        ▼
完成程序初始化
        │
        ▼
             INIT
        │
        ├─ 初始化 ego planner、executor 接口和内部数据
        ├─ 准备接收 executor 提供的任务信息
        ├─ 监测 PX4 状态
        └─ 等待 arm + OFFBOARD
                │
                │ 满足 arm + OFFBOARD
                ▼
             TAKEOFF
```

`INIT` 只用于表示“程序已经初始化，但尚未满足起飞前条件”的阶段。它不表示已经开始执行某个任务。

### 3.4 TAKEOFF 阶段

```text
TAKEOFF
   │
   ├─ 通过内部起飞进度计算目标高度
   └─ 判断是否达到 takeoff_height - 0.1 m
             │
             │ 达到切换条件
             ▼
      COMPLETED_HOLD
```

`TAKEOFF` 的规则是：arm + OFFBOARD 后进入 `TAKEOFF`；通过内部起飞进度计算目标高度；到达 `takeoff_height - 0.1 m` 后立即切换到 `COMPLETED_HOLD`；锁存当前位置和 yaw，开始持续发布保持 setpoint。

### 3.5 COMPLETED_HOLD 状态

```text
                         COMPLETED_HOLD
                                │
       ┌────────────────────────┼────────────────────────┐
       │                        │                        │
       │ 收到自主导航 command    │ 收到跟随 command        │ 收到 HOVER command
       ▼                        ▼                        ▼
  提供/接收 waypoints       提供 leader + offset       执行悬停保持
       │                        │                        │
       ▼                        ▼                        ▼
 EGO_EXECUTING             PI_EXECUTING            COMPLETED_HOLD

                         收到安全 command / 安全事件
                                      │
                                      ▼
                              EMERGENCY_HOLD
```

`COMPLETED_HOLD` 相当于当前代码中的 `HOLD` 状态，核心动作保持不变：

```text
COMPLETED_HOLD
        │
        ├─ 锁定当前位置/当前 yaw 的 setpoint
        ├─ 持续发布保持悬停所需控制
        ├─ 等待新的 command 或安全事件
        └─ 允许切换到其他合法状态
```

### 3.6 EGO_EXECUTING 状态

适用 command：

```text
MOVE_TO
FAULT_EXIT
FOLLOW_ROUTE（leader）
以及其他由本机根据 waypoints 自主执行的 command
```

状态内主流程：

```text
EGO_EXECUTING
        │
        ├─ 接收 executor 提供的 waypoints 序列
        ├─ 以 ego 算法生成、执行和必要时更新轨迹
        ├─ 持续发布自主导航 setpoint
        ├─ 处理 waypoint 推进和 end_pt 到达判断
        └─ 直接比较本机 pose 与邻机 pose，执行距离门禁
```

状态转换：

```text
EGO_EXECUTING
        │
        ├─ waypoints 序列完成，且到达 end_pt
        │  （当前 waypoint 推进使用 `reach_thresh_m`；最终 `end_pt` 使用
        │   `arrival_reach_thresh_m`，并连续满足 `arrival_stable_s`）
        │       ▼
        │   COMPLETED_HOLD
        │
        ├─ 直接 pose 距离门禁触发
        │  （仅对 fresh 邻机 pose 进行直接中心距离比较；距离小于
        │   `min_center_distance_m` 时触发，不使用 TTC 或未来轨迹预测）
        │       ▼
        │   EMERGENCY_HOLD
        │
        └─ 其他情况
                └─ 保持 EGO_EXECUTING
```

这里的距离门禁是当前设计中明确的安全切换条件：

```text
本机 pose + 邻机 pose
        │
        ▼
直接计算当前中心距离
        │
        ├─ 未触发门禁 → 继续 EGO_EXECUTING
        └─ 触发门禁   → 转入 EMERGENCY_HOLD
```

waypointsTracker 使用 `reach_thresh_m` 推进 waypoint 序列；最终 `end_pt` 使用 `arrival_reach_thresh_m` 判定到达，并连续满足 `arrival_stable_s` 后产生 `COMPLETE_REQUEST`，由请求仲裁器进入 `COMPLETED_HOLD`。直接 pose 距离门禁仅对 fresh 邻机 pose 进行直接中心距离比较，距离小于 `min_center_distance_m` 时直接进入 `EMERGENCY_HOLD`。具体动作分支见第 4 章。

### 3.7 PI_EXECUTING 状态

适用 command：

```text
FOLLOW_ROUTE（follower）
```

状态内主流程：

```text
PI_EXECUTING
        │
        ├─ 接收 executor 提供的 leader ID
        ├─ 接收本机相对 leader 的 offset
        ├─ 读取 leader 的运动状态/完成状态
        ├─ 计算 leader + offset 的跟随目标
        └─ 使用 PI 控制跟随并发布控制 setpoint
```

状态转换：

```text
PI_EXECUTING
        │
        ├─ leader 发布 SUCCESS
        │       │
        │       ├─ 本机尚未到达 leader + offset
        │       │       └─ 保持 PI_EXECUTING，继续跟随/收敛
        │       │
        │       └─ 本机到达 leader + offset
        │               │
        │               └─ 悬停保持达到规定时间
        │                       ▼
        │                 COMPLETED_HOLD
        │
        └─ 其他情况
                └─ 保持 PI_EXECUTING
```

leader 成功通知、follower 到达判定和悬停持续时间的精确定义记录在第 0 章，后续补充。

### 3.8 EMERGENCY_HOLD 状态

适用进入来源：

```text
HOLD command
EGO_EXECUTING 触发直接 pose 距离门禁
其他已确认的安全事件
```

当前设计目标：

```text
EMERGENCY_HOLD
        │
        ├─ 锁定当前安全位置/悬停 setpoint
        ├─ 停止正常 EGO 自主导航
        ├─ 停止 PI 跟随
        ├─ 不再允许普通 command 使其自动恢复
        ├─ 不再允许 START 或自动 replan 退出
        └─ 等待遥控器切换到 onboard 模式后人工接管
```

`EMERGENCY_HOLD` 的进入规则已经确定：`BRAKE_REQUEST` 经请求仲裁器接受后，直接锁定当前安全位置并进入 `EMERGENCY_HOLD`，不再先执行 EMERGENCY 制动轨迹。进入后停止正常自主导航、PI 跟随和正常 `trajectory_intent` 发布；不允许普通 command、START 或自动 replan 退出，只能等待遥控器切换到 onboard 模式后人工接管。

任一 UAV 报告单机安全事件后，由 `tcp_to_ros` 立即向全组传播 `HOLD`；其他 UAV 收到组级 `HOLD` 后统一进入 `EMERGENCY_HOLD`。

## 4. EGO_EXECUTING 状态内部动作分支

`EGO_EXECUTING` 不是由一个串行死循环独占完成，而是由三类并行动作分支共同维持：

```text
                         EGO_EXECUTING
                                │
          ┌─────────────────────┼─────────────────────┐
          │                     │                     │
          ▼                     ▼                     ▼
   A. 轨迹执行分支        B. 轨迹更新分支        C. 条件触发分支
   publishSetpoint()       request arbiter        条件检测器
          │                     │                     │
          │                     │                     ├─ replanTimer
          │                     │                     ├─ waypointsTracker
          │                     │                     ├─ collisionChecker
          │                     │                     └─ emergencyDog
          │                     │
          │                     └─ 按优先级消费请求
          │                           brake > complete > replan
          │
          └─ 持续向 PX4 发布当前轨迹 setpoint
```

三类分支的职责边界是：

```text
A：执行已经提交的 current trajectory
B：统一消费请求并执行轨迹更新
C：只检测条件并产生动作请求
```

C 分支不直接提交新轨迹；B 分支不区分普通 `REPLAN_REQUEST` 的具体来源。

### 4.1 A. 轨迹执行分支

```text
publishSetpoint() 周期运行
        │
        ├─ 读取当前已提交轨迹 current_
        ├─ 根据执行时间采样位置、速度、加速度和 yaw
        ├─ 发布当前轨迹 setpoint 给 PX4
        └─ 提供 waypointTracker 所需的当前位姿和执行时间
```

A 分支的职责是维持当前轨迹的实时执行，不负责提交新轨迹，也不直接决定普通 replan 或 brake。

```text
A. 轨迹执行分支
    = publish current trajectory setpoint
    = provide current pose and execution time
    = keep the current execution loop alive
```

### 4.2 B. 轨迹更新分支

B 分支是唯一执行 `replanOnce()` 并提交新轨迹的分支。

```text
B. 轨迹更新分支
        │
        ├─ replanTimer 产生的 REPLAN_REQUEST
        ├─ waypointsTracker 产生的 REPLAN_REQUEST
        └─ collisionChecker 产生的 REPLAN_REQUEST
                │
                ▼
       请求仲裁器确认无更高优先级请求
                │
                ▼
             replanOnce()
                │
        ┌───────┴────────┐
        │                │
        ▼                ▼
       成功             失败
        │                │
        ├─ 替换 current_  └─ 失败后的重试/结束策略待第 0 章确认
        ├─ 更新轨迹激活时间
        ├─ 发布新 intent
        └─ 保持 EGO_EXECUTING
```

B 分支不根据请求来自定时器、waypointTracker 还是 collisionChecker 选择不同的规划算法：

```text
REPLAN_REQUEST
    → 统一进入 replanOnce()
    → 统一执行规划、验证、提交和结果处理
```

### 4.3 C. 条件触发分支

C 分支只产生请求，不直接修改 `current_` 或提交新轨迹。

#### 4.3.1 replanTimer

`replanTimer` 属于 C 分支，因为它负责检查触发条件，而不是执行重规划。

```text
replanTimer 周期检查
        │
        ▼
读取当前最新已提交轨迹的年龄
        │
        │ trajectory_age = now - current_ 的激活时间
        │
        ├─ current_ 不存在
        │      └─ 不产生请求
        │
        ├─ trajectory_age < 1s
        │      └─ 不产生请求
        │
        └─ trajectory_age >= 1s
                └─ 产生 REPLAN_REQUEST
```

它不是“每隔 1 秒无条件调用一次 `replanOnce()`”，而是：

```text
周期检查轨迹年龄
    → 年龄达到 1s 后产生 REPLAN_REQUEST
    → B 分支消费请求并调用 replanOnce()
```

同一条轨迹年龄达到 1 秒后的重复请求抑制，以及 `replanOnce()` 失败后的重试/结束策略，暂不在本章定案，统一保留在第 0 章待确认。

#### 4.3.2 waypointsTracker

```text
waypointsTracker
        │
        ├─ 当前 waypoint 已到达
        │      ├─ 推进 waypoint 序列
        │      └─ 产生 REPLAN_REQUEST
        │
        ├─ 最终 end_pt 已到达且满足稳定条件
        │      └─ 产生 COMPLETE_REQUEST
        │
        └─ 其他情况
               └─ 不产生请求
```

waypointsTracker 不直接提交轨迹，也不直接设置最终状态，而是把结果交给请求仲裁器。

#### 4.3.3 collisionChecker

collisionChecker 在收到新的邻机 `trajectory_intent` 并完成合法性校验后立即检查：

```text
收到新的邻机 trajectory_intent
        │
        ▼
完成身份、协议、时间、版本和轨迹数据校验
        │
        ▼
使用：本机当前轨迹 vs 新收到的邻机 intent
        │
        ├─ 未发现碰撞
        │      └─ 不产生请求
        │
        └─ 发现预测碰撞
               └─ 产生 REPLAN_REQUEST
```

collisionChecker 的边界：

```text
· 使用当前本机轨迹与新邻机 intent 做同刻轨迹检查
· 与原版 ego 的收到新轨迹后检查逻辑一致
· 只产生 REPLAN_REQUEST
· 不直接 brake
· 不直接进入 EMERGENCY_HOLD
· 不把实时 pose 距离检查与 emergencyDog 重复
```

#### 4.3.4 emergencyDog

`emergencyDog` 是独立的最高优先级安全条件检测分支：

```text
emergencyDog（20 Hz）
        │
        ▼
读取本机 pose 和邻机 pose
        │
        ▼
直接计算当前中心距离
        │
        ├─ 未触发紧急距离门禁
        │      └─ 不产生请求
        │
        └─ 触发紧急距离门禁
               └─ 产生 BRAKE_REQUEST
```

其明确边界是：

```text
· 不使用 TTC 或其他未来轨迹预测
· 不等待邻机 intent 超时
· 不产生普通 REPLAN_REQUEST
· 不直接在检测回调中修改 current_
· 只提交 BRAKE_REQUEST
```

### 4.4 请求仲裁器和动作优先级

所有条件分支只提交请求，由统一仲裁器决定下一动作：

```text
请求集合：
  BRAKE_REQUEST
  COMPLETE_REQUEST
  REPLAN_REQUEST

优先级：
  BRAKE_REQUEST > COMPLETE_REQUEST > REPLAN_REQUEST
```

```text
请求仲裁器
        │
        ├─ 存在 BRAKE_REQUEST
        │      └─ 直接进入 EMERGENCY_HOLD
        │
        ├─ 否则存在 COMPLETE_REQUEST
        │      └─ 进入 COMPLETED_HOLD
        │
        ├─ 否则存在 REPLAN_REQUEST
        │      └─ B 分支调用 replanOnce()
        │
        └─ 没有请求
               └─ 保持 EGO_EXECUTING
```

优先级的含义：

```text
BRAKE_REQUEST 与 COMPLETE_REQUEST 同时存在
    → 先 brake，不允许以完成状态掩盖安全事件

COMPLETE_REQUEST 与 REPLAN_REQUEST 同时存在
    → 先 complete，不为已经到达 end_pt 的任务继续普通 replan

BRAKE_REQUEST 被接受
    → 清除/废弃普通 COMPLETE_REQUEST 和 REPLAN_REQUEST
    → 直接进入 EMERGENCY_HOLD
```

### 4.5 EGO_EXECUTING 内部总流程

```text
进入 EGO_EXECUTING
        │
        ▼
┌──────────────────────────────────────────────────────┐
│ A. publishSetpoint                                    │
│    持续采样 current_，发布 PX4 setpoint               │
│    提供当前 pose 和执行时间                            │
└──────────────────────┬───────────────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────────────┐
│ C. 条件触发                                            │
│    replanTimer：轨迹年龄 >= 1s → REPLAN_REQUEST       │
│    waypointsTracker：推进 → REPLAN_REQUEST             │
│    waypointsTracker：到达 end_pt → COMPLETE_REQUEST   │
│    collisionChecker：新 intent 冲突 → REPLAN_REQUEST  │
│    emergencyDog：pose 门禁 → BRAKE_REQUEST             │
└──────────────────────┬───────────────────────────────┘
                       │
                       ▼
┌──────────────────────────────────────────────────────┐
│ 请求仲裁器                                             │
│    BRAKE > COMPLETE > REPLAN                          │
└───────────────┬──────────────┬────────────────────────┘
                │              │
                ▼              ▼
          EMERGENCY_HOLD   COMPLETED_HOLD
                │
                └──────────────┐
                               ▼
┌──────────────────────────────────────────────────────┐
│ B. 轨迹更新                                            │
│    REPLAN_REQUEST → replanOnce()                      │
│    成功：替换 current_、更新时间、发布 intent           │
│    失败：按失败策略重试或结束                            │
│    成功后继续 EGO_EXECUTING                            │
└──────────────────────────────────────────────────────┘
```

本章确定了 `EGO_EXECUTING` 内部的职责拆分和请求优先级；尚未确定的 `replanOnce()` 失败重试策略、普通失败错误码细分和 PI_EXECUTING 完成反馈边界仍记录在第 0 章。
