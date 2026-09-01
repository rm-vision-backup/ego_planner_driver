# Offboard 新状态机设计（2026-09-02）

本文逐步整理 `swarm_uav_executor` 与 `ego_planner_driver` 的新状态机设计。任务名称与固定命令按《实物无人机上下层 TCP/IP 接口对接方案》区分：任务名称表示高层目的，可以任意命名；`MOVE_TO`、`FOLLOW_ROUTE`、`FAULT_EXIT`、`HOVER`、`HOLD` 等是固定的 `command` 字段。

---

## 0. 尚待确认的反馈细节

以下反馈、状态动作和联动细节尚未最终确认，暂不作为已经实现的状态机事实：

- `SUCCESS` 是否只对应 `COMPLETED_HOLD`
  - `HOVER` command 是否以 `COMPLETED_HOLD` 作为成功终态
- 普通失败和 `EMERGENCY_HOLD` 是否使用不同错误码
  - 规划失败、跟随失败、超时、通信失败是否分别编码
  - `EMERGENCY_HOLD` 是否使用独立安全故障码
- executor 是持续反馈，还是只在状态变化时反馈
  - `ACCEPTED`、`START`、终态是否足够
  - 是否还要持续上报 ego 当前状态
- 单机失败如何触发批次级失败和全组安全动作
  - executor 只反馈本机 `FAILURE`，还是主动触发组级 `HOLD`
  - tcp_to_ros 是否负责汇总 `FAILED` 和传播全组 `HOLD`
- `INIT` 和 `TAKEOFF` 的描述性边界
  - 两者是否只作为启动过程的方便命名，不计入正式长期运行状态集合
  - `TAKEOFF` 达到 `takeoff_height` 后需要持续保持条件多长时间
- `EGO_EXECUTING` 的完成和安全切换条件
  - `waypoints` 序列完成、到达 `end_pt` 的精确定义
  - pose 直接比较的距离门禁阈值、检查频率和失败反馈
  - 进入 `EMERGENCY_HOLD` 后需要停止哪些规划、执行和 intent 动作
- `PI_EXECUTING` 的完成条件
  - leader 的 `SUCCESS` 如何发布和被 follower 识别
  - follower 到达 `leader + offset` 后需要悬停保持多长时间
- `EMERGENCY_HOLD` 的内部动作
  - 是否保留 EMERGENCY 制动轨迹
  - 是否直接锁定当前位置 setpoint 并保持悬停
  - 如何停止正常轨迹发布、自动 replan 和普通 command
- ego 状态与 executor 反馈的映射
  - 普通失败和安全锁存是否分别映射为不同 `MotionResult`
  - 单机安全状态如何触发批次级失败和全组 HOLD

当前暂定的反馈边界：

```text
executor 监控 ego driver
        │
        ├─ 正常完成/进入 COMPLETED_HOLD → 向 tcp_to_ros 反馈成功结果
        ├─ 普通执行失败                 → 向 tcp_to_ros 反馈 FAILURE
        └─ 进入 EMERGENCY_HOLD          → 反馈安全故障/FAILURE
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
│    · TAKEOFF：满足 arm + OFFBOARD 后执行起飞（一次性描述阶段）        │
│    · EGO_EXECUTING：使用 ego 算法自主导航                            │
│    · PI_EXECUTING：使用 PI 控制跟随 leader                            │
│    · COMPLETED_HOLD：正常完成/悬停后的可退出保持                      │
│    · EMERGENCY_HOLD：安全故障后的锁存保持                            │
│  · 向 executor 提供当前状态和执行结果                                │
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
                         ACCEPTED/START
                               │
          ┌────────────────────┼────────────────────┐
          │                    │                    │
          ▼                    ▼                    ▼
   MOVE_TO / FAULT_EXIT   FOLLOW_ROUTE leader   FOLLOW_ROUTE follower
          │                    │                    │
          └──────────────┬─────┘                    │
                         ▼                          ▼
              executor 发布 waypoints       executor 发布 leader + offset
                         │                          │
                         ▼                          ▼
                  EGO_EXECUTING                PI_EXECUTING
                         │                          │
                         └──────────────┬───────────┘
                                        │ 正常完成/停止
                                        ▼
                                 COMPLETED_HOLD
                                        │
                         ┌──────────────┼──────────────┐
                         │              │              │
                         ▼              ▼              ▼
                  新自主导航       新跟随 command    HOVER command
                         │              │              │
                         ▼              ▼              ▼
                  EGO_EXECUTING    PI_EXECUTING   COMPLETED_HOLD

任意可运行状态 ── HOLD command / 安全事件 ──► EMERGENCY_HOLD
```

```text
EMERGENCY_HOLD
        │
        ├─ 锁存
        ├─ 不允许普通 command 或 START 退出
        ├─ 不允许自动 replan 恢复普通执行
        └─ 仅遥控器切换 onboard 后人工接管
```

此图是当前设计骨架，不代表所有切换条件、失败分支、反馈码和全组联动细节已经确定；这些内容在第 0 章确认后继续补充。
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

### 2.8 executor 监控和反馈

```text
ego driver 状态/运动结果
        │
        ▼
executor 监控
        │
        ├─ 正常完成/COMPLETED_HOLD
        │      └─ 向 tcp_to_ros 反馈成功结果
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

TCP 接口的线上状态应使用协议规定的 `ACCEPTED`、`COMPLETED`、`FAILED`；`SUCCESS`、`FAILURE` 是 executor/runtime 内部的任务结果语义，具体映射仍见第 0 章。

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
 TAKEOFF（一次性描述阶段：达到 takeoff_height 并持续保持）
    │
    │ 满足持续高度条件
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
   ├─ 继续向 PX4 发布起飞所需控制
   ├─ 监测本机高度/位姿
   └─ 判断是否达到 takeoff_height
             │
             │ 达到 takeoff_height
             ▼
      持续保持该条件
             │
             │ 满足现有的持续时间条件
             ▼
      COMPLETED_HOLD
```

`TAKEOFF` 结束条件保持当前程序语义：不是第一次达到 `takeoff_height` 就立即结束，而是达到高度后继续保持一段时间，满足持续条件后才进入 `COMPLETED_HOLD`。

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
        │       ▼
        │   COMPLETED_HOLD
        │
        ├─ 触发直接 pose 距离门禁
        │  （不使用预测；直接比较邻机和自身 pose）
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

该状态的规划、replan、waypoint 推进、end_pt 判定、setpoint 发布和距离门禁动作将在后续章节继续展开；具体阈值和边界记录在第 0 章。

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

`EMERGENCY_HOLD` 相当于当前代码中的 `BRAKE_HOLD`，但是否保留进入该状态前的 EMERGENCY 制动轨迹仍未确定：

```text
安全事件 / HOLD command
        │
        ├─ 方案 A：直接锁定当前位置 setpoint
        │       └─ 进入 EMERGENCY_HOLD
        │
        └─ 方案 B：先执行 EMERGENCY 制动轨迹
                └─ 制动完成后进入 EMERGENCY_HOLD
```

两种方案的选择、轨迹发布是否停止、是否需要组级 HOLD 传播以及状态锁存细节，均记录在第 0 章待确认。
