# Offboard 新状态机设计（2026-09-02）

本文逐步整理 `swarm_uav_executor` 与 `ego_planner_driver` 的新状态机设计。任务名称与固定命令按《实物无人机上下层 TCP/IP 接口对接方案》区分：任务名称表示高层目的，可以任意命名；`MOVE_TO`、`FOLLOW_ROUTE`、`FAULT_EXIT`、`HOVER`、`HOLD` 等是固定的 `command` 字段。

---

## 0. 尚待确认的反馈细节

以下反馈和联动细节尚未最终确认，暂不作为已经实现的状态机事实：

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

## 3. 当前已确认的状态机骨架

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
