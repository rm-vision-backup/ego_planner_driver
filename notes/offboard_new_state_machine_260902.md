# Offboard 新状态机设计（2026-09-02）

本文逐步整理 `swarm_uav_executor` 与 `ego_planner_driver` 的新状态机设计。当前章节只记录已经确认的系统分层和任务受理边界；后续状态、切换条件和状态内动作在逐条确认后追加。

---

## 1. 系统分层

### 1.1 总体职责关系

```text
┌──────────────────────────────────────────────────────────────────────┐
│                         tcp_to_ros / GCS                             │
│                                                                      │
│  · 接收批次任务                                                     │
│  · 执行批次级 prepare、排程和任务协调                               │
│  · 向各 UAV executor 发送单机任务请求                               │
│  · 接收各 UAV 的 ACCEPTED / START / SUCCESS / FAILURE 等反馈         │
└──────────────────────────────┬───────────────────────────────────────┘
                               │
                               │ bridge 转发任务与状态话题/服务
                               ▼
┌──────────────────────────────────────────────────────────────────────┐
│                      swarm_uav_executor                              │
│                                                                      │
│  任务接入与任务适配层                                                │
│                                                                      │
│  · 接收 tcp_to_ros 发来的单机任务                                   │
│  · 校验、保存并返回 ACCEPTED                                        │
│  · 等待 GCS 后续发送 START                                          │
│  · 按任务类型整理 ego driver 所需输入                               │
│  · 向 ego driver 提供 waypoints、leader ID、offset、HOVER/HOLD 信息  │
│  · 启动任务执行线程                                                  │
│  · 监控 ego driver 状态和本机运动结果                               │
│  · 向 tcp_to_ros 反馈 ACCEPTED、START、SUCCESS、FAILURE 等结果       │
└───────────────┬──────────────────────────────┬───────────────────────┘
                │                              │
                │ 提供不同任务信息             │ 接收 ego 状态/结果
                ▼                              │
┌──────────────────────────────────────────────────────────────────────┐
│                       ego_planner_driver                             │
│                                                                      │
│  飞行控制与内部状态机层                                              │
│                                                                      │
│  · 接收 executor 提供的任务信息                                     │
│  · 根据收到的信息自行进行内部状态转换                               │
│  · EGO_EXECUTING：使用 ego 算法自主导航                             │
│  · PI_EXECUTING：使用 PI 控制跟随 leader                             │
│  · COMPLETED_HOLD：正常完成/悬停后的可退出保持                       │
│  · EMERGENCY_HOLD：安全故障后的锁存保持                             │
│  · 向 executor 提供当前状态和执行结果                               │
└──────────────────────────────┬───────────────────────────────────────┘
                               │
                               │ setpoint / position / velocity control
                               ▼
┌──────────────────────────────────────────────────────────────────────┐
│                              PX4                                     │
│                                                                      │
│  · 接收 offboard 控制 setpoint                                      │
│  · 执行位置、速度或相关飞行控制                                     │
│  · 遥控器切换 onboard 后可接管                                       │
└──────────────────────────────────────────────────────────────────────┘
```

### 1.2 核心边界

```text
executor 不直接发送“切换 ego 状态”的专用命令

executor 接收任务
    │
    ├─ 识别任务类型
    ├─ 整理任务内容
    ├─ 发布 ego 所需信息
    └─ 启动对应 driver 执行路径
            │
            ▼
ego driver 根据收到的信息
自行进行内部状态转换
```

因此，文档中的“使 ego driver 进入某状态”表示：

```text
executor 提供了足够且正确的输入
    → ego driver 按自身状态机转换到预期状态
```

并不表示 executor 绕过 ego driver、直接写入或强制设置其内部状态。

---

## 2. ACCEPTED/START 机制

### 2.1 命名约定

本文使用：

```text
ACCEPTED/START 机制
```

避免把 executor 的单机任务受理阶段称为 `PREPARE`，因为 `prepare` 已经用于 tcp_to_ros 的批次任务协调。

```text
批次任务 prepare
    = GCS/tcp_to_ros 层面的批次排程、整批受理和启动协调

单机 ACCEPTED
    = 某一架 UAV 的 executor 已校验并保存任务，等待启动

START
    = GCS 命令 executor 启动已 ACCEPTED 的单机任务
```

### 2.2 ACCEPTED 阶段

```text
GCS / tcp_to_ros
        │
        │ 单机任务请求
        ▼
executor 接收任务
        │
        ├─ 校验请求和任务内容
        ├─ 检查本机 executor 是否可用
        ├─ 将任务转换为内部 goal
        ├─ 调用 driver 的受理检查
        ├─ 保存任务记录
        └─ 返回 ACCEPTED
                │
                ▼
        任务已受理，但尚未开始运动
```

`ACCEPTED` 的语义是：

```text
“本机 executor 已接受该任务，并已具备等待 START 的条件。”
```

它不表示：

```text
· 任务已经开始执行
· ego 已经进入 EGO_EXECUTING
· PI 跟随已经开始
· 任务已经成功完成
```

### 2.3 START 阶段

```text
GCS / tcp_to_ros
        │
        │ 等待批次协调完成
        │ 按计划发送 START
        ▼
executor 收到 START
        │
        ├─ 查找对应 ACCEPTED 任务
        ├─ 检查任务仍未启动且未终止
        ├─ 启动本机动作线程
        ├─ 向对应 driver 提供任务输入
        └─ 返回 START accepted / started
                │
                ▼
        driver 开始根据任务类型运行
```

START 的职责是启动已经 ACCEPTED 的任务，而不是重新解析或重新受理任务。

### 2.4 当前完整链路

```text
批次任务
   │
   ▼
tcp_to_ros 执行批次 prepare / 排程
   │
   ├─ 某 UAV-1 ──单机任务请求──► executor-1 ──► ACCEPTED
   ├─ 某 UAV-2 ──单机任务请求──► executor-2 ──► ACCEPTED
   └─ ...
   │
   ▼
全部或规定参与者 ACCEPTED
   │
   ▼
tcp_to_ros 按批次协调结果发送 START
   │
   ├─ executor-1 ──► 启动任务
   ├─ executor-2 ──► 启动任务
   └─ ...
   │
   ▼
各 executor 根据任务类型向 ego driver 提供不同输入
   │
   ▼
ego driver 自行进入预期运行状态
```

---

## 3. executor 的任务分类与 ego 预期状态

executor 不直接写入 ego 的内部状态，而是按任务类型提供不同信息，使 ego driver 进入相应的预期状态。

### 3.1 导航任务：EGO_EXECUTING

适用任务：

```text
MOVE_TO
FAULT_EXIT
FOLLOW_ROUTE（leader）
以及其他包含航点、由本机自主导航的任务
```

流程：

```text
ACCEPTED/START
        │
        ▼
executor 解析任务航点
        │
        ├─ 根据任务内容生成 waypoints 序列
        ├─ 进行必要的航点处理/密化
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

`EGO_EXECUTING` 与 `PI_EXECUTING` 的核心区别：

```text
EGO_EXECUTING
    = ego 算法根据目标航点自主规划和执行

PI_EXECUTING
    = PI 控制根据 leader 和 offset 跟随
```

### 3.2 跟随任务：PI_EXECUTING

适用任务：

```text
FOLLOW_ROUTE（follower）
```

流程：

```text
ACCEPTED/START
        │
        ▼
executor 解析跟随任务
        │
        ├─ 提供 leader ID（UAVn）
        ├─ 提供本机相对 leader 的 offset
        └─ 向 ego driver 发布跟随所需信息
                │
                ▼
        ego driver 根据跟随信息自行转换状态
                │
                ▼
             PI_EXECUTING
                │
                ▼
        使用 PI 控制跟随 leader
```

### 3.3 悬停任务：COMPLETED_HOLD

适用任务：

```text
HOVER
```

流程：

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

`COMPLETED_HOLD` 是正常流程中的保持状态，允许后续切换到其他合法状态。

```text
COMPLETED_HOLD
      │
      ├─ 新的导航任务 ─► EGO_EXECUTING
      ├─ 新的跟随任务 ─► PI_EXECUTING
      ├─ 新的 HOVER   ─► COMPLETED_HOLD
      └─ 安全 HOLD     ─► EMERGENCY_HOLD
```

具体切换条件和状态内动作待后续逐条确认。

### 3.4 安全任务：EMERGENCY_HOLD

适用任务：

```text
HOLD
```

流程：

```text
收到安全 HOLD 请求或安全事件
        │
        ▼
executor 提供安全保持所需信息
        │
        ▼
ego driver 自行转换状态
        │
        ▼
      EMERGENCY_HOLD
```

`EMERGENCY_HOLD` 与 `COMPLETED_HOLD` 的区别：

```text
COMPLETED_HOLD
    = 正常完成/悬停
    = 允许切换到其他状态

EMERGENCY_HOLD
    = 安全故障后的锁存保持
    = 不允许普通任务、START 或自动 replan 退出
    = 只能通过遥控器切换到 onboard 模式后人工接管
```

期望结构：

```text
EMERGENCY_HOLD
        │
        ├─ 拒绝新的普通任务启动
        ├─ 拒绝自动恢复到 EGO_EXECUTING
        ├─ 拒绝自动恢复到 PI_EXECUTING
        └─ 保持当前安全位置/悬停控制

        直到：
        遥控器切换到 onboard 模式
                │
                ▼
        人工接管，且不再依赖 offboard 程序
```

---

## 4. executor 的反馈职责

### 4.1 向 tcp_to_ros 反馈 ACCEPTED/START

```text
单机任务请求
        │
        ▼
executor 校验并保存
        │
        ▼
返回 ACCEPTED
        │
        ▼
等待 GCS START
        │
        ▼
收到 START 后启动任务
        │
        ▼
返回 START accepted / started
```

这组反馈用于 tcp_to_ros 判断：

```text
· 本机是否已经接受任务
· 本机是否可以进入启动阶段
· 本机是否已经启动动作线程
· 批次任务是否可以继续协调
```

### 4.2 监控 ego 状态并反馈运动结果

```text
ego driver 状态/结果
        │
        ▼
executor 监控
        │
        ├─ 正常完成
        │      └─ 向 tcp_to_ros 反馈 SUCCESS
        │
        ├─ 进入 COMPLETED_HOLD
        │      └─ 作为正常完成/保持结果反馈
        │
        ├─ 普通执行失败
        │      └─ 向 tcp_to_ros 反馈 FAILURE
        │
        └─ 进入 EMERGENCY_HOLD
               ├─ 反馈安全故障/FAILURE
               └─ 记录本机进入锁存安全状态
```

这里的本机运动结果可以理解为 executor 对 ego driver 结果的任务级封装：

```text
MotionResult
    = success / failure
    + error_code
    + message
```

后续需要继续确认：

```text
· SUCCESS 是否只对应 COMPLETED_HOLD
· FAILURE 是否区分普通规划失败和 EMERGENCY_HOLD
· EMERGENCY_HOLD 是否需要专门的状态反馈码
· executor 是否持续反馈状态，还是只在状态变化时反馈
· tcp_to_ros 是否负责把单机 FAILURE 传播为批次级失败和全组安全动作
```

---

## 5. 当前已确认的状态机骨架

```text
                         ACCEPTED/START
                               │
          ┌────────────────────┼────────────────────┐
          │                    │                    │
          ▼                    ▼                    ▼
       导航任务             跟随任务              HOVER
          │                    │                    │
          ▼                    ▼                    ▼
   executor 发布          executor 发布       executor 发布
   waypoints 等信息       leader + offset      HOVER 信息
          │                    │                    │
          ▼                    ▼                    ▼
   EGO_EXECUTING         PI_EXECUTING       COMPLETED_HOLD
          │                    │                    │
          │                    │                    ├─允许普通状态切换
          │                    │                    │
          └────────────┬───────┴────────────────────┘
                       │
                       │ 安全 HOLD / 安全事件
                       ▼
                EMERGENCY_HOLD
                       │
                       ├─锁存
                       ├─不允许普通状态退出
                       └─仅遥控器切换 onboard 后人工接管
```

此图是当前设计骨架，不代表所有切换条件、失败分支和全组联动细节已经确定。后续内容将在状态机每条边的功能描述确认后继续补充。
