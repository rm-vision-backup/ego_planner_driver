# 当前源码运行架构（现状图鉴，2026-09-08 重构）

> 只画「当前源码怎么跑」，不是设计。目标设计见 `offboard_new_state_machine_260902.md`（051e84e）；
> 将落地的加固见 `src/tcp_to_ros/plans/implementation_plan_26090802.md`（未实施，图中以 ⚑26090802 标注）。
> [源码事实] 可直接找到；[推导] 由源码连接得出；[待确认] 需实测。

## 1. 系统分层

```text
┌───────────────────────────────────────────────────────────────┐
│ runtime / TCP 对端（JSON line，任务层 Axx）                    │
└───────────────────────────┬───────────────────────────────────┘
                            │ TCP:39001
┌───────────────────────────▼───────────────────────────────────┐
│ tcp_to_ros / GCS_A  Master(11310)                              │
│  tcp_server_node  → ExecuteGroupTask Action                    │
│  gcs_a_backend_node：Axx→UAVn、t_off 排程、PREPARE/START/聚合   │
│  gcs_a_calibration（原点/航向冻结）                             │
└──────┬──────────────────────────────┬──────────────────────────┘
       │ 本机 Action                  │ 跨 Master Service/Topic
┌──────▼───────────────┐   ┌──────────▼──────────────────────────┐
│ ExecuteGroupTask     │   │ swarm_topology_bridge（通用代理）    │
└──────┬───────────────┘   │  Topic PUB/SUB + Service ROUTER     │
       │                  └──────┬──────────────────┬─────────────┘
       │                         │                  │
┌──────▼─────────────────────────▼──────┐   ┌───────▼──────────────┐
│ 每架 UAV 本机 Master(11311..11325)      │   │ /UAVn/uav_task*     │
│  uav_executor_node（PREPARE/START/HOLD） │   │ /UAVn/uav_task_state│
│  EgoSwarmDriver（EGO waypoints / follPI）│   │ /UAVn/intent·odom·  │
│  ego_planner_driver(C++)                 │   │  state 转发          │
│  pose_relay / setpoint_relay             │   └────────────────────┘
└──────────────────────┬──────────────────┘
                       │ /setpoint/ego 候选（C++）  /setpoint/follower（PI）
┌──────────────────────▼─────────────────────────────────────────┐
│ setpoint_relay 30Hz：选源/校验/失效 HOLD → /mavros/setpoint_raw │
└──────────────────────┬─────────────────────────────────────────┘
                       │
┌──────────────────────▼─────────────────────────────────────────┐
│ MAVROS → PX4（offboard；arm/OFFBOARD 归外部脚本）               │
└─────────────────────────────────────────────────────────────────┘
```

身份：任务层 Axx →(uav_identity_map, 静态)→ 执行层 UAVn。
并发：`tcp_to_ros` 同时只允许一个活动 GroupState（多余命令 → COMMAND_ALREADY_ACTIVE）。

## 2. 整组任务时序（backend）

```text
TCP COMMAND
  │
  ▼
tcp_server：解析/去重(command_id)/HOLD→EmergencyHold
  │
  ▼
ExecuteGroupTask goal ──► backend 校验：assignments 非空/标定就绪/编队与 leader/身份解析/无活动组
  │
  ├─ t_off 排程（仅 MOVE_TO/FAULT_EXIT 且启用）
  │    输入：bridge pose(x,y) + 目标(已转 local ENU) + ScheduleParams
  │    现状：geometric 1.0 + safety_margin 1.0 ⇒ 预测 2.0m；失败→SCHEDULE_* FAILED（不 PREPARE）
  │
  ▼
并发 PREPARE(/UAVn/uav_task)        全部 accepted？
  ├─ 任一拒/超时 ──► ABORT 已接受机 ──► 整组 FAILED
  └─ 全 accepted
        │
        ▼
START(/UAVn/uav_task_control)
  ├─ 排程路径：每架 worker 等 t0+schedule_tau[Axx] 再 START（独立线程，不串行累加）
  └─ 非排程：普通并发 START
        │   任一失败 → ABORT/安全 HOLD → 整组 FAILED
        ▼
Action ACCEPTED ──► 等各机 UavTaskState 终态
        ├─ 全 COMPLETED → Action/TCP COMPLETED
        ├─ 任一 FAILED  → 组 HOLD fan-out → Action/TCP FAILED
        ├─ timeout      → 组 HOLD → FAILED
        └─ cancel       → 组 HOLD → CANCELLED
```

| command | assignment 输入 |
|---|---|
| MOVE_TO | `target_pose` |
| FOLLOW_ROUTE leader | `waypoints`；leader_id ∈ assignments |
| FOLLOW_ROUTE follower | `formation_follow=true` + `formation_offset` |
| FAULT_EXIT | `waypoints`（最终退出点） |
| HOVER | 参与 UAV |
| HOLD | 安全语义：走 EmergencyHold/UavHold，非普通 driver 任务 |

> `hold-a-end` 是计划名，线上 command 仍是 HOVER，勿写成 TCP HOLD。

## 3. executor 单机生命周期

```text
UavTask PREPARE
  │
  ├─ validate：protocol/identity/command/timeout/assignment
  ├─ TaskKey=(mission,command,uav)；指纹幂等（同指纹重发，异指纹 DUPLICATE_CONFLICT）
  ├─ 空闲检查：BUSY / driver.health / prepare(armed+OFFBOARD+origin)
  └─ register → 发布 ACCEPTED（require_explicit_start=true → 等 START）

UavTaskControl START        UavHold
  │                          │ 置 cancel
  ├─ mark_started ─┬─► _run_task daemon（deadline=prepared_at+timeout）
  │                │         ├─ 运动 success → COMPLETED
  │                │         └─ 运动失败 → 先 driver.hold → FAILED/HOLD[_FAILED]
  │                └─ START 后 ABORT 被拒（须 HOLD）
```

## 4. EgoSwarmDriver（Python 适配层）

```text
             ┌─ EGO 路径（MOVE_TO/FAULT_EXIT/FOLLOW_ROUTE leader）
             │   分层关键点(15/8/12m) → 3m 密化 → /waypoints(Polygon) ──► C++
PREPARE ─────┤
             └─ follower 路径（FOLLOW_ROUTE follower）
                  leader odom + offset + own pose
                      │
                      ▼  PI loop(10Hz 算, 30Hz 发) /setpoint/follower(2496)
                      │  direct_control_active=true → relay 选 follower
                      ▼  退出 finally 置 false → relay 回 EGO
```

发布：`goal` `goal_yaw` `waypoints` `hold` `setpoint`(follower) `direct_control_active`
      `neighbor_intent`（远端 intent → C++ `neighbor_intent` 转发）
订阅：`exec_state` `local_pose` `mavros/state`(可选) `gp_origin_confirmed`
      各 `/UAVn/trajectory_intent`、邻机 odom(门禁)、leader odom(follower)

follower 失败：LEADER_LOST / POSE_STALE / MIN_DISTANCE_BREACH / LOCAL_TIMEOUT / COMMAND_HELD
`prepare()`：ego 可见+pose 新鲜+MAVROS connected+armed+OFFBOARD(+MOVE_TO origin_confirmed)；不自动 arm。

## 5. C++ driver 状态机（现状 8 态）

```text
程序启动 ──► HOLD（初始；常量另有 IDLE 仅供 arm+OFFBOARD guard 兼容）
             │
             │ arm + OFFBOARD
             ▼
          TAKEOFF ──软升到 takeoff_height(5m)──► HOLD
             │
   ┌─────────┴───────────────────────────────┐
   │ goal / waypoints                        │
   ▼                                         │
EXECUTING ◄──────────────────────────────────┘（有轨迹时）
   │ 推进(距当前目标<0.5m→切下一点) ──► 置位→replan（保持 EXECUTING）
   │ 到达(距最终点<0.5m 且稳定 1s) ──► COMPLETED ──► HOLD(冻结终点)
   │ 执行超时 ──► EGO_EXEC_TIMEOUT ──(executor)──► HOLD
   │ replan 连续失败 ≥10s ──► EGO_PLAN_FAILED ──(executor)──► HOLD
   │ EMERGENCY：replan 失败 + ttc<1s ──► EMERGENCY_BRAKE ──制动轨迹耗尽──► BRAKE_HOLD(锁存)
   │                                     ⚑26090802：改为直接锁存 BRAKE_HOLD（去制动轨迹），
   │                                       且 BRAKE_HOLD 不再可被新 goal/HOLD/direct_control 转出
   └─ 其他 → 保持 EXECUTING
```

运行线程/timer：`publishSetpoint`(30Hz) / `runSafetySupervisor`(25Hz) / `collisionCheckTick`(10Hz)
/ `replanLoop`(独立线程，条件变量)。发布 `/setpoint/ego` `/trajectory_intent` `/exec_state`(latch)。

### 5.1 触发 → 单一提交者（现状）

```text
定时 replan ─────────────┐
推进/到达 ──────────────► planning_requested_ = true ─┐
collision(10Hz,<1.3×cl) ┘                             ▼
                                          replanLoop ──► replanOnce()
SafetyPredictor EMERGENCY（预测）          （唯一轨迹提交者）   │
   min_d≤1.0 或 ttc≤1.5  ─► emergency_pending_ + 唤醒 ──┘    ├─ 常规 replan / 候选 / 制动
                                                                  └─ 提交 current_+intent
```

SafetyPredictor 判定基于【预测】：current_ vs intent 同刻采样 CPA/TTC/min_d；非实时位置。
实时位置门禁是另一层（executor 3D odom <1.0m → /hold）；`self_odom_dev_m` 仅诊断。

## 6. replanOnce 内部（现状决策流）

```text
replanOnce(timed_trigger)
  │
  ├─ 门控：hold/direct_control/TAKEOFF/无 trigger/无 pending_goal/steady 锁存/pose 过期 → 返回
  ├─ handleArrivalBeforeSnapshot（真实 odometry 推进/最终到达）
  ├─ 快照：起点 P/V/A = 旧轨迹同刻；目标 = waypoints_[waypoint_consumed_]
  │
  ├─ activation retry(≤3) planOnce ──► PlanResult
  │     ├─ RESOURCE_LIMIT    ──► EGO_PLAN_FAILED + HOLD target（不重试/不 yield）
  │     ├─ DEGENERATE_SEGMENT ─► 保留 current_（到达/重试路径）
  │     ├─ RETRYABLE_FAILURE  ─► 更新 assumed_activation 重试
  │     └─ SUCCESS            ─► checkClearance → 提交（锁内复验最新 intent + safety_generation_）
  │
  └─ 失败分支
        ├─ EMERGENCY_pending_ ? ──► tryEmergencyBrakeLocked（受控制动轨迹，现状 ⚑26090802 改直接锁存）
        └─ 普通失败 ──► tryYieldReplan（SLOW/LEFT/RIGHT + STAND_ON/YIELD）
              ├─ COMMITTED → 提交候选
              ├─ RESOURCE_LIMIT → EGO_PLAN_FAILED
              └─ NOT_AVAILABLE → 保留 current_/末端
                    └─ soft-landing：1s 定时+事件重试；连续 ≥ plan_fail_timeout_s(10s) → EGO_PLAN_FAILED
                          ⚑26090802：改 retry 0.6s×同输入最多5次（参数），去 soft-landing 词
```

现状**无**：`ReplanSlot/input_version/replan_running/retry_not_before/仲裁器对象`；
`safety_generation_` 只作安全/制动代次检查，不含 waypoint/intent 输入版本（⚑26090802 变更 B 补）。

## 7. setpoint 出口（relay）

```text
/setpoint/ego(C++,2048) ──┐
/setpoint/follower(PI,2496)─┼─► direct_control_active ? 选源
/direct_control_active ─────┘     │
                                  ├─ 校验 mask/frame/finite/freshness(cand≤0.2s,pose≤1.0s)
                                  ├─ 切换 generation++ 作废旧候选
                                  └─ 无效/过期 → 固定 position/yaw HOLD(mask2552,锁存 fresh pose)
                                          │
                                          ▼
                                  /mavros/setpoint_raw/local（唯一业务出口）→ PX4
```

## 8. 现状 - 目标差异速查

| 项 | 现状 | 目标(051e84e) / 26090802 |
|---|---|---|
| 提交入口 | replanLoop 唯一（✅一致） | 单一执行器 |
| 请求 | planning_requested_/emergency_pending_ bool | BRAKE>COMPLETE>REPLAN 仲裁对象（⚑A） |
| 结果新旧 | safety_generation_（仅安全/制动） | input_version 覆盖目标/intent（⚑B） |
| 失败重试 | soft-landing 1s / 10s 收口 | retry 0.6s×5 参数（⚑C） |
| 制动 | 受控轨迹→BRAKE_HOLD（可被转出） | 直接锁存 BRAKE_HOLD 且不转出（⚑D） |
| 状态名 | 8 态现状名 | 逻辑名（INIT/…_HOLD）未实施 |
| 身份 | 静态 IdentityMap | 动态 current map 未实施 |

⚑ = `implementation_plan_26090802.md` 已批准未实施；除 A/B/C/D 外其余目标项不在该计划。

## 9. 证据文件

TCP：`tcp_to_ros/scripts/tcp_server_node.py`、`tcp_to_ros/scripts/tcp_to_ros/protocol.py`
GCS：`gcs_a_backend_node.py`、`time_offset_scheduler.py`、`identity.py`、`config/uav_identity_map.yaml`
executor：`uav_executor_node.py`、`executor.py`、`validation.py`、`task_store.py`、`state_reporter.py`
适配：`drivers/ego_swarm.py`；relay：`setpoint_relay.py`、`scripts/setpoint_relay_node.py`、`pose_relay_node.py`
C++：`ego_planner_driver_node.h/.cpp`、`ego_planner_core.cpp`、`safety_predictor.cpp`、`yield_candidates.cpp`
桥：`swarm_topology_bridge/scripts/bridge_node.py`；launch：`uav_executor_ego.launch`、`gcs_a_backend.launch`
相关：`offboard_new_state_machine_260902.md`、`src/tcp_to_ros/plans/implementation_plan_26090802.md`
