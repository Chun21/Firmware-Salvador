# 模块边界与接口约束

## G1 平台层

- 新增 G1 支持时，应建立独立 adapter：
  - G1 SDK 初始化/关闭。
  - G1 high-level locomotion。
  - G1 lowstate / odom / IMU 订阅。
  - G1 头顶相机云台 DDS 订阅和控制。
- 不把 G1 SDK 调用散落到 agent、strategy 或 localization 业务层。

## Motion

- G1 第一阶段只支持 `WALK`、`STAND`、`NOTHING` 语义。
- `WALK` 映射到高层 `Move(vx, vy, vyaw)`。
- `STAND` / `NOTHING` 映射到 `StopMove` / `BalanceStand`。
- `JOINT_CONTROL` 在 G1 下必须硬拦截。
- `MoveBallGoalOrder` 在 G1 下代表推球/带球，不代表踢球。

## Head / Camera Servo

- G1 头顶相机云台是独立 2DOF，不属于 29DOF 本体数组。
- 现有 head focus 逻辑可以复用，但必须通过 G1 servo adapter 输出。
- 与 `g1_comp_servo_service` 通信时统一在 adapter 中处理单位转换。
- 默认限制：
  - yaw: `[-50°, 50°]`
  - pitch: `[-20°, 85°]`
  - yaw speed: `45°/s`
  - pitch speed: `30°/s`

## Vision / Detection

- G1 第一阶段默认使用仓库内 `vendor/g1/robocup_runtime/football_detectcpp` 复制过来的检测链路和 `vendor/g1/robocup_dds` 中的 DDS 类型；不得新增对 `~/robocup_deploy` 的构建依赖。
- 不得直接把该 YOLO engine 塞进现有 booster vision pipeline，除非同时完成类别顺序和后处理适配。
- 类别映射必须集中维护：
  - `Ball` -> ball hypothesis
  - `Goalpost` -> goalpost / landmark
  - `L` -> L spot
  - `T` -> T spot
  - `X` -> X spot
  - `PenaltyPoint` -> penalty spot
  - `Opponent` / `person` / `Robot` / `Human` -> robot / obstacle
- `xyz`、`offset_fov` 等外部检测字段必须在 adapter 内转换成项目内部语义。

## Localization

- G1 第一阶段主定位订阅 `rt/locationresults`。
- `robot2field_x/y/theta` 转换为当前项目内部定位结果。
- 坐标单位、方向、theta 正负号必须在 adapter 内集中处理。
- 定位超时或无效时，quality 必须降级，不允许策略层继续按有效定位盲走。

## Sensor / Odom / Fall

- G1 优先使用 SDK / DDS 提供的 odom 和 IMU。
- 不把当前 K1/T1 固定数组 `IMUJointState` 强行扩展成 G1 31DOF 混合结构。
- 如果通用模块只需要 IMU、odom、头部角度，应由 adapter 提供兼容视图。
- fall 判断第一阶段保守处理：异常姿态进入安全停止，不执行复杂 get-up policy。

## Agent / Strategy

G1 第一阶段允许注册：

- `GoaliePositioningAgent`
- `WalkRelativeAgent`
- `LocalizeAgent`
- `BallSearchAgent`
- `DribbleAgent`
- `WalkToPositionAgent`

G1 第一阶段禁止注册：

- `ShootAgent`
- `WalkCustomAgent`
- 任何会输出 `JointControl` 的 agent
- 任何 T1/K1 policy agent

若策略产生 `ShootOrder`，G1 第一阶段必须拒绝执行或转换为安全推球/带球行为，并记录日志。

## GameController / TeamCom

- GameController 和 TeamCom 可复用。
- 非 Playing 状态必须优先安全停止。
- G1 速度、walking time、角色切换参数应使用更保守默认值。
- 多机策略可以复用，但不得假设 G1 有踢球能力。
