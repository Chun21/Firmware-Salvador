# Unitree G1 RoboCup 支持完整规划（31DOF 约束版）

## Summary
- G1 按 **29DOF 本体 + 2DOF 头顶相机云台 = 31DOF** 规划；当前 Firmware 的 22/23DOF 结构、T1 policy、`JointControl(std::array<float,23>)` 全部不能直接复用。
- 第一阶段目标是安全完整跑 RoboCup 主流程：比赛状态、识别、定位、找球、走位、守门站位、推球/带球。
- 真正踢球、扑救、低层动作作为第二阶段单独建设 **G1 31DOF Action Layer**，不能依赖 `unitree_sdk2` 高层接口。

## Target Architecture
- **G1 平台层**
  - 新增 `ROBOT_MODEL=g1`，独立于 K1/T1。
  - 新增 `g1_sdk_adapter`：初始化 Unitree DDS、G1 LocoClient、lowstate、odom、secondary IMU。
  - 新增 `g1_servo_adapter`：管理头顶相机 2DOF，接 `rt/g1_comp_servo/cmd/state`。
  - 新增 `g1_motion_connector`：第一阶段只允许 `WALK/STAND/NOTHING`，强制拦截 `JOINT_CONTROL`。

- **G1 感知定位层**
  - G1 第一版使用仓库内 `vendor/g1/robocup_runtime/football_detectcpp` 复制过来的检测方式和 `weight.engine`。
  - 新增 `g1_perception_adapter`：
    - `detectionresults → ball_hypothesis_channel`
    - `detectionresults → point_features_channel`
    - `detectionresults → opponent_hypotheses_channel`
  - 新增 `g1_location_adapter`：
    - `rt/locationresults → loc_position_channel`
  - 当前 Firmware 自带 `booster_vision` 和 `Localization` 保留为 K1/T1 主线；G1 下默认不用它做主定位。

- **行为策略层**
  - 继续复用当前 `robocup` team strategy、GameController、TeamCom。
  - G1 模式下注册安全 agent 集合：
    - `GoaliePositioningAgent`
    - `WalkRelativeAgent`
    - `LocalizeAgent`
    - `BallSearchAgent`
    - `DribbleAgent`
    - `WalkToPositionAgent`
  - 禁止注册：
    - `ShootAgent`
    - `WalkCustomAgent`
    - T1/K1 policy executor
    - 任何会输出 23DOF `JointControl` 的 agent。

## Module-by-Module Plan
- **Build / CMake / Install**
  - `CMakeLists.txt` 增加 `ROBOT_MODEL_G1`。
  - `install.bash` 增加 `--g1`，构建目录 `build-g1`。
  - 新增 G1 SDK CMake 配置，默认使用仓库内 `vendor/g1/unitree_sdk2`，不依赖构建机上的 `~/robocup_deploy`。
  - G1 部署资源包含：
    - G1 firmware binary
    - G1 YOLO engine
    - G1 servo config
    - 可选启动脚本：`g1_comp_servo_service`、`football_detectcpp`、`location_fusion`。

- **31DOF 数据模型**
  - 新增 `G1JointState`，表达 29DOF 本体。
  - 新增 `G1HeadServoState`，表达头顶相机 yaw/pitch 2DOF。
  - 不把 31DOF 塞进当前 `IMUJointState` 的 22/23 固定数组。
  - 当前通用模块若只需要 IMU、头 yaw/pitch、odom，则通过 adapter 提供兼容视图；低层动作模块必须使用 G1 专用结构。

- **Motion**
  - 第一阶段：
    - `MotionCommand::WALK` → `LocoClient::Move(vx, vy, vyaw)`
    - `STAND/NOTHING` → `StopMove/BalanceStand`
    - GameController 禁止移动状态 → `StopMove`
    - 跌倒/异常 → `Damp` 或安全停止
  - G1 默认限速保守配置：
    - 前进速度、侧移速度、角速度都单独限幅。
    - 所有速度命令做 slew-rate 限制。
  - `JOINT_CONTROL` 在 G1 下硬拦截，记录 error log，绝不下发。

- **Kick / Shoot**
  - 当前 `ShootAgent` 依赖 `shoot_v1.tflite`、`T1Config`、23DOF，不能用于 G1。
  - 第一阶段不实现真踢球，进攻用 `DribbleAgent` 推球/带球。
  - 第二阶段新增 `g1_action_layer`：
    - 支持 29DOF 本体动作轨迹。
    - 2DOF 相机云台独立保持/看球。
    - 实现 `KickAction` 状态机：准备、对球、起脚、触球、恢复站立、失败回退。
    - 所有动作必须有限位、姿态检测、超时、急停。
  - 未完成第二阶段前，RoboCup 策略不得生成 `ShootOrder`。

- **Vision / Detection**
  - 采用 `vendor/g1/robocup_runtime/football_detectcpp/weight` 中复制过来的检测权重，不能直接丢给当前 `booster_vision` detector，因为类别顺序不同。
  - 类别映射固定为：
    - `Ball` → BALL
    - `L` → L_SPOT
    - `T` → T_SPOT
    - `X` → X_SPOT
    - `PenaltyPoint` → PENALTY_SPOT
    - `Opponent/person/Robot/Human` → ROBOT
  - 检测输出中的 `xyz`、`offset_fov` 用于球相对位置和头部跟踪。

- **Localization**
  - 第一阶段主定位使用与 `vendor/g1/robocup_runtime/robocup_locator_v1.1` 一致的 `rt/locationresults`。
  - 转换规则：
    - `robot2field_x/y/theta` → 当前 `LocPosition.position`
    - 定位有效时 `quality=0.8~1.0`
    - 超时或无效时降为 `quality=0`
  - 后续可做融合：
    - RGBD/marker 定位为主。
    - 当前场线定位只作为验证或 fallback。

- **Head / 2DOF Camera Servo**
  - 当前 `HeadControl` 输出 rad；`g1_comp_servo_service` 使用角度制 DDS command。
  - 新增转换和限位：
    - yaw：默认限制 `[-50°, 50°]`
    - pitch：默认限制 `[-20°, 85°]`
    - 速度限制参考 servo config：yaw 45°/s，pitch 30°/s
  - HeadFocus 逻辑复用：看球、找球、看定位点，但输出走 G1 servo adapter。

- **Odometry / Sensor / Fall**
  - G1 订阅：
    - `rt/lowstate`
    - `rt/odommodestate`
    - `rt/secondary_imu`
    - `rt/g1_comp_servo/state`
  - 优先使用 G1 odom；当前基于 motion command 的 `OdometerProcessor` 不作为 G1 主里程计。
  - Fall 判断第一阶段保守：
    - IMU 姿态超阈值 → fallen/unsafe
    - 不自动执行复杂 get-up policy，只进入安全停止或调用 SDK 可用站立接口。

- **Team Strategy / TeamCom / GameController**
  - 保留原逻辑。
  - G1 调整 walking time / max speed 参数，避免 striker 选择过于激进。
  - `MoveBallGoalOrder` 在 G1 下语义明确为“推球/带球进攻”，不是踢球。
  - 如果未来启用 `KickAction`，再引入 `ShootOrder`。

## Implementation Phases
- **Phase 0：安全隔离**
  - 增加 `ROBOT_MODEL_G1`。
  - 禁止 G1 编译/运行 T1 policy、23DOF `JointControl`、`ShootAgent`。
  - 建立 G1 SDK 初始化和停止逻辑。

- **Phase 1：G1 可跑比赛主流程**
  - 接入 G1 LocoClient 高层走路。
  - 接入 G1 2DOF 相机云台。
  - 接入仓库内 vendor 复制过来的检测和定位 DDS 协议。
  - 跑通 AlwaysPlayGC：找球、走位、推球、守门站位。

- **Phase 2：RoboCup 多机完整流程**
  - 接入真实 GameController。
  - 验证 TeamCom、多机器人 striker/supporter/goalie 角色。
  - 调整 G1 速度、避障、推球参数。

- **Phase 3：31DOF 动作层**
  - 新增 G1 低层动作接口，不复用当前 T1 23DOF policy。
  - 先做静态安全动作，再做短踢球动作。
  - 通过 feature flag 启用，不影响 Phase 1/2 稳定比赛能力。

## Test Plan
- **编译测试**
  - `--g1` 独立构建成功。
  - `--t1/--k1z/--k1bc` 不回归。
- **接口安全测试**
  - G1 下任何 `JointControl` 都被拦截。
  - `ShootAgent/WalkCustomAgent/PolicyExecutor` 不进入 G1 agent 列表。
- **实机基础测试**
  - DDS 初始化、lowstate、odom、IMU、servo state 正常。
  - `StopMove/BalanceStand/Move` 方向和限速正确。
  - 头顶相机 yaw/pitch 限位正确。
- **感知定位测试**
  - 检测类别映射无错位。
  - 球相对坐标、场地定位坐标方向正确。
  - 定位超时时策略不继续盲走。
- **比赛场景测试**
  - Ready 站位。
  - Playing 找球。
  - Striker 推球。
  - Supporter 走位。
  - Goalie 守门站位。
  - Penalized/Finished/Set 状态安全停止。

## Assumptions
- G1 第一版不实现真踢球、不实现扑救倒地动作。
- 31DOF 低层动作是独立第二阶段工程，必须单独验证安全。
- `vendor/g1/robocup_runtime` 中复制过来的检测/定位链路是 G1 第一版主数据源参考；构建不得依赖外部 `~/robocup_deploy`。
- 当前 Firmware 的 RoboCup 策略可复用，但必须在 G1 下禁用所有 T1/K1 低层动作路径。
