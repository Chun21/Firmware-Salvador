# G1 实现阶段约束

## Phase 0：安全隔离

目标：先建立 G1 构建和运行边界，禁止危险路径。

必须完成：

- 增加 `ROBOT_MODEL_G1`。
- 增加 `ROBOT_MODEL=g1` 构建分支。
- G1 下拦截 `JointControl`。
- G1 下禁用 `ShootAgent`、`WalkCustomAgent`、T1/K1 policy。
- 建立 G1 SDK 初始化和安全停止框架。

不得完成：

- 不实现真踢球。
- 不实现低层 31DOF action。
- 不改动 K1/T1 行为。

## Phase 1：G1 跑通 RoboCup 主流程

目标：用高层走路、外部检测定位完成比赛主流程。

必须完成：

- G1 high-level locomotion adapter。
- G1 2DOF camera servo adapter。
- 仓库内 `vendor/g1/robocup_dds` / `vendor/g1/robocup_runtime` 检测结果接入；不得依赖外部 `~/robocup_deploy`。
- `rt/locationresults` 定位接入。
- 找球、走位、推球/带球、守门站位。

验收：

- AlwaysPlayGC 或等价测试能跑通。
- 非 Playing 状态安全停止。
- 感知/定位超时能安全降级。

## Phase 2：多机比赛流程

目标：真实 RoboCup 多机流程可用。

必须完成：

- GameController 实战状态验证。
- TeamCom 多机器人信息验证。
- striker/supporter/goalie 角色切换验证。
- G1 速度、避障、推球参数调优。

不得完成：

- 不因多机需求开启未验证 kick / dive。

## Phase 3：31DOF Action Layer

目标：独立实现 G1 低层安全动作层。

必须完成：

- 29DOF 本体动作接口。
- 2DOF 头顶相机云台独立保持/跟踪。
- kick action 状态机。
- 姿态检测、限位、超时、急停、恢复站立。
- feature flag 默认关闭。

启用条件：

- 单机静态动作安全验证通过。
- 短动作验证通过。
- 失败恢复验证通过。
- 不影响 Phase 1/2 稳定比赛能力。
