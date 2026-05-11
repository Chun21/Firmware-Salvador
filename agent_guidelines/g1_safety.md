# G1 安全约束

## 自由度模型

G1 在本项目中固定按以下方式建模：

- 本体：29DOF
- 头顶相机云台：2DOF
- 总计：31DOF

禁止把 G1 当成 T1/K1 的 22/23DOF 机器人处理。

## 第一阶段禁止项

G1 第一阶段只允许使用高层、安全的行走和站立能力。以下行为禁止实现或启用：

- 真踢球动作。
- 扑救倒地动作。
- T1/K1 23DOF policy。
- `std::array<float, 23>` 形式的 G1 关节控制。
- 任何从策略层直接下发到 G1 低层关节的动作。
- 任何未做限位、超时、急停的 body action。

## 必须硬拦截的路径

G1 模式下必须拒绝以下路径：

- `MotionCommand::JointControl`
- `ShootAgent`
- `WalkCustomAgent`
- `PolicyExecutor`
- `T1Config::Policy::Control::num_dofs`
- 任何输出 23DOF action 的 agent 或 motion connector

拦截要求：

- 不下发到机器人。
- 输出明确 error/warn log。
- 机器人进入安全停止或保持站立。

## G1 高层运动边界

第一阶段 G1 motion 只允许映射到：

- `LocoClient::Move(vx, vy, vyaw)`
- `LocoClient::StopMove`
- `LocoClient::BalanceStand`
- 异常时 `Damp` 或安全停止

所有运动命令必须有：

- 最大速度限幅。
- slew-rate 限制。
- 命令超时。
- GameController 状态检查。
- 定位/感知有效性检查。

## 状态降级规则

以下情况默认安全停止：

- GameController 处于 `Set`、`Finished`、`Penalized` 或其他禁止移动状态。
- 运动命令超时。
- 定位长时间无效。
- IMU 姿态超过安全阈值。
- DDS / SDK 连接异常。
- 头部云台状态异常且影响相机输入。

## 31DOF Action Layer 约束

真踢球、扑救、倒地恢复只能在后续独立的 G1 31DOF Action Layer 中实现。该层必须单独具备：

- 29DOF 本体动作轨迹模型。
- 2DOF 头顶相机云台独立控制。
- 关节限位。
- 姿态检测。
- 超时。
- 急停。
- 失败恢复到安全站立。
- feature flag，默认关闭。

