# 测试与验收清单

## 编译检查

修改 G1 相关代码后至少检查：

- G1 构建配置可生成。
- K1/T1 构建入口不被破坏。
- G1 专用依赖不会泄漏到 K1/T1 链路。

建议命令按实际环境选择：

```bash
./install.bash --g1
./install.bash --t1
./install.bash --k1z
./install.bash --k1bc
```

## 静态安全检查

G1 模式下必须确认：

- `JointControl` 被硬拦截。
- `ShootAgent` 未注册。
- `WalkCustomAgent` 未注册。
- `PolicyExecutor` 不参与 G1 action。
- 不存在 G1 调用 T1 23DOF policy 的路径。

可辅助搜索：

```bash
grep -R "JointControl\\|ShootAgent\\|WalkCustomAgent\\|PolicyExecutor\\|T1Config::Policy" -n .
```

## 实机基础检查

上机前先确认：

- DDS / SDK 初始化成功。
- lowstate、odom、IMU 有数据。
- `StopMove`、`BalanceStand` 行为正确。
- `Move(vx, vy, vyaw)` 方向正确。
- 速度限幅和 slew-rate 生效。
- 禁止移动状态会立即停止。

## 头部云台检查

- yaw/pitch 单位转换正确。
- yaw 限位 `[-50°, 50°]` 生效。
- pitch 限位 `[-20°, 85°]` 生效。
- 速度限制生效。
- 找球和定位点关注不会让云台冲限。

## 感知定位检查

- 检测类别没有错位。
- ball、goalpost、L/T/X、penalty point、robot/obstacle 映射正确。
- 球相对坐标方向正确。
- `robot2field_x/y/theta` 转换方向正确。
- 定位超时后 quality 降级。

## RoboCup 场景验收

至少覆盖：

- Ready：站位。
- Set：停止。
- Playing：找球。
- Playing：striker 推球/带球。
- Playing：supporter 走位。
- Playing：goalie 守门站位。
- Penalized：停止。
- Finished：停止。
- 感知丢失：搜索或安全降级。
- 定位丢失：不盲走。

## 回归要求

- K1/T1 原有 agent 列表和 motion connector 不因 G1 改动改变语义。
- 现有 `robocup` strategy、GameController、TeamCom 在非 G1 模式下保持原行为。

