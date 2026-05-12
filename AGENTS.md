# AGENTS.md

本文件是后续 agent 在本仓库写代码时必须优先阅读和遵守的入口规范。  
G1 RoboCup 支持的总体方案见 `PLAN.md`；更细的规范见 `agent_guidelines/`。
裁判盒代码在/home/chunyu/programs/GameController
## 必读顺序

1. `PLAN.md`：确认 G1 支持的阶段目标和总体架构。
2. `agent_guidelines/g1_safety.md`：确认 G1 31DOF、安全禁用项和动作边界。
3. `agent_guidelines/module_contracts.md`：确认感知、定位、运动、策略等模块接口边界。
4. `agent_guidelines/code_style.md`：确认 C++/CMake 代码风格。
5. `agent_guidelines/testing_checklist.md`：修改完成前按清单验证。

## 最高优先级约束

- G1 按 **29DOF 本体 + 2DOF 头顶相机云台 = 31DOF** 建模。
- 第一阶段 G1 只跑 RoboCup 主流程：识别、定位、找球、走位、守门站位、推球/带球。
- 第一阶段 **不实现真踢球、不实现扑救倒地动作、不下发低层关节动作**。
- G1 模式下必须硬拦截所有 `JointControl` 和 23DOF policy 输出。
- 不得把 T1/K1 的 `ShootAgent`、`WalkCustomAgent`、`PolicyExecutor` 直接用于 G1。
- `unitree_sdk2` G1 高层接口不提供 RoboCup kick API；不得假设 SDK 支持踢球。
- G1 第一阶段感知定位使用仓库内 `vendor/g1/` 复制过来的检测、定位 DDS 类型、运行参考代码和 Unitree SDK2；不得新增对 `~/robocup_deploy` 等外部绝对路径的依赖。

## 修改原则

- 不做无关重构。
- 不添加需求外功能。
- 不破坏 K1/T1 现有构建和行为。
- 新增 G1 逻辑必须通过 `ROBOT_MODEL_G1` 或 G1 专用 adapter 隔离。
- 安全优先：不确定时停止、限幅、降级，而不是继续执行危险动作。

## 提交前最低检查

- 代码格式符合 `.clang-format`。
- K1/T1 分支没有被 G1 改动误伤。
- G1 下不会注册或执行任何 23DOF / T1 policy 路径。
- G1 下所有速度、头部角度和外部输入都有边界处理。
- 感知类别映射、定位坐标系、单位转换集中维护且可检查。
