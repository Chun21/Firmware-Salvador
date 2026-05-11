# GameController3 说明

当前代码已经切换为对齐 `~/GameController` 的 RoboCup GameController 协议
（`RoboCupGameControlData` version 19）。

历史上本仓库曾对齐 `~/GameController3` 的 version 18；如果后续需要重新支持
GameController3，请不要直接覆盖当前头文件，而应在 `game_controller` 模块中增加明确的
协议分支，并保证启动参数能选择 version 18 / version 19。
