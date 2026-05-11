# RoboCup GameController 通信说明

当前固件的 `GameController` 通信协议默认对齐仓库外的
`~/GameController/game_controller_msgs/headers/RoboCupGameControlData.h`。

## 当前协议

- 控制包端口：UDP `3838`
- 返回/心跳端口：UDP `3939`
- 控制包 magic：`RGme`
- 控制包版本：`19`
- 返回包 magic：`RGrt`
- 返回包版本：`4`

`game_controller/humanoid/RoboCupGameControlData.h` 是从
`~/GameController/game_controller_msgs/headers/RoboCupGameControlData.h` 复制过来的。
如果裁判盒协议更新，应重新复制该头文件并同步检查字段映射。

## 字段映射

固件接收 `RoboCupGameControlData` 后映射为内部 `GCState`：

- `state` -> `GameState`
- `gamePhase == GAME_PHASE_PENALTY_SHOOT_OUT` -> `GCState::GamePhase::PENALTY_SHOOT`
- `firstHalf` -> `FIRST_HALF` / `SECOND_HALF`
- `setPlay` -> `SecondaryState`
  - `SET_PLAY_DIRECT_FREE_KICK` -> `DirectFreeKick`
  - `SET_PLAY_INDIRECT_FREE_KICK` -> `IndirectFreeKick`
  - `SET_PLAY_PENALTY_KICK` -> `PenaltyKick`
  - `SET_PLAY_THROW_IN` -> `ThrowIn`
  - `SET_PLAY_GOAL_KICK` -> `GoalKick`
  - `SET_PLAY_CORNER_KICK` -> `CornerKick`
- `kickingTeam` -> `KickingTeam`
- `teams[*].players[*].penalty != PENALTY_NONE` -> `Player::is_penalized`

返回包 `RoboCupGameControlReturnData` 会尽量填写：

- `teamNum`
- `playerNum`
- `fallen`
- `pose[0..2]`，单位为 mm / rad
- `ballAge`
- `ball[0..1]`，单位为 mm

## 启动参数

推荐使用：

```bash
./fw_salvador --gc
```

`--gc3` 目前只是保留的旧别名，会启用同一个 `GameController` 接收器；当前协议仍然是
`~/GameController` 的 version 19，不再是 `~/GameController3` 的 version 18。
