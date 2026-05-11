# GameController3 通信说明

本项目已按 `~/GameController3/game_controller_msgs/headers/RoboCupGameControlData.h`
更新裁判盒协议。

## 协议

- 控制消息：UDP `3838`，header `RGme`，version `18`。
- 状态回传：UDP `3939`，header `RGrt`，version `4`。
- 裁判盒发送频率：约 `2Hz`。
- 机器人回传频率：约 `1Hz`。

## 启动

真实裁判盒：

```bash
./bin/fw_salvador --gc3 --team 44 --player 0
```

注意：本项目内部 `--player` 是 0-based；回传给裁判盒时会自动转成
GameController 协议要求的 1-based `playerNum`。

等价短参数：

```bash
./bin/fw_salvador --gc --team 44 --player 0
```

兼容旧参数：

```bash
./bin/fw_salvador --real_gamecontroller true --team 44 --player 0
```

未接裁判盒时使用 AlwaysPlay：

```bash
./bin/fw_salvador --team 44 --player 0
```

## 回传内容

机器人会向最近收到控制包的裁判盒 IP 回传：

- `teamNum`
- `playerNum`
- `fallen`
- `pose[0..2]`：场上位置，单位 mm / rad
- `ballAge`：最后看见球到现在的秒数，没球为 `-1`
- `ball[0..1]`：机器人相对球坐标，单位 mm

## G1 注意事项

- G1 默认用 `--g1` 构建，运行时仍通过同一 GameController 模块通信。
- G1 DDS 网络接口由 `G1_NETWORK_INTERFACE` 控制，默认 `lo`。
- 裁判盒通信是普通 UDP，与 G1 DDS 是两套网络；实机部署时需确认两者接口都正确。
