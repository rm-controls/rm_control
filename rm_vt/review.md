# rm_vt 协议审阅（基于 RoboMaster 2026 通信协议 V1.2.0）

## 审阅依据

- 权威来源（以此为准）：`src/rm_control/rm_referee/UTF-8__RoboMaster 2026 机甲大师高校系列赛通信协议 V1.2.0（20260209）.pdf`
- 图传链路/自定义客户端重点参考：
  - 表 1-38 `0x0302`、表 1-39 `0x0309`、表 1-40 `0x0310`、表 1-41 `0x0311`
  - 表 1-42 `0x0306`

## 结论摘要

- `rm_vt` 当前实现重点覆盖了 `0x0302` 和 `0x0304` 的接收解析，`unpack()` 的 payload 长度检查比 `rm_referee` 更安全。
- 当前代码已修正 `0x0310` 为 300 字节，并已定义 `0x0311` 命令码；但与 2026 协议相比仍存在命令码未对齐和解析缺项。
- 同时存在一条未在官方 PDF 中定义的私有帧解析路径（`0xA9 0x53`），建议明确标注为私有协议。

## 主要问题（按严重度排序）

### 1. 键鼠输入命令仍按 `0x0304` 实现，未对齐 V1.2.0 的 `0x0306`（高）

协议（V1.2.0）：

- 修改日志删除 `0x0304`（本轮使用从 PDF 抽取文本核对）：`/tmp/rm_referee_protocol_2026_v1_2_0.txt:7`
- 表 1-42 键鼠模拟命令为 `0x0306`：`src/rm_control/rm_referee/UTF-8__RoboMaster 2026 机甲大师高校系列赛通信协议 V1.2.0（20260209）.txt:2899`
- 第 2 章也将原 `RemoteControl` 拆分为 `KeyboardMouseControl` 与 `CustomControl`：`src/rm_control/rm_referee/UTF-8__RoboMaster 2026 机甲大师高校系列赛通信协议 V1.2.0（20260209）.txt:3945`, `src/rm_control/rm_referee/UTF-8__RoboMaster 2026 机甲大师高校系列赛通信协议 V1.2.0（20260209）.txt:4053`

代码当前定义：

- 仍将键鼠数据命令定义为 `ROBOT_COMMAND_CMD = 0x0304`：`src/rm_control/rm_vt/include/rm_vt/common/protocol.h:13`
- 接收解析 `switch` 也仅对 `0x0304` 进入 `KeyboardMouseData` 分支：`src/rm_control/rm_vt/src/video_transmission.cpp:132`

影响：

- 按 2026 协议发送的 `0x0306` 键鼠输入不会走当前解析路径。
- 当前实现对外语义仍停留在已删除命令码，容易在联调时产生“链路通但无数据”的问题。

### 2. `0x0311`（CustomControl）虽已定义，但未实现解析分支（中）

协议（表 1-41）：

- `0x0311` 为 30 字节，自定义客户端发送给机器人自定义指令：`src/rm_control/rm_referee/UTF-8__RoboMaster 2026 机甲大师高校系列赛通信协议 V1.2.0（20260209）.txt:2876`
- PDF 第 2 章 `CustomControl` 也明确对应 `0x0311`：`src/rm_control/rm_referee/UTF-8__RoboMaster 2026 机甲大师高校系列赛通信协议 V1.2.0（20260209）.txt:4055`

代码现状：

- 枚举已定义 `CUSTOM_TO_ROBOT_CMD = 0x0311`：`src/rm_control/rm_vt/include/rm_vt/common/protocol.h:16`
- 但 `switch` 仍仅处理 `0x0302`、`0x0304`：`src/rm_control/rm_vt/src/video_transmission.cpp:85`

影响：

- 头文件声明与实现支持范围不一致，`0x0311` 实际仍不可用。

### 3. 已声明 `0x0309/0x0310`，但未实现解析分支（中）

- 枚举声明了 `ROBOT_TO_CUSTOM_CMD = 0x0309`、`ROBOT_TO_CUSTOM_CMD_2 = 0x0310`：`src/rm_control/rm_vt/include/rm_vt/common/protocol.h:14`, `src/rm_control/rm_vt/include/rm_vt/common/protocol.h:15`
- `unpack()` 无对应 `case`：`src/rm_control/rm_vt/src/video_transmission.cpp:85`

影响：

- 协议覆盖面与头文件声明不一致，容易误导上层认为已支持。

### 4. 收包窗口处理鲁棒性不足（中）

- 当 `rx_len_ >= 256` 时，不更新 `unpack_buffer_`，可能直接丢数据：`src/rm_control/rm_vt/src/video_transmission.cpp:21`
- 扫描起始位置仅遍历前 128 字节：`src/rm_control/rm_vt/src/video_transmission.cpp:30`

影响：

- 高吞吐或串口读 burst 场景下可能漏帧。

### 5. 存在官方 PDF 未定义的私有帧解析路径（信息/需文档化）

代码会解析 `0xA9 0x53` 起始、固定 21 字节的控制帧：

- 检测入口：`src/rm_control/rm_vt/src/video_transmission.cpp:38`
- 解包实现：`src/rm_control/rm_vt/src/video_transmission.cpp:175`
- 结构定义 `ControlData`：`src/rm_control/rm_vt/include/rm_vt/common/protocol.h:71`

说明：

- 在官方 2026 PDF 表 1-38~1-42 中未见该帧格式定义（至少不属于公开常规/图传命令码表）。
- 这不一定是错误，但属于“额外私有协议”，应在包内文档显式说明来源（图传接收机私有数据流/厂商帧）与官方协议的边界。

## 已有优点（保留）

- `0x0310` 长度定义已经与协议对齐（300 字节）并有 `static_assert`：`src/rm_control/rm_vt/include/rm_vt/common/protocol.h:56`, `src/rm_control/rm_vt/include/rm_vt/common/protocol.h:114`
- `0x0311` 命令码已在枚举中定义：`src/rm_control/rm_vt/include/rm_vt/common/protocol.h:16`
- `unpack()` 已校验 `frame_len > rx_data_len`，并对 `0x0302/0x0304` 做 payload 长度检查：`src/rm_control/rm_vt/src/video_transmission.cpp:65`, `src/rm_control/rm_vt/src/video_transmission.cpp:71`
- 这部分实现明显优于 `rm_referee` 的当前解包安全性。

## 建议整改顺序

1. 优先完成命令码对齐：将键鼠输入路径从旧的 `0x0304` 迁移/兼容到 V1.2.0 的 `0x0306`（并明确 `0x0304` 是否仅保留兼容）。
2. 明确支持范围：若要支持 2026 自定义客户端链路，补充 `0x0311`（以及需要时的 `0x0309/0x0310`）解析/转发。
3. 给 `0xA9 0x53` 私有帧增加注释或 README 说明，避免与官方命令码协议混淆。
4. 优化收包窗口逻辑，避免 `rx_len_ >= 256` 和后半窗口漏检问题。
