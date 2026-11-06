# HC-05 串口协议与 OTA

HC-05 通过 Classic Bluetooth 2.0 + EDR SPP 提供透明 UART。配对后使用系统提供的串口连接。原规格的 BLE 4.0 无连接广播由连接后的设备身份帧承载。UART 参数以模块实际数据模式配置为准，UART 逻辑电平为 3.3 V，STM32 与模块共地。 [HC-05 模块数据手册](https://www.tgltechnologies.com/assets/data-sheets/Bluetooth%20HC05.pdf)

## 帧结构

所有帧固定 20 字节，串口可任意分割或合并帧。多字节数字使用小端。

| 字节索引 | 内容 |
| --- | --- |
| 0–1 | 同步字 A5 5A |
| 2 | 消息类型 |
| 3 | 序号 |
| 4 | 有效载荷长度 0–14 |
| 5–18 | 有效载荷及尾部零填充 |
| 19 | 字节 2–18 的 CRC-8/SMBUS，初值 0，多项式 0x07 |

| 类型 | 方向 | 有效载荷 |
| --- | --- | --- |
| 0x01 IDENTITY | 设备 → 客户端 | 4 字节设备 ID，1 字节电量百分比，0xFF 表示未知 |
| 0x02 TELEMETRY | 设备 → 客户端 | 4 字节启动后毫秒，1 字节运动模式，2 字节计数，1 字节 BPM，0 表示心率不可用 |
| 0x03 INTERVAL | 客户端 → 设备 | 2 字节间隔，单位毫秒，范围 100–1000，默认 500 |
| 0x10 START | 客户端 → 设备 | 4 字节固件长度，4 字节固件 CRC32，可附加 0x01 请求从零擦除重传 |
| 0x11 DATA | 客户端 → 设备 | 固件数据，通常 14 字节，最后一包可短于 14 字节 |
| 0x12 FINISH | 客户端 → 设备 | 空 |
| 0x80 REPLY | 设备 → 客户端 | 1 字节请求类型，1 字节状态，4 字节已接收偏移，4 字节更新区容量 |

REPLY 原样带回请求序号。状态码为 0 成功、1 帧错误、2 会话缺失或未完成、3 参数越界、4 序号错误、5 Flash 操作失败、6 固件 CRC32 校验失败。

## 升级过程

1. 客户端计算待升级应用二进制的 CRC32/ISO-HDLC。START 成功后以设备返回的偏移为准。
2. DATA 每次发送一包并等待 REPLY，序号为 `(偏移 / 14) % 256`。客户端以 500 ms 为 DATA 确认时限，最多重发三次。START 首次擦除允许 20 秒，FINISH 读回校验允许 10 秒。设备收到超过五个连续坏帧、错序包或错长度包时丢弃内存会话。
3. 设备每十个成功写入的包提交一次持久检查点，并在最后一个包提交实际总长度。重新连接后对同一长度及 CRC32 发送 START，设备返回可续传偏移。检查点之后已经落盘的数据可能重发，Flash 端口须允许内容相同的重放。
4. FINISH 流式读取整个更新区。CRC32 匹配后写入启动标记，STM32 板级端口在应答发送后复位。CRC32 检测传输和存储损坏。应用固件仅应从受信任来源获取，配对连接仅应向授权操作人员开放。
5. 双区为 STM32F407 单物理 Flash Bank 内的两个逻辑区。应用从运行区链接，上传镜像暂存在更新区。引导器负责安装和中断安装后的恢复，具体布局见板级文档。

Windows 上配对模块后选择出站 SPP COM 口。关闭占用相同端口的串口终端，然后运行以下命令。速率与 STM32 UART 和 HC-05 的数据模式配置一致。

```powershell
python -m pip install pyserial
python tools/ota_upload.py COM7 path/to/leappulse.bin --baud 115200
```

断开或掉电后重新运行同一命令可续传。确认更新区数据错误时加 `--restart`，设备将擦除更新区并重新接收完整镜像。

## 调研与实现选择

| 项目 | 设计、依赖和限制 |
| --- | --- |
| [Zephyr MCUmgr](https://github.com/zephyrproject-rtos/zephyr) | Apache-2.0。镜像管理用镜像标识和设备返回的偏移续传，BLE/串口传输使用 SMP。其上传会话状态位于 RAM，设备重启后续传需要应用补充持久状态。LeapPulse 采用设备确认偏移和重复包应答，用自定义 20 字节 UART 帧接入 FreeRTOS。 [镜像管理规范](https://docs.zephyrproject.org/latest/services/device_mgmt/smp_groups/smp_group_1.html) |
| [MCUboot](https://github.com/mcu-tools/mcuboot) | Apache-2.0。参考其运行槽、更新槽和校验后激活的分工。完整引入需移植 Flash map、镜像格式及交换元数据，槽区布局也取决于扇区大小。当前实现使用 F407 板级端口的逻辑分区和 CRC32。 [上游设计](https://github.com/mcu-tools/mcuboot/blob/main/docs/design.md) |
| [littlefs](https://github.com/littlefs-project/littlefs) | BSD-3-Clause。通过带 CRC 的提交记录判断最新完整状态。这里采用固定用途的 Flash 进度记录，文件系统及其块设备抽象对单一升级会话没有必要。 [提交格式](https://github.com/littlefs-project/littlefs/blob/master/SPEC.md) |

固件使用独立实现的帧解析、校验与状态机。PC 客户端使用 [pySerial](https://github.com/pyserial/pyserial) 提供串口 API。
