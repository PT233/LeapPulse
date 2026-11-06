# LeapPulse 智能跳绳计数器

STM32F407 固件原型，使用 FreeRTOS、BMI270、MAX30102、SSD1306、DRV2605L 和 HC-05。旋转编码器选择自由跳、计时跳或计数跳，按下启动键开始或停止。初始目标为 60 秒或 100 次，目标达成和高心率分别触发振动提示。

- **运动** · 200 Hz 姿态估计、15 Hz 五阶 Butterworth 滤波、动态阈值和陀螺仪门控。单摇、双摇、交叉跳使用启发式识别。
- **光学** · 运动时读取 100 Hz 红光和红外 FIFO。输出心率、RMSSD 和试验性 SpO₂ 估计。待机时每秒短暂唤醒光学器件。
- **交互** · LVGL 单色界面、增量 OLED 刷新、编码器防抖、DRV2605L 振动。
- **通信** · HC-05 SPP，5 字节设备身份、8 字节运动数据，发送周期 100–1000 ms，默认 500 ms。
- **升级** · 20 字节帧、CRC8、整镜像 CRC32、每十包保存进度，以及断点续传。应用与引导程序分开构建。

## 构建与测试

Windows MinGW 或 Linux GCC 可运行可移植测试。

```sh
cmake -S . -B build-host -G Ninja
cmake --build build-host
ctest --test-dir build-host --output-on-failure
```

Windows 上的部分 Ninja 版本无法处理中文路径。遇到 `FindFirstFileExA` 时，在 ASCII 路径检出项目后运行上述命令。

STM32F407 需要按实物接线生成 CubeMX 工程，再按 [STM32F407 接入说明](docs/stm32f407.md) 构建。另见 [OLED、振动与编码器](docs/peripherals.md)、[串口协议与 OTA 工具](docs/wireless.md) 和 [开源方案对比](docs/upstream.md)。

## 与原图规格的差异

- 普通 HC-05 支持经典蓝牙 SPP。设备身份在连接后发送，固件无法发出 BLE 4.0 广播。见 [HC-05 厂商手册](https://www.hc01.com/downloads/HC-05.pdf)。
- F407 的 Flash 使用两个逻辑槽。引导程序校验并复制暂存镜像，掉电后可以重做复制。传输完整镜像，保留暂存镜像以完成安装，覆盖运行区后无法回滚旧版本。见 [STM32F407 参考手册](https://www.st.com/resource/en/reference_manual/rm0090-stm32f4xx-reference-manual-stmicroelectronics.pdf)。
- BMI270 任意运动唤醒的持续时间按 20 ms 配置。300 ms 计数防抖将连续计数上限约束在 3.33 Hz。
- SSD1306 是单色控制器。DRV2605L 用芯片内部 LRA 波形，短振脉宽由波形决定。
- PPG LED 电流使用可调的初始寄存器值，光学标定和体动伪影需要实测。1 Hz 待机短采样不输出 BPM。SpO₂ 为工程估计。
- 计数准确率、姿态分类误差、待机电流、刷新率、蓝牙唤醒延迟和 OTA 掉电恢复需要实板验证。移动端界面与云端分析未实现。
