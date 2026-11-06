# 开源方案调研

只连接上游库或参考其公开设计。仓库未复制第三方驱动源码或附图中的文件。

| 候选 | 许可证与依赖 | 采用方式与限制 |
| --- | --- | --- |
| [STM32F4 HAL](https://github.com/STMicroelectronics/stm32f4xx-hal-driver) 和 [FreeRTOS Kernel 11.3.0 LTS](https://github.com/FreeRTOS/FreeRTOS-Kernel/tree/V11.3.0) | BSD-3-Clause / MIT，使用 CubeF4 CMSIS 与 Cortex-M4F port | 直接使用 I²C、UART、Flash 和调度 API。FreeRTOS 默认 tickless 使用 SysTick，F407 STOP 另需 RTC 和恢复系统时钟。 |
| [Bosch BMI270 SensorAPI](https://github.com/boschsensortec/BMI270_SensorAPI/tree/41129fcfe39c583ee5462d79195741945d51c1fe) | BSD-3-Clause，C，无操作系统绑定 | 直接集成该提交的 `bmi2.c` 和 `bmi270.c`，复用官方配置数据、量程和任意运动中断。配置加载需要芯片规定的延时。 |
| [Fusion](https://github.com/xioTechnologies/Fusion) 和 [CMSIS-DSP](https://github.com/ARM-software/CMSIS-DSP) | MIT / Apache-2.0，前者为 C 姿态估计库，后者可用 ARM SIMD | 参考重力修正门控和 SOS 的设计，自行实现可移植核心。六轴航向会随时间漂移，长时间线加速度会影响姿态。 |
| [Zephyr MAX30102 驱动](https://github.com/zephyrproject-rtos/zephyr/pull/108697) 与 [SparkFun MAX3010x](https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library) | Apache-2.0，依赖 Zephyr / MIT，依赖 Arduino Wire，部分算法文件有单独许可声明 | 参考红光和红外的 FIFO 组织。STM32 侧按 ADI 手册接 HAL I²C，检查 FIFO 溢出并重置估计。SparkFun 的公开 issue 报告过异常读数。 |
| [LVGL v9.6.0](https://github.com/lvgl/lvgl/releases/tag/v9.6.0) | MIT，嵌入式 C/CMake | 直接用上游标签组件与 1 位缓冲区，限制启用的组件。SSD1306 使用列式页布局，驱动需转换 LVGL 行位图。 |
| [Zephyr DRV2605](https://github.com/zephyrproject-rtos/zephyr/tree/v4.5.0/drivers/haptics) 与 [Adafruit DRV2605](https://github.com/adafruit/Adafruit_DRV2605_Library) | Apache-2.0，依赖 Zephyr / MIT，依赖 Arduino I²C | 参考驱动的 LRA 库选择与硬件波形序列。用 TI 寄存器实现非阻塞触发。 |
| [MCUboot](https://github.com/mcu-tools/mcuboot) 与 [ST X-CUBE-DBFU](https://www.st.com/en/embedded-software/x-cube-dbfu.html) | Apache-2.0 / ST 扩展包，依赖目标 Flash 布局 | 参考暂存、校验和启动阶段安装。HC-05 的 20 字节帧仍走项目协议。F407 只有一个物理 Flash bank，不适用 ST 的原生双 bank 读写/切换。 |

主要硬件依据为 [RM0090](https://www.st.com/resource/en/reference_manual/rm0090-stm32f4xx-reference-manual-stmicroelectronics.pdf)、[MAX30102 数据手册](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX30102.pdf)、[HC-05 模块手册](https://www.hc01.com/downloads/HC-05.pdf) 和 [TI DRV2605L 数据手册](https://www.ti.com/lit/ds/symlink/drv2605l.pdf)。ADI 说明 [MAX30102 的 R 曲线取决于整机光学设计和标定](https://ez.analog.com/optical_sensing/a/documents/c/max30102efd-t-faq/DO19495/does-the-max30102-have-an-r-curve-associated-with-it-to-correlate-the-r-acred-dcred-acir-dcir)。

```sh
git clone https://github.com/boschsensortec/BMI270_SensorAPI.git
git -C BMI270_SensorAPI checkout 41129fcfe39c583ee5462d79195741945d51c1fe
git clone --branch v9.6.0 --depth 1 https://github.com/lvgl/lvgl.git
```
