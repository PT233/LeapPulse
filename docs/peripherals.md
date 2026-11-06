# 显示、旋钮和触觉

- SSD1306 使用 I²C 7 位地址 `0x3c`，面板为 128×64，启用内部电荷泵。若模块使用 `0x3d`，修改 `firmware/src/peripherals.c` 的地址。启动时先写入显存再打开屏幕。界面显示三位计数和心率，无有效心率时显示 `---`，超过三位时显示 `999`。16×16 模式标识 `F`、`T`、`C` 对应自由、计时、计数。
- LVGL 在 UI 任务内工作，约每 33 ms 检查一次画面更新间隔，只有数据变化才重绘。I1 位图转成 SSD1306 的纵向页格式。每次 UI 轮询只发送一页的变更范围，最长 128 字节，地址和数据传输分别释放 I²C 锁。30 FPS 是绘制上限，实际 OLED 刷新和 BMI270 采样抖动需要用实板测量。
- 机械编码器使用 A/B 两相，默认四个有效格雷码边沿对应一个刻度。完成一个刻度后执行 20 ms 防抖。接触弹跳导致的反向边沿会抵消累积步数。5 ms 轮询适合手动旋钮，快速转动可能漏过边沿。方向相反时互换 A/B 接线或交换板级配置。
- DRV2605L 使用 I²C 地址 `0x5a` 和内置闭环 LRA 波形库。把使能置高并将 LRA 接至模块的 OUT+ 和 OUT−。目标达成触发一次短震，心率告警触发两次，中间插入 50 ms 等待。单个波形的脉宽由芯片 ROM 决定，驱动频率由闭环跟踪，均需实测。芯片独立完成震动，随后 UI 任务切回待机。固件保留芯片的额定电压寄存器并将过驱钳位寄存器配置为 `0x60`，上板前按具体 LRA 的额定参数核对并校准。
- 板级未安装电量采集时，电压读取接口返回负值表示未知。电池分压和 SoC 换算留给实际硬件实现。

## 开源实现对照

| 上游 | 依赖与限制 | 采用的设计 |
| --- | --- | --- |
| [LVGL 9.6](https://github.com/lvgl/lvgl/releases/tag/v9.6.0) · MIT | 由上游 C 库渲染。I1 数据包含 8 字节调色板，位图以行存储。SSD1306 需要纵向页转换，旧版软件旋转曾有单色缓冲问题。 | 直接依赖 LVGL，固定方向，显式转换位图格式。 |
| [Adafruit SSD1306](https://github.com/adafruit/Adafruit_SSD1306) · BSD-3-Clause | Arduino Wire 与 Adafruit GFX；上游处理过 I²C 超时和 Wire 缓冲长度限制。 | 在 STM32 HAL 抽象上独立实现命令与数据分离，并限制单次 I²C 传输长度。 |
| [Zephyr DRV2605 驱动](https://github.com/zephyrproject-rtos/zephyr/blob/v4.5.0/drivers/haptics/ti/drv2605.c) · Apache-2.0 | 依赖 Zephyr 设备模型；RTP 实现会在系统工作队列上等待各段时长。 | 参考其 ROM 波形序列机制，在本项目板级 I²C 接口上实现有限长度硬件播放。 |
| [Zephyr GPIO QDEC](https://github.com/zephyrproject-rtos/zephyr/blob/v4.5.0/drivers/input/input_gpio_qdec.c) · Apache-2.0 | 依赖 Zephyr 输入设备与 GPIO；根据相位变化累计步数。 | 采用四相转移表，通过 FreeRTOS UI 任务周期读取 A/B。 |

单色 LVGL 的缓冲和旋转限制见 [LVGL 显示格式](https://lvgl.io/docs/open/9.5/main-modules/display/color_format) 和 [上游问题 #3414](https://github.com/lvgl/lvgl/issues/3414)。SSD1306 I²C 超时处理见 [Adafruit 发布记录](https://github.com/adafruit/Adafruit_SSD1306/releases)。DRV2605L 内部 ROM 和 LRA 驱动模式见 [TI 数据手册](https://www.ti.com/lit/ds/symlink/drv2605l.pdf)。
