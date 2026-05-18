# Bioelec-Impedance-TCM

基于 RK3568 SoC 与 AD5940 AFE 芯片的高精度生物阻抗采集与分析系统，支持幅频/相频响应的实时采集、传输与可视化，为中医穴位阻抗频谱分析提供数据支撑。

---

## 一、项目简介

### 1.1 概述

<img width="600" alt="b60868f44850190c2fdfd4b50db2e61b" src="https://github.com/user-attachments/assets/e105a707-1ae3-43d6-a2ac-6f048057cc74" />

本项目实现了一套完整的 **Linux 嵌入式生物阻抗（BIA）采集系统**，覆盖从内核驱动到用户态可视化应用的全栈开发。系统以 ADI AD5940 高精度模拟前端（AFE）为核心，通过 **开尔文四线法（Kelvin 4-wire）** 测量连接至人体皮肤，采用四电极配置（CE1/CE2 激励电极 + SE1/SE2 采样电极）消除引线电阻误差，在 50μA~750μA 可编程激励电流下完成人体生物阻抗宽频高精度测量。通过 SPI 接口与 RK3568 通信，利用 AFE 内部序列器（Sequencer）自动完成激励信号发生、ADC 采样、DFT 运算等测量流程，FIFO 阈值中断触发内核 IIO triggered buffer 机制，将 4 通道 DFT 原始数据推送至用户态。用户态守护进程通过 libiio 流式读取 IIO buffer，完成 18-bit 符号扩展与阻抗计算后，经 Unix DGRAM Socket 将结果推送至 Qt GUI 前端实时绘制 Bode 图。

### 1.2 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        用户态 (Userspace)                       │
│                                                                │
│   ┌──────────────────┐  cmd DGRAM→   ┌────────────────────┐   │
│   │   Qt GUI 前端    │ ───────────►  │  ad5940_bia_daemon │   │
│   │  (mainwindow)    │ ←data DGRAM─  │  (C 后端)          │   │
│   └──────────────────┘                └─────────┬──────────┘   │
│                                                │ libiio        │
│                                                │ IIO buffer    │
│                                    ┌───────────▼───────────┐   │
│                                    │   /dev/iio:deviceX    │   │
│                                    │   /sys/bus/iio/...    │   │
│                                    └───────────┬───────────┘   │
└────────────────────────────────────────────────┼───────────────┘
                                                 │
                   内核/用户边界                  │
                                                 │
┌────────────────────────────────────────────────┼───────────────┐
│                        内核态 (Kernel)          │               │
│                                    ┌───────────▼───────────┐   │
│                                    │     IIO 子系统        │   │
│                                    │  (triggered buffer)   │   │
│                                    └───────────┬───────────┘   │
│                                    ┌───────────▼───────────┐   │
│                                    │    ad5940_drv.c       │   │
│                                    │  (SPI driver + IIO)   │   │
│                                    └───────────┬───────────┘   │
│                                    ┌───────────▼───────────┐   │
│                                    │    ad5940_core.c      │   │
│                                    │  (AFE 寄存器 + BIA)   │   │
│                                    └───────────┬───────────┘   │
│                                    ┌───────────▼───────────┐   │
│                                    │     SPI 子系统        │   │
│                                    └───────────┬───────────┘   │
│                                    ┌───────────▼───────────┐   │
│                                    │  AD5940 AFE (SPI0)    │   │
│                                    └───────────────────────┘   │
└────────────────────────────────────────────────────────────────┘
```

**核心数据流**：Qt 发送 START 命令（`/tmp/bia_cmd.sock`）→ WUPT 周期触发 → AFE 序列器执行测量 → FIFO 达阈值 → GP0 中断 → 内核 IRQ handler → IIO trigger handler 读取 FIFO → iio_push_to_buffers → 用户态 libiio stream → 阻抗计算 → 数据推送（`/tmp/bia_sample.sock`）→ Qt 绘图。

---

## 二、技术亮点

### 2.1 脱离 ADI 官方库的 AFE 序列器独立重写（内核态实现 SEQGen）

ADI 官方库 (`ad5940lib`) 仅提供裸机/C51 的 `SEQGen` 序列生成器，无法在 Linux 内核中使用。本项目在 `ad5940_core.c` 中基于对官方库逻辑的理解，独立重写了完整的**序列器命令生成器**，包括：
- 影子寄存器（shadow register）读-改-写机制，确保序列器命令与当前 AFE 状态一致
- 9 个等价函数：`seq_afe_ctrl`、`seq_adc_mux_cfg`、`seq_sw_matrix_cfg`、`seq_ref_cfg`、`seq_adc_filter_cfg`、`seq_dft_cfg`、`seq_gpio_ctrl`、`seq_enter_sleep` 等
- 自动生成 Init/Measure 两段序列并写入 SRAM，完全复现 `AppBIASeqCfgGen` + `AppBIASeqMeasureGen` 的功能

### 2.2 三频段自适应扫频与动态序列切换

AD5940 的 ADC/DFT 滤波器参数在不同频段差异巨大（低频需 SINC2+Notch+高 OSR，高频需 SINC3+低 OSR），无法用单一配置覆盖全频段。本项目实现了：
- **三频段 DSP 参数自动选择**：Band0 (≤100Hz) / Band1 (≤2kHz) / Band2 (>2kHz)，各段独立配置 SINC3/SINC2 OSR、DFT 点数、功耗模式、等待时钟数
- **频段切换时动态重生成测量序列**：扫频步进检测到频段变化后，实时重新生成 Measure 序列并写入 SRAM + 更新 PMBW，无需重启测量
- **RTIA 三频段独立校准**：每个活跃频段单独执行 RCAL/RTIA 双次 DFT 测量并存储校准因子（幅值 mΩ + 相位 mdeg），确保全频段测量精度

### 2.3 IIO triggered buffer 全流程集成

在 Linux IIO 子系统中完整实现了 `triggered_buffer` 机制：
- 8 通道 IIO 通道定义（4×DFT + 频率 + RTIA幅值/相位 + 时间戳），含 18-bit 有符号数 scan_type
- 硬中断（GPIO 下降沿）→ `disable_irq_nosync` 防重入 → `iio_trigger_poll` 调度线程化 handler → FIFO 批量读取 + 扫频步进 → `iio_trigger_notify_done` 重新使能 IRQ
- 扫频计数逻辑：自动检测一轮扫频完成后停止 buffer，防止数据回绕

### 2.4 多线程生产者-消费者解耦架构

用户态守护进程采用三线程 + 环形缓冲区架构，将采集、通信与命令控制解耦：
- **main_thread**：绑定命令 Socket（`/tmp/bia_cmd.sock`），接收 Qt 端 S/T/?/Q 命令，控制采集启停
- **acq_thread**：阻塞式 libiio stream 读取 + 阻抗计算 + ring_push（生产者）
- **comm_thread**：ring_pop + sendto 数据 Socket（`/tmp/bia_sample.sock`，消费者），首包发送 `bia_meta_t` 元信息
- **环形缓冲区**：16 槽位，满则丢弃最旧数据，空则条件等待，兼顾实时性与可靠性

### 2.5 开机自启动与 Weston 异步等待

`S50systemui` 脚本解决了 Buildroot 启动时序问题：
- `insmod` 阻塞至 probe 完成（含 RTIA 校准），轮询等待 IIO 设备就绪
- Qt 前端以后台子进程等待 `wayland-0` socket 出现后再启动，不阻塞 init 流程

---

## 三、目录结构

```
ad5940_driver/
├── ad5940_core.h              # AFE 寄存器定义、数据结构、API 声明
├── ad5940_core.c              # SPI 底层通信 + BIA 核心实现（序列器、扫频、RTIA 校准）
├── ad5940_drv.c               # SPI 平台驱动 + IIO triggered buffer 集成
├── ad5940_bia_daemon.c        # 用户态守护进程（libiio 采集 + 阻抗计算 + Unix Socket 通信）
├── Makefile                   # 内核模块 + 用户态程序交叉编译
├── S50systemui                # 开机自启动脚本（insmod → daemon → Qt GUI）
├── scripts/
│   ├── AFE_enable.sh          # 调试用：开启 IIO scan_elements + buffer
│   ├── AFE_disable.sh         # 调试用：关闭 IIO buffer
│   └── cpu_test.sh            # 调试用：采样 10s 计算 CPU 占用率
├── myQtProcess/               # Qt 前端（Bode 图实时绘制）
│   └── myQtProcess/
│       ├── mainwindow.h
│       ├── mainwindow.cpp
│       └── ...
├── adi_mcu_lib/               # ADI 官方库参考（ad5940lib + 示例代码，仅作对照）
├── libiio/                    # libiio 源码（交叉编译后静态链接至 daemon）
├── datasheet/
│   └── ad5940-5941.pdf        # AD5940 数据手册
└── project_framework.md       # 详细项目框架文档
```

---

## 四、使用方式

### 4.1 设备树配置

在 RK3568 设备树中添加 AD5940 SPI 子节点：

```dts
&spi1 {
    pinctrl-0 = <&spi1m1_cs0 &spi1m1_pins>;
    pinctrl-1 = <&spi1m1_cs0_hs &spi1m1_pins_hs>;
    pinctrl-names = "default", "high_speed";
    cs-gpios = <&gpio3 RK_PA1 GPIO_ACTIVE_LOW>;
    status = "okay";

    ad5940@0 {
        compatible = "adi,ad5940";
        reg = <0>;
        spi-max-frequency = <4000000>;
        interrupt-parent = <&gpio3>;
        interrupts = <8 IRQ_TYPE_EDGE_FALLING>;
        reset-gpios = <&gpio1 RK_PB1 GPIO_ACTIVE_LOW>;
    };
};
```

**引脚说明**：

| 功能 | GPIO | 说明 |
|------|------|------|
| SPI CS | GPIO3_A1 | SPI 片选 |
| SPI IRQ | GPIO3_A8 | AFE 中断输出（FIFO 阈值 → GP0，下降沿触发） |
| AFE Reset | GPIO1_B1 | 硬件复位（低电平有效） |

### 4.2 IIO 通道说明

驱动注册 8 个 IIO 通道：

| 通道 | IIO 类型 | 精度 | 说明 |
|------|----------|------|------|
| `in_current0` | CURRENT | s18/32 | 电流 DFT 实部 |
| `in_current1` | CURRENT | s18/32 | 电流 DFT 虚部 |
| `in_voltage0` | VOLTAGE | s18/32 | 电压 DFT 实部 |
| `in_voltage1` | VOLTAGE | s18/32 | 电压 DFT 虚部 |
| `in_altvoltage0` | ALTVOLTAGE | u32 | 激励频率 (Hz) |
| `in_resistance0` | RESISTANCE | s64 | RTIA 校准幅值 (mΩ) |
| `in_phase0` | PHASE | s32 | RTIA 校准相位 (mdeg) |
| `timestamp` | — | s64 | IIO 时间戳 |

**BIA 4 线电极配置**：
- 电流路径：CE0 → 人体 → AIN1 → HSTIA (RTIA)
- 电压检测：AIN3(+) / AIN2(-)，独立高阻抗检测

### 4.3 模块参数

通过 `insmod` 命令行或 `/sys/module/ad5940/parameters/` 在线配置：

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `sweep_en` | bool | true | 使能扫频模式 |
| `sweep_start_hz` | uint | 10 | 扫频起始频率 (Hz) |
| `sweep_stop_hz` | uint | 200 | 扫频终止频率 (Hz) |
| `sweep_points` | uint | 15† | 扫频点数 |
| `sweep_type` | uint | 2 | 扫频类型：0=线性，1=对数，2=自定义频率表 |

> **†** `sweep_points` 默认值 15 对线性/对数模式有效；自定义频率表模式（`sweep_type=2`）下，probe 时自动覆盖为频率表长度（当前 12 个频点）。

**示例**：线性扫频 20Hz ~ 100kHz：
```bash
insmod ad5940.ko sweep_type=0 sweep_start_hz=20 sweep_stop_hz=100000
```

### 4.4 编译与安装

#### 前置条件

- RK3568 Linux SDK（含内核源码与交叉编译工具链，内核版本 4.19）
- `libiio` 源码（已包含在 `libiio/` 目录下，需交叉编译）

#### 内核 Kconfig 配置

> **注意**：RK3568 默认内核配置（4.19）不会编译 `CONFIG_IIO_TRIGGERED_BUFFER`，而本驱动依赖该选项实现 triggered buffer 机制。若不手动使能，编译时会因缺少 `iio_triggered_buffer_setup` 等符号而报错。
>
> 解决方法：在板级 Kconfig 中添加 AD5940 驱动选项，通过 `select IIO_TRIGGERED_BUFFER` 自动拉起依赖：
>
> ```kconfig
> config AD5940
>     tristate "Analog Devices AD5940 AFE driver"
>     depends on IIO && SPI && IIO_BUFFER
>     select IIO_TRIGGERED_BUFFER
> ```
>
> 配置后在 `make menuconfig` 中使能 `AD5940`（或直接在 defconfig 中追加 `CONFIG_AD5940=y`），`IIO_TRIGGERED_BUFFER` 会被自动选中编译。

#### 编译

```bash
# 1. 修改 Makefile 中的 KERNELDIR 和 CROSS_COMPILE 指向你的 SDK 路径
# 2. 编译内核模块 + 用户态程序
make build

# 产物：
#   ad5940.ko              — 内核模块
#   ad5940_bia_daemon      — 用户态守护进程（静态链接 libiio）
```

#### 部署

```bash
# 将以下文件拷贝至目标板 /mydrivers/ 目录
adb push ad5940.ko /mydrivers/
adb push ad5940_bia_daemon /mydrivers/
adb push myQtProcess /mydrivers/

# 安装开机自启动脚本
adb push S50systemui /etc/init.d/S50systemui
chmod +x /etc/init.d/S50systemui
```

### 4.5 测试验证

#### 方式一：开机自启动

```bash
/etc/init.d/S50systemui start
# 自动执行：insmod → 等待 IIO 就绪 → 启动 daemon → 启动 Qt GUI
```

#### 方式二：手动逐步测试

```bash
# Step 1: 加载内核模块
insmod /mydrivers/ad5940.ko sweep_type=0 sweep_start_hz=2200 sweep_stop_hz=100000

# Step 2: 确认 IIO 设备已注册
cat /sys/bus/iio/devices/iio:device*/name
# 预期输出: ad5940

# Step 3: 启动守护进程
/mydrivers/ad5940_bia_daemon &

# Step 4: 启动 Qt 前端
/mydrivers/myQtProcess
```

#### 方式三：sysfs 直接调试（无需 daemon/Qt）

```bash
DEVICE=/sys/bus/iio/devices/iio:device1

# 使能所有 scan_elements
echo 1 > $DEVICE/scan_elements/in_current0_en
echo 1 > $DEVICE/scan_elements/in_current1_en
echo 1 > $DEVICE/scan_elements/in_voltage0_en
echo 1 > $DEVICE/scan_elements/in_voltage1_en

# 使能 buffer（开始采集）
echo 1 > $DEVICE/buffer/enable

# 读取数据
cat /dev/iio:device1 | hexdump -C

# 停止采集
echo 0 > $DEVICE/buffer/enable
```

也可使用 `scripts/` 目录下的快捷脚本：
```bash
sh scripts/AFE_enable.sh    # 开启采集
sh scripts/AFE_disable.sh   # 关闭采集
sh scripts/cpu_test.sh      # 检测内核态 CPU 占用
```

### 4.6 预期测试现象

系统正常采集时，可从两个窗口同时观察数据：

#### (A) 屏幕 — Qt GUI Bode 图

Qt 前端启动后点击 **"启动"** 按钮，屏幕上实时绘制阻抗幅频/相频响应曲线：

<img width="600" alt="image" src="https://github.com/user-attachments/assets/77cea791-8f4a-4722-8de1-5e764667cca0" />


- X 轴：对数频率坐标，覆盖扫频范围（如 2.2kHz ~ 100kHz）
- 左 Y 轴：阻抗幅值 |Z|（Ω）
- 右 Y 轴：相位角（°）
- 状态栏显示采集进度：`采集中... N/M 个频点`
- 一轮扫频完成后自动停止并更新状态标签

#### (B) 串口终端 — 实时原始数据

通过串口登录开发板，可实时查看每个频点的完整测量值。根据 daemon 运行方式不同：

**前台运行**（手动启动，stdout 直连终端）：数据直接打印到串口。

**后台运行**（开机自启 / `&` 后台）：stdout 被重定向至日志文件，需通过以下命令实时跟踪：

```bash
# 清空旧日志
> /var/log/ad5940_samples.log

# 实时跟踪采样数据
tail -f /var/log/ad5940_samples.log

# 查看完整日志
cat /var/log/ad5940_samples.log
```

预期输出格式：

```text
===== Acquisition STARTED (expecting 12 points) =====

[ 1/12]   2200 Hz  |Z|=  1234.56Ω  Ph=  +12.34° R=  1205.23Ω  X=    +265.10Ω
               (I:+45230,-12890  V:+8923,+4567  Rtia=1000.5mΩ/0.3mdeg)
[ 2/12]   5000 Hz  |Z|=   987.65Ω  Ph=   -5.67° R=   983.45Ω  X=    -97.32Ω
               (I:+38120,-9230   V:+7654,+3210  Rtia=1000.8mΩ/0.2mdeg)
 ...
[12/12] 100000 Hz  |Z|=    45.67Ω  Ph=  +67.89° R=    17.10Ω  X=    +42.31Ω
               (I:+520,-180     V:+15,+18       Rtia=1001.2mΩ/0.5mdeg)

===== Sweep COMPLETE: 12/12 samples collected =====
```

各字段含义：

| 字段 | 说明 |
|------|------|
| `\|Z\|` | 阻抗幅值 (Ohms) |
| `Ph` | 阻抗相位角 (degrees) |
| `R` | 电阻分量 = \|Z\| × cos(Ph) (Ohms) |
| `X` | 电抗分量 = \|Z\| × sin(Ph) (Ohms) |
| `I:±d,±d` | 电流通道 DFT 原始值（实部, 虚部，18-bit 符号扩展后） |
| `V:±d,±d` | 电压通道 DFT 原始值（实部, 虚部） |
| `Rtia` | 当前频段 RTIA 校准因子（幅值 mΩ / 相位 mdeg） |

## License

SPDX-License-Identifier: GPL-2.0-only

---

<p align="center">Developed by <b>Mason Wang</b></p>
