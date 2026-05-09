```
AD5940 BIA Linux 项目框架 (RK3568 / Buildroot / Linux 4.19)
│
│   ┌─────────────────────────────────────────────────────────────────┐
│   │                        用户态 (Userspace)                       │
│   │                                                                 │
│   │   ┌──────────────────┐   Unix DGRAM   ┌────────────────────┐   │
│   │   │   Qt GUI 前端    │ ◄────────────► │  ad5940_bia_daemon │   │
│   │   │  (mainwindow)    │  /tmp/bia_*.sock│  (C 后端)         │   │
│   │   └──────────────────┘               └─────────┬──────────┘   │
│   │                                                │ libiio        │
│   │                                                │ IIO buffer    │
│   │                                    ┌───────────▼───────────┐   │
│   │                                    │   /dev/iio:device0    │   │
│   │                                    │   /sys/bus/iio/...    │   │
│   │                                    └───────────┬───────────┘   │
│   └────────────────────────────────────────────────┼───────────────┘
│                                                    │
│   ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┼ ─ ─ ─ ─ ─ ─
│                         内核/用户边界              │
│   ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┼ ─ ─ ─ ─ ─ ─
│                                                    │
│   ┌────────────────────────────────────────────────┼───────────────┐
│   │                        内核态 (Kernel)         │               │
│   │                                    ┌───────────▼───────────┐   │
│   │                                    │     IIO 子系统        │   │
│   │                                    │  (triggered buffer)   │   │
│   │                                    └───────────┬───────────┘   │
│   │                                    ┌───────────▼───────────┐   │
│   │                                    │    ad5940_drv.c       │   │
│   │                                    │  (SPI driver + IIO)  │   │
│   │                                    └───────────┬───────────┘   │
│   │                                    ┌───────────▼───────────┐   │
│   │                                    │    ad5940_core.c      │   │
│   │                                    │  (AFE 寄存器 + BIA)  │   │
│   │                                    └───────────┬───────────┘   │
│   │                                    ┌───────────▼───────────┐   │
│   │                                    │     SPI 子系统        │   │
│   │                                    └───────────┬───────────┘   │
│   │                                    ┌───────────▼───────────┐   │
│   │                                    │  AD5940 AFE (SPI0)   │   │
│   │                                    └───────────────────────┘   │
│   └─────────────────────────────────────────────────────────────────┘
│
│
├── ═══════════════════════ 内核态 ═══════════════════════
│
├── ad5940_core.h ── 寄存器定义、数据结构、API 声明
│   │
│   ├── SPI 命令字节
│   │   ├── CMD_SETADDR (0x20)     → 设置待访问寄存器地址
│   │   ├── CMD_WRITEREG (0x2d)    → 写寄存器
│   │   ├── CMD_READREG  (0x6d)    → 读寄存器
│   │   └── CMD_READFIFO (0x5f)    → 读 FIFO
│   │
│   ├── 寄存器地址 (0x0000 ~ 0x3000+)
│   │   ├── AFECON (0x2000)        → AFE 主控制
│   │   ├── SEQCON (0x2004)        → 序列器控制
│   │   ├── FIFOCON (0x2008)       → FIFO 控制
│   │   ├── ADCCON (0x21A8)        → ADC 控制 (MUX, PGA)
│   │   ├── DFTCON (0x20D0)       → DFT 控制
│   │   ├── WGFCW (0x2030)        → 波形发生器频率字
│   │   ├── WUPTCON (0x0800)      → 唤醒定时器
│   │   ├── INTCCLR (0x3004)       → 中断清除
│   │   └── FIFOCNT (0x2200)      → FIFO 计数
│   │
│   ├── 位字段常量 (AFECON/ADCCON/DFTCON/FIFOCON 等)
│   │
│   ├── 序列器命令编码
│   │   ├── SEQ_WAIT(clks)         → 等待指定时钟数
│   │   ├── SEQ_WR(addr, data)     → 写寄存器 (序列器执行)
│   │   ├── SEQ_STOP()             → 停止序列器
│   │   └── SEQ_SLP()              → AFE 进入休眠
│   │
│   ├── 硬件配置宏
│   │   ├── AD5940_BIA_RTIA_SEL    → HSTIA RTIA 选择 (默认 1kΩ)
│   │   ├── AD5940_BIA_RTIA_CTIA   → CTIA 补偿电容 (默认 31)
│   │   ├── AD5940_BIA_RCAL_OHM    → PCB 上 RCAL 电阻 (默认 10kΩ)
│   │   └── AD5940_BIA_RTIA_OHM    → 标称 RTIA 电阻值 (自动推导)
│   │
│   ├── 自定义扫频频率表
│   │   └── ad5940_custom_freq_table[] → {1, 10, 25, 50, 100, 200, 1k, 10k, 40k, 50k, 100k, 200k}
│   │
│   ├── 关键数据结构
│   │   ├── ad5940_freq_params     → DSP 滤波器参数 (DFT/SINC/等待时钟/功耗)
│   │   ├── seq_shadow_regs       → 序列器生成器的影子寄存器
│   │   ├── ad5940_rtia_cal_result → RTIA 校准结果 (幅值mΩ, 相位mdeg)
│   │   └── ad5940_priv            → 驱动私有数据
│   │       ├── SPI/GPIO/IRQ/IIO 句柄
│   │       ├── 运行状态 (running, irq_disabled)
│   │       ├── 扫频配置与状态 (sweep_en/type/start/stop/points/index/curr/next)
│   │       ├── 当前频段 DSP 参数 (curr_freq_params)
│   │       ├── 影子寄存器 (meas_shadow) + 序列地址/长度
│   │       └── 频段校准表 (band_cal_table[3], sweep_band_map[])
│   │
│   └── 导出 API 声明
│       ├── 核心: spi_xfer/write/read/fifo_read, reset/wakeup/init
│       ├── BIA: bia_init/bia_rtia_cal/bia_start/bia_stop
│       ├── 序列器: seq_cmd_write
│       └── 扫频: wg_freq_word_cal, sweep_calc_freq, bia_sweep_step
│
├── ad5940_core.c ── 底层寄存器访问 + BIA 核心实现
│   │
│   ├── 【SPI 通信层】
│   │   ├── ad5940_spi_xfer()       → 全双工 SPI 传输 (单次 CS)
│   │   ├── ad5940_spi_write()      → 两事务写: SETADDR + WRITEREG (16/32-bit)
│   │   ├── ad5940_spi_read()       → 两事务读: SETADDR + READREG (16/32-bit)
│   │   └── ad5940_fifo_read()      → FIFO 批量读: SETADDR + READFIFO (burst)
│   │
│   ├── 【硬件控制】
│   │   ├── ad5940_reset()          → GPIO 硬件复位 (10ms 脉冲 + 10ms 等待)
│   │   ├── ad5940_wakeup()         → 唤醒 AFE (读 ADIID, 最多 10 次重试)
│   │   ├── ad5940_init()           → 上电后初始化寄存器表 (14 条写操作)
│   │   └── ad5940_seq_cmd_write()  → 写序列命令到 SRAM (地址+数据逐条)
│   │
│   ├── 【序列器生成器 (复制 ADI SEQGen 逻辑)】
│   │   ├── seq_gen_buf             → 命令缓冲区 + 影子寄存器状态
│   │   ├── seq_afe_ctrl()          → AFECtrlS 等价: 影子 AFECON 读-改-写 + SEQ_WR
│   │   ├── seq_adc_mux_cfg()       → ADCMuxCfgS 等价: 影子 ADCCON + SEQ_WR
│   │   ├── seq_sw_matrix_cfg()     → SWMatrixCfgS 等价: 写 DSW/NPSW/TSW + SEQ_WR
│   │   ├── seq_ref_cfg()           → REFCfgS 等价: 影子 BUFSENCON/LPREFBUFCON
│   │   ├── seq_adc_filter_cfg()    → ADCFilterCfgS 等价: 影子 ADCFILTERCON
│   │   ├── seq_dft_cfg()           → DFTCfgS 等价: 影子 DFTCON
│   │   ├── seq_gpio_ctrl()         → SEQGpioCtrlS 等价: 写 SYNCEXTDEVICE
│   │   ├── seq_enter_sleep()       → EnterSleepS 等价: SEQ_SLP + SEQ_STOP
│   │   │
│   │   ├── ad5940_bia_gen_init_seq()   → ★生成初始化序列 (等价 AppBIASeqCfgGen)
│   │   │   ├── 配置 HP 参考电源 (Bandgap, 1V1, 1V8)
│   │   │   ├── 配置 HSLoop: HSDAC增益/更新率 + HSTIA(Rtia/Ctia/Bias) + SW矩阵 + WG正弦
│   │   │   ├── 配置 LPLoop: LPDAC0(Vbias/Vzero) + LPTIA(Rf/开关)
│   │   │   ├── 配置 DSP: ADC(MUX/PGA/SINC3/SINC2) + DFT(Num/Src/Hanning)
│   │   │   ├── 使能 AFE 模块 (HPRef/HSTIA/WG/DACRef/HSDAC/Sinc2Notch)
│   │   │   └── 返回命令数组 + 影子寄存器快照
│   │   │
│   │   └── ad5940_bia_gen_measure_seq() → ★生成测量序列 (等价 AppBIASeqMeasureGen)
│   │       ├── Step 1: 电流 DFT
│   │       │   ├── SW矩阵: D→CE0, P→CE0, N→AIN1, T→AIN1+TRTIA
│   │       │   ├── ADC MUX: HSTIA_P / HSTIA_N
│   │       │   ├── 使能 WG+ADCPWR → 等 50us
│   │       │   ├── 使能 ADCCNV+DFT → 等 WaitClks
│   │       │   └── 关闭 ADCCNV+DFT (WG 保持运行)
│   │       ├── Step 2: 电压 DFT
│   │       │   ├── ADC MUX: AIN3(+) / AIN2(-)
│   │       │   ├── 使能 ADCCNV+DFT → 等 WaitClks
│   │       │   └── 关闭全部 (ADCCNV+DFT+WG+ADCPWR)
│   │       ├── 恢复 SW矩阵为 PL/NL 闭环
│   │       ├── AFE 进入休眠
│   │       └── 返回命令数组 + 影子寄存器快照
│   │
│   ├── 【三频段 DSP 参数】ad5940_get_freq_params()
│   │   ├── Band0 (≤100Hz):  SINC2NOTCH, S2OSR=667, S3OSR=4,  DFT=8192, LP, WaitClks≈437M
│   │   ├── Band1 (≤2kHz):   SINC2NOTCH, S2OSR=22,  S3OSR=4,  DFT=8192, LP, WaitClks≈14M
│   │   └── Band2 (>2kHz):   SINC3,      S3OSR=2,             DFT=8192, HP, WaitClks≈328K
│   │
│   ├── 【扫频辅助】
│   │   ├── ad5940_wg_freq_word_cal()  → 频率→WGFCW (26-bit, f*2^26/SysClk)
│   │   ├── ad5940_sweep_calc_freq()   → 计算第 i 个频点
│   │   │   ├── CUSTOM: 查 ad5940_custom_freq_table[]
│   │   │   ├── LOG: Q16 定点 Newton-Raphson 增量乘法
│   │   │   └── LINEAR: 等间距线性插值
│   │   └── ad5940_bia_sweep_step()    → ★扫频步进
│   │       ├── 推进 sweep_index (循环回绕)
│   │       ├── 计算下一频率 + 更新 freq_of_data_hz
│   │       └── 频段变化时: 重新生成测量序列 + 写 SRAM + 更新 PMBW
│   │
│   ├── 【RTIA 校准】ad5940_bia_rtia_cal()
│   │   ├── 建立频段映射 (sweep_band_map[]) 和活跃频段列表
│   │   ├── 对每个活跃频段:
│   │   │   ├── 配置该频段的 ADC/DFT 滤波参数
│   │   │   ├── 用 RCAL 测量 DFT: 写 WGFCW → 使能 ADCCNV+DFT → 读 DFT 结果
│   │   │   ├── 用 RTIA 测量 DFT (切换 SW 矩阵)
│   │   │   └── 计算 Rtia = Rcal × |DftRtia/DftRcal| × exp(j(θrtia-θrcal))
│   │   │       → 存入 band_cal_table[band_id] (magnitude_mohm, phase_mdeg)
│   │   └── 首个活跃频段结果存入 priv->rtia_cal (当前校准值)
│   │
│   ├── 【BIA 初始化】ad5940_bia_init()  ★probe 调用, 包含全部 AFE 配置
│   │   ├── 1. CLKCfg: 使能 HFOSC(16MHz) + LFOSC(32kHz), 配 CLKCON0/CLKSEL
│   │   ├── 2. FIFOCfg: Src=DFT, Threshold=4
│   │   ├── 3. INTCfg: INTC1=全监控, INTC0=FIFO阈值→GP0
│   │   ├── 4. AGPIOCfg: GP0=INT输出, GP2=TRIG输入
│   │   ├── 5. SleepKey: 允许 AFE 进入休眠
│   │   ├── 6. RTIA 校准: ad5940_bia_rtia_cal() (遍历活跃频段, 可能耗时数秒~60s)
│   │   ├── 7. 生成 Init 序列: ad5940_bia_gen_init_seq() → 写 SRAM (SEQID_1)
│   │   ├── 8. 生成 Measure 序列: ad5940_bia_gen_measure_seq() → 写 SRAM (SEQID_0)
│   │   ├── 9. 执行 Init 序列: SEQMmrTrig(SEQID_1) → 等 ENDSEQ
│   │   ├── 10. 配置 Measure 序列信息: SEQ0INFO (地址+长度)
│   │   └── 11. AFEPwrBW(LP, 250kHz) + SWMUX 配置
│   │
│   ├── 【BIA 启动】ad5940_bia_start()
│   │   ├── 唤醒 AFE
│   │   ├── 清理 FIFO + INTC (确保 GP0 拉高, 产生下降沿)
│   │   ├── 初始化扫频状态 (sweep_index=0, 当前频率→WGFCW)
│   │   └── 使能 WUPT (周期=32kHz/ODR, 触发 SEQID_0)
│   │
│   └── 【BIA 停止】ad5940_bia_stop()
│       ├── 唤醒 AFE + 锁休眠键
│       ├── 禁用 WUPT (写两次)
│       ├── 复位 FIFO
│       ├── 清除 INTC 标志
│       └── 解锁休眠键
│
├── ad5940_drv.c ── SPI 平台驱动 + IIO 集成
│   │
│   ├── 【模块参数】(insmod 时指定, /sys/module/ad5940/parameters/ 可读写)
│   │   ├── sweep_en (bool, 默认 true)     → 使能扫频
│   │   ├── sweep_start_hz (uint, 默认 10)  → 扫频起始频率
│   │   ├── sweep_stop_hz (uint, 默认 200)  → 扫频终止频率
│   │   ├── sweep_points (uint, 默认 15)    → 扫频点数
│   │   └── sweep_type (uint, 默认 2)       → 扫频类型: 0=线性, 1=对数, 2=自定义
│   │
│   ├── 【IIO 通道定义】ad5940_dft_channels[]
│   │   ├── current0 (s18/32)  → 电流 DFT 实部
│   │   ├── current1 (s18/32)  → 电流 DFT 虚部
│   │   ├── voltage0 (s18/32) → 电压 DFT 实部
│   │   ├── voltage1 (s18/32) → 电压 DFT 虚部
│   │   ├── altvoltage0 (u32) → 激励频率 (Hz)
│   │   ├── resistance0 (s64) → RTIA 校准幅值 (毫欧)
│   │   ├── phase0 (s32)      → RTIA 校准相位 (毫度)
│   │   └── timestamp (s64)    → IIO 时间戳 (自动填充)
│   │
│   ├── 【IIO Trigger 操作】
│   │   ├── ad5940_trigger_set_state()  → buffer 使能时启动 WUPT, 禁用时停止
│   │   └── ad5940_trigger_reenable()   → trigger handler 完成后重新使能 IRQ
│   │
│   ├── 【IIO Trigger Handler】ad5940_trigger_handler()
│   │   ├── 唤醒 AFE + 锁休眠键
│   │   ├── 读 FIFOCNT → 对齐到 4-word 帧
│   │   ├── 逐帧读取 FIFO:
│   │   │   ├── ad5940_fifo_read(4 words)
│   │   │   ├── 填充 freq 通道 (当前测量频率)
│   │   │   ├── 填充 RTIA 校准通道 (当前频段校准值)
│   │   │   ├── iio_push_to_buffers_with_timestamp()
│   │   │   └── 扫频步进: ad5940_bia_sweep_step()
│   │   ├── 清除 FIFO 阈值中断标志
│   │   └── 解锁休眠键 + iio_trigger_notify_done()
│   │
│   ├── 【硬中断处理】ad5940_irq_handler()
│   │   ├── disable_irq_nosync() → 防止重入
│   │   ├── 记录 irq_disabled = true
│   │   └── iio_trigger_poll() → 调度 trigger handler (线程上下文)
│   │
│   ├── 【probe】ad5940_probe()
│   │   ├── 分配 IIO 设备 + 私有数据
│   │   ├── 解析模块参数 → priv->sweep_* (含 custom 表自动更新 sweep_points)
│   │   ├── SPI 配置 (Mode 0, 4MHz, 8-bit)
│   │   ├── 获取 reset GPIO
│   │   ├── 硬件复位: ad5940_reset()
│   │   ├── AFE 初始化: ad5940_init()
│   │   ├── 读 ADIID (0x4144) + CHIPID (0x5502) 验证芯片
│   │   ├── ★BIA 初始化: ad5940_bia_init() (含 RTIA 校准, 可能耗时 ms~60s)
│   │   ├── 配置 IIO 设备 (7 通道 + triggered buffer)
│   │   ├── 注册 IIO trigger + buffer
│   │   ├── 注册 IIO 设备 (devm_iio_device_register)
│   │   └── 申请 IRQ (GPIO 下降沿触发)
│   │
│   ├── 【remove】ad5940_remove()
│   │   ├── 停止测量: ad5940_bia_stop()
│   │   ├── 释放 trigger 引用 (put_device)
│   │   └── assert reset GPIO → AFE 低功耗
│   │
│   └── 【驱动注册】
│       ├── of_match: "adi,ad5940"
│       └── module_spi_driver(ad5940_driver)
│
│
├── ═══════════════════════ 用户态 ═══════════════════════
│
├── ad5940_bia_daemon.c ── C 后端守护进程 (libiio + Unix Socket)
│   │
│   ├── 架构
│   │   ┌──────────┐  cmd DGRAM  ┌────────────────────┐  IIO stream  ┌─────────┐
│   │   │  Qt GUI  │ ◄─────────► │  ad5940_bia_daemon │ ◄──────────── │ AD5940  │
│   │   │          │  S/T/?/Q   │  (本文件)          │               │ driver  │
│   │   └──────────┘             └────────────────────┘               └─────────┘
│   │
│   ├── 【协议定义】
│   │   ├── 命令 Socket: /tmp/bia_cmd.sock (DGRAM)
│   │   │   ├── 'S' → START (使能 IIO buffer, 开始采集)
│   │   │   ├── 'T' → STOP  (禁用 IIO buffer)
│   │   │   ├── '?' → STATUS (回复 sweep_points)
│   │   │   └── 'Q' → QUIT  (退出守护进程)
│   │   │
│   │   ├── 数据 Socket: /tmp/bia_sample.sock (DGRAM)
│   │   │   ├── bia_meta_t  → 首包元信息 (magic + sweep_points + sweep_type)
│   │   │   └── bia_sample_t → 每次测量结果
│   │   │
│   │   └── 数据结构
│   │       ├── bia_sample_t { magnitude, phase, resistance, reactance,
│   │       │                   freq_hz, curr_real/imag, volt_real/imag }
│   │       ├── bia_meta_t   { magic=0xB1A00000, sweep_points, sweep_type }
│   │       └── sign_extend_18bit() → 18-bit 符号扩展
│   │
│   ├── 【阻抗计算】compute_impedance()
│   │   ├── V_mag = |Vr + j*Vi|,  I_mag = |Cr + j*Ci|
│   │   ├── V_phase = atan2(-Vi, Vr),  I_phase = atan2(-Ci, Cr)
│   │   ├── Z_mag   = V_mag / I_mag × Rtia_mag
│   │   ├── Z_phase = V_phase - I_phase + Rtia_phase
│   │   ├── R = Z_mag × cos(Z_phase)
│   │   ├── X = Z_mag × sin(Z_phase)
│   │   └── 输出: bia_sample_t (Ω, °, Ω, Ω, Hz, raw...)
│   │
│   ├── 【环形缓冲区】ringbuf_t (16 个 sample 槽位)
│   │   ├── ring_push() → 生产者 (acq_thread), 满则丢弃最旧
│   │   └── ring_pop()  → 消费者 (comm_thread), 空则等待 cond
│   │
│   ├── 【采集线程】acq_thread_fn()
│   │   ├── 等待 START 命令 (cond_wait)
│   │   ├── 每轮创建新 iio_stream (cancel 后不可复用)
│   │   ├── 采集循环:
│   │   │   ├── iio_stream_get_next_block() → 阻塞等待数据
│   │   │   ├── 读取 7 通道 (4 DFT + freq + RTIA_mag + RTIA_phase)
│   │   │   ├── 18-bit 符号扩展 + 阻抗计算
│   │   │   ├── ring_push() → 推入环形缓冲区
│   │   │   └── 自动停止: sample_count >= sweep_points (防止扫频回绕)
│   │   ├── 轮结束: cancel stream + disable buffer
│   │   └── 回到等待 START
│   │
│   ├── 【通信线程】comm_thread_fn()
│   │   ├── 发送首包 bia_meta_t (sweep_points + sweep_type)
│   │   ├── 循环 ring_pop() → sendto() 数据 Socket
│   │   └── 采集停止时重置 meta_sent 标志
│   │
│   └── 【主线程】main()
│       ├── 创建 IIO context (local)
│       ├── 查找 ad5940 设备 + 7 个通道
│       ├── 获取 IIO buffer + 创建 channels mask
│       ├── 启动 comm_thread + acq_thread
│       ├── 绑定命令 Socket → 命令循环
│       │   ├── START: 读 sysfs sweep_params → 唤醒 acq_thread
│       │   ├── STOP:  iio_stream_cancel() + g_acquiring=0
│       │   ├── STATUS: 回复 sweep_points
│       │   └── QUIT:  退出
│       └── cleanup: join 线程 + 销毁 IIO context
│
├── myQtProcess/ ── Qt 前端 (BIA 阻抗频谱图)
│   │
│   ├── mainwindow.h ── 窗口类定义
│   │   ├── 协议常量 (与 daemon 保持一致)
│   │   │   ├── BIA_DATA_SOCK_PATH = "/tmp/bia_sample.sock"
│   │   │   ├── BIA_CMD_SOCK_PATH  = "/tmp/bia_cmd.sock"
│   │   │   ├── CMD_START/STOP/STATUS/QUIT
│   │   │   └── BIA_META_MAGIC = 0xB1A00000
│   │   │
│   │   ├── 数据结构 (与 daemon 保持一致)
│   │   │   ├── bia_sample_t { magnitude, phase, resistance, reactance,
│   │   │   │                   freq_hz, curr_real/imag, volt_real/imag }
│   │   │   └── bia_meta_t   { magic, sweep_points, sweep_type }
│   │   │
│   │   └── MainWindow 成员
│   │       ├── 数据 Socket: m_dataFd + QSocketNotifier
│   │       ├── 数据存储: m_samples (QMap<freq, bia_sample_t>)
│   │       ├── 图表: QChart + QLineSeries(幅值/相位) + QLogValueAxis(X) + QValueAxis(Y×2)
│   │       ├── 控制: QPushButton(启动/取消) + QLabel(状态)
│   │       └── 状态: m_acquiring, m_sweepPoints, m_receivedPoints
│   │
│   └── mainwindow.cpp ── 窗口实现
│       │
│       ├── 【命令发送】sendCommand(char cmd)
│       │   └── 创建临时 DGRAM socket → sendto → close
│       │
│       ├── 【采集控制】
│       │   ├── startAcquisition()  → 清数据 + sendCommand('S') + setAcquiring(true)
│       │   ├── stopAcquisition()   → sendCommand('T') + setAcquiring(false)
│       │   └── setAcquiring(bool)   → 更新按钮文本/颜色 + 状态标签
│       │
│       ├── 【图表初始化】initChart()
│       │   ├── |Z| (Ω) 蓝色折线 + Phase (°) 红色折线
│       │   ├── X 轴: QLogValueAxis (对数频率, base=10)
│       │   ├── 左 Y: QValueAxis (|Z| 幅值)
│       │   └── 右 Y: QValueAxis (相位, ±90°)
│       │
│       ├── 【数据 Socket】initDataSocket()
│       │   ├── bind /tmp/bia_sample.sock (DGRAM)
│       │   ├── 设置 SO_RCVBUF=64K + O_NONBLOCK
│       │   └── QSocketNotifier → onDataReady()
│       │
│       └── 【数据接收 + 图表刷新】onDataReady()
│           ├── recvfrom → 判断包类型:
│           │   ├── magic == BIA_META_MAGIC → bia_meta_t (更新 m_sweepPoints)
│           │   └── 否则 → bia_sample_t (存入 m_samples)
│           ├── 更新状态标签 (已收到/总频点)
│           ├── refreshChart():
│           │   ├── 重建 Series (删除+新建, 避免追加)
│           │   ├── 按频率排序遍历 m_samples → append 数据点
│           │   └── 自动调整坐标轴范围 (X: 0.5×~2×, Y: 0~1.05×max)
│           └── 扫频完成自动停止 (receivedPoints >= sweepPoints)
│
│
├── ═══════════════════════ 启动脚本 ═══════════════════════
│
└── S50systemui ── 开机自启动脚本 (替代 Buildroot 默认 systemui)
    │
    ├── 启动顺序:
    │   ├── Step 1: insmod ad5940.ko (阻塞至 probe 完成, 含 RTIA 校准)
    │   ├── Step 2: 轮询等待 IIO 设备就绪 (检测 /sys/bus/iio/devices/iio:device*/name == "ad5940")
    │   ├── Step 3: 启动 ad5940_bia_daemon (后台守护进程)
    │   └── Step 4: 启动 Qt GUI (后台等待 weston 就绪后启动)
    │
    ├── 关键函数:
    │   ├── wait_for_device()  → 轮询 IIO 设备 (间隔 2s, 超时 120s)
    │   ├── load_module()      → insmod + MODULE_PARAMS
    │   ├── start_daemon()     → start-stop-daemon + PID 文件
    │   └── start_qt()         → (后台子进程) 等 wayland-0 socket → 启动 Qt
    │
    └── 可配置变量:
        ├── MODULE_PATH="/mydrivers/ad5940.ko"
        ├── MODULE_PARAMS="sweep_type=0 sweep_start_hz=2200 sweep_stop_hz=100000"
        ├── DAEMON_BIN="/mydrivers/ad5940_bia_daemon"
        ├── QT_BIN="/mydrivers/myQtProcess"
        ├── POLL_INTERVAL=2, POLL_TIMEOUT=120
        └── LOG="/var/log/ad5940_bia.log"
```
