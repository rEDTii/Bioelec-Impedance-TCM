```
AD5940_BIA 项目框架
│
├── AD5940Main.c ── 顶层入口与平台配置
│   │
│   ├── AD5940PlatformCfg() ── AFE硬件平台初始化
│   │   ├── AD5940_HWReset()          → 硬件复位
│   │   ├── AD5940_Initialize()       → SPI通信初始化
│   │   ├── AD5940_CLKCfg()           → 时钟配置(HFOSC 16MHz, LFOSC 32kHz)
│   │   ├── AD5940_FIFOCfg()          → FIFO先禁用再使能(复位FIFO), Src=DFT, Thresh=4
│   │   ├── AD5940_INTCCfg()          → INTC1:全中断监控; INTC0:FIFO阈值中断→GP0→MCU
│   │   ├── AD5940_INTCClrFlag()      → 清除所有中断标志
│   │   ├── AD5940_AGPIOCfg()         → GP0=INT输出, GP2=TRIG输入
│   │   └── AD5940_SleepKeyCtrlS()    → 允许AFE进入睡眠
│   │
│   ├── AD5940BIAStructInit() ── 覆盖默认应用参数
│   │   ├── SeqStartAddr=0, MaxSeqLen=512
│   │   ├── RcalVal=10kΩ, DftNum=8192
│   │   ├── NumOfData=-1(永不停止), BiaODR=20Hz, FifoThresh=4
│   │   └── ADCSinc3Osr=2
│   │
│   ├── BIAShowResult() ── 数据展示
│   │   ├── AppBIACtrl(BIACTRL_GETFREQ) 获取当前频率
│   │   └── printf 阻抗幅值(RzMag)和相位(RzPhase)
│   │
│   └── AD5940_Main() ── 主循环
│       ├── AD5940PlatformCfg()        → 平台初始化
│       ├── AD5940BIAStructInit()      → 参数初始化
│       ├── AppBIAInit(AppBuff, 512)   → 应用初始化(生成序列+执行Init序列)
│       ├── AppBIACtrl(BIACTRL_START)  → 启动WUPT驱动周期测量
│       └── while(1) 轮询中断标志
│           ├── AD5940_GetMCUIntFlag()  → 检查MCU GPIO中断
│           ├── AppBIAISR()            → 中断服务(FIFO读取+数据处理)
│           └── BIAShowResult()        → 打印结果
│
├── BodyImpedance.h ── 数据结构与接口定义
│   │
│   ├── AppBIACfg_Type ── 应用配置结构体
│   │   ├── 序列器空间: SeqStartAddr, MaxSeqLen, SeqStartAddrCal, MaxSeqLenCal
│   │   ├── 时钟频率: SysClkFreq(16M), WuptClkFreq(32k), AdcClkFreq(16M)
│   │   ├── 采样参数: BiaODR(20Hz), FifoThresh(4), NumOfData(-1)
│   │   ├── 激励参数: SinFreq(50kHz), DacVoltPP(800mV), ExcitBufGain, HsDacGain
│   │   ├── ADC参数: ADCPgaGain, ADCSinc3Osr, ADCSinc2Osr
│   │   ├── DFT参数: DftNum(8192), DftSrc(SINC3), HanWinEn(true)
│   │   ├── TIA参数: HstiaRtiaSel(1kΩ), CtiaSel(16pF)
│   │   ├── 扫频: SweepCfg(SweepEn, Start=10k, Stop=150k, Points=100, Log=true)
│   │   ├── 内部状态: RtiaCalTable[100][2], SweepCurrFreq, SweepNextFreq
│   │   └── 序列信息: InitSeqInfo(SEQID_1), MeasureSeqInfo(SEQID_0)
│   │
│   ├── 控制命令
│   │   ├── BIACTRL_START(0)     → 启动测量
│   │   ├── BIACTRL_STOPNOW(1)   → 立即停止
│   │   ├── BIACTRL_STOPSYNC(2)  → 同步停止(在下次ISR中停止)
│   │   ├── BIACTRL_GETFREQ(3)   → 获取当前数据频率
│   │   └── BIACTRL_SHUTDOWN(4)  → 关闭AFE进入休眠
│   │
│   └── 公共接口
│       ├── AppBIAGetCfg()   → 获取配置结构体指针
│       ├── AppBIAInit()     → 应用初始化
│       ├── AppBIAISR()      → 中断服务
│       └── AppBIACtrl()     → 控制命令
│
└── BodyImpedance.c ── 核心实现
    │
    ├── AppBIACfg ── 全局配置实例(默认值)
    │   └── PwrMod=LP, Rtia=1kΩ, SinFreq=50kHz, SweepEn=false ...
    │
    ├── 【配置阶段 ── AppBIAInit() 入口】
    │   │
    │   ├── 1. AD5940_WakeUp()           → 唤醒AFE
    │   ├── 2. AD5940_SEQCfg()           → 配置序列器(2KB SRAM, 先禁用)
    │   ├── 3. AppBIARtiaCal()           → RTIA校准
    │   │   ├── 扫频模式: 遍历100个频率点, 每点调用AD5940_HSRtiaCal()
    │   │   │   → 结果存入RtiaCalTable[i][Mag, Phase]
    │   │   └── 单频模式: 仅校准SinFreq, 结果存RtiaCurrValue
    │   │
    │   ├── 4. AD5940_FIFOCfg()          → 重配FIFO(Src=DFT, Thresh=4)
    │   ├── 5. AppBIASeqCfgGen()         → ★生成初始化序列(SEQID_1)
    │   │   ├── 配置HP参考电源(Bandgap, 1V1, 1V8)
    │   │   ├── 配置HSLoop:
    │   │   │   ├── HSDAC: 增益, 更新率, 激励电压
    │   │   │   ├── HSTIA: Rtia=1kΩ, Ctia=16pF, Bias=1.1V
    │   │   │   ├── SW矩阵: PL/NL闭环(初始化态)
    │   │   │   └── WG: 正弦波, 频率Word计算(SweepStart或SinFreq)
    │   │   ├── 配置LPLoop:
    │   │   │   ├── LPDAC0: Vbias=1100mV, Vzero=31
    │   │   │   └── LPTIA: Rf=20kΩ, 开关配置
    │   │   ├── 配置DSP:
    │   │   │   ├── ADC: Mux=HSTIA_P/N, PGA, Sinc3Osr, Sinc2Osr
    │   │   │   └── DFT: Num=8192, Src=SINC3, Hanning窗
    │   │   ├── 使能AFE模块(HPRef, HSTIA, WG, DACRef, HSDAC, Sinc2Notch)
    │   │   └── 写入SRAM → InitSeqInfo
    │   │
    │   ├── 6. AppBIASeqMeasureGen()     → ★生成测量序列(SEQID_0)
    │   │   ├── 计算DFT等待时钟数(AD5940_ClksCalculate)
    │   │   │
    │   │   │  ┌── Step1: 电流DFT ──────────────────────┐
    │   │   ├──│  SW矩阵: D→CE0, P→CE0, N→AIN1, T→AIN1+TRTIA │
    │   │   │  ADC Mux: HSTIA_P / HSTIA_N                │
    │   │   │  使能WG+ADCPWR → 等50us                    │
    │   │   │  使能ADCCNV+DFT → 等WaitClks               │
    │   │   │  关闭ADCCNV+DFT+WG+ADCPWR                  │
    │   │   │  └──────────────────────────────────────────┘
    │   │   │
    │   │   │  ┌── Step2: 电压DFT ──────────────────────┐
    │   │   ├──│  ADC Mux: AIN3(+) / AIN2(-)            │
    │   │   │  使能WG+ADCPWR → 等50us                    │
    │   │   │  使能ADCCNV+DFT → 等WaitClks               │
    │   │   │  关闭ADCCNV+DFT+WG+ADCPWR                  │
    │   │   │  └──────────────────────────────────────────┘
    │   │   │
    │   │   ├── 恢复SW矩阵为PL/NL闭环
    │   │   ├── AFE进入睡眠
    │   │   ├── 计算MaxODR, 修正BiaODR不超过MaxODR
    │   │   └── 写入SRAM → MeasureSeqInfo
    │   │
    │   ├── 7. 执行Init序列
    │   │   ├── AD5940_SEQInfoCfg()  → 配置InitSeqInfo
    │   │   ├── AD5940_SEQCfg(Enable) → 使能序列器
    │   │   ├── AD5940_SEQMmrTrig(SEQID_1) → 触发Init序列
    │   │   └── while(等待ENDSEQ标志)
    │   │
    │   ├── 8. 配置Measure序列(等待WUPT触发)
    │   │   ├── AD5940_SEQInfoCfg()  → 配置MeasureSeqInfo
    │   │   └── AD5940_SEQCfg(Enable) → 使能序列器, 等待触发
    │   │
    │   └── 9. AD5940_AFEPwrBW(LP, 250kHz) + SWMUX配置
    │       └── BIAInited = bTRUE
    │
    ├── 【工作阶段 ── 周期性触发】
    │   │
    │   AppBIACtrl(BIACTRL_START) → 启动Wakeup Timer
    │   └── WUPT周期 = WuptClkFreq/BiaODR, 每次触发SEQID_0
    │
    │   每个WUPT周期自动执行:
    │   ┌─────────────────────────────────────────────┐
    │   │ WUPT触发 → SEQID_0(测量序列)               │
    │   │   Step1: 电流DFT → FIFO写入2个word(Real+Imag)│
    │   │   Step2: 电压DFT → FIFO写入2个word(Real+Imag)│
    │   │   FIFO达阈值4 → GP0拉低 → MCU GPIO中断      │
    │   │   AFE自动进入睡眠                           │
    │   └─────────────────────────────────────────────┘
    │
    ├── 【中断处理 ── AppBIAISR()】
    │   │
    │   ├── AD5940_WakeUp()              → 唤醒AFE
    │   ├── SLPKEY_LOCK                  → 禁止AFE睡眠(操作期间)
    │   ├── 检查INTC0 DATAFIFOTHRESH标志
    │   ├── AD5940_FIFOGetCnt()          → 获取FIFO数据量(对齐到4)
    │   ├── AD5940_FIFORd()              → 批量读取FIFO
    │   ├── AD5940_INTCClrFlag()         → 清除FIFO阈值中断标志
    │   ├── AppBIARegModify()            → 寄存器修改
    │   │   ├── 检查NumOfData是否到达 → 停止WUPT
    │   │   ├── 检查StopRequired       → 停止WUPT
    │   │   └── SweepEn? → AD5940_WGFreqCtrlS() 切换到下一个频率
    │   ├── SLPKEY_UNLOCK               → 允许AFE睡眠
    │   ├── AppBIADataProcess()         → ★数据处理
    │   │   ├── 每4个FIFO word = 1次阻抗结果
    │   │   │   word[0]: 电流DFT实部  word[1]: 电流DFT虚部
    │   │   │   word[2]: 电压DFT实部  word[3]: 电压DFT虚部
    │   │   ├── 18bit符号扩展
    │   │   ├── 计算电压/电流的幅值和相位
    │   │   │   Z = V_mag / I_mag × Rtia
    │   │   │   θ = V_phase - I_phase + Rtia_phase
    │   │   ├── 输出: Magnitude(Ω), Phase(rad)
    │   │   └── 扫频推进: SweepCurrFreq→FreqofData, 调用AD5940_SweepNext()
    │   └── 返回数据计数
    │
    └── AppBIACtrl() ── 控制命令分发
        ├── BIACTRL_START    → 启动WUPT(周期触发测量序列)
        ├── BIACTRL_STOPNOW  → 立即禁用WUPT
        ├── BIACTRL_STOPSYNC → 置StopRequired标志(ISR中停止)
        ├── BIACTRL_GETFREQ  → 返回当前数据频率(扫频/单频)
        └── BIACTRL_SHUTDOWN → 停止+关闭参考+关闭LP环路+休眠
```
```
WUPT周期触发
  → SEQID_0测量序列执行
    → Step1: CE0/AIN1电流通路 → HSTIA → ADC → DFT → FIFO[I_real, I_imag]
    → Step2: AIN3/AIN2电压通路 → ADC → DFT → FIFO[V_real, V_imag]
      → FIFO达阈值4 → INTC0 → GP0下降沿 → MCU中断
        → AppBIAISR: 读FIFO → 切频率(扫频时) → 数据处理
          → Z = |V|/|I| × Rtia, θ = ∠V - ∠I + ∠Rtia
```