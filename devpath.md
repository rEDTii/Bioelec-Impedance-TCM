```
Phase 1: 单频点BIA（验证信号链）
  ├── 移植 AppBIASeqCfgGen 的寄存器序列（方案B：静态数组）
  ├── 移植 AppBIASeqMeasureGen 的测量序列（静态数组）
  ├── 在 probe 中执行 Init 序列 + Measure 序列
  ├── 启动 WUPT，验证 FIFO 中断 + trigger_handler 读到4个DFT word
  └── 在用户空间做阻抗计算（暂不内嵌驱动）

Phase 2: RTIA 校准
  ├── 移植 AD5940_HSRtiaCal
  ├── 在 trigger_set_state(true) 中执行校准
  └── 校准结果写入驱动或通过 sysfs 输出

Phase 3: 扫频
  ├── 增加 SweepCfg 到 ad5940_priv
  ├── 在 trigger_handler 中切换频率 + 推进 SweepIndex
  ├── 增加频率 channel 或通过 sysfs 输出当前频率
  └── 预计算 RtiaCalTable 或在启动时批量校准

Phase 4: 产品化
  ├── 参数通过 DT / sysfs 可配置
  ├── 错误处理 + 恢复
  └── 省电模式（休眠/唤醒）

```