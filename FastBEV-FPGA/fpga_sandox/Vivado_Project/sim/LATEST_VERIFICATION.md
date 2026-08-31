# 最新验证结果（2026-08-21）

目标器件：`xc7z030ffg676-2`，Vivado 2018.3，目标 HP 时钟 200 MHz。

## 布局布线

- Setup：WNS `+0.030 ns`，TNS `0 ns`，失败端点 `0`；
- Hold：WHS `+0.028 ns`，THS `0 ns`，失败端点 `0`；
- SA 引擎内部最差 setup：`+0.275 ns`；
- 路由：101892/101892 个可路由网络全部完成，路由错误 `0`；
- DRC error：`0`；
- 资源：LUT 48770（62.05%）、寄存器 59530（37.87%）、BRAM Tile
  140（52.83%）、DSP 38（9.50%）。

报告中仍有 2 个脉宽失败端点，`WPWS=-0.409 ns`。两者均位于受保护的
`ps_ai_wrap_demo.edf` 内部 1 GHz `AI_CORE_CLK` PLL/BUFG 路径，不属于可修改
RTL，按工程约定忽略。全局最差 setup 路径同样位于该受保护网表中。

发布包中的 Vivado 报告归档在：
`Verification_Results/synthesis&implement_result/`。

## RTL 回归

整理后的 `Vivado_Project/sim/run_all.ps1` 完整通过：

- Part1/Part3 网络接口与 RAW 文件检查；
- 160000 项真实 LUT 格式、范围及 SHA256 检查；
- FP32→TF32→FP16 转换；
- FP32 加法器数值与 5 拍延迟；
- DDR 回压与写请求稳定性；
- 顶层启动/完成脉冲 CDC；
- 整帧仿真：20000 次 LUT burst、624500 次特征读取、320000 次输出写回。

最终结果：`ALL_RTL_CHECKS_PASS`。
