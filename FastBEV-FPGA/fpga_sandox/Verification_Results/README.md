# fpga_sandox 验证与实现结果

本目录归档新版 FastBEV Part2 FPGA 工程的 RTL 回归摘要及 Vivado 2018.3
综合、布局布线报告。目标器件为 `xc7z030ffg676-2`，HP 目标时钟为 200 MHz。

## 2026-08-21 发布回归

在整理后的 `fpga_sandox` 目录结构上重新运行完整回归，结果为
`ALL_RTL_CHECKS_PASS`：

- Part1 输出：FP32 NHWC `[6,120,160,64]`，29,491,200 bytes；
- Part3 对接：Conv256 输入 value 362，FP16 NCHWc16 `[1,16,200,200,16]`，
  20,480,000 bytes；
- LUT：160,000 项、1,280,000 bytes、156,125 项有效，SHA-256 为
  `84187065e7fa9b7c2dd7aecf7992770bfa4f21b809696f14f9e0ee610a566383`；
- 单元回归：FP32→TF32→FP16、FP32 加法器、LUT/DDR 回压、顶层 CDC 均通过；
- 整帧回归：20,000 次 LUT burst、624,500 次特征读取、320,000 次输出写回。

## Vivado 2018.3 实现摘要

| 指标 | 结果 |
|---|---:|
| Setup WNS / TNS | `+0.030 ns / 0 ns` |
| Hold WHS / THS | `+0.028 ns / 0 ns` |
| SA 内部最差 setup | `+0.275 ns` |
| 可路由网络 | `101892 / 101892`，错误 0 |
| DRC error | 0 |
| Slice LUT | `48,770 / 78,600 (62.05%)` |
| Slice Register | `59,530 / 157,200 (37.87%)` |
| BRAM Tile | `140 / 265 (52.83%)` |
| DSP | `38 / 400 (9.50%)` |

剩余两个 `WPWS=-0.409 ns` 端点均位于受保护
`ps_ai_wrap_demo.edf` 的 1 GHz `AI_CORE_CLK` PLL/BUFG 路径，按工程边界保留并
忽略；自写 RTL 的 setup/hold 及 SA 内部路径均无负裕量。

详细报告位于 `synthesis&implement_result/`。DCP 属于体积较大的中间产物，
未收入发布目录；需要复现时运行
`Vivado_Project/sim/run_vivado_ascii.ps1`。
