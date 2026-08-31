# part2_sa_v1.0 验证与实现结果

本目录保留原有 RTL 定向验证日志，并归档 2026-07-19 重新运行 Vivado 后的
综合与实现报告。实现数据以 `part2_sa_1.0.runs` 为准。

## 最终实现摘要

| 指标 | 结果 |
|---|---:|
| Setup WNS / TNS | `+0.030 ns / 0 ns` |
| Hold WHS / THS | `+0.027 ns / 0 ns` |
| Part2 `clk_pll_i` setup WNS | `+0.224 ns` |
| Pulse width WPWS / TPWS | `-0.409 ns / -0.479 ns`，2 endpoints |
| 全局 timing | `not met`，setup/hold 通过，pulse width 未通过 |
| Fully routed nets | `97,346 / 97,346`，0 routing errors |
| DRC | 0 errors，130 warnings |
| Methodology DRC | 40 warnings，64 advisories |
| Slice LUT | `42,918 / 78,600 (54.60%)` |
| Slice Register | `58,139 / 157,200 (36.98%)` |
| Block RAM Tile | `124 / 265 (46.79%)` |
| DSP48E1 | `97 / 400 (24.25%)` |
| Total on-chip power | `3.963 W`，Confidence Level = Low |
| Bitstream | 生成成功 |

