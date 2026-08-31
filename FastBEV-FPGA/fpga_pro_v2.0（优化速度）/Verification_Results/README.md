# part2_sa_v2.0 验证与实现结果

本目录归档 2026-07-19 重新运行 Vivado 后的综合与实现报告。实现数据以
`part2_sa_2.0.runs` 为准。

## 最终实现摘要

| 指标 | 结果 |
|---|---:|
| Setup WNS / TNS | `+0.030 ns / 0 ns` |
| Hold WHS / THS | `+0.025 ns / 0 ns` |
| Part2 `clk_pll_i` setup WNS | `+0.084 ns` |
| Pulse width WPWS / TPWS | `-0.409 ns / -0.479 ns`，2 endpoints |
| 全局 timing | `not met`，setup/hold 通过，pulse width 未通过 |
| Fully routed nets | `102,206 / 102,206`，0 routing errors |
| DRC | 0 errors，113 warnings |
| Methodology DRC | 48 warnings，64 advisories |
| Slice LUT | `48,102 / 78,600 (61.20%)` |
| Slice Register | `61,296 / 157,200 (38.99%)` |
| Block RAM Tile | `140 / 265 (52.83%)` |
| DSP48E1 | `65 / 400 (16.25%)` |
| Total on-chip power | `4.059 W`，Confidence Level = Low |
| Bitstream | 生成成功 |

