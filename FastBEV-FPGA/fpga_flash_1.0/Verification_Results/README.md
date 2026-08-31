# Flash v1.0 验证与实现结果

本目录归档 2026-07-20 使用 Vivado 2018.3 重新运行综合与实现后提取的关键
报告与日志。完整原始运行目录保存在
`../Vivado_Runs/flash_1.0.runs/`。

## 最终实现摘要

目标器件为 `xc7z030ffg676-2`，设计顶层为 `bev_edif_top`。

| 指标 | 结果 |
|---|---:|
| Setup WNS / TNS | `+0.030 ns / 0 ns` |
| Hold WHS / THS | `+0.029 ns / 0 ns` |
| Part2 `clk_pll_i` setup WNS | `+0.107 ns` |
| Pulse width WPWS / TPWS | `-0.409 ns / -0.479 ns`，2 endpoints |
| 全局 timing | `not met`；setup/hold 通过，pulse width 未通过 |
| Fully routed nets | `86,175 / 86,175`，0 routing errors |
| DRC | 0 errors；报告列出 64 warning violations |
| Methodology DRC | 93 warnings，64 advisories |
| Slice LUT | `39,934 / 78,600 (50.81%)` |
| Slice Register | `50,646 / 157,200 (32.22%)` |
| Block RAM Tile | `124 / 265 (46.79%)` |
| DSP48E1 | `3 / 400 (0.75%)` |
| Total on-chip power | `3.652 W`，Confidence Level = Low |
| Bitstream | `write_bitstream completed successfully` |

## 报告说明

- `bev_edif_top_timing_summary_routed.rpt` 是最终 routed timing 的权威来源。
  setup 和 hold 均无 failing endpoint；全局状态由受保护高速时钟树上的两个
  pulse-width endpoint 标记为 `not met`。
- `bev_edif_top_route_status.rpt` 显示 86,175 条可布线网络全部完成路由，
  routing error 为 0。
- `bev_edif_top_drc_routed.rpt` 的 warning 主要包含厂商封装内存异步控制、
  clock buffering/pipelining 和无可布线负载等条目；不能将 warning 数量解释为
  0 DRC 问题。
- 功耗报告没有仿真活动文件，Confidence Level 为 Low，只能用于粗略估算。

## 日志与文件

| 文件 | 内容 |
|---|---|
| `synthesis_runme.log` | 完整综合日志，0 errors，综合成功 |
| `implementation_runme.log` | place/route/bitstream 日志，0 errors，bitstream 成功 |
| `bev_edif_top_utilization_synth.rpt` | 综合后资源报告 |
| `bev_edif_top_utilization_placed.rpt` | 布局后资源报告 |
| `bev_edif_top_timing_summary_routed.rpt` | 最终时序报告 |
| `bev_edif_top_route_status.rpt` | 最终路由状态 |
| `bev_edif_top_drc_routed.rpt` | routed DRC |
| `bev_edif_top_methodology_drc_routed.rpt` | methodology DRC |
| `bev_edif_top_power_routed.rpt` | routed power 估算 |
| `bev_edif_top_clock_utilization_routed.rpt` | 时钟资源报告 |
| `bev_edif_top_control_sets_placed.rpt` | control-set 报告 |

离线数据契约检查可运行：

```text
python golden_layout_check.py
```

该脚本检查量化边界、ZYX LUT 索引、blocked output address、64→256 通道复制
和 512-bit beat 对齐。当前目录未保留 RTL 仿真 PASS 日志，Vivado 实现成功也
不能替代数值仿真、板级缓存一致性和 Decoder 端到端验证。
