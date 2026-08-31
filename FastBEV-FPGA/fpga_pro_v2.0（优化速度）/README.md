# Fast-BEV Part2_sa_v2.0 FPGA 工程

## 项目简介

本工程实现 Fast-BEV Part2 的 INT8 Group4 FPGA 加速，只保留四帧 Group4
处理模式，不包含 legacy 单步 LUT/SA 模式。设计完成 FP32 feature 量化、LUT
映射、前三帧 history 保存与空间对齐，并生成供 Part3 使用的四帧 temporal BEV。

工程面向 Vivado 2018.3 和 AI7030/Zynq-7030 平台。PS 端寄存器协议、DDR
buffer 规划、四帧调用流程及 Part3 输出布局与上一版兼容。

## 四帧处理流程

1. frame1：Quant + LUT，写入 history slot 0。
2. frame2：Quant + LUT，写入 history slot 1。
3. frame3：Quant + LUT，写入 history slot 2。
4. frame4：Quant + LUT，current 直接写入最终 concat。
5. 依次执行 SA0、SA1、SA2，将前三帧对齐结果写入 concat。

最终输出顺序与网络要求一致：

```text
frame4 current, frame3 aligned, frame2 aligned, frame1 aligned
即：[current, prev1, prev3, prev5]
```

输出为 signed INT8，物理布局和尺寸为：

```text
[N][Z][C/32][X][Y][C%32] = [1][4][8][200][200][32]
```

第四帧 current 不进行 SA 近似；三个历史 SA 顺序复用同一个 SA engine。

## 目录结构

```text
part2_sa_v2.0/
├─ README.md
├─ Bitstreams/                     最终 bitstream、BOOT.bin 与哈希清单
├─ Documentation/                  技术手册
├─ Verification_Results/
│  ├─ README.md                    本轮实现结果摘要
│  └─ synthesis&implement_result/  综合、实现、时序、资源、DRC 与功耗报告
├─ part2_sa_2.0.runs/              Vivado synthesis/implementation 原始结果
└─ Vivado_Project/
   ├─ constrs/                     官方及工程 XDC
   ├─ netlist/                     厂商 PS/AI EDIF 网表
   ├─ rtl/                         RTL 与顶层 wrapper
   └─ sim/                         自检 testbench
```

## v2.0 核心优化

- SA 使用 signed Q16.16 DDA 增量坐标。
    SA1.0中对于每个voxcel，u = a*x + b*y + tx；v = c*x + d*y + ty。
    SA2.0中先计算每一行的起始坐标，再用u_next = u_current + a；v_next = v_current + c
    通过减少重复的乘法运算降低组合逻辑的压力。

- 16-lane 可分离双线性插值流水，默认 Q0.6，可编译回退 Q0.8。
    SA1.0：R=P00​(1−fx​)(1−fy​)+P01​fx​(1−fy​)+P10​(1−fx​)fy​+P11​fx​fy​
    SA2.0：top = P00 + (P01 - P00) * fx；bottom = P10 + (P11 - P10) * fx；
         result = top + (bottom - top) * fy；先算x再算y。更好的运用乘加融合运算。fx精度目前是Q0.6，精度1/64
    每个voxel对应64个int8通道，16line每次并行读取16个通道

- 1/2/4 邻点快速路径、4-line cache 和最多 4 个 outstanding read。
    4line全相联cache保留最近使用的4个邻点数据，减少DDR读取，利用的是双线性插值的领域重叠
    设置输出缓冲区，在DDR读取未响应之前最多可以连续发送四次DDR请求

- SA cache lookup 使用可停顿两级流水，隔离 tag compare、512-bit mux 和 pbuf。
  先执行tag compare 和 cache data select；再执行结果写入和DDR请求发送

- 采用与流水线版同样的优化思路：将相邻两个y拼接后写入，充分利用512bit带宽
  由于DDR接口一次64Byte，我们把相邻两个y拼接成一个beat写回，每次先计算y_even后暂存，再计算y_odd后暂存，然后拼接起来写入PLDDR
  先处理偶数y的4个z暂存，再处理奇数y的4个z暂存，然后拼接偶数 `bev_y` 的 SA 输出合并为完整 512-bit 写；odd-Y 保留 RMW fallback。

- Quant engine在读取fp32数据和写入int8的时候，使用 4 outstanding 和两个 ping-pong pack context。
  可以连续发送四次请求，两个 ping-pong pack context同时处理两个voxel体素，读阻塞的时候不影响写入

## 板端延迟记录

| 项目 | 延迟 |
| Part2 LUT+concat | `385.17 ms` |

## Vivado 2018.3 实现结果

目标器件：`xc7z030ffg676-2`。

| 指标 | 结果 |
|---|---:|
| Setup WNS / TNS | `+0.030 ns / 0 ns` |
| Hold WHS / THS | `+0.025 ns / 0 ns` |
| Part2 `clk_pll_i` setup WNS | `+0.084 ns` |
| Pulse width WPWS / TPWS | `-0.409 ns / -0.479 ns`，2 endpoints |
| 全局 timing 状态 | `not met`，仅 pulse-width 检查未通过 |
| Fully routed nets | `102,206 / 102,206` |
| Routing errors | `0` |
| DRC | `0 errors / 113 warnings` |
| Methodology DRC | `48 warnings / 64 advisories` |
| Slice LUT | `48,102 / 78,600 (61.20%)` |
| Slice Register | `61,296 / 157,200 (38.99%)` |
| Block RAM Tile | `140 / 265 (52.83%)` |
| DSP48E1 | `65 / 400 (16.25%)` |
| Total on-chip power | `4.059 W`，Confidence Level = Low |
| Bitstream | `write_bitstream completed successfully` |

Setup、hold 和 routing 均通过；Vivado 全局 timing summary 因受保护 AI clock
tree 中的 2 个 1 GHz pulse-width endpoint 标记为 `not met`。归档报告来自
`part2_sa_2.0.runs/synth_1` 与 `part2_sa_2.0.runs/impl_1`。

## 发布物校验

| 文件 | 字节数 | SHA-256 |
|---|---:|---|
| `ai7030_edif_top.bit` | 5,980,024 | `70443C9C07E168EF36A05A8E3E697D56472F0E37D4EBB7113AA7832D94FBDD74` |
| `BOOT.bin` | 7,023,040 | `3065E5797F7C2C1CDA9B9205AED76A14150289BB63CAED0EBD437B1DE0AF9746` |
