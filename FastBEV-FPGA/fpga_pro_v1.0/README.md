# Fast-BEV Part2_sa_v1.0 FPGA 工程

## 项目简介

本工程实现 Fast-BEV Part2 的 INT8 Group4 FPGA 加速，位于 Part1 NPU
Extractor 与 Part3 NPU Decoder 之间。设计接收六相机 FP32 2D feature，完成
FP32 到 signed INT8 的量化、LUT 视角变换、四帧时序管理和前三帧历史 BEV
的空间对齐，最终输出 Part3 可直接读取的 blocked INT8 temporal BEV。

工程只保留 INT8 Group4 主路径，不提供 legacy LUT/SA 单步运行模式。目标平台为
XC7Z030FFG676-2，完整综合与实现使用 Vivado 2018.3 及厂商 patched flow。

## 四帧处理流程

一组输入必须按时间顺序连续提交四帧：

1. frame1：Quant + LUT，写入 history slot 0。
2. frame2：Quant + LUT，写入 history slot 1。
3. frame3：Quant + LUT，写入 history slot 2。
4. frame4：Quant + LUT，将 current 写入最终 concat 的通道 0-63。
5. 依次执行 SA0、SA1、SA2，将三帧 history 对齐到 frame4 坐标系，并写入
   concat 的通道。

最终输出顺序与网络要求一致：

```text
frame4 current, frame3 aligned, frame2 aligned, frame1 aligned
即：[current, prev1, prev3, prev5]
```


## 数据格式与存储规模

| 对象 | 逻辑尺寸 | 字节数 | 用途 |
|---|---:|---:|---|
| FP32 feature | `6 × 64 × 176 × 64 × 4B` | 17,301,504 | Part1 → Quant |
| INT8 temporary | `6 × 64 × 176 × 64` | 4,325,376 | Quant → LUT |
| LUT | `200 × 200 × 4 × 8B` | 1,280,000 | 2D → BEV 映射 |
| 单个 history slot | `200 × 200 × 4 × 64` | 10,240,000 | 共三个历史帧 |
| Concat output | `1 × 4 × 8 × 200 × 200 × 32` | 40,960,000 | Part3 输入 |

LUT entry 固定为 8 bytes：

```text
{pad[16], v[16], u[16], cam_id[16]}
```

invalid voxel 输出全零。所有主路径 PLDDR 基地址必须按 64 bytes 对齐，LUT、
feature、INT8 temporary、三个 history slot 和 concat output 不得非法重叠。

Part3 输出为 signed INT8，物理布局为：

```text
[N][Z][C/32][X][Y][C%32] = [1][4][8][200][200][32]
```

地址公式为：

```text
addr = concat_base + (((z * 8 + c/32) * X + x) * Y + y) * 32 + c%32
```

## 目录结构

```text
part2_sa_v1.0/
├─ README.md
├─ Bitstreams/
│  ├─ bev_edif_top.bit             最终 FPGA bitstream
│  └─ BOOT.bin                     启动镜像
├─ Documentation/
│  └─ FastBEV_part2_sa1.0技术手册.pdf
├─ part2_sa_1.0.runs/              Vivado 综合与实现原始运行目录
├─ Verification_Results/
│  ├─ README.md                    本轮实现结果摘要
│  ├─ Phase_A_Quant/               Quant 单元验证
│  ├─ Phase_B_LUT/                 LUT history/current 验证
│  ├─ Phase_C_SA/                  SA 仿射与插值验证
│  ├─ Phase_D_Group4_Control/      寄存器、控制器和仲裁验证
│  ├─ Phase_E_Full_Pipeline/       完整四帧 Phase F 行为验证日志
│  └─ synthesis&implement_result/  综合、实现、时序、资源和 DRC 报告
└─ Vivado_Project/
   ├─ constraints/                 官方及工程 XDC
   ├─ netlist/                     厂商 PS/AI EDIF 网表
   ├─ rtl/                         Part2 RTL 与顶层 wrapper
   └─ sim/                         各阶段 testbench
```


## RTL 模块说明

| 模块 | 作用 |
|---|---|
| `fp32_int8_quant.v` | 单 lane FP32 → signed INT8 量化 |
| `quant_engine.v` | 16 lane 并行量化及四个 FP32 beat 到一个 INT8 beat 的打包 |
| `lut_engine_int8.v` | LUT 采样、history 连续写入和第四帧 current 融合写入 |
| `sa_engine_int8.v` | Q16.16 affine、Q0.8 双线性插值、边界清零和饱和 |
| `pipeline_ctrl_group4.v` | 四帧 phase、history mask、stage、错误和 output-ready 管理 |
| `dma_arbiter_stage.v` | Quant/LUT/SA stage-exclusive DDR 所有权仲裁 |
| `bev_reg_ctrl.v` | PS 寄存器配置、状态、错误和性能计数器 |
| `bev_accel_top.v` | Part2 顶层连接、CDC、控制器、引擎和 DMA 仲裁 |
| `bev_edif_top.v` | 板级顶层，连接厂商 `ps_ai_wrap_demo` 网表 |

### Quant

默认量化公式为：

```text
q = round(fp32 / 0.06905783)
q = clamp(q, -128, 127)
```

RTL 参数 `SHIFT_BASE=156`，不包含额外的 256 因子。Quant engine 每次处理一个
512-bit FP32 beat（16 个 FP32 lane），连续四个输入 beat 打包成一个 512-bit
INT8 beat，并支持读写 backpressure。

### LUT

LUT engine 提供两种 Group4 内部目标模式：

- `HISTORY_CONTIG`：前三帧写入各自 history slot，地址为
  `hist_base + voxel_idx * 64`。
- `FUSED_CURRENT`：第四帧写入最终 blocked concat 的 current 通道，同时保留
  同一 512-bit beat 内的历史通道数据。

### SA

SA engine 为三个 history slot 分别使用一组 2×3 Q16.16 affine：

```text
u = a00 * x + a01 * y + a02
v = a10 * x + a11 * y + a12
```

四邻域采样使用 Q0.8、权重和为 256 的 signed INT8 双线性插值。越界采样输出
零，插值结果 round 后饱和到 `[-128, 127]`。三个 history 顺序复用同一个
SA engine。

## 主要寄存器

寄存器按 32-bit word 编址：

```text
byte_address = 0x400C0000 + index * 4
```

| Index | 名称 | 属性 | 说明 |
|---:|---|---|---|
| `0x00` | `CTRL_START` | W | 写 bit0=1 启动当前帧 Group4 pipeline |
| `0x01` | `LUT_BASE` | RW | LUT PLDDR 基地址 |
| `0x02` | `LUT_SIZE` | RW | 默认 160,000 voxel |
| `0x09` | `BEV_PARAMS` | RW | 打包 C/X/Y/Z |
| `0x0A` | `IMG_PARAMS` | RW | 打包 cameras/H/W |
| `0x11..0x13` | `FRAME0..2_ADDR` | RW | 三个 history slot |
| `0x15` | `FRAME_SIZE` | RW | 默认 10,240,000 bytes |
| `0x20` | `COMP_DONE` | R | 当前帧完成锁存 |
| `0x24` | `EXT_MODE` | RW | Group4 软件配置为 1 |
| `0x25` | `FEAT2D_FP32` | RW | 当前帧 FP32 feature 地址 |
| `0x26` | `FEAT2D_INT8` | RW | INT8 temporary 地址 |
| `0x27` | `CONCAT_OUT` | RW | 最终输出基地址 |
| `0x28` | `GROUP_STATUS` | R | error/busy/ready/phase/history/stage/state |
| `0x29` | `GROUP_CTRL` | RW | 清 history metadata 或强制 phase reset |
| `0x2A` | `ERROR_STATUS` | R/W1C | alignment/phase/history/overlap/mode 错误 |
| `0x2B` | `PERF_STAGE_SEL` | RW | 性能计数器选择 |
| `0x2C..0x2D` | `PERF_STAGE_CNT` | R | 64-bit 性能计数值 |
| `0x30` | `VERSION` | R | `0x20260707` |
| `0x40..0x51` | `XFORM_H0/H1/H2` | RW | 三套 2×3 Q16.16 affine |
| `0x77` | `RESET` | W | 软件复位 |

性能计数器可选择总周期、Quant、LUT、SA、DDR read/write wait 和 DDR
read/write grant。每帧启动前必须完成当前 feature 写入和 cache flush；观察到
`COMP_DONE` 后再读取状态或复用输入缓冲。第四帧完成后，应 invalidate
40,960,000-byte concat buffer，再交给 Part3。

## 验证结果

当前目录保留的验证日志表明：

- Phase A：Quant lane order、四 beat 打包、backpressure 和 flush 通过。
- Phase B：LUT history、fused current、invalid zero 和 Part3 layout 通过。
- Phase C：SA identity、整数平移、fractional bilinear、signed affine 和越界清零通过。
- Phase D：Group4 phase/stage/arbiter 连续 1000 组压力测试通过。
- 完整流水线：四帧 numerical、layout、backpressure 和性能计数检查通过。


## 板端延迟记录

v1.0 版本一次整网运行的 Timing Summary 中，与 Part2 直接相关的记录如下：

| 项目 | 延迟 |
|---|---:|
| Part2 LUT+concat | `1067.32 ms` |
| Input data2Tensor | `34.54 ms` |

其中 `Part2 LUT+concat` 为 Part2 主处理阶段的系统侧计时，`Input data2Tensor`
为单独列出的输入数据转换时间。

## Vivado 2018.3 实现结果

目标器件：`xc7z030ffg676-2`。

| 指标 | 结果 |
|---|---:|
| Setup WNS / TNS | `+0.030 ns / 0 ns` |
| Hold WHS / THS | `+0.027 ns / 0 ns` |
| Part2 `clk_pll_i` setup WNS | `+0.224 ns` |
| Pulse width WPWS / TPWS | `-0.409 ns / -0.479 ns`，2 endpoints |
| 全局 timing 状态 | `not met`，仅 pulse-width 检查未通过 |
| Fully routed nets | `97,346 / 97,346` |
| Routing errors | `0` |
| DRC | `0 errors / 130 warnings` |
| Methodology DRC | `40 warnings / 64 advisories` |
| Slice LUT | `42,918 / 78,600 (54.60%)` |
| Slice Register | `58,139 / 157,200 (36.98%)` |
| Block RAM Tile | `124 / 265 (46.79%)` |
| DSP48E1 | `97 / 400 (24.25%)` |
| Total on-chip power | `3.963 W`，Confidence Level = Low |
| Bitstream | `write_bitstream completed successfully` |

Setup、hold 和 routing 均通过；Vivado 全局 timing summary 因受保护 AI clock
tree 中的 2 个 1 GHz pulse-width endpoint 标记为 `not met`。归档报告来自
`part2_sa_1.0.runs/synth_1` 与 `part2_sa_1.0.runs/impl_1`。

## 发布物校验

| 文件 | 字节数 | SHA-256 |
|---|---:|---|
| `bev_edif_top.bit` | 5,980,022 | `A298C0A634282492D987EDA926E2C4D7C056F0361AA05C3B9B9FB8BDF9C49026` |
| `BOOT.bin` | 7,023,040 | `080F9E93ABE2F69CE9A9460B3D74DC30A875776F46E7A167BB61271D246857B6` |


详细接口、算法和实现结果见：

```text
Documentation/FastBEV_part2_sa1.0技术手册.pdf
```
