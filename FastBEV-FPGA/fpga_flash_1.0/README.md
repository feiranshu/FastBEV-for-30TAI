# FastBEV Part2 Flash v1.0 FPGA 工程

## 项目简介

本工程实现 FastBEV 当前帧 Part2 的 FPGA 数据通路：从 PLDDR 读取 Part1
输出的六相机 FP32 feature，根据 LUT 完成 2D-to-3D 映射与 INT8 量化，将
64 个源通道复制为 256 个通道，并直接写出 Part3 Decoder 使用的 blocked
INT8 数据。

本版本只包含 LUT/current-frame 路径，不包含时序 history、SA 空间对齐和
四帧 Group4 控制。寄存器版本号为 `0xBE080001`。

## 目录结构

```text
fpga_flash_1.0/
├─ README.md
├─ Vivado_Project/
│  ├─ rtl/                  可编辑 RTL、顶层 wrapper、厂商接口 stub 和 filelist
│  ├─ constrs/              AI7030 板级约束与实现约束
│  └─ netlist/              厂商 PS/AI EDIF 网表
├─ Vivado_Runs/             完整 synth_1/impl_1 原始运行结果
├─ Verification_Results/    关键报告、运行日志与离线 golden check
├─ Bitstreams/              bev_edif_top.bit
├─ Documentation/           技术手册、接口约定、联调和修复说明
├─ driver/                  PS 端参考驱动骨架
└─ Archive/                 不参与当前构建的历史 RTL
```

当前目录不包含 `.xpr` 和 BOOT.bin，但保留完整 `.runs`、提取后的实现报告、
bitstream 和正式 PDF 技术手册。重新创建工程时需使用 Vivado 2018.3，并导入
`Vivado_Project/` 下匹配的 RTL、EDIF 和 XDC。

## 主要模块

| 模块 | 作用 |
|---|---|
| `bev_edif_top.v` | AI7030 板级顶层，连接厂商 PS/AI 封装 |
| `bev_accel_top.v` | 寄存器、LUT engine、DMA mux 和 CDC 集成 |
| `bev_reg_ctrl.v` | Part2 控制、地址、尺寸、状态和版本寄存器 |
| `lut_engine.v` | LUT 查询、FP32 feature 读取、量化缓存、通道复制和写回 |
| `fp32_int8_quant.v` | FP32 到 signed INT8 的固定点近似量化 |
| `dma_arbiter.v` | LUT/SA 接口兼容的单活动引擎 DDR 仲裁器 |
| `pulse_cross.v` | start/done 脉冲跨时钟域同步 |
| `ps_ai_wrap_demo.v/.edf` | 厂商接口 stub 与受保护 EDIF 网表 |

`Vivado_Project/rtl/filelist.f` 给出可编辑 Verilog 的依赖顺序；EDIF 和 XDC
仍需单独加入 Vivado 工程。

## 数据与内存约定

| 对象 | 布局/规模 | 字节数 |
|---|---:|---:|
| LUT | `200 × 200 × 4 × 8B` | 1,280,000 |
| Part1 feature | `[6][64][176][64]` FP32 NHWC | 17,301,504 |
| FPGA Part2 output | `[1][4][8][200][200][32]` signed INT8 | 40,960,000 |

LUT entry 为 `{pad[16], v[16], u[16], cam_id[16]}`，索引顺序为：

```text
lut_idx = (z * bev_y + y) * bev_x + x
```

输出物理布局为：

```text
[N][Z][C/32][X][Y][C%32] = [1][4][8][200][200][32]
```

对逻辑坐标 `(x,y,z,c)`，其中 `c=0..255`：

```text
c_blk   = c / 32
c_inner = c % 32
addr = base + ((((z * 8 + c_blk) * 200 + x) * 200 + y) * 32 + c_inner)
value(x,y,z,c) = quantized_source(x,y,z,c % 64)
```

DDR 接口宽度为 512 bit。RTL 将相邻的偶数/奇数 `y` 各 32 bytes 拼成一个
64-byte 写回 beat，因此本交付固定使用 `bev_z=4` 和偶数 `bev_y=200`，所有
PLDDR 基地址必须按 64 bytes 对齐。

## 量化约定

当前 RTL 使用 `SHIFT_BASE=156`，有效公式为：

```text
q = round(fp32 / 0.06905783)
q = clamp(q, -128, 127)
```

NaN 和极小值输出 0，溢出饱和到 signed INT8 范围。`256` 是最终输出通道数，
不能乘入量化 scale；详细算法和实现约定见
[Flash v1.0 FPGA 技术手册](Documentation/FastBEV_part2_flash_v1.0_技术手册.pdf)。

## 主要寄存器

寄存器按 32-bit word 编址，CPU byte address 为
`0x400C0000 + index * 4`。

| Index | 名称 | 说明 |
|---:|---|---|
| `0x00` | `CTRL_START` | 写 `0x03` 启动 LUT 模式 |
| `0x01` | `LUT_BASE_ADDR` | LUT PLDDR 基地址 |
| `0x02` | `LUT_SIZE` | 默认 160,000 entries |
| `0x03` | `FEAT2D_BASE_ADDR` | Part1 FP32 feature 基地址 |
| `0x04` | `FEAT3D_WR_ADDR` | Decoder INT8 输出基地址 |
| `0x05` | `FEAT3D_WR_SIZE` | 默认 40,960,000 bytes |
| `0x09` | `BEV_PARAMS` | 打包 C/X/Y/Z |
| `0x0A` | `IMG_PARAMS` | 打包 cameras/H/W |
| `0x20` | `COMP_DONE` | 当前任务完成状态 |
| `0x30` | `VERSION` | `0xBE080001` |
| `0x77` | `RESET` | 软件复位 |

完整的软件时序、缓存一致性和非法操作约束见
[`Documentation/Handoff_INT8.md`](Documentation/Handoff_INT8.md)。

## 验证状态

运行离线契约检查：

```text
python Verification_Results/golden_layout_check.py
```

该脚本检查 40,960,000-byte 输出边界、blocked layout 地址、64→256 通道
复制、LUT ZYX 索引及量化边界，当前运行结果为 PASS。

2026-07-20 的 Vivado 2018.3 运行结果如下：

| 指标 | 结果 |
|---|---:|
| Setup WNS / TNS | `+0.030 ns / 0 ns` |
| Hold WHS / THS | `+0.029 ns / 0 ns` |
| Pulse width WPWS / TPWS | `-0.409 ns / -0.479 ns`，2 endpoints |
| Fully routed nets | `86,175 / 86,175`，0 routing errors |
| Slice LUT | `39,934 / 78,600 (50.81%)` |
| Slice Register | `50,646 / 157,200 (32.22%)` |
| Block RAM Tile | `124 / 265 (46.79%)` |
| DSP48E1 | `3 / 400 (0.75%)` |
| Total on-chip power | `3.652 W`，Confidence Level = Low |
| Bitstream | 生成成功 |

Setup、hold 和 routing 通过；全局 timing 因受保护高速时钟树上的 2 个
pulse-width endpoint 标记为 `not met`。完整摘要见
[`Verification_Results/README.md`](Verification_Results/README.md)，原始结果见
`Vivado_Runs/flash_1.0.runs/`。

## 发布物校验

| 文件 | 字节数 | SHA-256 |
|---|---:|---|
| `Bitstreams/bev_edif_top.bit` | 5,980,024 | `D01A659FB2CC4A726AC1744C064D3E1ADC3956C049899D4D489E0529661B30D7` |

详细架构、接口、数据布局、软件时序和实现结果见
[Flash v1.0 FPGA 技术手册](Documentation/FastBEV_part2_flash_v1.0_技术手册.pdf)。

当前包仍未保留 RTL 仿真 PASS 日志，综合和 bitstream 成功不能替代数值仿真、
板级缓存一致性和 Decoder 端到端验证。

`Archive/lut_engine_legacy.v` 是整理前 `temp/` 中的历史版本，仅供差异追溯，
不在当前 filelist 中，也不得替换现行 `Vivado_Project/rtl/lut_engine.v`。
