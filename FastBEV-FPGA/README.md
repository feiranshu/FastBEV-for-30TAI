# Fast-BEV FPGA 工程总览

本仓库整理了四份面向 AI7030/Zynq-7030 的 Fast-BEV FPGA 工程输入与交付资料。目标器件均为 `xc7z030ffg676-2`，原始实现环境为 Vivado 2018.3，板级顶层为 `bev_edif_top`。

## 1. 四份工程的关系

四份工程分为两条数据通路：

- `fpga_flash_1.0` 与 `fpga_flash_2.0` 处理当前帧 Part2，包括 FP32 量化、LUT 2D-to-3D 映射、64→256 通道复制和 Decoder INT8 写回；v2.0 进一步加入 cache、请求流水、性能计数和三级 LED 预警。
- `fpga_pro_v1.0` 与 `fpga_pro_v2.0` 处理四帧时序融合 Group4，包含 history 保存、SA 仿射对齐和 temporal BEV 输出；v2.0 是 v1.0 的性能优化版。
- Flash 与 Pro 系列的寄存器、缓存规划和处理流程不同；厂商 EDIF、板级 XDC、顶层 RTL 与 bitstream 必须按工程成套使用，不能跨目录任意替换。

| 工程 | 数据链 | 主要功能 | 建议用途 |
|---|---|---|---|
| [`fpga_flash_1.0`](fpga_flash_1.0/README.md) | 当前帧 LUT | 基础 INT8 Part2、原生 blocked 输出、参考驱动 | RTL、EDIF、XDC、完整 runs、实现报告、bitstream、手册 |
| [`fpga_flash_2.0`](fpga_flash_2.0/README.md) | 当前帧 LUT | LUT cache、连续写回、性能计数、J1/M6 三级 LED | FastBEV3.0 预警功能与当前帧 Part2 优化 |
| [`fpga_pro_v1.0`](fpga_pro_v1.0/README.md) | 四帧 Group4 | Quant、LUT、三帧 history、SA 仿射对齐、四帧 concat | Pro 基线、算法对照和完整验证追溯 |
| [`fpga_pro_v2.0`](fpga_pro_v2.0/README.md) | 四帧 Group4 | 兼容 v1.0 流程，优化 SA、Quant、cache、outstanding 和写回 | Pro 性能优化版本 |

## 2. 目录结构

```text
FastBEV-FPGA/
├─ README.md
├─ fpga_flash_1.0/
│  ├─ Vivado_Project/         RTL、约束和厂商 EDIF
│  ├─ Vivado_Runs/            完整 synth_1/impl_1 原始运行结果
│  ├─ Verification_Results/   实现报告、日志与 Python golden check
│  ├─ Bitstreams/             bev_edif_top.bit
│  ├─ Documentation/          技术手册、交接、联调与修复说明
│  ├─ driver/                 PS 端参考驱动骨架
│  └─ Archive/                不参与当前构建的历史 RTL
├─ fpga_flash_2.0/
│  ├─ rtl/                    顶层、Part2、LED RTL 和厂商 EDIF
│  ├─ sim/                    自检 testbench
│  ├─ constrs/                板级与实现约束
│  ├─ latency_opt_reports/    综合、实现、路由、时序和 DRC 报告
│  ├─ bitstream/              bev_edif_top.bit 与 BOOT.bin
│  └─ Documentation/          FPGA 技术手册
├─ fpga_pro_v1.0/
│  ├─ Vivado_Project/         RTL、仿真、约束和厂商 EDIF
│  ├─ Verification_Results/   分阶段日志及完整实现报告
│  ├─ Bitstreams/             bev_edif_top.bit 与 BOOT.bin
│  └─ Documentation/          Pro v1.0 技术手册
└─ fpga_pro_v2.0/
   ├─ Vivado_Project/         优化后 RTL、仿真、约束和厂商 EDIF
   ├─ Verification_Results/   验证摘要及完整实现报告
   ├─ Bitstreams/             ai7030_edif_top.bit 与 BOOT.bin
   └─ Documentation/          Pro v2.0 技术手册
```

当前四个目录均未包含 `.xpr` 文件。`Vivado_Project/` 表示已整理好的工程输入，而不是可直接打开的完整 Vivado Project；重新实现时需在 Vivado 2018.3 中创建工程并导入对应版本的 RTL、EDIF、XDC 和 testbench。

## 3. 公共硬件与接口约定

| 项目 | 约定 |
|---|---|
| FPGA | Xilinx Zynq-7000 `xc7z030ffg676-2` |
| 开发平台 | AI7030/Zynq-7030 |
| 工具版本 | Vivado 2018.3，完整实现需使用与原工程一致的厂商 patched flow |
| 板级顶层 | `bev_edif_top` |
| 厂商封装 | `ps_ai_wrap_demo.v` 与 `ps_ai_wrap_demo.edf` |
| 寄存器基址 | `0x400C0000`，32-bit word 编址 |
| DDR 数据宽度 | 主数据通路以 512-bit/64-byte beat 组织 |
| BEV 几何 | 默认 `X=200`、`Y=200`、`Z=4`、单帧 64 通道 |
| Decoder 输入 | signed INT8，`[N][Z][C/32][X][Y][C%32] = [1][4][8][200][200][32]`，40,960,000 字节 |

厂商 EDIF、板级 XDC 和对应 RTL 必须成套使用。不要在不同工程间单独替换 `bev_reg_ctrl.v`、`bev_accel_top.v`、`bev_edif_top.v` 或 `bev_edif_top.bit`。

## 4. Flash 系列

### 4.1 Flash v1.0

Flash v1.0 是当前帧 INT8 Part2 基线：读取六相机 FP32 feature 和 ZYX 顺序
LUT，在 FPGA 内量化并将 64 个源通道复制为 256 个通道。输出采用 Part3 原生
blocked layout：

```text
addr = base + ((((z * 8 + c/32) * 200 + x) * 200 + y) * 32 + c%32)
```

量化有效公式为 `round(fp32 / 0.06905783)` 后饱和到 signed INT8，版本寄存器
为 `0xBE080001`。目录保留完整原始 runs、关键实现报告和 bitstream。最终
setup WNS 为 `+0.030 ns`、hold WHS 为 `+0.029 ns`，86,175 条可布线网络全部
完成路由；全局 timing 因 2 个 pulse-width endpoint 仍为 `not met`。详细说明见
[`fpga_flash_1.0/README.md`](fpga_flash_1.0/README.md) 和
[Flash v1.0 FPGA 技术手册](fpga_flash_1.0/Documentation/FastBEV_part2_flash_v1.0_技术手册.pdf)。

### 4.2 Flash v2.0 功能

- `fp32_int8_quant.v`：FP32 到 signed INT8 四级流水量化。
- `lut_engine.v`：1K x 512-bit LUT line cache、Feature 请求流水、连续写回和 stall/DDR 计数。
- `alert_led_ctrl.v`：J1/M6 注意、危险、紧急三级灯效。
- `bev_reg_ctrl.v`：Part2 参数、告警命令、状态、版本、能力和性能计数器。
- 生产顶层固定 `FEAT_MAX_OUTSTANDING=1`；4 outstanding 只作为离线候选配置验证。

### 4.3 Flash v2.0 版本标识

```text
BEV_VERSION = 0xBE080003
ALERT_CAPS  = 0x414C0001
PART2_CAPS  = 0x50320001
```

### 4.4 Flash v2.0 验证状态

RTL 回归覆盖 LED、寄存器、量化、LUT cache 和随机背压。当前目录保留完整实现报告，88,774 条可布线网络全部完成路由，setup WNS 为 `+0.030 ns`，hold WHS 为 `+0.029 ns`。真实灯效、音频协同、Decoder 输入一致性和板端毫秒延迟仍需结合完整 FastBEV3.0 系统验证。

详细说明见 [`fpga_flash_2.0/README.md`](fpga_flash_2.0/README.md) 和
[Flash v2.0 FPGA 技术手册](fpga_flash_2.0/Documentation/FastBEV3.0_预警优化版_FPGA技术手册.pdf)。

## 5. fpga_pro_v1.0

### 5.1 四帧 Group4 流程

1. frame1 至 frame3 分别执行 Quant + LUT，并写入三个 history slot。
2. frame4 执行 Quant + LUT，将 current 写入最终 concat 的通道 0-63。
3. SA0、SA1、SA2 顺序复用同一个 SA engine，将前三帧对齐到 frame4 坐标系。
4. 最终输出顺序为 `frame4 current, frame3 aligned, frame2 aligned, frame1 aligned`，
   即 `[current, prev1, prev3, prev5]`。

输出物理布局为：

```text
[N][Z][C/32][X][Y][C%32] = [1][4][8][200][200][32]
```

### 5.2 主要实现

- 16-lane Quant 和四个 FP32 beat 到一个 INT8 beat 的打包。
- LUT history 连续写入与第四帧 fused current 写入。
- Q16.16 affine、Q0.8 signed INT8 双线性插值、越界清零和饱和。
- Group4 phase/history/stage 控制与 stage-exclusive DMA 仲裁。
- 分阶段 Quant、LUT、SA、控制及完整四帧验证日志。

版本寄存器为 `0x20260707`。板端记录中 `Part2 LUT+concat` 为 `1067.32 ms`，另有 `Input data2Tensor` 为 `34.54 ms`。

详细说明见 [`fpga_pro_v1.0/README.md`](fpga_pro_v1.0/README.md) 和
[FastBEV Part2 Pro v1.0 技术手册](fpga_pro_v1.0/Documentation/FastBEV_part2_sa1.0技术手册.pdf)。

## 6. fpga_pro_v2.0

SA v2.0 保持 v1.0 的 PS 寄存器协议、DDR 缓冲规划、四帧调用顺序和 Part3 输出布局，重点优化内部吞吐与 DDR 访问。

### 6.1 相比 v1.0 的优化

- SA 坐标生成改为 signed Q16.16 DDA，每行只计算一次起始坐标，后续按增量推进。
- 双线性插值改为 16-lane 可分离流水，默认 Q0.6，可编译回退 Q0.8。
- 增加 1/2/4 邻点快速路径、4-line 全相联 cache 和最多 4 个 outstanding read。
- SA cache lookup 使用可停顿两级流水，隔离 tag compare、512-bit mux 和 pbuf。
- 偶数 `bev_y` 将相邻两个 y 合并为完整 512-bit 写；odd-Y 保留 RMW fallback。
- Quant 使用最多 4 个 outstanding read 和两个 ping-pong pack context。

板端记录中 `Part2 LUT+concat` 为 `385.17 ms`。按与 v1.0 同名计时项计算，耗时下降约 63.9%，约为 v1.0 的 2.77 倍速度。

### 6.2 当前验证边界

- `Vivado_Project/sim/` 保留 SA、odd-Y、Quant、ping-pong、LUT 和 DMA 的优化版 testbench 源码。
- 当前目录未保留这些 testbench 的 PASS 日志；上板前应优先重跑
  `tb_dma_arbiter_stage_ready.v`、`tb_quant_dma_queued.v` 和
  `tb_sa_engine_int8_optimized.v` 等回归。
- `Verification_Results/` 保留 2026-07-19 的综合、实现、时序、资源、DRC、
  功耗报告和 run log，但未保留完整 `.runs/` 原始目录。

详细说明见 [`fpga_pro_v2.0/README.md`](fpga_pro_v2.0/README.md)、
[FastBEV Part2 Pro v2.0 技术手册](fpga_pro_v2.0/Documentation/FastBEV_part2_sa_v2.0_技术手册.pdf)
和 [Pro v2.0 验证结果](fpga_pro_v2.0/Verification_Results/README.md)。

## 7. Vivado 2018.3 实现结果对比

| 指标 | Flash v1.0 | Flash v2.0 | Pro v1.0 | Pro v2.0 |
|---|---:|---:|---:|---:|
| Setup WNS / TNS | `+0.030 ns / 0` | `+0.030 ns / 0` | `+0.030 ns / 0` | `+0.030 ns / 0` |
| Hold WHS / THS | `+0.029 ns / 0` | `+0.029 ns / 0` | `+0.027 ns / 0` | `+0.025 ns / 0` |
| Routing errors | `0` | `0` | `0` | `0` |
| Slice LUT | `39,934 / 50.81%` | `41,356 / 52.62%` | `42,918 / 54.60%` | `48,102 / 61.20%` |
| Slice Register | `50,646 / 32.22%` | `51,385 / 32.69%` | `58,139 / 36.98%` | `61,296 / 38.99%` |
| Block RAM Tile | `124 / 46.79%` | `138 / 52.08%` | `124 / 46.79%` | `140 / 52.83%` |
| DSP48E1 | `3 / 0.75%` | `3 / 0.75%` | `97 / 24.25%` | `65 / 16.25%` |

四个工程的 setup、hold 和 routing 均通过。报告中的受保护 AI clock
tree pulse-width 检查仍需结合厂商 patched flow 解读，不能只依据正 WNS/WHS
忽略全局 timing 状态。

## 8. Bitstream 与启动镜像

四个工程的镜像内容不同，烧录前必须核对目标目录、文件名和 SHA-256。

| 工程 | 文件 | 字节数 | SHA-256 |
|---|---|---:|---|
| Flash v1.0 | `Bitstreams/bev_edif_top.bit` | 5,980,024 | `D01A659FB2CC4A726AC1744C064D3E1ADC3956C049899D4D489E0529661B30D7` |
| Flash v2.0 | `bitstream/bev_edif_top.bit` | 5,980,024 | `4DBD22BD0CF3FB176BB818240DBEB6DBD2B4151C2BA49FDEA88B246C8905897D` |
| Flash v2.0 | `bitstream/BOOT.bin` | 7,023,040 | `7814D707656FF2DA9C258CE8968176A121958C3DFD0E022258B7D6C98FB3BB93` |
| Pro v1.0 | `Bitstreams/bev_edif_top.bit` | 5,980,022 | `A298C0A634282492D987EDA926E2C4D7C056F0361AA05C3B9B9FB8BDF9C49026` |
| Pro v1.0 | `Bitstreams/BOOT.bin` | 7,023,040 | `080F9E93ABE2F69CE9A9460B3D74DC30A875776F46E7A167BB61271D246857B6` |
| Pro v2.0 | `Bitstreams/ai7030_edif_top.bit` | 5,980,024 | `70443C9C07E168EF36A05A8E3E697D56472F0E37D4EBB7113AA7832D94FBDD74` |
| Pro v2.0 | `Bitstreams/BOOT.bin` | 7,023,040 | `3065E5797F7C2C1CDA9B9205AED76A14150289BB63CAED0EBD437B1DE0AF9746` |

## 9. 文档入口

- [Flash v1.0 README](fpga_flash_1.0/README.md)
- [Flash v1.0 FPGA 技术手册](fpga_flash_1.0/Documentation/FastBEV_part2_flash_v1.0_技术手册.pdf)
- [Flash v1.0 验证与实现结果](fpga_flash_1.0/Verification_Results/README.md)
- [Flash v1.0 CPU/RTL 交接约定](fpga_flash_1.0/Documentation/Handoff_INT8.md)
- [Flash v2.0 README](fpga_flash_2.0/README.md)
- [Flash v2.0 FPGA 技术手册](fpga_flash_2.0/Documentation/FastBEV3.0_预警优化版_FPGA技术手册.pdf)
- [Pro v1.0 README](fpga_pro_v1.0/README.md)
- [Pro v1.0 技术手册](fpga_pro_v1.0/Documentation/FastBEV_part2_sa1.0技术手册.pdf)
- [Pro v2.0 README](fpga_pro_v2.0/README.md)
- [Pro v2.0 技术手册](fpga_pro_v2.0/Documentation/FastBEV_part2_sa_v2.0_技术手册.pdf)
- [Pro v2.0 验证结果](fpga_pro_v2.0/Verification_Results/README.md)
