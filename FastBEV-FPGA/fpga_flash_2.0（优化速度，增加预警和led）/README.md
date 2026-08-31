# FastBEV 3.0 FPGA 工程

## 目录

```text
fpga/
├─ rtl/            顶层、寄存器、Part2 与 LED 控制 RTL
├─ constrs/        管脚与实现约束
├─ bitstream/      当前 BOOT.bin 和 bev_edif_top.bit
├─ Documentation/ FPGA 技术手册
└─ README.md
```

## 主要模块

| 模块 | 作用 |
|---|---|
| `bev_edif_top.v` | 工程顶层；将告警 LED 接到 J1/M6 |
| `bev_accel_top.v` | Part2 数据通路与寄存器/LED 控制集成 |
| `bev_reg_ctrl.v` | 控制、状态、版本、能力和性能计数寄存器 |
| `alert_led_ctrl.v` | 三级 LED 灯效，100 MHz 时钟域 |
| `lut_engine.v` | LUT line cache、Feature 请求流水与连续写回 |
| `fp32_int8_quant.v` | FP32 到 INT8 的四级流水量化 |
| `dma_arbiter.v` | DDR 访问仲裁 |
| `ps_ai_wrap_demo.v/.edf` | 受保护的厂商 PS/AI 封装 |

## 内存重排序问题

addr = B + ((((z × 8 + c_blk) × 200 + x) × 200 + y) × 32) + c_inner
c = r × 64 + s ，r = 1 ，2 ，3 ，4
同样一个voxel的结果会先存在   qbuf0 → 源通道  0～15  c_inner 0-15
                            qbuf1 → 源通道 16～31  c_inner 16-31
                            qbuf2 → 源通道 32～47  c_inner 0-15
                            qbuf3 → 源通道 48～63  c_inner 16-31
c_blk依次从0取到7，c_blk为偶数 → qbuf0、qbuf1，c_blk为奇数 → qbuf2、qbuf3；
由于DDR接口一次64Byte，我们把相邻两个y拼接成一个beat写回，每次先计算y_even后暂存，再计算y_odd后暂存，然后拼接起来写入PLDDR
先处理偶数y的4个z暂存，再处理奇数y的4个z暂存，然后拼接

## 优化部分

1.0、增加约 14 个 RAMB36 和 1 个 RAMB18 ，建立片上cache，利用DDR512bit的带宽，每次读取8个lut 
     entry；原工程虽然读取了整个 512-bit beat，但每处理一个体素都会重新请求一次。利用z，y作为缓存索引。
2.0、建立缓冲队列，对于每个体素，quant_engine可以在上个队列未返回的时候处理下一个。

## Vivado 实现结果

| 指标 | 基础pipeline版本 | 当前版本(优化后) | 变化 |
|---|---:|---:|---:|
| Slice LUT | 38,919 / 49.52% | 41,356 / 52.62% | +2,437 / +3.10 pct |
| Slice Register | 50,908 / 32.38% | 51,385 / 32.69% | +477 / +0.31 pct |
| BRAM Tile | 124 / 46.79% | 138 / 52.08% | +14 / +5.29 pct |
| DSP | 3 / 0.75% | 3 / 0.75% | 0 |
| setup WNS/TNS | +0.013 ns / 0 | +0.030 ns / 0 | 无 setup 违例 |
| hold WHS/THS | +0.015 ns / 0 | +0.029 ns / 0 | 无 hold 违例 |
| WPWS/TPWS | -0.409/-0.479 ns | -0.409/-0.479 ns | 2 个既有端点不变 |
| DRC warning | 64 | 64 | 未增加 |


