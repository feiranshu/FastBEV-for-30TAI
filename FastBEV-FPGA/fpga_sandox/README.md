# FastBEV Part2 FPGA Sandbox 工程

本目录是参照 `fpga_pro_v1.0` 发布结构整理的 FastBEV 新版网络 Part2 FPGA
工程包。FPGA 位于 Part1 与
Part3 之间：读取 Part1 的六相机 FP32 特征和 LUT，完成 BEV gather、数据类型
转换及可选的空间对齐（SA），将结果直接写入 Part3 `Conv2dNode 256` 所消费的
value 362 缓冲区。

## 1. 当前接口契约

| 项目 | 规格 |
|---|---|
| Part1 输出 | FP32，NHWC，`[6,120,160,64]` |
| Part1 数据量 | 29,491,200 B |
| LUT | 160,000 项；小端 `int16[4] = [cam_id,u,v,pad]` |
| LUT 遍历顺序 | ZYX，`x` 最快 |
| FPGA 输出 | FP16，NCHWc16，`[1,16,200,200,16]` |
| FPGA 输出量 | 20,480,000 B |
| Part3 对接点 | value 362，即 `HardOp 254 ReshapeNode` 的输出和 `HardOp 256 Conv2dNode` 的输入 |
| DDR 数据宽度 | 512 bit / 64 B 每拍 |
| 目标器件 | Xilinx `xc7z030ffg676-2` |
| 工具版本 | Vivado 2018.3 |
| HP 目标时钟 | 200 MHz（5 ns） |
| RTL 版本寄存器 | `0x20260818` |

输出不是送入 Part3 原始入口重新执行前置变换，而是直接写入 value 362 对应
缓冲区。软件侧必须从 Conv256 开始继续执行，或明确跳过 Part3 入口到
Reshape254 的预处理链，避免 value 362 被覆盖。

## 2. 目录结构

```text
fpga_sandox/
├─ README.md
├─ Bitstreams/                bev_edif_top.bit 与 BOOT.bin
├─ Documentation/             工程文档（DOCX/PDF）
├─ Verification_Results/      RTL 回归摘要与 Vivado 实现报告
├─ Vivado_Project/
│  ├─ constraints/            引脚与实现约束
│  ├─ netlist/                受保护厂商 EDIF
│  ├─ rtl/                    当前有效 RTL
│  └─ sim/                    Icarus/Vivado 回归脚本及 testbench
└─ Archive/legacy_rtl/        不参与当前构建的旧版参考 RTL
```

本发布目录不包含 `.Xil/`、临时日志、DCP、仿真编译产物或第三方工具副本。
`Vivado_Project/netlist/ps_ai_wrap_demo.edf` 是受保护网表，不得编辑或重新生成替换。

## 3. RTL 组成

| 文件 | 当前用途 |
|---|---|
| `Vivado_Project/rtl/bev_edif_top.v` | 工程顶层，连接 PS/DDR 包装模块和 Part2 加速器 |
| `Vivado_Project/rtl/bev_accel_top.v` | Part2 控制顶层，完成寄存器、CDC、性能计数和引擎调度 |
| `Vivado_Project/rtl/bev_reg_ctrl.v` | GP 时钟域寄存器接口 |
| `Vivado_Project/rtl/lut_engine_fp32.v` | 当前 LUT gather 与 FP32→TF32→FP16 实现 |
| `Vivado_Project/rtl/sa_engine_fp32.v` | 当前 FP32 空间对齐和双线性插值实现 |
| `Vivado_Project/rtl/dma_arbiter.v` | LUT/SA 两引擎共享 512-bit PLDDR 接口的仲裁器 |
| `Vivado_Project/rtl/pulse_cross.v` | 启动/完成脉冲跨时钟域握手 |
| `Vivado_Project/rtl/ps_ai_wrap_demo.v` | 受保护 EDIF 的 Verilog 接口声明 |
| `Vivado_Project/netlist/ps_ai_wrap_demo.edf` | 平台受保护网表，只读 |
| `Archive/legacy_rtl/*.v` | 旧版参考模块，不参与当前实现 |

当前有效模块虽然文件名带 `_fp32`，其模块名仍为 `lut_engine` 和
`sa_engine`。不要同时把旧版模块改回同名，否则会产生重复定义。

## 4. 数据布局与地址计算

Part1 输入像素的 64 个 FP32 通道连续存放，每个像素 256 B，需要四次 512-bit
DDR 读取：

```text
feature_pixel = (cam_id * IMG_H + v) * IMG_W + u
feature_addr  = FEAT2D_BASE_ADDR + feature_pixel * 64 * 4
```

LUT 索引与 BEV 坐标关系：

```text
lut_index = (z * BEV_Y + y) * BEV_X + x
```

输出将 Z 维折叠到 Part3 的 16 个 cblk16 中：

```text
global_channel = z * 64 + c
global_cblk16  = z * 4 + floor(c / 16)
c_inner        = c mod 16
```

输出字节地址：

```text
FEAT3D_WR_ADDR
  + ((((z*4 + source_cblk16)*BEV_X + x)*BEV_Y + y)*16 + c_inner)*2
```

一个 512-bit 写拍打包同一 `(x,cblk16)` 下相邻的两行：低 256 bit 对应
`y_even`，高 256 bit 对应 `y_odd`。浮点转换严格执行 FP32→TF32 RNE→FP16
RNE，而不是直接截断或直接 FP32→FP16。

## 5. 寄存器接口

寄存器控制器使用 `itf_ra_awaddr[17:2]`，下表“索引”是 RTL 内部 word address，
“字节偏移”是软件通常使用的 `索引×4`。

| 索引 | 字节偏移 | 名称 | 属性 | 复位值/说明 |
|---:|---:|---|:---:|---|
| `0x00` | `0x000` | `CTRL_START` | R/W | `[0]` start；`[2:1]` mode；`[3]` frame_shift |
| `0x01` | `0x004` | `LUT_BASE_ADDR` | R/W | LUT DDR 基地址，默认 0 |
| `0x02` | `0x008` | `LUT_SIZE` | R/W | 默认 160000 |
| `0x03` | `0x00C` | `FEAT2D_BASE_ADDR` | R/W | Part1 输出 DDR 基地址 |
| `0x04` | `0x010` | `FEAT3D_WR_ADDR` | R/W | value 362 输出缓冲区基地址 |
| `0x05` | `0x014` | `FEAT3D_WR_SIZE` | R/W | 默认 20,480,000；当前用于软件读回，不直接控制 LUT 状态机 |
| `0x06` | `0x018` | `SA_SRC_ADDR` | R/W | SA 输入基地址 |
| `0x07` | `0x01C` | `SA_DST_ADDR` | R/W | SA 输出基地址 |
| `0x08` | `0x020` | `SA_SIZE` | R/W | 默认 40000（200×200） |
| `0x09` | `0x024` | `BEV_PARAMS` | R/W | `{bev_z,bev_y,bev_x,channels}`，默认 `{4,200,200,64}` |
| `0x0A` | `0x028` | `IMG_PARAMS` | R/W | `{cameras[7:0],img_h[11:0],img_w[11:0]}`，默认 `{6,120,160}` |
| `0x0B`–`0x10` | `0x02C`–`0x040` | `XFORM_A00`–`A12` | R/W | Q16.16 仿射矩阵，默认单位矩阵 |
| `0x11`–`0x14` | `0x044`–`0x050` | `FRAME0_ADDR`–`FRAME3_ADDR` | R/W | 帧槽地址；当前顶层仅保存/读回 |
| `0x15` | `0x054` | `FRAME_SIZE` | R/W | 默认 40,960,000；当前顶层仅保存/读回 |
| `0x20` | `0x080` | `COMP_DONE` | R | bit0=完成，下一次 start 时清零 |
| `0x21` | `0x084` | `STATUS` | R | 低 2 bit 为 HP 域当前活动模式 |
| `0x22` | `0x088` | `PERF_CNT_LO` | R | HP 时钟周期计数低 32 bit |
| `0x23` | `0x08C` | `PERF_CNT_HI` | R | HP 时钟周期计数高 32 bit |
| `0x30` | `0x0C0` | `VERSION` | R | `0x20260818` |
| `0x77` | `0x1DC` | `RESET` | W | 软件复位控制 |
| `0xFF` | `0x3FC` | `DEBUG` | R/W | 调试寄存器 |

启动值：

- LUT：向 `CTRL_START` 写 `0x00000003`；
- SA：向 `CTRL_START` 写 `0x00000005`；
- bit3 `frame_shift` 当前仅被寄存器保存/读回，未连接到 LUT/SA 数据通路。

## 6. 推荐的软件启动流程

1. 将 Part1 输出、LUT 和 FPGA 输出缓冲区放在 PLDDR 中，并保证 64 B 对齐。
2. 写入 `LUT_BASE_ADDR`、`FEAT2D_BASE_ADDR` 和 `FEAT3D_WR_ADDR`。
3. 写入 `LUT_SIZE=160000`、`BEV_PARAMS=0x04C8C840`、
   `IMG_PARAMS=0x060780A0`。
4. 确保上述多位配置在发出 start 前已经稳定。
5. 向 `CTRL_START` 写 `0x3` 启动 LUT 引擎。
6. 轮询 `COMP_DONE[0]`；完成后读取 `PERF_CNT_HI/LO`。
7. 将 Part3 的 Conv256 输入指向 `FEAT3D_WR_ADDR`，从 Conv256 继续执行。

建议软件通过 `(6<<24) | (120<<12) | 160` 计算 `IMG_PARAMS`，避免手工拼接
字段时出错。

## 7. RTL 仿真

回归需要 Python 3 和 Icarus Verilog。脚本优先使用 PATH 中的 `iverilog`/`vvp`，
也可以用 `-IverilogRoot` 指定便携版 Icarus 根目录。完整回归还会验证
Part1/Part3 编译网络和真实 LUT，因此需用 `-DataRoot` 指向包含下列目录的数据根：

```text
part1/
part2_lut/
part3/
```

它们不是综合 RTL 的依赖，只是 `run_all.ps1` 的数据一致性验证输入。准备好后：

```powershell
Set-Location E:\集创赛复微杯\Fast-BEV\FastBEV-FPGA\fpga_sandox
powershell -NoProfile -ExecutionPolicy Bypass -File .\Vivado_Project\sim\run_all.ps1 `
  -DataRoot E:\集创赛复微杯\Fast-BEV\FastBEV4.0_sandbox
```

快速回归跳过整帧测试：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\Vivado_Project\sim\run_all.ps1 `
  -DataRoot E:\集创赛复微杯\Fast-BEV\FastBEV4.0_sandbox -SkipFullFrame
```

测试范围包括网络接口、LUT 哈希和范围、FP32→TF32→FP16、FP32 加法器 5 拍
延迟、DDR 回压、顶层 CDC、顶层展开和整帧 320000 次输出写回。

## 8. Vivado 综合与实现复验

Vivado 2018.3 在中文源码路径下推断 BRAM 时可能出现 `TclStackFree` 崩溃。
脚本会将 RTL/约束复制到纯英文临时目录再完成独立综合、布局和布线：

```powershell
Set-Location E:\集创赛复微杯\Fast-BEV\FastBEV-FPGA\fpga_sandox
powershell -NoProfile -ExecutionPolicy Bypass -File .\Vivado_Project\sim\run_vivado_ascii.ps1
```

默认 Vivado 启动器：

```text
C:\vivado2018.3\vivado\Vivado\2018.3\bin\vivado.bat
```

如安装路径不同，可传入参数：

```powershell
.\Vivado_Project\sim\run_vivado_ascii.ps1 `
  -VivadoBat 'D:\Xilinx\Vivado\2018.3\bin\vivado.bat'
```

实现检查会强制验证全局 setup、hold 和 SA 内部 slack。脉宽负裕量只有在报告
行明确属于 `U0_ps_ai_wrap_demo` 受保护网表时才自动忽略；自写 RTL 中出现负
裕量会使脚本报错退出。

## 9. 已验证结果

| 指标 | 结果 |
|---|---:|
| Setup WNS / TNS | `+0.030 ns / 0 ns` |
| Hold WHS / THS | `+0.028 ns / 0 ns` |
| SA 内部最差 setup | `+0.275 ns` |
| 可路由网络 | 101892/101892，错误 0 |
| DRC Error | 0 |
| Slice LUT | 48,770 / 78,600（62.05%） |
| Slice Register | 59,530 / 157,200（37.87%） |
| BRAM Tile | 140 / 265（52.83%） |
| DSP | 38 / 400（9.50%） |

剩余 `WPWS=-0.409 ns` 的 2 个失败端点位于受保护 EDIF 内部 1 GHz
`AI_CORE_CLK` PLL/BUFG 路径，按工程边界忽略。不得通过修改自写 RTL、添加
错误 false path 或修改 EDIF 来“消除”该告警。

完整 RTL 回归的最终结果为 `ALL_RTL_CHECKS_PASS`；整帧统计为 20000 次 LUT
burst、624500 次特征读取、320000 次输出写回。

## 10. Bitstream

当前配置文件：`Bitstreams/bev_edif_top.bit`

```text
Size   : 5,980,024 bytes
SHA256 : 451FCBF412699F4611D633E3A512B09F1BFB82D1EFBA834F3532B4396AA0D95A
```

部署前建议重新运行完整 RTL 回归，并核对使用的 bitstream、RTL 版本寄存器和
软件寄存器表来自同一发布版本。

## 11. 已知边界与维护规则

- `ps_ai_wrap_demo.edf` 受保护且不可修改；来自该网表的时序问题按报告层级分类。
- 当前生产规格固定为输入 `[6,120,160,64]`、输出 `[1,16,200,200,16]`。
- `FEAT3D_WR_ADDR` 必须指向 Part3 value 362 的实际缓冲区。
- 配置总线采用“先写配置、保持稳定、再发启动握手”的 CDC 协议；运行期间不要
  改写本次计算使用的地址和维度。
- LUT 必须是 1,280,000 B；预期 SHA256 为
  `84187065e7fa9b7c2dd7aecf7992770bfa4f21b809696f14f9e0ee610a566383`。
- 修改浮点流水级后，必须同步更新 testbench 的延迟期望并重新跑完整实现。
- 修改约束时不要重复创建 `sys_clk_p`，也不要恢复当前顶层不存在的 SDI/FMC
  端口约束。

更完整的体系结构、寄存器说明和验证记录见 `Documentation/` 与
`Verification_Results/`。
