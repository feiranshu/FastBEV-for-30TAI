# FastBEV Part1–FPGA–Part3 接口说明

本目录以 `lut_engine_fp32.v` 和 `sa_engine_fp32.v` 为当前有效实现；旧版
`lut_engine.v`、`sa_engine.v` 已移至 `../../Archive/legacy_rtl/`，不参与构建。

## 已核对的网络接口

- Part1 编译网络输出：`FP32 NHWC [6,120,160,64]`，共 29,491,200 字节；
- FPGA LUT 路径输入：同一物理布局，每个像素的 64 个 FP32 连续存放，分四个
  512-bit DDR 拍读取；
- FPGA 输出/Part3 节点 362：`FP16 [1,16,200,200,16]`，物理布局
  `NCHWc16`，共 20,480,000 字节；
- Part3 的 `Conv2dNode 256` 直接消费 value 362。

原始 Part3 ONNX 的入口变换是：

```text
[N,C,X,Y,Z]
  --Transpose(0,2,3,4,1)--> [N,X,Y,Z,C]
  --Reshape---------------> [N,X,Y,256]
  --Transpose(0,3,1,2)----> [N,256,X,Y]
```

因此 FPGA 必须使用以下通道折叠关系：

```text
global_channel = z*64 + c
global_cblk16  = z*4 + floor(c/16)
c_inner        = c mod 16
```

当前 RTL 与该关系一致。输出字节地址为：

```text
FEAT3D_WR_ADDR
  + ((((z*4 + source_cblk16)*BEV_X + x)*BEV_Y + y)*16 + c_inner)*2
```

一个 `(x,cblk16)` 的相邻 `y_even/y_odd` 对打包成一个 512-bit 写拍：低
256 bit 为 `y_even`，高 256 bit 为 `y_odd`。

注意：`FEAT3D_WR_ADDR` 必须指向 Part3 value 362/Conv256 输入对应的缓冲区，
并从 Conv256 开始执行或跳过 Part3 原入口到 Reshape254 的预处理链；否则该链会
重新生成并覆盖节点 362。

## LUT 与输入地址

LUT 文件为 `part2_lut/fastbev_lut_table.bin`：160000 项，每项为小端
`int16[4] = [cam_id,u,v,pad]`，顺序为 ZYX：

```text
lut_index = (z*BEV_Y + y)*BEV_X + x
feature_pixel = (cam_id*IMG_H + v)*IMG_W + u
feature_byte_addr = FEAT2D_BASE_ADDR + feature_pixel*64*4
```

默认参数为相机 6、图像特征高 120、宽 160、BEV `200×200×4`。

## 时序与跨时钟域修正

- GP 时钟域的模式信号采用两级同步；启动脉冲到达 HP 域时锁存全部多位配置，
  再延迟一拍启动引擎，运行中不再直接使用异步配置总线；
- 软件复位写入放回 GP 时钟域，HP 复位使用两级同步后再生成高扇出内部复位；
- FP32→TF32 与 TF32→FP16 之间加入流水寄存器，FP16 舍入结果在写入
  voxel block 前再寄存一拍，切断 200 MHz 组合长路径；
- FP32 加法器的指数差改为只比较两个指数，不再让 24-bit 尾数比较串入
  移位量寄存器路径；该修改不改变 5 拍加法延迟；
- NHWC 特征地址使用分级乘加，输出平面 stride 在启动时预计算；
- 偶数 Y 行缓冲拆分为 16 个 `256×64-bit` bank，Vivado 2018.3 实际映射为
  16 个 RAMB36E1，避免原来 262144 bit 展开成触发器；
- 写请求在 DDR 回压期间保持地址和数据稳定；
- 请求/应答脉冲及模式、复位同步寄存器均标注 `ASYNC_REG`；已删除不属于
  `bev_edif_top` 的旧 SDI/FMC 时钟约束和重复的 `sys_clk_p` 时钟定义。

目标器件 `xc7z030ffg676-2` 的完整布局布线复验中，全局 setup/hold 的
WNS/WHS 分别为 `+0.030 ns` / `+0.028 ns`，均无违例；SA 引擎内部最差
setup 裕量为 `+0.275 ns`。全局最差 setup 路径及剩余 `WPWS=-0.409 ns`
脉宽告警均位于受保护的 `ps_ai_wrap_demo.edf`，按工程边界保留且不修改。

Vivado 2018.3 在中文源码路径下综合推断 BRAM 可能触发 `TclStackFree`
崩溃。完整实现验证请用 `../sim/run_vivado_ascii.ps1` 复制到纯英文目录。

## 仿真

在工程根目录运行，并用 `-DataRoot` 指向包含 `part1`、`part2_lut`、`part3`
的 FastBEV 数据根目录：

```powershell
powershell -ExecutionPolicy Bypass -File .\Vivado_Project\sim\run_all.ps1 `
  -DataRoot E:\集创赛复微杯\Fast-BEV\FastBEV4.0_sandbox
```

快速回归可添加 `-SkipFullFrame`。完整回归会校验两份编译网络及 RAW、真实 LUT、
FP 转换边界、DDR 回压、跨时钟域配置快照、顶层展开，并用真实 160000 项 LUT
完成整帧 320000 次输出写回检查。

完整 Vivado 实现验证：

```powershell
powershell -ExecutionPolicy Bypass -File .\Vivado_Project\sim\run_vivado_ascii.ps1
```
