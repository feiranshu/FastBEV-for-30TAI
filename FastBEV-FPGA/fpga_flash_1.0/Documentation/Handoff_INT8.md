# FastBEV Part2 INT8 FPGA Handoff

## 1. Data Flow

The FPGA performs all Part2 data formatting:

```text
Part1 FP32 NHWC feature
  -> read 4 fp32 beats per valid LUT pixel
  -> quantize 16 fp32 lanes per beat to int8
  -> map by LUT to BEV P index
  -> replicate 64 channels four times
  -> write Decoder blocked INT8 layout
```

CPU responsibilities are reduced to:

1. Load the LUT table.
2. Configure Part2 registers.
3. Pass the Part1 output PLDDR address to `FEAT2D_BASE_ADDR`.
4. Pass the Decoder INT8 input PLDDR address to `FEAT3D_WR_ADDR`.
5. Start LUT mode with `CTRL_START = 0x03`.
6. Wait for `COMP_DONE`.

## 2. Register Map

CPU byte address is `0x400C_0000 + word_addr * 4`.

| Word | Name | Default | Description |
|---:|---|---:|---|
| `0x00` | `CTRL_START` | - | bit0 start, bit[2:1] mode. LUT mode is `0x03`. |
| `0x01` | `LUT_BASE_ADDR` | 0 | PLDDR byte address of LUT table. |
| `0x02` | `LUT_SIZE` | 160000 | Number of BEV voxels. Must be a multiple of 4. |
| `0x03` | `FEAT2D_BASE_ADDR` | 0 | Part1 FP32 NHWC feature PLDDR byte address. |
| `0x04` | `FEAT3D_WR_ADDR` | 0 | Final Decoder INT8 input PLDDR byte address. |
| `0x05` | `FEAT3D_WR_SIZE` | 40960000 | Final output bytes. |
| `0x09` | `BEV_PARAMS` | `0x04C8C840` | `{z=4,y=200,x=200,ch=64}`. |
| `0x0A` | `IMG_PARAMS` | `0x060400B0` | `{cams=6,h=64,w=176}`. |
| `0x20` | `COMP_DONE` | 0 | Sticky done, cleared by next start. |
| `0x21` | `STATUS` | 0 | bit0 is LUT done pulse status. |
| `0x22/0x23` | `PERF_CNT` | 0 | 64-bit cycle counter. |
| `0x30` | `VERSION` | `0xBE080001` | INT8 package version. |
| `0x77` | `RESET` | - | Software reset. |

## 3. Input and LUT Format

Part1 feature:

```text
[cam][v][u][channel]
channel = 64 fp32
pixel bytes = 64 * 4 = 256
pixel_addr = feat2d_base + (((cam * img_h + v) * img_w + u) * 256)
```

LUT table:

```text
lut_idx = (z * bev_y + y) * bev_x + x
entry = {pad[63:48], v[47:32], u[31:16], cam_id[15:0]}
cam_id < 0 means invalid and produces zeros.
```

## 4. Output Layout

The output is already the Decoder INT8 input:

```text
shape  = [1, 4, 8, 200, 200, 32]
layout = [N][Z][C/32][X][Y][C%32]
dtype  = sint8
bytes  = 1 * 4 * 8 * 200 * 200 * 32 = 40960000
```

Mapping:

```text
c_blk = c / 32
c_inner = c % 32
addr = base + ((((z * 8 + c_blk) * 200 + x) * 200 + y) * 32 + c_inner)
```

The source has 64 channels. The engine writes four identical channel groups:

```text
repeat 0: c =   0.. 63
repeat 1: c =  64..127
repeat 2: c = 128..191
repeat 3: c = 192..255
```

## 5. Quantization

The hardware module `fp32_int8_quant` implements:

```text
q = round(fp32 / 0.06905783)
q = clamp(q, -128, 127)
```

NaN and tiny values become 0. The module has 3-cycle latency.

## 6. Software Contract and Prohibited Behaviors

The current RTL is intended to work with fully cooperative software. Software must
treat the register interface as a strict contract. Violating any item below may
hang the DMA path, drop a start command, produce corrupted output, or read/write
outside the intended PLDDR buffers.

### 6.1 Execution ordering

Software must follow exactly one in-flight job at a time:

1. Reset the block only during initialization or while the engine is idle.
2. Configure all static registers before the first run.
3. For each frame, wait for the Part1 FP32 feature tensor to be ready.
4. Write `FEAT2D_BASE_ADDR`.
5. Write `FEAT3D_WR_ADDR`.
6. Write `CTRL_START = 0x03` exactly once.
7. Poll `COMP_DONE[0]` until it becomes 1.
8. Do not reuse, free, overwrite, or remap any involved PLDDR buffer until
   `COMP_DONE[0] == 1`.

Software must not issue a second start before the previous job is complete.

### 6.2 Register writes that are forbidden while a job is running

After writing `CTRL_START = 0x03` and before observing `COMP_DONE[0] == 1`,
software must not write any of the following registers:

- `CTRL_START`
- `LUT_BASE_ADDR`
- `LUT_SIZE`
- `FEAT2D_BASE_ADDR`
- `FEAT3D_WR_ADDR`
- `FEAT3D_WR_SIZE`
- `BEV_PARAMS`
- `IMG_PARAMS`
- `RESET`

The hardware does not snapshot all multi-bit configuration registers internally.
Therefore, runtime writes may change active addresses, dimensions, mode, or size
while the DMA engine is already using them.

### 6.3 Allowed mode and fixed dimensions

Software must only use LUT mode:

```text
CTRL_START = 0x03
```

All other `CTRL_START` values and modes are forbidden for this handoff.

The current hardware package must be treated as fixed-shape:

```text
BEV_X        = 200
BEV_Y        = 200
BEV_Z        = 4
SRC_CHANNELS = 64
CAMERAS      = 6
IMG_W        = 176
IMG_H        = 64
LUT_SIZE     = 160000
FEAT3D_SIZE  = 40960000
```

Software must not program alternative dimensions, channel counts, camera counts,
LUT sizes, or output sizes. In particular:

- `LUT_SIZE` must be exactly `160000`.
- `LUT_SIZE` must be non-zero and divisible by 4.
- `FEAT3D_WR_SIZE` must be exactly `40960000`.
- `BEV_PARAMS` must be exactly `{z=4,y=200,x=200,ch=64}`.
- `bev_y` must be even because one DDR beat packs an even/odd `y` pair.
- `IMG_PARAMS` must be exactly `{cams=6,h=64,w=176}`.

### 6.4 PLDDR address and buffer rules

All PLDDR addresses passed to the FPGA must be 32-bit byte addresses. Software
must reject addresses above `0xFFFF_FFFF` instead of truncating them.

Required buffers:

```text
LUT buffer            >= 1,280,000 bytes
Part1 FP32 feature    >= 17,301,504 bytes
Decoder INT8 output   >= 40,960,000 bytes
```

Address and buffer restrictions:

- `LUT_BASE_ADDR`, `FEAT2D_BASE_ADDR`, and `FEAT3D_WR_ADDR` must be at least
  64-byte aligned.
- `FEAT2D_BASE_ADDR` is preferably 256-byte aligned because each source pixel is
  256 bytes.
- The LUT, Part1 FP32 feature, and Decoder INT8 output buffers must not overlap.
- The LUT buffer must remain valid and unchanged while the engine is running.
- The Part1 FP32 feature buffer must remain valid and unchanged while the engine
  is running.
- The Decoder INT8 output buffer must not be read by Decoder or CPU until
  `COMP_DONE[0] == 1`.
- If software writes the LUT or feature data through a cached CPU mapping, it
  must flush/synchronize the data before starting the FPGA job.

### 6.5 LUT content rules

Every LUT entry must be either invalid or in range.

Invalid entry:

```text
cam_id = -1 / 0xFFFF
```

Valid entry constraints:

```text
0 <= cam_id < 6
0 <= u      < 176
0 <= v      < 64
```

Software must not provide valid-looking LUT entries with out-of-range `cam_id`,
`u`, or `v`. The current hardware checks invalid entries by `cam_id < 0`; it does
not enforce the upper bounds for camera, `u`, or `v`.

### 6.6 Polling and status rules

Software must use `COMP_DONE[0]` as the completion indicator.

Software must not use `STATUS[0]` as the completion condition because it is a
short pulse-style status signal. `PERF_CNT` is diagnostic only and should be read
after completion, not used for synchronization.

### 6.7 Concurrency rules

Only one software thread/process may own this register block at a time. Software
must serialize all register access to this accelerator and must not let another
thread reset, reconfigure, or start the engine while a job is active.

### 6.8 Register ordering and barriers

Software must ensure that all configuration writes reach the register interface
before `CTRL_START = 0x03` reaches the hardware. If the register API uses posted
or weakly ordered writes, use ordered writes, read-back, or an explicit memory
barrier before the start write.

Polling `COMP_DONE` must also use a register read path that cannot be satisfied
from a stale cached value.

## 7. Decoder Boundary

This handoff targets the INT8 Matmul-pre boundary. In the current full decoder
JSON, the public input is still FP32. Integration must either:

- cut/regenerate Decoder so the public input is
  `sint8 [1,4,8,200,200,32] [N][Z][C/32][X][Y][C%32]`, or
- bind the FPGA output to the internal value with this layout and dtype.
