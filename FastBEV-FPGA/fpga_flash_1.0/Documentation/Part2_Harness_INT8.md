# FastBEV Part2 Harness Summary - INT8 FPGA Output

## Fixed Interfaces

- Register access remains word-addressed through `itf_ra_awaddr[17:2]`.
- PLDDR data width remains 512-bit, one beat is 64 bytes.
- `CTRL_START = 0x03` starts LUT mode.
- SA mode is not part of this INT8 delivery.

## Data Layouts

### Part1 Feature Input

```text
layout = [cam][v][u][64]
dtype  = fp32
bytes per pixel = 256
```

### LUT

```text
entry = {pad16, v16, u16, cam_id16}
invalid = cam_id < 0
lut_idx = (z * bev_y + y) * bev_x + x
```

### FPGA Output

```text
layout = [N][Z][C/32][X][Y][C%32]
shape  = [1][4][8][200][200][32]
dtype  = sint8
bytes  = 40960000
```

The output is no longer a 64-channel intermediate BEV tensor. It is the final
256-channel Decoder INT8 input.

## Address Checks

For `(x,y,z,c)`, where `c=0..255`:

```text
c_blk = c / 32
c_inner = c % 32
byte_addr = base + ((((z * 8 + c_blk) * 200 + x) * 200 + y) * 32 + c_inner)
```

Each DDR write packs all 32 channels from an even `y` followed by the same
channel block from the adjacent odd `y`:

```text
beat_addr = base + ((((z * 8 + c_blk) * 200 + x) * 200 + y_even) * 32)
y_even % 2 == 0
bytes  0..31 = y_even, c_inner 0..31
bytes 32..63 = y_even + 1, c_inner 0..31
```

Therefore every write is 64-byte aligned.

## Size Summary

| Item | Old FP32 path | New INT8 FPGA path |
|---|---:|---:|
| Per source voxel | `64 * fp32 = 256B` | `64 * int8 = 64B` internally |
| Final Decoder input | CPU-created FP32 | FPGA-created INT8 |
| Final buffer | `160000 * 256 * 4B` | `160000 * 256 * 1B` |
| Final bytes | `163,840,000` | `40,960,000` |

## Harness Expectations

- Invalid LUT entries write zeros for all four repeated channel groups.
- The four 64-channel groups must be bit-identical.
- `LUT_SIZE` must be 160000, `bev_z` must be 4, and `bev_y` must be even.
- All PLDDR base addresses must be 64-byte aligned.
