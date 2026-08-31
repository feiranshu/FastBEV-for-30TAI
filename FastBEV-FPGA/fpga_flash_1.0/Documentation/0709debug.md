# FastBEV Part2 INT8 — Quantizer scale fix & output-layout fix

> Archive note: this document records the original fix and mentions the one-off
> `quant_model.py` and `layout_model.py` used at that time. Those scripts are not
> included in the delivery; the maintained runnable contract check is
> `../Verification_Results/golden_layout_check.py`.

This change set touches **only two files** and their documentation. No unrelated
logic, interfaces, or module boundaries were refactored.

- `fp32_int8_quant.v` — remove a spurious `256x` factor from the effective scale.
- `lut_engine.v` — change the final output layout to `N, Z, C/32, X, Y, C%32`.

---

## 1. `fp32_int8_quant.v` — correct quantization formula

The effective quantization formula is:

```text
int8 = round(fp32 / 0.06905783)
```

Clarification (this is the important part):

```text
0.06905783 is the effective scale.
256 is the number of channels / repeated scale count, NOT a numeric scale multiplier.
256 * 0.06905783 must NOT be used as the effective scale.
```

### What was wrong

The datapath approximates `1/scale` with a constant multiply then an arithmetic
shift:

```text
|fp32|  = sig * 2^(exp-150)          , sig = {1, mantissa} (24-bit)
s0_prod = sig * 927                  , 927 ≈ (1/0.06905783) * 2^6 = 14.4816 * 64
result  = round( s0_prod >> (SHIFT_BASE - exp) )
```

Solving for the exponent so that `result == fp32 / 0.06905783`:

```text
2^(SHIFT_BASE - 150) = 927 * 0.06905783 = 64.017  ≈  2^6
=> SHIFT_BASE - 150 = 6
=> SHIFT_BASE = 156
```

The old `SHIFT_BASE = 164` gives `2^(164-150) = 2^14 = 16384 = 927 * (256*0.069058)`,
i.e. an extra `>> 8` (`/256`) on the shift path. That is exactly
`int8 = round(fp32 / (256 * 0.06905783))` — the `256x` scale-amplification bug.

### The fix

```text
SHIFT_BASE changed from 164 to 156
```

That is the only numeric change. The derived localparams follow automatically and
stay correct:

```text
ZERO_E = SHIFT_BASE - 40   (124 -> 116)   flush-to-zero threshold
SAT_E  = SHIFT_BASE        (164 -> 156)   coarse exponent guard (shamt <= 0)
```

The constant `927 = (1<<10)-(1<<7)+(1<<5)-1` is unchanged; `927/64 = 14.4844`
approximates `1/0.06905783 = 14.4816` to within ~0.02%.

### Saturation is a post-quant clamp (NOT part of the scale)

The int8-range clamp is kept and is described as a quantization-output
clamp/saturation, separate from the scale:

```text
positive overflow ->  127
negative overflow -> -128
NaN / tiny        ->    0
```

---

## 2. `lut_engine.v` — output layout

### Final output layout

```text
N, Z, C/32, X, Y, C%32
```

### Shape for this delivery

```text
1, 4, 8, 200, 200, 32          (N=1, Z=4, C/32=8, X=200, Y=200, C%32=32)
total = 1*4*8*200*200*32 = 40,960,000 bytes
```

Row-major (rightmost index fastest), with `c = c_blk*32 + c_inner`,
`c_blk = 0..7`, `c_inner = 0..31`:

```text
for n in range(1):
  for z in range(4):
    for c_blk in range(8):
      for x in range(200):
        for y in range(200):
          for c_inner in range(32):
            output[n][z][c_blk][x][y][c_inner]

byte_addr = feat3d_wr_addr
          + ((((z*(C/32) + c_blk)*bev_x + x)*bev_y + y)*32 + c_inner)
```

Strides: `c_inner=1`, `y=32`, `x=bev_y*32`, `c_blk=bev_x*bev_y*32`,
`z=(C/32)*bev_x*bev_y*32`.

### How the engine produces it

- The read / LUT / quantize path is unchanged. For each voxel column `(x,y)` the
  engine still produces all 4 z-planes × 64 source channels (4 source cblocks of
  16) into `qbuf`.
- `c_inner` (32 channels = 32 bytes) is half of a 512-bit DDR beat. So a full
  64-byte write packs `c_inner=0..31` of **two consecutive `y` values** (an
  `(even, odd)` pair) for the same `(z, c_blk, x)`:

  ```text
  wr_data[127:0]   = (y  , c_inner 0..15)  = src cblkA
  wr_data[255:128] = (y  , c_inner 16..31) = src cblkB
  wr_data[383:256] = (y+1, c_inner 0..15)  = src cblkA
  wr_data[511:384] = (y+1, c_inner 16..31) = src cblkB
  c_blk even -> (cblkA,cblkB)=(0,1) source channels 0..31
  c_blk odd  -> (cblkA,cblkB)=(2,3) source channels 32..63
  ```

- Replication to 256 channels is preserved: output channel `c` uses source
  channel `c % 64`. The 8 `c_blk` values are 4 repeats × 2 parity of the 64
  source channels.

### Implementation notes (minimal, interface-preserving)

- `qbuf0..3` became `[y_parity][z]` so an `(even, odd)` y-pair is held before the
  write. Even-`y` groups are buffered and advance without writing; the odd-`y`
  group flushes 32 beats (`write_k = z*(C/32) + c_blk`, 0..31).
- Address walks by a precomputed run-constant stride `zc_stride_r = bev_x*bev_y*32`
  (one multiply at `engine_start`, none on the per-beat path). The pair base is
  `feat3d_wr_addr + ((xy_idx-1) << 5)` (wire shift, no DSP).
- Module ports, the DMA/DDR handshake, and the `req/grant` protocol are unchanged.
- Assumption for this delivery: `bev_z = 4` and `bev_y` even (200), so each `x`
  row splits into complete `(even, odd)` y-pairs.

---

## 3. Verification

No simulator was available in the build environment, so both data paths were
verified with **bit-exact Python models** of the RTL arithmetic and address
generation (`quant_model.py`, `layout_model.py`).

### 3.1 Quantizer (`quant_model.py`)

Replicates the exact fixed-point datapath (sig×927, round-half-up shift, clamp).

- `SHIFT_BASE=156` vs `round(fp32/0.06905783)` over 200k random values plus edge
  cases (0, ±scale, saturation boundary, NaN, ±Inf): **0 gross mismatches**;
  worst `|diff| = 1`, only at rounding half-boundaries caused by the `927/64`
  constant approximation (inherent to the fixed-point design, unrelated to the
  256 bug).
- `SHIFT_BASE=164` reproduces `round(fp32/(256*0.06905783))` exactly — confirming
  the old code had the extra `256x` divide.
- Spot checks: `1.0 -> 14`, `8.0 -> 116`, `8.8 -> 127`, `9.0 -> 127`,
  `-9.0 -> -128`, `0.06905783 -> 1`.

### 3.2 Output layout (`layout_model.py`)

Replicates the FSM's write-address + beat packing.

- 640,000 beats emitted; all beat bases distinct and tiling `[0, 40,960,000)` in
  exact 64-byte steps → a **perfect partition** (no gaps, no overlap).
- Sampled byte-level check: every byte's address equals
  `target_addr(n,z,c_blk,x,y,c_inner)` for the target row-major layout, and the
  channel value at each byte satisfies the `c % 64` replication rule.
- Independent dense-ordering check confirms the target layout is contiguous
  row-major `0..40,960,000`.

Result: **engine output == `N, Z, C/32, X, Y, C%32` = `1,4,8,200,200,32`.**

### 3.3 Adversarial review

- `fp32_int8_quant.v`: no remaining `*256`, `<<8`, or `256*0.06905783` in any
  expression — only in explanatory comments. Effective scale is `0.06905783`.
- `lut_engine.v`: write order verified to be `N,Z,C/32,X,Y,C%32`; no dangling
  references to the removed `write_cblock/repeat_id/wr_addr_next/lut_size_bytes_r`.
- Structural balance (`begin/end`, `case/endcase`, `function/endfunction`,
  `generate/endgenerate`, parentheses) checks pass on both files.

---

## 4. Items for human review

1. **No RTL simulation was run** (no `iverilog`/`verilator`, network disabled).
   The Python models are bit-exact replicas, but a real elaboration + gate-level
   sim against Part1/Decoder golden data is recommended before tape-in.
2. **`0.06905783` vs `927/64`.** The RTL uses the constant `927` (≈0.02% error).
   If the Decoder needs the exact scale, confirm this tolerance is acceptable or
   widen the constant (would also shift `SHIFT_BASE`).
3. **Layout assumption `bev_y` even / `bev_z=4`.** Only the fixed
   `200×200×4` geometry was verified. Non-even `bev_y` or `bev_z≠4` would need the
   y-pairing / z-grouping revisited.
4. **`p_abs`** in `lut_engine.v` is written but not read (pre-existing in the
   original); left untouched.
5. Confirm the **downstream Decoder** truly expects `N,Z,C/32,X,Y,C%32` with the
   `c%64` replication (this doc encodes that assumption from the task spec).
