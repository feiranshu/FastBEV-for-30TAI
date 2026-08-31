#!/usr/bin/env python3
"""Reference checks for the FastBEV Flash v1.0 INT8 output contract."""

import math

BEV_X = 200
BEV_Y = 200
BEV_Z = 4
SRC_CHANNELS = 64
REPEAT_TIMES = 4
DEC_CHANNELS = SRC_CHANNELS * REPEAT_TIMES
CHANNEL_BLOCK = 32
CHANNEL_BLOCKS = DEC_CHANNELS // CHANNEL_BLOCK
LUT_SIZE = BEV_X * BEV_Y * BEV_Z
DECODER_INT8_BYTES = LUT_SIZE * DEC_CHANNELS
QUANT_SCALE = 0.06905783


def quant_fp32_to_int8(value: float) -> int:
    """Model round-half-away-from-zero followed by signed-int8 saturation."""
    if math.isnan(value) or value == 0.0:
        return 0
    if math.isinf(value):
        return 127 if value > 0 else -128
    magnitude = math.floor(abs(value / QUANT_SCALE) + 0.5)
    quantized = int(math.copysign(magnitude, value))
    return max(-128, min(127, quantized))


def output_offset(x: int, y: int, z: int, channel: int) -> int:
    """Return the byte offset for [N][Z][C/32][X][Y][C%32]."""
    channel_block, channel_inner = divmod(channel, CHANNEL_BLOCK)
    return (
        ((((z * CHANNEL_BLOCKS + channel_block) * BEV_X + x) * BEV_Y + y)
         * CHANNEL_BLOCK)
        + channel_inner
    )


def lut_index_zyx(x: int, y: int, z: int) -> int:
    return (z * BEV_Y + y) * BEV_X + x


def main() -> None:
    assert BEV_Y % 2 == 0
    assert CHANNEL_BLOCKS == 8
    assert LUT_SIZE == 160_000
    assert DECODER_INT8_BYTES == 40_960_000

    assert output_offset(0, 0, 0, 0) == 0
    assert output_offset(0, 1, 0, 0) == CHANNEL_BLOCK
    assert output_offset(0, 0, 0, 32) == BEV_X * BEV_Y * CHANNEL_BLOCK
    assert output_offset(0, 0, 1, 0) == CHANNEL_BLOCKS * BEV_X * BEV_Y * CHANNEL_BLOCK
    assert output_offset(199, 199, 3, 255) == DECODER_INT8_BYTES - 1

    # One 512-bit write contains 32 channels for y_even followed by y_odd.
    for z in range(BEV_Z):
        for channel_block in range(CHANNEL_BLOCKS):
            channel = channel_block * CHANNEL_BLOCK
            beat_base = output_offset(7, 8, z, channel)
            assert beat_base % 64 == 0
            assert output_offset(7, 9, z, channel) == beat_base + 32
            assert output_offset(7, 9, z, channel + 31) == beat_base + 63

    # Output channels repeat the same 64-channel source group four times.
    for source_channel in range(SRC_CHANNELS):
        assert [
            (source_channel + repeat * SRC_CHANNELS) % SRC_CHANNELS
            for repeat in range(REPEAT_TIMES)
        ] == [source_channel] * REPEAT_TIMES

    assert quant_fp32_to_int8(float("nan")) == 0
    assert quant_fp32_to_int8(float("inf")) == 127
    assert quant_fp32_to_int8(float("-inf")) == -128
    assert quant_fp32_to_int8(0.0) == 0
    assert quant_fp32_to_int8(QUANT_SCALE) == 1
    assert quant_fp32_to_int8(-QUANT_SCALE) == -1
    assert quant_fp32_to_int8(1.0) == 14
    assert quant_fp32_to_int8(1e9) == 127
    assert quant_fp32_to_int8(-1e9) == -128

    assert lut_index_zyx(0, 0, 0) == 0
    assert lut_index_zyx(0, 0, 1) == 40_000
    assert lut_index_zyx(199, 199, 3) == 159_999
    print("golden_layout_check: OK")


if __name__ == "__main__":
    main()
