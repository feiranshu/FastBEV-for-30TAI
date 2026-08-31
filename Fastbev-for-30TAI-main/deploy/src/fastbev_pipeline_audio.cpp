/*
 * Audio-enabled variant of the stable synchronous FastBEV pipeline.
 *
 * Keeping this as a thin translation-unit wrapper means the inference path
 * stays in the shared synchronous implementation while FPGA LED and USB-audio
 * alert code is compiled only into this executable.
 */
#define FASTBEV_ENABLE_AUDIO 1
#include "fastbev_pipeline.cpp"
