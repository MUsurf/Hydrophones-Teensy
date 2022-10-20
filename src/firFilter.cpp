#include "firFilter.h"

// FIR bandpass filter variables. Coeffs in filter_taps[]
arm_fir_instance_f32 fir_instance_0, fir_instance_1;
float32_t firState0[FILTER_TAP_NUM + BLOCK_SIZE - 1], firState1[FILTER_TAP_NUM + BLOCK_SIZE - 1];

// FFT variables
arm_rfft_fast_instance_f32 f32_instance0, f32_instance1, f32_instance_rxy;