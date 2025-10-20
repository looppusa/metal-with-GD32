#include "filter.h"

void filter_init(filter_t *filter)
{
	switch(filter->type)
	{
		case FIR:
			arm_fir_init_f32(filter->fir.S,  filter->fir.numTaps,  filter->fir.pCoeffs,  filter->fir.pState,  filter->fir.blockSize);
			break;
		case IIR:
			arm_biquad_cascade_df1_init_f32(filter->iir.S, filter->iir.numStages, filter->iir.pCoeffs, filter->iir.pState);
			break;
		default:
			break;
	}
}


//数字滤波器运行
float32_t filter_run(float32_t *src, filter_t *filter)
{
	float32_t dst = 0;
	switch(filter->type)
	{
		case FIR:
			arm_fir_f32((filter->fir.S), src, &dst, filter->fir.blockSize);
			dst *= filter->fir.scale;
			break;
		case IIR:
			arm_biquad_cascade_df1_f32((filter->iir.S), src, &dst, filter->iir.blockSize);
			dst *= filter->iir.scale;
			break;
		default:
			break;
	}
	return dst;
}
