#ifndef _FILTER_H_
#define _FILTER_H_

#include <rtthread.h>
#include <board.h>
#include "arm_math.h"


#define ADC_BLOCK_SIZE 1
#define MAX_NUM_TAPS    32
#define MAX_BLOCK_SIZE  32
#define FIR 0
#define IIR 1

typedef struct{
    float32_t real;
    float32_t imag;
}complex;

//FIR滤波器的参数
typedef struct 
{
	arm_fir_instance_f32  *S;	//浮点型FIR滤波器结构体的实例
	uint16_t numTaps;			//滤波器的阶数
	float32_t pCoeffs[MAX_NUM_TAPS];		//滤波器系数
	float32_t pState[MAX_NUM_TAPS + MAX_BLOCK_SIZE - 1];			//函数内部计算数据的缓存
	uint32_t blockSize;			//每次需要处理的样本数
	float32_t scale;	//放大倍数
}filter_fir;

//IIR滤波器的参数
typedef struct 
{
	arm_biquad_casd_df1_inst_f32 *S;	//浮点型FIR滤波器结构体的实例
	uint16_t numStages;			//滤波器的阶数
	float32_t pCoeffs[MAX_NUM_TAPS];		//滤波器系数
	float32_t pState[MAX_NUM_TAPS + MAX_BLOCK_SIZE - 1];			//函数内部计算数据的缓存
	uint32_t blockSize;			//每次需要处理的样本数
	float32_t scale;	//放大倍数
}filter_iir;

//滤波器前后的数据
typedef struct {
	complex src;
	complex dst;
	complex dst_bj;
}filter_complex_data;

// filter_fir fir = {
// 	.numTaps = 2,
// 	.blockSize = ADC_BLOCK_SIZE,
// 	.pCoeffs = {
// 			1,2,1,1.972324544845770244450022801174782216549,-0.978340933957856928593344036926282569766,
// 			1,2,1,1.946764350938011256531012804771307855844,-0.948444394718458094750701548036886379123
// 																							},
// };

//数字滤波参数
typedef struct {
	uint8_t type;
	filter_fir fir;
	filter_iir iir;
}filter_t;

void filter_init(filter_t* filter);
float32_t filter_run(float32_t *src, filter_t* filter);


#endif
