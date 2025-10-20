#include "data_process.h"
#include "filter.h"
#include "ad7190.h"
#include "ad983x.h"
#include "dac7811.h"

rt_sem_t dp_sem, bj_sem;

arm_biquad_casd_df1_inst_f32 hpf_filter[2];
arm_biquad_casd_df1_inst_f32 lpf_filter[4];

// 滤波器实例化 lps 5Hz	hps 3Hz
filter_t filter_lpf_r = {
	.type = IIR,
	.iir = {
		.S = &lpf_filter[0],
		.numStages = 2,
		.blockSize = ADC_BLOCK_SIZE,
		.pCoeffs = {
			1, 2, 1, 1.972324544845770244450022801174782216549, -0.978340933957856928593344036926282569766,
			1, 2, 1, 1.946764350938011256531012804771307855844, -0.948444394718458094750701548036886379123},
		.scale = 0.001504097278021694454597234624770862865 * 0.001504097278021694454597234624770862865 * 0.891250938133745562730325673328479751945,
	},
};

filter_t filter_lpf_i = {
	.type = IIR,
	.iir = {
		.S = &lpf_filter[1],
		.numStages = 2,
		.blockSize = ADC_BLOCK_SIZE,
		.pCoeffs = {
			1, 2, 1, 1.972324544845770244450022801174782216549, -0.978340933957856928593344036926282569766,
			1, 2, 1, 1.946764350938011256531012804771307855844, -0.948444394718458094750701548036886379123},
		.scale = 0.001504097278021694454597234624770862865 * 0.001504097278021694454597234624770862865 * 0.891250938133745562730325673328479751945,
	},
};

filter_t filter_hpf_r = {
	.type = IIR,
	.iir = {
		.S = &hpf_filter[0],
		.numStages = 1, // 二阶IIR滤波器的个数
		.blockSize = ADC_BLOCK_SIZE,
		.pCoeffs = {
			1, -2, 1, 1.955578240315035243312991042330395430326, -0.956543676511203200263366852595936506987},
		.scale = 0.978030479206559610894089473731582984328,
	},
};

filter_t filter_hpf_i = {
	.type = IIR,
	.iir = {
		.S = &hpf_filter[1],
		.numStages = 1, // 二阶IIR滤波器的个数
		.blockSize = ADC_BLOCK_SIZE,
		.pCoeffs = {
			1, -2, 1, 1.955578240315035243312991042330395430326, -0.956543676511203200263366852595936506987},
		.scale = 0.978030479206559610894089473731582984328,
	},
};

// 1hz
filter_t filter_lpf_bj_r = {
	.type = IIR,
	.iir = {
		.S = &lpf_filter[2],
		.numStages = 2, // 二阶IIR滤波器的个数
		.blockSize = ADC_BLOCK_SIZE,
		.pCoeffs = {
			1, 2, 1, 1.999919843215512749168283335166051983833, -0.999919854181300138407095801085233688354,
			1, 2, 1, 1.999806510877087717403810529503971338272, -0.999806521842253936860345220338786020875},
		.scale = 0.000000002741446913418854483167073411189 * 0.000000002741291559897852368484304135606,
	},
};

filter_t filter_lpf_bj_i = {
	.type = IIR,
	.iir = {
		.S = &lpf_filter[3],
		.numStages = 2, // 二阶IIR滤波器的个数
		.blockSize = ADC_BLOCK_SIZE,
		.pCoeffs = {
			1, 2, 1, 1.999919843215512749168283335166051983833, -0.999919854181300138407095801085233688354,
			1, 2, 1, 1.999806510877087717403810529503971338272, -0.999806521842253936860345220338786020875},
		.scale = 0.000000002741446913418854483167073411189 * 0.000000002741291559897852368484304135606,
	},
};

int32_t amplitude, phase;
filter_complex_data fdata;
float32_t pack[2000] = {0};

void send(float32_t K0,float32_t K1,float32_t K2,float32_t K3,float32_t K4)
{
		int32_t data_t[5] = {0x47,0,0,0};
		data_t[0] = K0;
		data_t[1] = K1;
		data_t[2] = K2;
		data_t[3] = K3;
		data_t[4] = K4;
		rs232_send_msg((rt_uint8_t *)data_t, 5*4);
}


void balance_process(void)
{
	//	rt_uint8_t data_protocol[4] = {0};
	//	float data_real[25];
	//	float data_imag[25];
	//	complex data;
	//	static int i=0;
	int32_t real, imag;
	//	static int i=0;
	while (1)
	{
		rt_sem_take(bj_sem, RT_WAITING_FOREVER);
		//		if(i<1000)
		//			i++;
		fdata.dst_bj.real = fdata.src.real;
		fdata.dst_bj.imag = fdata.src.imag;

		// 进行数字低通滤波,将金属信号滤除
		//		fdata.dst_bj.real = filter_run(&(fdata.src.real), &filter_lpf_r);
		//		fdata.dst_bj.imag = filter_run(&(fdata.src.imag), &filter_lpf_r);

		if (state_balance) // 停止按钮
		//		if(!in_detection_zone)
		{

			balance_adjust(&(fdata.dst_bj));

			// 未触发红外，进行底噪阈值获取
			//			if(i==1000)
			//			{
			//				if(noise_background_max.real < fdata.dst.real)	noise_background_max.real = fdata.dst.real;
			//				if(noise_background_max.imag < fdata.dst.imag)	noise_background_max.imag = fdata.dst.imag;
			//				if(noise_background_min.real > fdata.dst.real)	noise_background_min.real = fdata.dst.real;
			//				if(noise_background_min.imag > fdata.dst.imag)	noise_background_min.imag = fdata.dst.imag;
			//
			//			}

			// #ifdef BJ
			// 采集的数据
			real = (int32_t)fdata.dst_bj.real;
			imag = (int32_t)fdata.dst_bj.imag;
			// 采集数据获取的相位
			//		real = (int32_t)sqrtf(fdata.dst_bj.real*fdata.dst_bj.real+fdata.dst_bj.imag*fdata.dst_bj.imag);
			//		imag = (int32_t)atan2(fdata.dst_bj.imag, fdata.dst_bj.real) / 2.5 * 8388608 * 180.0f / 3.1415926f;

			// 将滤波后的数据打包发送
			ad7190_pack(CH_AN1_AN2, imag); // 蓝
			ad7190_pack(CH_AN3_AN4, real); // 红
			// #endif
		}

		//
		// #ifdef BJ
		//					//采集的数据
		//					real = (int32_t)fdata.dst_bj.real;
		//					imag = (int32_t)fdata.dst_bj.imag;
		
		
		
		//					//采集数据获取的相位
		////		real = (int32_t)sqrtf(fdata.dst_bj.real*fdata.dst_bj.real+fdata.dst_bj.imag*fdata.dst_bj.imag);
		////		imag = (int32_t)atan2(fdata.dst_bj.imag, fdata.dst_bj.real) / 2.5 * 8388608 * 180.0f / 3.1415926f;

		//					// 将滤波后的数据打包发送
		//					ad7190_pack(CH_AN1_AN2, imag);//蓝
		//					ad7190_pack(CH_AN3_AN4, real);//红
		// #endif
	}
}

void reject_process(void)
{
	while (1)
	{
		// 获取tick值
		reject_opt.tick = rt_tick_get();
		if (rm_reject(reject_opt.tick))
		{
			DetectCtrl(state_detect);
		}
		rt_thread_delay(1); // 1ms
	}
}

// 数据处理任务
void data_process(void)
{
	int32_t real, imag;
	uint32_t i=0;
//	static int i = 0;
	while (1)
	{
		rt_sem_take(dp_sem, RT_WAITING_FOREVER);

		// 将滤波器的原始数据获取到，再输出数字滤波后的数据
		imag = -ad7190_read_channel(CH_AN1_AN2);
		real = ad7190_read_channel(CH_AN3_AN4);

		fdata.src.real = (float32_t)real;
		fdata.src.imag = (float32_t)imag;
		rt_sem_release(bj_sem);

#ifdef FILTER
		// 进行数字低通滤波
//		fdata.dst.real = filter_run(&(fdata.src.real), &filter_lpf_r);
//		fdata.dst.imag = filter_run(&(fdata.src.imag), &filter_lpf_i);

		// 进行数字高通滤波
		fdata.dst.real = filter_run(&(fdata.src.real), &filter_hpf_r);
		fdata.dst.imag = filter_run(&(fdata.src.imag), &filter_hpf_i);

#else
		// 原始数据
		fdata.dst.real = fdata.src.real;
		fdata.dst.imag = fdata.src.imag;
#endif


		if (!state_balance)
		{
			// #ifndef BJ

			real = (int32_t)fdata.dst.real;
			imag = (int32_t)fdata.dst.imag;
			
						// 采集数据获取的相位
//			real = (int32_t)sqrtf(fdata.dst_bj.real*fdata.dst_bj.real+fdata.dst_bj.imag*fdata.dst_bj.imag);
//			imag = (int32_t)atan2(fdata.dst_bj.imag, fdata.dst_bj.real) / 2.5 * 8388608 * 180.0f / 3.1415926f;
			// 将滤波后的数据打包发送
			ad7190_pack(CH_AN1_AN2, imag);
			ad7190_pack(CH_AN3_AN4, real);
			// #endif
		}
		//	if((real > 0.00005f*8388608.0f/2.5f || -real > 0.00005f*8388608.0f/2.5f || -imag > 0.00005f*8388608.0f/2.5f || imag > 0.00005f*8388608.0f/2.5f) && 1 && state_motor != 0)
		//		in_detection_zone = 1;

		// 判断是否进入探测区域
		if (in_detection_zone) // 进入探测区域
							   //		if(1)//目前默认探测状态
		{
			if (data_pack(fdata.dst))
			{
				in_detection_zone = 0;
				// 判断是进入学习状态，开始探测状态
				// 注意：学习状态必须保持为矩阵处理方式，而非实时处理方式
				switch (statework)
				{
				// 学习状态：计算相位与最值
				case LEARNING:
				{
					learning_process(&paradetection, &paralearning);
					
					// 学习波形
					for(i=0;i<SIZE;i++)
					{
						wavei[i] += paradetection.data_rotation[i]/SIZE;
						waveq[i] += paradetection.data_rotation[i+1]/SIZE;
					}
				
					
//					send(0x47,paralearning.amp_max.real,paralearning.amp_max.imag,paralearning.amp_min.real,paralearning.amp_min.imag);
					break;
				}
				// 探测状态：旋转相位与能量检测
				case DETECTION:
				{
					// 判断是否存在金属异物
					// 是：进入剔除状态
					// 否：计算相位与最值
					detection_process(&paradetection, &paralearning);
//					send(0x50,paradetection.amp_max.real,paradetection.amp_max.imag,paradetection.amp_min.real,paradetection.amp_min.imag);
					break;
				}
				case IDLE:
				{
					break;
				}
				default:
					break;
				}
			}
		}
	}
}

/* 平衡调节，基于PID*/
void balance_adjust(complex *data0)
{
//	uint32_t data_t[4] = {0x47,0,0,0};
//	int size = 1;
//	static int i = 0;
	static float32_t Ki1 = 50; // 积分项参数
	static float32_t Ki2 = 1;// 积分项参数
	static float32_t Ki3 = 0.5;// 积分项参数
	float32_t Ki = Ki2;
	static float32_t Kp = 0;  // 积分项参数
	static float32_t Kd = 0;  // 积分项参数
	complex data = {0};
	static float32_t amp, ph;
	static complex data_out;
	static complex err_i = {0};
	static complex err_d = {0};
	static complex err = {0};
	static complex err_last = {0};
//	static complex data_3000 = {0};
	data.real = data0->real * 2.5f / 8388608.0f;
	data.imag = data0->imag * 2.5f / 8388608.0f;
	//	data.real = data0->real;
	//	data.imag = data0->imag;

	//	data_3000.real+=data.real/size;
	//	data_3000.imag+=data.imag/size;
	//		i++;

	//	if(i==size)
	//	{

	// IQ距离目标值的差值
	err.real = TARGET - data.real;
	err.imag = TARGET - data.imag;

	// IQ距离目标值差值的积分
	err_i.real += err.real;
	err_i.imag += err.imag;

	err_d.real = err.real - err_last.real;
	err_d.imag = err.imag - err_last.imag;

#ifdef SINGLE_PID	
	// 输出的参数值
	data_out.real = (Kp * err.real + Ki * err_i.real + Kd * err_d.real);
	data_out.imag = (Kp * err.imag + Ki * err_i.imag + Kd * err_d.imag);
//	if(i==0)
//	{
//		data_t[1] = 256;
//		data_t[2] = 0;
//		data_t[3] = 256;
//		rs232_send_msg((rt_uint8_t *)data_t, 4*4);
//		i=1;
//	}

#else
	switch(pidstate)
	{
		case state1:
			// 输出的参数值
			data_out.real = (Kp * err.real + Ki * err_i.real + Kd * err_d.real);
			data_out.imag = (Kp * err.imag + Ki * err_i.imag + Kd * err_d.imag);
			if((err.imag <= 0.05f && err.imag>=-0.05f) || (err.real <= 0.05f && err.imag >= -0.05f))
			{
				pidstate = state3;
				Ki = Ki3;
//				send(Ki);
			}
			else if((err.imag <= 0.1f && err.imag>=-0.1f) || (err.real <= 0.1f && err.imag >= -0.1f))
			{
				pidstate = state2;
				Ki = Ki2;
//				send(Ki);
			}
			break;
		case state2:
			// 输出的参数值
			data_out.real = (Kp * err.real + Ki * err_i.real + Kd * err_d.real);
			data_out.imag = (Kp * err.imag + Ki * err_i.imag + Kd * err_d.imag);	
			if(((err.imag <= 0.05f && err.imag>=-0.05f) || (err.real <= 0.05f && err.imag >= -0.05f))) 
			{
				pidstate = state3;
				Ki = Ki3;
//				send(Ki);
			}
			else if((err.imag <= 0.1f && err.imag>=-0.1f) || (err.real <= 0.1f && err.imag >= -0.1f))
			{
				pidstate = state2;
				Ki = Ki2;
			}
			else{
				pidstate = state1;
				Ki = Ki1;
//				send(Ki);
			}
			break;
		case  state3:
			// 输出的参数值
			data_out.real = (Kp * err.real + Ki * err_i.real + Kd * err_d.real);
			data_out.imag = (Kp * err.imag + Ki * err_i.imag + Kd * err_d.imag);	
			
			if(!((err.imag <= 0.05f && err.imag>=-0.05f) || (err.real <= 0.05f && err.imag >= -0.05f))) 
			{
				pidstate = state2;
				Ki = Ki2;
//				send(Ki);
			}
			else if(!((err.imag <= 0.1f && err.imag>=-0.1f) || (err.real <= 0.1f && err.imag >= -0.1f)))
			{
				pidstate = state1;
				Ki = Ki1;
//				send(Ki);
			}
			else{
				pidstate = state3;
				Ki = Ki3;
			}
			break;
		default:
			break;
	}
#endif

	if (data_out.real >= 4095)
		data_out.real -= 4095;
	else if (data_out.real <= -4095)
		data_out.real -= -4095;

	if (data_out.imag >= 4095)
		data_out.imag -= 4095;
	else if (data_out.imag <= -4095)
		data_out.imag -= -4095;

//	i = 0;
	// 根据输出的参数值，获取相位与幅值（浮点型）
	amp = sqrtf(data_out.real * data_out.real + data_out.imag * data_out.imag);
	ph = atan2(data_out.imag, data_out.real) * 180.0f / 3.1415926f * (4096 / 360);
	amplitude = (int32_t)(amp + 0.5f);
	phase = (int32_t)(ph + 0.5f);
	if (phase < 0)
		phase += 360;
	if (amplitude >= 4095)
		amplitude = amplitude-4095;
	dac7811_set_shift_reg(0x00, (DAC7811_LOAD | ((amplitude)&0xFFF))); // 控制幅值
	ad983x_set_phase(AD983X_REG_PHASE0, phase, 0x00);				   // 控制相位
	//		data_out.real = 0;
	//		data_out.imag = 0;
	//		err_i.real = 0;
	//	 err_i.imag = 0;
//}

	err_last.imag = err.imag;
	err_last.real = err.real;
}

/**
 * @brief：   学习流程
 * @param：   探测数据结构体、学习数据结构体
 * @param:    经过对探测数据包中的探测数据
 *                      计算相位与旋转矩阵
 *                      进行数据相位旋转
 *                      能量检测/获取最值
 * @retval:   NONE
 **/
void learning_process(Para_Detection *paradetection, Para_Learning *paralearning)
{
	//	int i=0;
	uint8_t init = paralearning->init; // 重新学习
	if (detection_ob == 0x02)		   // 金属+探测物
	{
		paralearning->phase = get_phase(paradetection->matrix_pack->pData, init);										 // 获取相位(弧度转角度)
		get_matrix_rotation(paralearning->phase, paralearning->matrix_rotation->pData);									 // 获取旋转矩阵
		phase_rotation((paradetection->matrix_pack), (paralearning->matrix_rotation), (paradetection->matrix_rotation)); // 旋转相位
		matrix_range((paradetection->matrix_rotation), LEARNING);														 // 获取最值
		
	}
	else if (detection_ob == 0x01) // 金属	//获取数据峰值
	{
		matrix_range((paradetection->matrix_pack), LEARNING);
	}
}

/**
 * @brief:		探测处理流程
 * @param：		根据探测数据结构体
 * 					获取相位，方便之后作为学习参数
 * 					相位旋转，将产品相位消除
 * 					获取最值，进行数值比较
 * 						在区域内部表示未有金属
 * 							将相位与旋转矩阵/最值存储在学习中
 *						超出区域表示存在金属
 *  						剔除功能，使用环形buf进行剔除功能筛选
 */
void detection_process(Para_Detection *paradetection, Para_Learning *paralearning)
{
	float32_t test_f32_ERR[SIZE] = {0};
	//	float32_t phase;
	if (detection_ob == 0x02) // 金属+探测物
	{
		// 相位旋转
		phase_rotation((paradetection->matrix_pack), (paralearning->matrix_rotation), (paradetection->matrix_rotation));
//		arm_lms_f32_test1(paradetection->matrix_rotation->pData, paradetection->matrix_rotation->pData,wavei,test_f32_ERR);
		// 获取最值
		matrix_range((paradetection->matrix_rotation), DETECTION);
	}
	else if (detection_ob == 0x01) // 金属
	{
		matrix_range((paradetection->matrix_pack), DETECTION);
	}

	// 比较最值
	//	if (paradetection->amp_max > paralearning->amp_max*TIMES || paradetection->amp_min*TIMES < paralearning->amp_min) // 超出范围：有金属异物
	if (is_range(paradetection, paralearning))
	{
		DetectCtrl(state_detect);
		// 进行剔除操作
		//		reject_opt.tick = rt_tick_get();
		//		add_reject(reject_opt.tick + TICK_DELAY);
	}
	else // 在范围内：无金属异物
	{
		//		if(detection_ob == 0x02)
		//		{
		//			// 获取相位
		//			phase = get_phase(paradetection->data, paralearning->init);
		//			paralearning->phase = phase;
		//			get_matrix_rotation(phase, paralearning->matrix_rotation.pData);
		//			paralearning->amp_max.real = paradetection->amp_max.real;
		//			paralearning->amp_min.real = paradetection->amp_min.real;
		//			paralearning->amp_max.imag = paradetection->amp_max.imag;
		//			paralearning->amp_min.imag = paradetection->amp_min.imag;
		//		}
	}

	// 获取旋转矩阵
	// 传递到学习结构体中
}

rt_uint8_t data_pack(complex data)
{
	static int i = 0;
	static int index = 0;
	//	paradetection.data[i++] = data.real;
	//	paradetection.data[i++] = data.imag;
//	if(index >= FROM && index < TO)
//	{
//		paradetection.matrix_pack->pData[i++] = data.real;
//		paradetection.matrix_pack->pData[i++] = data.imag;		
//	}
//	index++;
//	if(index == TO) 
//	{
//		index = 0;
//		i=0;
//		return 1;
//	}
	
	if(index<SIZE)
	{
		paradetection.matrix_pack->pData[i++] = data.real;
		paradetection.matrix_pack->pData[i++] = data.imag;	
		index++;
	}
	if(index==SIZE)
	{
		i=0;
		index = 0;
//		paradetection.matrix_pack->pData[i++] = data.real;
//		paradetection.matrix_pack->pData[i++] = data.imag;	
		return 1;
	}
	return 0;	
}

// 输出相位(角度值)
float32_t get_phase(float32_t *pack, rt_uint8_t init)
{
	static float32_t phase;
	static complex sum = {0};
	static complex sum_2 = {0};
	float32_t k = 0;
	static int times = 0;
	int i = 0;
	if (init) // 初始化
	{
		phase = 0;
		sum.real = 0;
		sum.imag = 0;
		sum_2.real = 0;
		sum_2.imag = 0;
		times++;
	}
	for (i = 0; i < SIZE; i++)
	{
		sum.real += pack[2 * i]/8388608*2.5;					 // x
		sum.imag += pack[2 * i + 1]/8388608*2.5;				 // y
		sum_2.real += pack[2 * i] /8388608*2.5* pack[2 * i + 1]/8388608*2.5; // x*y
		sum_2.imag += pack[2 * i] /8388608*2.5* pack[2 * i]/8388608*2.5;	 // x*x
	}
	times++;
	k = (sum.imag * sum_2.imag - sum_2.real * sum.real) / (SIZE * i * sum_2.imag - sum.real * sum.real);

	phase = atan2(k, 1); // 输出为弧度制

	return phase * 180.0f / 3.1415926f; // 弧度制转角度
}

// 输出旋转矩阵
void get_matrix_rotation(float32_t phase, float32_t *matrix_data)
{
	double ph = 0.0f;
	ph = phase * PI / 180.0f; // 角度值转弧度
	/*	real imag
	 *			cos			-sin
	 *			sin			 cos
	 */
	matrix_data[0] = cos(ph);
	matrix_data[1] = -sin(ph);
	matrix_data[2] = sin(ph);
	matrix_data[3] = cos(ph);
}
// 旋转矩阵
void phase_rotation(arm_matrix_instance_f32 *src1, arm_matrix_instance_f32 *src2, arm_matrix_instance_f32 *dst)
{
	arm_mat_mult_f32(src1, src2, dst);
}


//阈值范围的判断
//需要保证探测阈值范围比学习获取的阈值范围要大
//
rt_uint8_t is_range(Para_Detection *paradetection, Para_Learning *paralearning)
{
	uint8_t max_real = 0, max_imag = 0, min_real = 0, min_imag = 0;
	uint8_t threshold_min = 0, threshold_max = 0;
	
//#ifndef THRESHOLD
	
	max_real = (paradetection->amp_max.real > paralearning->amp_max.real);
	max_imag = (paradetection->amp_max.imag > paralearning->amp_max.imag);	
	min_real = (paradetection->amp_min.real < paralearning->amp_min.real);
	min_imag = (paradetection->amp_min.imag < paralearning->amp_min.imag);
//	return (max_real || max_imag || min_real || min_imag);
	
//#else	
	threshold_max = (paradetection->threshold_max > paralearning->threshold_max);
	threshold_min = (paradetection->threshold_min < paralearning->threshold_min);
	
//	return threshold_max ;
//#endif	
	
	return max_real || max_imag || min_real || min_imag;// || threshold_max || threshold_min;
}

// 获取数据包中最值
void matrix_range(arm_matrix_instance_f32 *src, StateWork statework)
{
	uint32_t index = 0;
	float32_t width = 0;
	float32_t height = 0;
	float32_t thre = 0;
	float32_t threshold = 0;
	if (statework == LEARNING)
	{
		paralearning.amp_max.real = src->pData[index * 2];
		paralearning.amp_max.imag = src->pData[index * 2 + 1];
		paralearning.amp_min.real = src->pData[index * 2];
		paralearning.amp_min.imag = src->pData[index * 2 + 1];
		paralearning.threshold_max = sqrt(src->pData[index * 2]*src->pData[index * 2]+src->pData[index * 2+1]*src->pData[index * 2+1]);
		paralearning.threshold_min = sqrt(src->pData[index * 2]*src->pData[index * 2]+src->pData[index * 2+1]*src->pData[index * 2+1]);
		for (index = 0; index < SIZE; index++)
		{
			threshold = sqrt(src->pData[index * 2]*src->pData[index * 2]+src->pData[index * 2+1]*src->pData[index * 2+1]);
			if (paralearning.threshold_max<threshold)
			{
				paralearning.threshold_max = threshold;
			}
			if (paralearning.threshold_min>threshold)
			{
				paralearning.threshold_min = threshold;
			}
			if (paralearning.amp_max.real < src->pData[index * 2])
			{
				paralearning.amp_max.real = src->pData[index * 2];
			}
			if (paralearning.amp_max.imag < src->pData[index * 2 + 1])
			{
				paralearning.amp_max.imag = src->pData[index * 2 + 1];
			}
			if (paralearning.amp_min.real > src->pData[index * 2])
			{
				paralearning.amp_min.real = src->pData[index * 2];
			}
			if (paralearning.amp_min.imag > src->pData[index * 2 + 1])
			{
				paralearning.amp_min.imag = src->pData[index * 2 + 1];
			}
		}
		//扩大阈值范围
		paralearning.width = paralearning.amp_max.real - paralearning.amp_min.real;
		paralearning.height = paralearning.amp_max.imag - paralearning.amp_min.imag;
		paralearning.thre = paralearning.threshold_max - paralearning.threshold_min;
		
		paralearning.amp_max.real = (paralearning.amp_max.real + paralearning.amp_min.real+paralearning.width*multiple)/2;
		paralearning.amp_max.imag = (paralearning.amp_max.imag + paralearning.amp_min.imag+paralearning.height*multiple)/2;
		paralearning.amp_min.real = (paralearning.amp_max.real + paralearning.amp_min.real-paralearning.width*multiple)/2;
		paralearning.amp_min.imag = (paralearning.amp_max.imag + paralearning.amp_min.imag-paralearning.height*multiple)/2;
		paralearning.threshold_max = (paralearning.threshold_max + paralearning.threshold_min + paralearning.thre*multiple)/2;
		paralearning.threshold_min = (paralearning.threshold_max + paralearning.threshold_min - paralearning.thre*multiple)/2;
		
		
	}
	else
	{
		paradetection.amp_max.real = src->pData[index * 2];
		paradetection.amp_max.imag = src->pData[index * 2 + 1];
		paradetection.amp_min.real = src->pData[index * 2];
		paradetection.amp_min.imag = src->pData[index * 2 + 1];
		paradetection.threshold_max = sqrt(src->pData[index * 2]*src->pData[index * 2]+src->pData[index * 2+1]*src->pData[index * 2+1]);
		paradetection.threshold_min = sqrt(src->pData[index * 2]*src->pData[index * 2]+src->pData[index * 2+1]*src->pData[index * 2+1]);
		
		for (index = 0; index < SIZE; index++)
		{
			if (paradetection.threshold_max<threshold)
			{
				paradetection.threshold_max = threshold;
			}
			if (paradetection.threshold_min>threshold)
			{
				paradetection.threshold_min = threshold;
			}
			if (paradetection.amp_max.real < src->pData[index * 2])
			{
				paradetection.amp_max.real = src->pData[index * 2];
			}
			if (paradetection.amp_max.imag < src->pData[index * 2 + 1])
			{
				paradetection.amp_max.imag = src->pData[index * 2 + 1];
			}
			if (paradetection.amp_min.real > src->pData[index * 2])
			{
				paradetection.amp_min.real = src->pData[index * 2];
			}
			if (paradetection.amp_min.imag > src->pData[index * 2 + 1])
			{
				paradetection.amp_min.imag = src->pData[index * 2 + 1];
			}
		}
	}

	//	arm_cmplx_mag_f32(src->pData, dst_data, SIZE); // 取模值
	//	if (statework == DETECTION)
	//	{
	//		arm_max_f32(dst_data, SIZE, &(paradetection.amp_max), &index);
	//		arm_min_f32(dst_data, SIZE, &(paradetection.amp_min), &index);
	//	}
	//	else if (statework == LEARNING)
	//	{
	//		arm_max_f32(dst_data, SIZE, &(paralearning.amp_max), &index);
	//		arm_min_f32(dst_data, SIZE, &(paralearning.amp_min), &index);
	//	}
}


#define BLOCK_SIZE           SIZE/* 调用一次arm_lms_norm_f32处理的采样点个数 */
#define NUM_TAPS             20      /* 滤波器系数个数 */

static float32_t testInput_f32_50Hz_200Hz[SIZE]={0}; /* 源波形 */
static float32_t testInput_f32_REF[SIZE]={0};        /* 参考波形 */
static float32_t test_f32_ERR[SIZE]={0};             /* 误差数据 */
static float32_t testOutput[SIZE]={0};               /* 滤波后的输出 */
static float32_t lmsStateF32[BLOCK_SIZE + NUM_TAPS - 1]={0};        /* 状态缓存，大小numTaps + blockSize - 1 */
static float32_t lmsCoeffs32[NUM_TAPS] = {0};                       /* 滤波器系数 */

uint32_t blockSize = BLOCK_SIZE; 
uint32_t numBlocks = SIZE/BLOCK_SIZE;                /* 需要调用arm_lms_norm_f32的次数 */

void arm_lms_f32_test1(float32_t * testInput_f32_50Hz_200Hz, float32_t *testOutput, float32_t *testInput_f32_REF, float32_t *test_f32_ERR)
{
	static int flag = 1;
	uint32_t i;
	float32_t  *inputF32, *outputF32, *inputREF, *outputERR;
	arm_lms_norm_instance_f32 lmsS={0};
	
	if(flag)
	{
		/* 如果是实时性的滤波，仅需清零一次 */
		memset(lmsCoeffs32,0,sizeof(lmsCoeffs32));
		memset(lmsStateF32,0,sizeof(lmsStateF32));	
		flag = 0;
	}
	
	/* 初始化输入输出缓存指针 */
	inputF32 = (float32_t *)&testInput_f32_50Hz_200Hz[0]; /* 原始波形 */
	outputF32 = (float32_t *)&testOutput[0];              /* 滤波后输出波形 */
	inputREF = (float32_t *)&testInput_f32_REF[0];        /* 参考波形 */  
	outputERR = (float32_t *)&test_f32_ERR[0];            /* 误差数据 */  
	
	/* 归一化LMS初始化 */
	arm_lms_norm_init_f32 (&lmsS,                         /* LMS结构体 */
						   NUM_TAPS,                      /* 滤波器系数个数 */
							(float32_t *)&lmsCoeffs32[0], /* 滤波 */ 
							&lmsStateF32[0],              /* 滤波器系数 */
							0.1,                    	  /* 步长 */
							blockSize);                   /* 处理的数据个数 */

	/* 实现LMS自适应滤波，这里每次处理1个点 */
	for(i=0; i < numBlocks; i++)
	{
		
		arm_lms_norm_f32(&lmsS, /* LMS结构体 */
						inputF32 + (i * blockSize),   /* 输入数据 */
						inputREF + (i * blockSize),   /* 输出数据 */
						outputF32 + (i * blockSize),  /* 参考数据 */
						outputERR + (i * blockSize),  /* 误差数据 */
						blockSize);					  /* 处理的数据个数 */

	}

}

// 数据处理初始化
int data_process_init(void)
{
	// 低通滤波与高通滤波初始化
	filter_init(&filter_hpf_r);
	filter_init(&filter_hpf_i);
	filter_init(&filter_lpf_r);
	filter_init(&filter_lpf_i);
	filter_init(&filter_lpf_bj_r);
	filter_init(&filter_lpf_bj_i);

	paradetection.matrix_pack = &matrix_pack_detection;
	paradetection.matrix_rotation = &matrix_rotation_detection;
	paralearning.matrix_rotation = &matrix_rotation_learning;

	// 数据包与旋转矩阵初始化
	arm_mat_init_f32((paralearning.matrix_rotation), 2, 2, (float32_t *)paralearning.data);
	arm_mat_init_f32((paradetection.matrix_pack), SIZE, 2, (float32_t *)paradetection.data);
	arm_mat_init_f32((paradetection.matrix_rotation), SIZE, 2, (float32_t *)paradetection.data_rotation);

	// 互斥锁创建
	dp_sem = rt_sem_create("dp_sem", 1, RT_IPC_FLAG_FIFO);
	bj_sem = rt_sem_create("bj_sem", 1, RT_IPC_FLAG_FIFO);

	// 数据处理任务创建
	rt_thread_t thread_DP = rt_thread_create("DP", (void (*)(void *parameter))data_process, RT_NULL, 2048, 2, 2);
	if (thread_DP != RT_NULL)
	{
		rt_thread_startup(thread_DP);
	}
#ifdef BP
	// 数据处理任务创建
	rt_thread_t thread_BP = rt_thread_create("BP", (void (*)(void *parameter))balance_process, RT_NULL, 2048, 3, 2);
	if (thread_BP != RT_NULL)
	{
		rt_thread_startup(thread_BP);
	}
#endif

	//	// 数据处理任务创建
	//	rt_thread_t thread_TC = rt_thread_create("TC", (void (*)(void *parameter))reject_process, RT_NULL, 2048, 1, 0);
	//	if (thread_TC != RT_NULL)
	//	{
	//		rt_thread_startup(thread_TC);
	//	}
	return RT_EOK;
}

INIT_APP_EXPORT(data_process_init);
