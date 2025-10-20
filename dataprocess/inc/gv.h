#ifndef _GV_H_
#define _GV_H_

#include <board.h>
#include <rtthread.h>
#include "arm_math.h"
#include "detectctrl.h"
#include "filter.h"

#define TICK_DELAY	1000

//#define THRESHOLD (1.2f)
#define FROM 80
#define TO	250
#define SIZE (300)
#define TIMES_LEARNING 5

typedef enum{
	LEARNING = 0u,
	DETECTION,
	IDLE
}StateWork;

typedef struct{
	rt_uint8_t init;
	float32_t phase;		//相位
	float32_t width;
	float32_t height;
	float32_t thre;
	complex amp_max;	//幅值最大值
	complex amp_min;	//幅值最小值
	float32_t threshold_max;	//能量最大值
	float32_t threshold_min;	//能量最小值
	float32_t data[4];	//旋转矩阵具体数据
	arm_matrix_instance_f32 *matrix_rotation;
}Para_Learning;

typedef struct{
	float32_t phase;
	complex amp_max;	//幅值最大值
	complex amp_min;	//幅值最小值
	float32_t threshold_max;	//能量最大值
	float32_t threshold_min;	//能量最小值
	float32_t data[SIZE*2];
	float32_t data_rotation[SIZE*2];
	arm_matrix_instance_f32 *matrix_pack;
	arm_matrix_instance_f32 *matrix_rotation;
}Para_Detection;

typedef struct{
	uint32_t rejectbuf[100];
	uint8_t p_current;
	uint8_t p_reject;
	uint8_t size;
	uint32_t tick;	//设置为当前时间，判断与buf时间是否一致
}Reject_Buf;


typedef enum{
	state1 = 0u,
	state2,
	state3
}PID_State;

extern State_Detect state_detect;													//探测状态
extern StateWork statework;															//工作状态
extern Para_Learning paralearning;											//自学习参数
extern Para_Detection paradetection;										//探测数据
extern Reject_Buf reject_opt;														//剔除操作
extern uint8_t in_detection_zone;
extern uint8_t detection_ob;														//探测物选择
extern uint8_t state_motor;
extern uint8_t state_balance;
extern uint8_t data_insert;
extern arm_matrix_instance_f32 matrix_rotation_learning;		//旋转矩阵
extern arm_matrix_instance_f32 matrix_pack_detection;				//采集数据矩阵
extern arm_matrix_instance_f32 matrix_rotation_detection;		//相位旋转后的数据
extern complex noise_background_max;												//底噪获取阈值
extern complex noise_background_min;												//底噪获取阈值
extern PID_State pidstate;																	//PID 参数调整
extern float32_t multiple;																	//阈值扩大倍数
extern int times_learning;
extern float32_t wavei[SIZE];
extern float32_t waveq[SIZE];

void reject_init(void);
void add_reject(uint32_t tick);
uint8_t rm_reject(uint32_t tick);

#endif
