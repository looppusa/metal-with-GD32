#ifndef _DATA_PROCESS_H_
#define _DATA_PROCESS_H_

#include "arm_math.h"
#include "board.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "filter.h"
#include "gv.h"
#include "rs232_link.h"

#define FILTER
//#define BJ	//平衡调节输出
#define BP	//双路 平衡调节/采集判断

#define SINGLE_PID

//#define THRESHOLD

#define TIMES 3
#define TARGET (0.2f)



int data_process_init(void);
void data_process(void);
void balance_process(void);
void balance_adjust(complex *data0);

void learning_process(Para_Detection *paradetection, Para_Learning *paralearning);
//void learning_process(Para_Detection *paradetection, Para_Learning *paralearning, uint8_t init);
void detection_process(Para_Detection *paradetection, Para_Learning *paralearning);

rt_uint8_t data_pack(complex data);                                                                              // 数据打包
void phase_rotation(arm_matrix_instance_f32 *src1, arm_matrix_instance_f32 *src2, arm_matrix_instance_f32 *dst); // 相位旋转

float32_t get_phase(float32_t *pack,rt_uint8_t init);
void get_matrix_rotation(float32_t phase, float32_t *matrix_data);
void matrix_range(arm_matrix_instance_f32 *src,StateWork statework);
rt_uint8_t is_range(Para_Detection *paradetection, Para_Learning *paralearning);


void arm_lms_f32_test1(float32_t * testInput_f32_50Hz_200Hz, float32_t *testOutput, float32_t *testInput_f32_REF, float32_t *test_f32_ERR);

#endif
