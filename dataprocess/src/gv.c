#include "gv.h"

uint8_t in_detection_zone = 0;

//״̬
StateWork statework = IDLE;
State_Detect state_detect = JY;
uint8_t detection_ob = 0;
uint8_t state_motor = 0;
uint8_t state_balance = 0;

Para_Learning paralearning = {0};
Para_Detection paradetection = {0};
Reject_Buf reject_opt;
uint8_t data_insert = 0;
float32_t multiple = 1.2f;
int times_learning = 0;
PID_State pidstate = state1;


float32_t wavei[SIZE] = {0};
float32_t waveq[SIZE] = {0};

complex noise_background_max = {0};
complex noise_background_min = {0};

arm_matrix_instance_f32 matrix_rotation_learning={2,2,paralearning.data};
arm_matrix_instance_f32 matrix_pack_detection={SIZE,2,paradetection.data};
arm_matrix_instance_f32 matrix_rotation_detection={SIZE,2,paradetection.data_rotation};

void reject_init(void)
{
    reject_opt.size = 100;
    reject_opt.p_current = 0;
    reject_opt.p_reject = 0;
}

void add_reject(uint32_t tick)
{
    reject_opt.rejectbuf[reject_opt.p_current] = tick;
    reject_opt.p_current++;
    if (reject_opt.p_current == reject_opt.size)
        reject_opt.p_current = 0;
}

uint8_t rm_reject(uint32_t tick)
{
    if (tick <= reject_opt.rejectbuf[reject_opt.p_reject]+10 && tick >= reject_opt.rejectbuf[reject_opt.p_reject]-10)
    {
			reject_opt.rejectbuf[reject_opt.p_reject] = 0;
        reject_opt.p_reject++;
        if (reject_opt.p_reject == reject_opt.size)
            reject_opt.p_reject = 0;
        return 1;
    }
    return 0;
}
