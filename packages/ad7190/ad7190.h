/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-03     Haojie       the first version
 */


#ifndef __AD7190_H
#define __AD7190_H

#ifdef __cplusplus
extern "C" {
#endif

#include <rthw.h>
#include <rtthread.h>
#include <stdint.h>
	
/* Channels definition */
#define CH_AN1_AN2 0x0
#define CH_AN3_AN4 0x1
#define CH_TEMP_SENS 0x2
#define CH_AN2_AN2 0x3
#define CH_AN1_COM 0x4
#define CH_AN2_COM 0x5
#define CH_AN3_COM 0x6
#define CH_AN4_COM 0x7


#define AD7190_SCK_PIN GET_PIN(B, 13)
#define AD7190_DOUT0_PIN GET_PIN(B, 14)
#define AD7190_CS0_PIN GET_PIN(B, 12)

#define AD7190_SYNC_PIN GET_PIN(C, 6)
#define AD7190_RDY_PIN AD7190_DOUT0_PIN

extern rt_mutex_t dp_mutex;	//互斥锁
extern rt_sem_t dp_sem;	//互斥锁

#define  AD7190_SMP_NUM     10        /* 两个通道的数据 */
extern int32_t ad7190_buf[2][AD7190_SMP_NUM*2+1+8];

int ad7190_read_spi(uint32_t *ad0, uint32_t *ad1);
int ad7190_read_channel(uint8_t chd);
int ad7190_pack(uint8_t chd, int32_t value);
int ad7190_variance1(uint8_t cnt, int32_t value);
int ad7190_variance0(uint8_t cnt, int32_t value);

#ifdef __cplusplus
}
#endif

#endif /* __AD7190_H */

/*****END OF FILE****/
