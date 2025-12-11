/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-03     Haojie       the first version
 */


#ifndef __DAC7811_H
#define __DAC7811_H

#ifdef __cplusplus
extern "C" {
#endif

#include <rthw.h>
#include <rtthread.h>

/* control bits definition */
#define   DAC7811_NOP                (0x0 << 12)
#define   DAC7811_LOAD               (0x1 << 12)
#define   DAC7811_READ_BACK          (0x2 << 12)
#define   DAC7811_CHAIN_DISABLE      (0x9 << 12)
#define   DAC7811_CLK_RISING         (0xA << 12)
#define   DAC7811_DAC_ZERO           (0xB << 12)
#define   DAC7811_DAC_MID            (0xC << 12)


rt_uint16_t dac7811_set_shift_reg(rt_uint8_t chip, rt_uint16_t data);


#ifdef __cplusplus
}
#endif

#endif /* __DAC7811_H */

/*****END OF FILE****/
