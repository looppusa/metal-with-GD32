/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-07     Haojie       the first version
 */


#include <stdio.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_gpio.h"
#include "drv_spi.h"
#include "drv_soft_spi.h"

#include "dac7811.h"



#define DAC7811_SYNC_0_PIN      GET_PIN(B, 1)
// #define DAC7811_SYNC_1_PIN      GET_PIN(B, 0)

static struct rt_spi_device *spi_dev[1];


/**
  * @brief GPIO and SPI Initialization Function
  * @param None
  * @retval None
  */
static void dac7811_hw_init(void)
{
    /* 挂载SPI设备 */
    rcu_periph_clock_enable(RCU_GPIOB);
    rt_sw_spi_device_attach("spi5", "spi51", DAC7811_SYNC_0_PIN);
    // rt_sw_spi_device_attach("spi3", "spi32", DAC7811_SYNC_1_PIN);
 
    struct rt_spi_configuration cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_2 | RT_SPI_MSB;
    cfg.max_hz = 10 * 1000 *1000;                           /* 10M */
    /* 配置SPI */
    spi_dev[0] = (struct rt_spi_device *)rt_device_find("spi51");
    // spi_dev[1] = (struct rt_spi_device *)rt_device_find("spi32");

    rt_spi_configure(spi_dev[0], &cfg);
    // rt_spi_configure(spi_dev[1], &cfg);
}

rt_uint16_t dac7811_set_shift_reg(rt_uint8_t chip, rt_uint16_t data)
{
    rt_uint8_t buf[2]={0};
		rt_uint16_t recv_data;
	
    buf[1] = (rt_uint8_t)(data & 0xFF);
    buf[0] = (rt_uint8_t)(data >> 8 & 0xFF);

    struct rt_spi_device *_spi_dev;

    _spi_dev = spi_dev[chip];

    if (_spi_dev)
    {
        rt_spi_transfer(_spi_dev, buf, buf, 2);
    }
		
	  recv_data =  (rt_uint16_t)(buf[1] | buf[0] << 8);
    return recv_data;
}


static int ad7811_init(void)
{
    /* hardware initialization */
    dac7811_hw_init();
    dac7811_set_shift_reg(0, (rt_uint16_t)(DAC7811_LOAD | (0x00 & 0xFFF)));
    // dac7811_set_shift_reg(1, (rt_uint16_t)(DAC7811_LOAD | (0x555 & 0xFFF)));
		return 0;
}

/* 
 * @brief   DAC控制命令: cmd_name chip_number dac_value
 * @example dac7811 0 4095
 * @example dac7811 1 2048
 */ 

static void dac7811_set(int argc, char**argv)
{
    int value;
    rt_uint8_t chip;
		rt_uint16_t data;
    if (argc < 3)
    {
        rt_kprintf("Please input'dac7811 <0|1> value'\n");
        return;
    }

    // 判断chip是否存在
    chip = atoi(argv[1]);
    if (chip > 1)
    {
        rt_kprintf("chip number must less than 2\n");
        return;
    }

		
    // 获取寄存器及器数值
    value   = atoi(argv[2]);
    if (value < 0) {
        value = 0;
    } else if (value > 4095){
        value = 4095;
    }
    dac7811_set_shift_reg(chip, (rt_uint16_t)(DAC7811_LOAD | (value & 0xFFF)));
		dac7811_set_shift_reg(chip, (rt_uint16_t)(DAC7811_READ_BACK));
		data = dac7811_set_shift_reg(chip, (rt_uint16_t)(DAC7811_NOP));
		rt_kprintf("DAC7811 READ BACK DATA: %d\n", data);
    return;
}

INIT_DEVICE_EXPORT(ad7811_init);  /* dac7811外设驱动初始化 */
MSH_CMD_EXPORT(dac7811_set, dac7811 cmd sample: dac7811 0 4096);
