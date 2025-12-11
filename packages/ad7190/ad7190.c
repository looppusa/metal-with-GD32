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
#include "gv.h"
#include "ad7190.h"
#include "arm_math.h"

#include "rs232_link.h"



static struct rt_spi_device *spi_dev;

/* Channels data */
__IO int32_t							ad7190_data[2] = {0};
int32_t             ad7190_buf[2][AD7190_SMP_NUM*2+1+8];
static rt_uint8_t           buf_index = 0;
__IO rt_uint8_t           cnt = 1;

//static uint32_t k = 0;

/**
 * @brief GPIO and SPI Initialization Function
 * @param None
 * @retval None
 */
static void ad7190_hw_init(void)
{
    rt_pin_mode(AD7190_SYNC_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(AD7190_CS0_PIN, PIN_MODE_OUTPUT);

    /* 复位AD7190: 拉低SYNC延时之后再拉高 */
    rt_pin_write(AD7190_SYNC_PIN, PIN_LOW);
    rt_thread_mdelay(2);
    rt_pin_write(AD7190_SYNC_PIN, PIN_HIGH);
    rt_thread_mdelay(100);

    /* 挂载SPI设备 */
    rcu_periph_clock_enable(RCU_GPIOB);
    rt_sw_spi_device_attach("spi4", "spi40", AD7190_CS0_PIN);

    struct rt_spi_configuration cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_MSB;
    cfg.max_hz = 10 * 1000; /* 10K */
    /* 配置SPI */
    spi_dev = (struct rt_spi_device *)rt_device_find("spi40");
    rt_spi_configure(spi_dev, &cfg);
}

// 设置drdy为外部中断
static int ad9710_set_drdy_it(void)
{
    /* 使能中断 */
    rt_pin_irq_enable(AD7190_RDY_PIN, PIN_IRQ_ENABLE);
		return 0;
}

// 设置drdy为spi miso
static int ad9710_set_drdy_spi(void)
{
    /* 除能中断 */
    rt_pin_irq_enable(AD7190_RDY_PIN, PIN_IRQ_DISABLE);
	return 0;
}

//static int ad7190_verify(void)
//{
//    uint8_t buf[3] = {0};

//    // buf[0] = 0x48; //读模式寄存器
//    buf[0] = 0x50; //读配置寄存器

//    rt_spi_send_then_recv(spi_dev, buf, 1, buf, 3);

//    rt_kprintf("Verify ad7190 mode reg: %x, %x, %x\r\n", buf[0], buf[1], buf[2]);
//		return 0;
//}

static int gpio_udelay(rt_uint32_t us)
{
    int i = us * 10; // CPU CLK = 120MHz

    while (i)
    {
        i--;
    }
		return 0;
}

int ad7190_read_spi(uint32_t *ad0, uint32_t *ad1)
{
    uint32_t data_0 = 0;
    uint32_t data_1 = 0;

    int i = 0;
    GPIO_BC(GPIOB) = GPIO_PIN_15; // DIN PB15=LOW
    for (i = 0; i < 32; i++)
    {
        GPIO_BC(GPIOB) = GPIO_PIN_13; // PB13=LOW
        gpio_udelay(2);
        data_0 <<= 1;
        data_1 <<= 1;
        if (GPIO_ISTAT(GPIOB) & (GPIO_PIN_14))
        {
            data_0 |= 0x01;
        }
//        if (GPIO_ISTAT(GPIOC) & (GPIO_PIN_8))
//        {
//            data_1 |= 0x01;
//        }

        GPIO_BOP(GPIOB) = GPIO_PIN_13; // PB13=HIGH
        gpio_udelay(2);
    }

    *ad0 = data_0;
    *ad1 = data_1;
		return data_0;
}
/**
  * @brief      读取ad7190通道数据
  * @param[in]  chd     -   通道号
  * @param[in]  data    -   读数据指针
  * @retval     0-读取失败, 1-读取成功
  */

int ad7190_read_channel(uint8_t chd)
{
//    switch (chd)
//    {
//    case 0:
//		{
////			*data = ad7190_buf[buf_index][cnt*2-1];
//			*data = (int32_t)ad7190_data[1];
////        return 1;
//        break;
//		}			
//    case 1:
//		{
////				*data = ad7190_buf[buf_index][cnt*2-0];
//		*data = (int32_t)ad7190_data[0];
////        return 1;
//        break;			
//		}

//    default:
//        break;
//    }
	
		//通道12 imag
		if(chd == CH_AN1_AN2)	return ad7190_data[1];//q
		//通道34 real
		else if(chd == CH_AN3_AN4) return ad7190_data[0];//r
		else return 1;
}

int ad7190_variance1(uint8_t cnt, int32_t value)
{

		static int r_index1 = 0;
//		static int i_index = 0;
//		static int r_fc = 0;
//		static int i_fc = 0;
		static float32_t data1[60] = {0};
		static float32_t Var1 = 0;
		
		data1[r_index1++] = (float32_t)value;
		if(r_index1 == 60) r_index1 = 0;
		
		
		arm_var_f32(data1,60,&Var1);
		
		return (int)Var1;
}
int ad7190_variance0(uint8_t cnt, int32_t value)
{

		static int r_index0 = 0;
//		static int i_index = 0;
//		static int r_fc = 0;
//		static int i_fc = 0;
		static float32_t data0[60] = {0};
		static float32_t Var0 = 0;
		
		data0[r_index0++] = (float32_t)value;
		if(r_index0 == 60) r_index0 = 0;
		
		
		arm_var_f32(data0,60,&Var0);
		
		return (int)Var0;
}

int ad7190_pack(uint8_t chd, int32_t value)
{
		static int cnt = 1;
	
		switch (chd)	//将数据写入
    {
				case CH_AN1_AN2://蓝色//虚部
						ad7190_buf[buf_index][cnt*2-1] = value;
//				ad7190_buf[buf_index][cnt*2-1] = ad7190_variance1(100, value);
//						ad7190_data[0] = value0;
						break;
				case CH_AN3_AN4://红色//实部
						ad7190_buf[buf_index][cnt*2-0] = value;
//				ad7190_buf[buf_index][cnt*2-0] = ad7190_variance1(100, value);
//						ad7190_buf[buf_index][cnt*2-0] = (int32_t)sqrt(ad7190_variance0(100, value)*ad7190_variance0(100, value)+ad7190_buf[buf_index][cnt*2-1]*ad7190_buf[buf_index][cnt*2-1]);
//						ad7190_data[1] = value0; 
						cnt++;
						break;
				default:
						break;
    }
		
		
		if (cnt == AD7190_SMP_NUM+1)
    {
			ad7190_buf[buf_index][AD7190_SMP_NUM*2+1+0] = paralearning.amp_max.real;
			ad7190_buf[buf_index][AD7190_SMP_NUM*2+1+1] = paralearning.amp_max.imag;
			ad7190_buf[buf_index][AD7190_SMP_NUM*2+1+2] = paralearning.amp_min.real;
			ad7190_buf[buf_index][AD7190_SMP_NUM*2+1+3] = paralearning.amp_min.imag;
			
			ad7190_buf[buf_index][AD7190_SMP_NUM*2+1+4] = paradetection.amp_max.real;
			ad7190_buf[buf_index][AD7190_SMP_NUM*2+1+5] = paradetection.amp_max.imag;
			ad7190_buf[buf_index][AD7190_SMP_NUM*2+1+6] = paradetection.amp_min.real;
			ad7190_buf[buf_index][AD7190_SMP_NUM*2+1+7] = paradetection.amp_min.imag;
		
//			ad7190_buf[buf_index][AD7190_SMP_NUM*2+1+0] = 1;
//			ad7190_buf[buf_index][AD7190_SMP_NUM*2+1+1] = 2;
//			ad7190_buf[buf_index][AD7190_SMP_NUM*2+1+2] = 3;
//			ad7190_buf[buf_index][AD7190_SMP_NUM*2+1+3] = 4;
//			
//			ad7190_buf[buf_index][AD7190_SMP_NUM*2+1+4] = 5;
//			ad7190_buf[buf_index][AD7190_SMP_NUM*2+1+5] = 6;
//			ad7190_buf[buf_index][AD7190_SMP_NUM*2+1+6] = 7;
//			ad7190_buf[buf_index][AD7190_SMP_NUM*2+1+7] = 8;

			rs232_send_msg((rt_uint8_t *)ad7190_buf[buf_index], AD7190_SMP_NUM*8+4+32);
		
        buf_index++;
        if (buf_index > 1) {
            buf_index = 0; 
        }
        cnt = 1;
        ad7190_buf[0][0] = 0x01010040;
        ad7190_buf[1][0] = 0x01010040;//协议帧开头
    }
		return 0;
}

static void ad7190_read_adc(void *args)
{
    /* 连续读取模式下DIN、CS应一直保持为低电平 */
    // uint8_t buf[4] = {0};
    // uint32_t data = 0;
    // uint8_t chd;
    // int32_t data_2;

    // struct rt_spi_message msg;
    // msg.send_buf   = buf;
    // msg.recv_buf   = buf;
    // msg.length     = 4;
    // msg.cs_take    = 0;
    // msg.cs_release = 0;
    // msg.next        = RT_NULL;

    // /* 配置drdy脚为spi */
    // ad9710_set_drdy_spi();

    // rt_spi_transfer_message(spi_dev, &msg);

    // /* 拼接24位有效数据 */
    // data = buf[2];
    // data |= buf[1] << 8;
    // data |= buf[0] << 16;



    uint32_t ad0 = 0;
    uint32_t ad1 = 0;
    uint8_t chd0;
//    uint8_t chd1 = 0;
    int32_t value0 = 0;
//    int32_t value1 = 0;
		
//		rt_interrupt_enter();
		
//		rt_mutex_take(dp_mutex, RT_WAITING_FOREVER);//获取互斥锁

    /* 配置drdy脚为spi */
    ad9710_set_drdy_spi();

    ad7190_read_spi(&ad0, &ad1);

    /* 得到通道号 */
    chd0 = ad0 & 0x07;
    /* AD7190配置为双极性模式时, 输出为偏移二进制码
    * 减去0x00800000转换为二进制补码形式
    */
    value0 = (int32_t)(ad0 >> 8) - 0x00800000;
    
    /* 判断通道号 
    * 然后依次在缓冲区中写入两个通道的数据
    */
    switch (chd0)
    {
		case CH_AN1_AN2:
	//        ad7190_buf[buf_index][cnt*2-1] = value0;
				ad7190_data[1] = value0;//Q
			break;
		case CH_AN3_AN4:
	//        ad7190_buf[buf_index][cnt*2-0] = value0;
	//        cnt++;
				ad7190_data[0] = value0;//I
			break;
		default:
			break;
    }

//    if (cnt == AD7190_SMP_NUM+1)
//    {
//        rs232_send_msg((rt_uint8_t *)ad7190_buf[buf_index], AD7190_SMP_NUM*8+4);
//        buf_index++;
//        if (buf_index > 1) {
//            buf_index = 0;
//        }
//        cnt = 1;
//        ad7190_buf[0][0] = 0x01010040;
//        ad7190_buf[1][0] = 0x01010040;
//    }

//		rt_mutex_release(dp_mutex);//释放互斥锁
		rt_sem_release(dp_sem);
    // 配置drdy为外部中断
    ad9710_set_drdy_it();
//		rt_interrupt_leave();
		
}

static int ad7190_init(void)
{
    uint8_t buf[4];
    uint8_t id = 0;

    /* spi message */
    struct rt_spi_message msg1;

    msg1.send_buf   = buf;
    msg1.recv_buf   = RT_NULL;
    msg1.length     = 4;
    msg1.cs_take    = 0;
    msg1.cs_release = 0;
    msg1.next       = RT_NULL;

    /* AD7190 double buffer */
    ad7190_buf[0][0] = 0x01010040;
    ad7190_buf[0][1] = 0x01010040;

    buf_index = 0;
    cnt = 1;

    /* hardware initialization */
    ad7190_hw_init();
    if (!spi_dev)
    {
        return RT_ERROR;
    }
    /* 复位串行接口：DIN输出40以上个1 */
    rt_spi_send(spi_dev, RT_NULL, 6);
    rt_thread_mdelay(10);
    // while ((id & 0x0F) != 0x04)
    {
        // 读ID寄存器
        buf[0] = 0x60;
        rt_spi_send_then_recv(spi_dev, buf, 1, &id, 1);
        rt_kprintf("Find ad7190: ID =  %x\r\n", id);
    }
    // 写配置寄存器
    buf[0] = 0x10; // 通信寄存器
    buf[1] = 0x00; // CON23-CON16 Chop=1
    buf[2] = 0x03; // CON15-CON8 通道选择 手册表20

    // 0x00:GAIN=1;
    // 0x03:GAIN=8;
    // 0x04:GAIN=16;
    // 0x07:GAIN=128; (GAIN=128时，5V基准时 ADC 输入范围为+-39.06mV)
    buf[3] = 0x00; // CON7-CON0

    rt_spi_send(spi_dev, buf, 4);

    rt_thread_mdelay(5);
    // 写模式寄存-零电平校准
    // buf[0] = 0x08;
    // buf[1] = 0x88; // MR23-MR16 10001000 内部零电平校准
    // buf[2] = 0x03; // MR15-MR8 使用sinc4滤波器 REJ60 = 0
    // /* 由于每次转换新的通道时内部滤波器需要复位，多通道采样率降低4倍。*/
    // buf[3] = 0xFF; // MR7-MR0 fs(MR9-MR9) = 0x01,即每通道数据输出速率为4804/480=10sps
    // rt_pin_write(AD7190_CS1_PIN, PIN_LOW);
    // rt_spi_send(spi_dev, buf, 4);
    // rt_pin_write(AD7190_CS1_PIN, PIN_HIGH);
    // rt_thread_mdelay(3000);

    // // // 写模式寄存-满量程校准
    // buf[0] = 0x08;
    // buf[1] = 0xA8; // MR23-MR16 10001000 内部满量程校准
    // buf[2] = 0x03; // MR15-MR8 使用sinc4滤波器 REJ60 = 0
    // /* 由于每次转换新的通道时内部滤波器需要复位，多通道采样率降低4倍。*/
    // buf[3] = 0xFF; // MR7-MR0 fs(MR9-MR9) = 0x01,即每通道数据输出速率为4804/480=10sps
    // rt_pin_write(AD7190_CS1_PIN, PIN_LOW);
    // rt_spi_send(spi_dev, buf, 4);
    // rt_pin_write(AD7190_CS1_PIN, PIN_HIGH);
    // rt_thread_mdelay(3000);

    // 写 ADC 模式寄存
    buf[0] = 0x08;
    buf[1] = 0x1C; // MR23-MR16 00011100 使能DAT_SET 状态寄存器的内容和数据器存器一起输出 使用内部时钟并输出到MCLK2
    buf[2] = 0x00; // MR15-MR8 使用sinc4滤波器 REJ60 = 0
    /* 由于每次转换新的通道时内部滤波器需要复位，多通道采样率降低4倍。*/
    buf[3] = 8; // MR7-MR0 fs(MR9-MR9) = 0x01,即每通道数据输出速率为4804/10/2=240sps
    rt_pin_write(AD7190_CS0_PIN, PIN_LOW); 
    rt_spi_transfer_message(spi_dev, &msg1);
    // rt_spi_send(spi_dev, buf, 4);
    rt_pin_write(AD7190_CS0_PIN, PIN_HIGH); 

    rt_thread_mdelay(500);

    // ad7190_verify();

    /* 设置数据寄存器连续读取 */
    buf[0] = 0x5C;
    rt_spi_send(spi_dev, buf, 1);

    rt_thread_mdelay(5);
    /* ADC配置完成后需要将CS置LOW，转换完成时DRDY才会拉低 */
    rt_pin_write(AD7190_CS0_PIN, PIN_LOW);

    /* 绑定中断回调函数 */
    rt_pin_attach_irq(AD7190_RDY_PIN, PIN_IRQ_MODE_FALLING, ad7190_read_adc, RT_NULL);

    /* 复位AD7190: 拉低SYNC延时之后再拉高 */
    rt_pin_write(AD7190_SYNC_PIN, PIN_LOW);
    rt_thread_mdelay(2);
    rt_pin_write(AD7190_SYNC_PIN, PIN_HIGH);

    /* 使能中断 */
    ad9710_set_drdy_it();

    return RT_EOK;
}

INIT_DEVICE_EXPORT(ad7190_init); /* a7190外设驱动初始化 */
