#include <rtthread.h>
#include <rtdevice.h>
#include "rs232_link.h"
#include "cobs.h"
#include "struct_pack.h"
#include "ad983x.h"
#include "dac7811.h"
#include "data_process.h"
#include "gv.h"


#define TAG_SPI_CMD 0x40
#define TAG_MOTOR 0x41
#define TAG_DDS 0x42
#define TAG_DAC 0x43
#define TAG_DPOT 0x44
#define TAG_LEARN	0x45
#define TAG_DETECT_MOD 0x46
#define TAG_THRESHOLD	0x47
#define TAG_BALANCE  0x48
#define TAG_DETECTION_SEL 0x49

#define SAMPLE_UART_NAME "uart0"

/* 串口设备句柄 */
static rt_device_t serial;
/* 用于接收消息的信号量 */
static struct rt_semaphore rx_sem;
/* 用于发送消息的信号量 */
static struct rt_semaphore tx_sem;

/* COBS解码相关 */
struct cobs_decode_handler cobs_handler;

/* 接收数据回调函数 */
static rt_err_t uart_rx_ind(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断，调用此回调函数，然后释放接收信号量 */
    rt_err_t result = rt_sem_release(&rx_sem);
    return RT_EOK;
}

/* 发送数据回调函数 */
static rt_err_t uart_tx_cmplt(rt_device_t dev, void *buffer)
{
    /* 串口发送完数据后产生中断，调用此回调函数，然后释放发送信号量 */
    rt_sem_release(&tx_sem);
    return RT_EOK;
}

/* 消息帧处理函数 */
static rt_err_t rs232_process_frame(void *buffer, rt_uint16_t size)
{
    rt_uint8_t *ptr = buffer;
    rt_uint8_t tag = *ptr++;	//数组0
    rt_uint8_t seq = *ptr++;	//数组1


    switch (tag)
    {
    case TAG_SPI_CMD:
    {
        /* spi parameter set */
        rt_uint8_t spi_chip = unpack_uint8(&ptr);
        rt_uint8_t length = unpack_uint8(&ptr);

        RT_ASSERT(spi_chip < 1);

//        rt_kprintf("spi parameter set = %x, %x\n", spi_chip, length);
        break;
    }


    case TAG_MOTOR:
    {
				/* motor parameter set */
				rt_uint8_t motor_state = unpack_uint8(&ptr);
				state_motor = motor_state;
				switch(motor_state)
				{
					case 0x00:
						//电机停止
						MotorCtrl(STOP);
						break;
					case 0x01:
						//电机启动正转
						MotorCtrl(START);
						break;
					case 0x02:
						//电机反转
						MotorCtrl(ROTATE);
						break;
					default:
						break;
				}
        break;
    }
    case TAG_DDS:
    {
        /* dds parameter set */
        rt_uint8_t dds_num = unpack_uint8(&ptr);
        rt_uint32_t dds_freq = unpack_uint32(&ptr);
        rt_uint16_t dds_phase = unpack_uint16(&ptr);

        RT_ASSERT(dds_num < 3);
//        ad983x_set_frequency(AD983X_REG_FREQ0, dds_freq, dds_num);
        ad983x_set_phase(AD983X_REG_PHASE0, dds_phase, dds_num);
        break;
    }

    case TAG_DAC:
    { 
        /* dac parameter set */
        rt_uint8_t dac_num = unpack_uint8(&ptr);
        rt_uint16_t dac_value = unpack_uint16(&ptr);

        if (dac_num > 0)
				{
					return RT_ERROR;
				}
        dac7811_set_shift_reg(dac_num, (DAC7811_LOAD | (dac_value & 0xFFF)));

//        rt_kprintf("dac parameter set = %x, %x\n", dac_num, dac_value);
        break;
    }
		case TAG_LEARN:
		{
			rt_uint8_t learn = unpack_uint8(&ptr);
			if(learn)
			{
//				in_detection_zone = 1;
				statework = LEARNING;	
				paralearning.init = 1;				
			}

			else
//				in_detection_zone = 0;
				statework = DETECTION;
//				detector_send_signal(&detector, SIGNAL_LEARN);
//			detector_switch_to_learn(&detector);//将探测器转换为学习模式
			break;
		}
		case TAG_DETECT_MOD:
		{
			rt_uint8_t detect = unpack_uint8(&ptr);
//			statework = DETECTION;
			switch(detect)
			{
				case 0x00:
					state_detect = JY;
					break;
				case 0x01:
					state_detect = BJ;
					break;
				case 0x02:
					state_detect = TC;
					break;
				case 0x03:
					state_detect = TZ;
					break;
				default:
					break;
			}
//			detector.reg.detect_mode = detect;//切换探测剔除形式
			break;
		}
		case TAG_THRESHOLD:
		{
			rt_uint8_t state = unpack_uint8(&ptr);
			
			
			paralearning.amp_max.real = paralearning.amp_max.real*2-paralearning.width*multiple;
			paralearning.amp_max.imag = paralearning.amp_max.imag*2-paralearning.height*multiple;
			paralearning.amp_min.real = paralearning.amp_min.real*2+paralearning.width*multiple;
			paralearning.amp_min.imag = paralearning.amp_min.imag*2+paralearning.height*multiple;
			paralearning.threshold_max = paralearning.threshold_max*2-paralearning.thre*multiple;
			paralearning.threshold_min = paralearning.threshold_min*2+paralearning.thre*multiple;
			
			multiple = (float32_t)state/100.0;
			paralearning.init = 1;	
			
			
		paralearning.amp_max.real = (paralearning.amp_max.real+paralearning.width*multiple)/2;
		paralearning.amp_max.imag = (paralearning.amp_max.imag+paralearning.height*multiple)/2;
		paralearning.amp_min.real = (paralearning.amp_min.real-paralearning.width*multiple)/2;
		paralearning.amp_min.imag = (paralearning.amp_min.imag-paralearning.height*multiple)/2;
		paralearning.threshold_max = (paralearning.threshold_max + paralearning.thre*multiple)/2;
		paralearning.threshold_min = (paralearning.threshold_min - paralearning.thre*multiple)/2;
			
			
			break;
		}
		case TAG_BALANCE:
		{
			rt_uint8_t state = unpack_uint8(&ptr);
			
			if(state)
				state_balance = 0;
			else
				state_balance = 1;	
			break;
		}
		case TAG_DETECTION_SEL://探测物品选择
		{
			rt_uint8_t data = unpack_uint8(&ptr);
			switch(data)
			{
				case 0x00:
					detection_ob = 0;
					break;
				case 0x01:	//金属
					detection_ob = 1;
					break;
				case 0x02:	//金属+物品
					detection_ob = 2;
					break;
				default:
					break;
			}
		}

    default:
        break;
    }

    return RT_EOK;
}

static rt_uint8_t rx_buffer[RT_SERIAL_RB_BUFSZ];

static void rs232_rx_thread_entry(void *parameter)
{
    rt_uint32_t rx_length = 0;

    while (1)
    {
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        rt_sem_control(&rx_sem, RT_IPC_CMD_RESET, RT_NULL);
        /* 从串口读取全部数据*/
        rx_length = rt_device_read(serial, 0, rx_buffer, RT_SERIAL_RB_BUFSZ);
        rt_kprintf("get serial length = %d\n", rx_length);

        /* 通过串口设备 serial 输出读取到的消息 */

        if (rx_length != 0)
        {
            /* 解码一段缓冲区数据，并执行消息处理函数 */
            cobs_decode_frame(rx_buffer, rx_length, &cobs_handler);
            
        }
    }
}

/******* 数据发送任务 *******/

/* 串口接收消息结构 */
struct rs232_tx_msg
{
    rt_uint8_t *buffer;
    rt_size_t size;
};

/* 消息缓冲区 */
static char rs232_tx_msg_pool[32];

/* 消息队列控制块 */
static struct rt_messagequeue rs232_tx_msg_queue;

/* 数据编码处理块 */
struct cobs_encode_handler encode_handler;

/* 将缓冲区地址和长度写入消息队列，然后发送任务从消息队列获取缓冲区地址和数据
 * 并从串口发出
 */
rt_err_t rs232_send_msg(rt_uint8_t *buffer, rt_size_t size)
{
    RT_ASSERT(size < 512-3);
    struct rs232_tx_msg msg;
    rt_err_t result;
    msg.buffer = buffer;
    msg.size = size;

    result = rt_mq_send(&rs232_tx_msg_queue, &msg, sizeof(msg));
    if ( result == -RT_EFULL)
    {
        /* 消息队列满 */
        rt_kprintf("tx message queue full！\n");
    }
    return result;
}

static void rs232_tx_thread_entry(void *parameter)
{
    struct rs232_tx_msg msg;
    rt_err_t result;

    while (1)
    {
        rt_memset(&msg, 0, sizeof(msg));
        /* 从消息队列中读取消息*/
        result = rt_mq_recv(&rs232_tx_msg_queue, &msg, sizeof(msg), RT_WAITING_FOREVER);
        if (result == RT_EOK)
        {
            /* 从缓冲区读取数据并使用COBS进行编码、以及CRC校验 */
            cobs_encode_reset(&encode_handler);
            cobs_encode_frame(msg.buffer, msg.size, &encode_handler);
            cobs_encode_end_frame(&encode_handler);

            rt_size_t length = encode_handler.dst - encode_handler.buffer;
            rt_device_write(serial, 0, encode_handler.buffer, length);
        }
    }
}




/* rs232 数据接收发送初始化函数 */
static int rs232_link(void)
{
    /* 初始化配置参数 */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    cobs_decode_init(&cobs_handler, rs232_process_frame);

    /* step1：查找串口设备 */
    serial = rt_device_find(SAMPLE_UART_NAME);

    /* step2：修改串口配置参数 */
    config.baud_rate = BAUD_RATE_115200; //修改波特率为
    config.data_bits = DATA_BITS_8;      //数据位 8
    config.stop_bits = STOP_BITS_1;      //停止位 1
    config.bufsz = RT_SERIAL_RB_BUFSZ;   //默认缓冲区大小
    config.parity = PARITY_NONE;         //无奇偶校验位

    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);

    /* step4：打开串口设备。以DMA接收及DMA发送模式打开串口设备 */
    rt_device_open(serial, RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX);

    /* 初始化信号量 */
    rt_sem_init(&rx_sem, "u0_rx_sem", 0, RT_IPC_FLAG_PRIO);
    rt_sem_init(&tx_sem, "u0_tx_sem", 0, RT_IPC_FLAG_PRIO);

    /* 初始化消息队列 */
    rt_mq_init(&rs232_tx_msg_queue, "rs232_tx_mq",
               rs232_tx_msg_pool,                   /* 存放消息的缓冲区 */
               sizeof(struct rs232_tx_msg),         /* 一条消息的最大长度 */
               sizeof(rs232_tx_msg_pool),           /* 存放消息的缓冲区大小 */
               RT_IPC_FLAG_PRIO);                   /* 如果有多个线程等待，按照先来先得到的方法分配消息 */

    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial, uart_rx_ind);
    rt_device_set_tx_complete(serial, uart_tx_cmplt); 

    /* 创建 serial 线程 */
    rt_thread_t thread = rt_thread_create("rs232_rx", (void (*)(void *parameter))rs232_rx_thread_entry, RT_NULL, 1024, 2, 10);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }

    thread = RT_NULL;
    thread = rt_thread_create("rs232_tx", (void (*)(void *parameter))rs232_tx_thread_entry, RT_NULL, 2048, 2, 11);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    return RT_EOK;
}

INIT_PREV_EXPORT(rs232_link); /* rs232_link 线程初始化 */
