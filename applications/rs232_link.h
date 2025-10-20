#ifndef __RS232_LINK_H
#define __RS232_LINK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <rtthread.h>
#include "motor.h"
rt_err_t rs232_send_msg(rt_uint8_t *buffer, rt_size_t size);

#ifdef __cplusplus
}
#endif

#endif /* __RS232_LINK_H */

/*****END OF FILE****/
