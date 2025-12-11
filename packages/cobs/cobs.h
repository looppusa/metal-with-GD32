#ifndef __COBS_H
#define __COBS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <rtthread.h>

#define COBS_DECODE_BUF_SIZE 64     /* 命令数据的最大长度 */
#define COBS_ENCODE_BUF_SIZE 512    //一包数据最大512字节

typedef struct crc_lut
{
    rt_uint8_t hi_lut;
    rt_uint8_t lo_lut;
} crc_lut_t;

typedef struct cobs_decode_handler
{
    rt_uint8_t buffer[COBS_DECODE_BUF_SIZE];
    rt_uint8_t *dst;
    rt_uint8_t code;
    rt_uint8_t block;
    crc_lut_t crc;
    /* cmd call back */
    rt_err_t (*cobs_complete)(void *buffer, rt_uint16_t size);

} cobs_decode_handler_t;

typedef struct cobs_encode_handler
{
    rt_uint8_t buffer[COBS_ENCODE_BUF_SIZE]; 
    rt_uint8_t *dst;
    rt_uint8_t code;
    rt_uint8_t *code_ptr;
    crc_lut_t crc;
} cobs_encode_handler_t;


void cobs_decode_init(cobs_decode_handler_t *handler,
                                rt_err_t (*cobs_complete)(void *buffer, rt_uint16_t size));
void cobs_decode_frame(const rt_uint8_t *src, rt_size_t length, cobs_decode_handler_t *handler);


void cobs_encode_reset(cobs_encode_handler_t *handler);
void cobs_encode_frame(rt_uint8_t *src, rt_uint16_t length, cobs_encode_handler_t *handler);
void cobs_encode_end_frame(cobs_encode_handler_t *handler);

#ifdef __cplusplus
}
#endif

#endif /* __COBS_H */

/*****END OF FILE****/
