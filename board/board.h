/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-12-18     BruceOu      first implementation
 */
#ifndef __BOARD_H__
#define __BOARD_H__

#include "gd32f30x.h"
#include "drv_usart.h"
#include "drv_gpio.h"


#include "gd32f30x_exti.h"

#define EXT_SDRAM_BEGIN    (0xC0000000U) /* the begining address of external SDRAM */
#define EXT_SDRAM_END      (EXT_SDRAM_BEGIN + (32U * 1024 * 1024)) /* the end address of external SDRAM */

// <o> Internal SRAM memory size[Kbytes] <8-64>
//  <i>Default: 64
#ifdef __ICCARM__
// Use *.icf ram symbal, to avoid hardcode.
extern char __ICFEDIT_region_RAM_end__;
#define GD32_SRAM_END          &__ICFEDIT_region_RAM_end__
#else
#define GD32_SRAM_SIZE         48
#define GD32_SRAM_END          (0x20000000 + GD32_SRAM_SIZE * 1024)
#endif

#ifdef __ARMCC_VERSION
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define HEAP_BEGIN    (&Image$$RW_IRAM1$$ZI$$Limit)
#elif __ICCARM__
#pragma section="HEAP"
#define HEAP_BEGIN    (__segment_end("HEAP"))
#else
extern int __bss_end;
#define HEAP_BEGIN    (&__bss_end)
#endif

#define HEAP_END          GD32_SRAM_END

/* SPI0 CS*/

#define SPI0_CS0_GPIO_Port  GPIOA
#define SPI0_CS0_Pin        GPIO_PIN_4

#define SPI0_CS1_GPIO_Port  GPIOB
#define SPI0_CS1_Pin        GPIO_PIN_1

#define SPI0_CS2_GPIO_Port  GPIOB
#define SPI0_CS2_Pin        GPIO_PIN_0

#define SPI0_CS3_GPIO_Port  GPIOC
#define SPI0_CS3_Pin        GPIO_PIN_5

#define SPI0_CS4_GPIO_Port  GPIOC
#define SPI0_CS4_Pin        GPIO_PIN_4

#define SPI0_CS5_GPIO_Port  GPIOC
#define SPI0_CS5_Pin        GPIO_PIN_9

#define SPI1_CS0_GPIO_Port  GPIOB
#define SPI1_CS0_Pin        GPIO_PIN_12

#define SPI1_CS1_GPIO_Port  GPIOC
#define SPI1_CS1_Pin        GPIO_PIN_7

/* UART DE */
#define UART_DE1_GPIO_Port  GPIOA
#define UART_DE1_Pin        GPIO_PIN_12

#define UART_DE0_GPIO_Port  GPIOA
#define UART_DE0_Pin        GPIO_PIN_11

#define RM_IN0_GPIO_Port  GPIOA
#define RM_IN0_Pin        GPIO_PIN_2

/* CPU IN */
#define CPU_IN0_GPIO_Port  GPIOD
#define CPU_IN0_Pin        GPIO_PIN_2

#define CPU_IN1_GPIO_Port  GPIOC
#define CPU_IN1_Pin        GPIO_PIN_12

#define CPU_IN2_GPIO_Port  GPIOB
#define CPU_IN2_Pin        GPIO_PIN_4

#define CPU_IN3_GPIO_Port  GPIOB
#define CPU_IN3_Pin        GPIO_PIN_3

#define CPU_IN4_GPIO_Port  GPIOB
#define CPU_IN4_Pin        GPIO_PIN_5

#define CPU_IN5_GPIO_Port  GPIOB
#define CPU_IN5_Pin        GPIO_PIN_6

#define CPU_IN6_GPIO_Port  GPIOB
#define CPU_IN6_Pin        GPIO_PIN_7

#define CPU_IN7_GPIO_Port  GPIOB
#define CPU_IN7_Pin        GPIO_PIN_8

/* CPU OUT */
#define CPU_OUT0_GPIO_Port  GPIOA
#define CPU_OUT0_Pin        GPIO_PIN_0

#define CPU_OUT1_GPIO_Port  GPIOC
#define CPU_OUT1_Pin        GPIO_PIN_3

#define CPU_OUT2_GPIO_Port  GPIOC
#define CPU_OUT2_Pin        GPIO_PIN_2

#define CPU_OUT3_GPIO_Port  GPIOC
#define CPU_OUT3_Pin        GPIO_PIN_1

#define CPU_OUT4_GPIO_Port  GPIOC
#define CPU_OUT4_Pin        GPIO_PIN_0

#define CPU_OUT5_GPIO_Port  GPIOC
#define CPU_OUT5_Pin        GPIO_PIN_13

#define CPU_OUT6_GPIO_Port  GPIOC
#define CPU_OUT6_Pin        GPIO_PIN_14

#define CPU_OUT7_GPIO_Port  GPIOC
#define CPU_OUT7_Pin        GPIO_PIN_15

#endif

