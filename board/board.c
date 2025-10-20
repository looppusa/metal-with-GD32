/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-12-18     BruceOu      first implementation
 */
#include <stdint.h>
#include <rthw.h>
#include <rtthread.h>
#include <board.h>

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler */
    /* User can add his own implementation to report the HAL error return state */
    while (1)
    {
    }
    /* USER CODE END Error_Handler */
}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{
    SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);
    NVIC_SetPriority(SysTick_IRQn, 0);
}

/**
 * This is the timer interrupt service routine.
 *
 */
void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();


    /* leave interrupt */
    rt_interrupt_leave();
}

/**
 * This function will initial GD32 board.
 */
void rt_hw_board_init()
{
    /* NVIC Configuration */
#define NVIC_VTOR_MASK 0x3FFFFF80
#ifdef VECT_TAB_RAM
    /* Set the Vector Table base location at 0x10000000 */
    SCB->VTOR = (0x10000000 & NVIC_VTOR_MASK);
#else /* VECT_TAB_FLASH  */
    /* Set the Vector Table base location at 0x08000000 */
    SCB->VTOR = (0x08000000 & NVIC_VTOR_MASK);
#endif

    SystemClock_Config();

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

#ifdef RT_USING_CONSOLE
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif

#ifdef BSP_USING_SDRAM
    rt_system_heap_init((void *)EXT_SDRAM_BEGIN, (void *)EXT_SDRAM_END);
#else
    rt_system_heap_init((void *)HEAP_BEGIN, (void *)HEAP_END);
#endif
    // Config special pins
    rcu_periph_clock_enable(RCU_AF);
    gpio_pin_remap_config(GPIO_SWJ_NONJTRST_REMAP, ENABLE);

    // Config all CS pins to high level
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);

    gpio_bit_set(SPI0_CS0_GPIO_Port, SPI0_CS0_Pin);
    gpio_bit_set(SPI0_CS1_GPIO_Port, SPI0_CS1_Pin);
    gpio_bit_set(SPI0_CS2_GPIO_Port, SPI0_CS2_Pin);
    gpio_bit_set(SPI0_CS3_GPIO_Port, SPI0_CS3_Pin);
    gpio_bit_set(SPI0_CS4_GPIO_Port, SPI0_CS4_Pin);
    gpio_bit_set(SPI0_CS5_GPIO_Port, SPI0_CS5_Pin);
    gpio_bit_set(SPI1_CS0_GPIO_Port, SPI1_CS0_Pin);
    gpio_bit_set(SPI1_CS1_GPIO_Port, SPI1_CS1_Pin);
    gpio_bit_set(UART_DE1_GPIO_Port, UART_DE1_Pin);

    gpio_init(SPI0_CS0_GPIO_Port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SPI0_CS0_Pin);
    gpio_init(SPI0_CS1_GPIO_Port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SPI0_CS1_Pin);
    gpio_init(SPI0_CS2_GPIO_Port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SPI0_CS2_Pin);
    gpio_init(SPI0_CS3_GPIO_Port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SPI0_CS3_Pin);
    gpio_init(SPI0_CS4_GPIO_Port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SPI0_CS4_Pin);
    gpio_init(SPI0_CS5_GPIO_Port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SPI0_CS5_Pin);
    gpio_init(SPI1_CS0_GPIO_Port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SPI1_CS0_Pin);
    gpio_init(SPI1_CS1_GPIO_Port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SPI1_CS1_Pin);
    gpio_init(UART_DE1_GPIO_Port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, UART_DE1_Pin);
}

/*@}*/
