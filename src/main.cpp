/**
  ******************************************************************************
  * @file    src/main.cpp
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    10 января 2018
  * @brief   Начало всего
  ******************************************************************************
  * @attention
  * Всем очко!!
  *
  * <h2><center>&copy; 2018 ООО "РПС"</center></h2>
  ******************************************************************************
  */

/**
 * \mainpage Test main page
 *
 * Просто заглавная страница
 * \section Процессор
 * В проекте используется процеесор STM32F103VB со следующими параметрами:
 * <table>
 * <tr><td>Ядро <td>Cortex-M3
 * <tr><td>FLASH <td>128 kB
 * <tr><td>RAM <td>20 kB
 * <tr><td> Fcpu <td>72 MHz
 * <tr><td> Fapb1 <td> 36 MHz
 * <tr><td> Fapb2 <td> 72 MHz
 * <tr><td> Fext <td> 8 MHz
 * </table>
 */


/* Includes */
#include <stddef.h>
#include <stdio.h>
#include "stm32f10x.h"
#include "SEGGER_RTT.h"
#include "main.h"
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "tlsf.h"


SemaphoreHandle_t xMutexSegger;

/* Private typedef */
/* Private define  */


/* Private macro */
/* Private variables */
static char pool_ram[configTOTAL_HEAP_SIZE] __attribute__ ((aligned(8)));
/* Private function prototypes */
/* Private functions */

#if configGENERATE_RUN_TIME_STATS == 1
#error Configure timers first!!!

static void setupRunTimeStats();
portTASK_FUNCTION_PROTO(vStatTask, pvParameters);
volatile unsigned long ulRunTimeStatsClock;
#endif


/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void) {
	SystemCoreClockUpdate();
	init_memory_pool(configTOTAL_HEAP_SIZE, pool_ram);

	xMutexSegger = xSemaphoreCreateRecursiveMutex();
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
	SEGGER_RTT_printf(0, "Hello\n");


TaskHandle_t handle = NULL;

#if configGENERATE_RUN_TIME_STATS == 1
	setupRunTimeStats();
	xTaskCreate(vStatTask, "Stat", configMINIMAL_STACK_SIZE*3, NULL, tskIDLE_PRIORITY, &handle);
	assert_param(handle);
#endif

	vTaskStartScheduler();
	for (;;) {
	}
	return 0;
}


#if configGENERATE_RUN_TIME_STATS == 1

static void setupRunTimeStats() {
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = (uint16_t)(SystemCoreClock/20000) - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM7, ENABLE);
}

void TIM7_IRQHandler() {
	if (TIM_GetITStatus(TIM7, TIM_IT_Update)) {
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		ulRunTimeStatsClock++;
	}
}


char buffer[48*20];
portTASK_FUNCTION(vStatTask, pvParameters) {
(void)pvParameters;
	for (;;) {
		vTaskDelay(10000);
		portENTER_CRITICAL();
			vTaskGetRunTimeStats(buffer);
			SEGGER_RTT_printf(0, "\n===============RUN TIME====================\n");
			SEGGER_RTT_printf(0, buffer);
			SEGGER_RTT_printf(0,   "===========================================\n\n");
		portEXIT_CRITICAL();
	}
}

#endif


void vApplicationStackOverflowHook(TaskHandle_t handle, char *name) {
(void)handle;
	SEGGER_RTT_printf(0, "Stack: %s", name);
	for (;;){}
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line) {
	/* User can add his own implementation to report the file name and line number,
		ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1) {}
}
#endif

/*
 * Minimal __assert_func used by the assert() macro
 * */
extern "C" void __assert_func(const char *file, int line, const char *func, const char *failedexpr) {
	while(1) {}
}

/*
 * Minimal __assert() uses __assert__func()
 * */
extern "C" void __assert(const char *file, int line, const char *failedexpr) {
	__assert_func (file, line, NULL, failedexpr);
}

