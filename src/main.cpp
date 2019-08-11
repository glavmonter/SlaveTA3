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
//#include "SEGGER_RTT.h"
#include "hardware.h"
#include "main.h"
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "trcRecorder.h"
#include "maintask.h"


SemaphoreHandle_t xMutexSegger;

/* Private typedef */
/* Private define  */


/* Private macro */
/* Private variables */
/* Private function prototypes */
static void prvHardwareSetup();
portTASK_FUNCTION_PROTO(vTestTask1, pvParameters);
portTASK_FUNCTION_PROTO(vTestTask2, pvParameters);
portTASK_FUNCTION_PROTO(vConsumer, pvParameters);
portTASK_FUNCTION_PROTO(vProducer, pvParameters);

/* Private functions */

#if configGENERATE_RUN_TIME_STATS == 1
//#error Configure timers first!!!

static void setupRunTimeStats();
portTASK_FUNCTION_PROTO(vStatTask, pvParameters);
volatile unsigned long ulRunTimeStatsClock;
#endif

#define APPLICATION_ADDRESS 	0x1FFFF000


typedef void (*pFunction)(void);

pFunction Jump_To_Application;
uint32_t JumpAddress;

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void) {
	SystemCoreClockUpdate();

//	xMutexSegger = xSemaphoreCreateRecursiveMutex();
	SEGGER_RTT_ConfigUpBuffer(0, "UP", NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
	SEGGER_RTT_ConfigDownBuffer(0, "DOWN", NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

#if configUSE_TRACE_FACILITY == 1
	SEGGER_RTT_ConfigUpBuffer(1, "traceUP", NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
	SEGGER_RTT_ConfigDownBuffer(1, "traceDOWN", NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

	vTraceEnable(TRC_START);
#endif
	SEGGER_RTT_printf(0, "Hello\n");

	prvHardwareSetup();

	if ((BKP_ReadBackupRegister(BKP_DR1) == 0x1234) and
		(BKP_ReadBackupRegister(BKP_DR5) == 0xCE94)) {

//		SEGGER_RTT_printf(0, "Boot\n");

		BKP_WriteBackupRegister(BKP_DR1, 0x0000);
		BKP_WriteBackupRegister(BKP_DR5, 0x0000);

		/* Start Bootloader */
		StartExternalApp(0x1FFFF000);
	}

#if configGENERATE_RUN_TIME_STATS == 1
TaskHandle_t handle = NULL;
	setupRunTimeStats();
	xTaskCreate(vStatTask, "Stat", configMINIMAL_STACK_SIZE*3, NULL, tskIDLE_PRIORITY, &handle);
	assert_param(handle);
#endif

	MainTask &mt = MainTask::Instance();
	return mt.run();
}


static void prvHardwareSetup() {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |
						   RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP | RCC_APB1Periph_PWR, ENABLE);
	PWR_BackupAccessCmd(ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

/*  */
GPIO_InitTypeDef GPIO_InitStructure;
	PORTA_DATA4_OUT = PORTA_DATA5_OUT = PORTA_DATA6_OUT = PORTA_DATA7_OUT = 1;
	PORTB_DATA4_OUT = PORTB_DATA5_OUT = PORTB_DATA6_OUT = PORTB_DATA7_OUT = 1;

	GPIO_InitStructure.GPIO_Pin = PORTA_DATA_OUT_PINS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PORTA_DATA_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PORTB_DATA_OUT_PINS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PORTB_DATA_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PORTA_DATA_IN_PINS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(PORTA_DATA_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PORTB_DATA_IN_PINS;
	GPIO_Init(PORTB_DATA_PORT, &GPIO_InitStructure);

	CLIMATE_HEATER_EN = 0;
	GPIO_InitStructure.GPIO_Pin = CLIMATE_HEATER_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CLIMATE_HEATER_PORT, &GPIO_InitStructure);

	CLIMATE_COOLER_EN = 0;
	GPIO_InitStructure.GPIO_Pin = CLIMATE_COOLER_PIN;
	GPIO_Init(CLIMATE_COOLER_PORT, &GPIO_InitStructure);
}

QueueHandle_t xQueue = NULL;


portTASK_FUNCTION(vConsumer, pvParameters) {
traceString ss = xTraceRegisterString("Consumer_str");
char buf[16];

	for (;;) {
		if (xQueue != NULL) {
			if (xQueueReceive(xQueue, buf, 200) == pdTRUE) {
				vTracePrintF(ss, buf);
			}
		}

		vTaskDelay(200);
	}
}


portTASK_FUNCTION(vProducer, pvParameters) {
char buf[16];
uint32_t index = 0;

	xQueue = xQueueCreate(10, 16);
	vTraceSetQueueName(xQueue, "ProdQueue");

	for (;;) {
		snprintf(buf, 16, "DATA: %d", index);
		index++;
		xQueueSend(xQueue, buf, portMAX_DELAY);
	}
}

portTASK_FUNCTION(vTestTask1, pvParameters) {
traceString ss = xTraceRegisterString("Task1");

	for (;;) {
		PORTA_DATA4_OUT ^= 1;
		vTaskDelay(250);
		PORTA_DATA5_OUT ^= 1;
		vTaskDelay(250);
		PORTA_DATA6_OUT ^= 1;
		vTaskDelay(250);
		PORTA_DATA7_OUT ^= 1;
		vTaskDelay(250);

		vTracePrintF(ss, "Ping");
	}
}

portTASK_FUNCTION(vTestTask2, pvParameters) {
traceString ss = xTraceRegisterString("Task2");

	for (;;) {
		PORTB_DATA4_OUT ^= 1;
		vTaskDelay(250);
		PORTB_DATA5_OUT ^= 1;
		vTaskDelay(250);
		PORTB_DATA6_OUT ^= 1;
		vTaskDelay(250);
		PORTB_DATA7_OUT ^= 1;
		vTaskDelay(250);

		vTracePrintF(ss, "Ping");
	}
}

#if configGENERATE_RUN_TIME_STATS == 1

static void setupRunTimeStats() {
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = (uint16_t)(SystemCoreClock/20000) - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM4, ENABLE);
}

void TIM4_IRQHandler() {
	if (TIM_GetITStatus(TIM4, TIM_IT_Update)) {
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		ulRunTimeStatsClock++;
	}
}


char buffer[48*20];
portTASK_FUNCTION(vStatTask, pvParameters) {
(void)pvParameters;
	for (;;) {
		vTaskDelay(10000);
		_log("FreeHeap: %d\n", xPortGetFreeHeapSize());

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
//	SEGGER_RTT_printf(0, "Stack: %s", name);
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

