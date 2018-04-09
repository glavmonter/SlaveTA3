/**
  ******************************************************************************
  * @file    include/maintask.h
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    21 сентября 2016
  * @brief   Реализация класса главного потока
  ******************************************************************************
  * @attention
  * Всем очко!!
  *
  * <h2><center>&copy; 2016 ФГУП "18 ЦНИИ" МО РФ</center></h2>
  ******************************************************************************
  */

#include <cstdlib>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <SEGGER_RTT.h>
#include "hardware.h"
#include "maintask.h"


static void TimerCallback(TimerHandle_t xTimer) {
SemaphoreHandle_t sem = static_cast<SemaphoreHandle_t>(pvTimerGetTimerID(xTimer));
	xSemaphoreGive(sem);
}


/**
 * @brief Конструктор.
 *
 * Создание таска ОСРВ с минимальным приоритетом (tskIDLE_PRIORITY)
 */
MainTask::MainTask() {
	SEGGER_RTT_printf(0, "Free: %d\n", xPortGetFreeHeapSize());
	xTaskCreate(task_main, "Main", configTASK_MAIN_STACK, this, configTASK_MAIN_PRIORITY, &handle);
}

/**
 * \brief Включение планировщика ОСРВ
 */
int MainTask::run() {
	vTaskStartScheduler();
	return 0;
}

const uint8_t wake_in[] = {0xC0, 0x85, 0x06, 0x07, 0x20, 0xDB, 0xDC, 0x22, 0x23, 0xF5, 0xDB, 0xDD, 0x05, 0x29};

/**
 * \brief Рабочие цикл таска.
 *
 * Рабочий цикл MainTask. Создание дочерних тасков и обслуживание связей между ними
 */
void MainTask::task() {
	_log("FreeHeap: %d\n", xPortGetFreeHeapSize());
	Init();
	_log("Start Timer\n");

	xTimerStart(xTimer, 5);

uint8_t received_byte = 0;

	for (;;) {
		QueueSetHandle_t event = xQueueSelectFromSet(xQueueSet, portMAX_DELAY);
		if (event == xTimerSemaphore) {
			xSemaphoreTake(event, 0);

		} else if (event == xQueueUsartRx) {
			xQueueReceive(event, &received_byte, 0);
			_log("Received: 0x%02X\n", received_byte);

			if (m_pWake->ProcessInByte(received_byte) == Wake::STA_READY) {
				if (m_pWake->GetCommand() == CMD_INFO) {
					strcpy((char *)m_pWake->TxData, "Hello");
					m_pWake->ProcessTx(0x01, m_pWake->GetCommand(), sizeof("Hello")-1);
				}
				_log("Wake receive data\n");
			}

		} else {
			_log("!!!Pizda!!!\n");
		}
	}
}


void MainTask::Init() {
	portENTER_CRITICAL();
		InitHardware();

		m_pWake = new Wake(0x02, m_pUSART);
		assert_param(m_pWake);

		xTimerSemaphore = xSemaphoreCreateBinary();
		assert_param(xTimerSemaphore);
		xSemaphoreTake(xTimerSemaphore, 0);

		xTimer = xTimerCreate("Main", 1000, pdTRUE, xTimerSemaphore, TimerCallback);
		assert_param(xTimer);

		xQueueUsartRx = xQueueCreate(20, sizeof(uint8_t));
		assert_param(xQueueUsartRx);

		xQueueSet = xQueueCreateSet(1 + 20);
		assert_param(xQueueSet);

		xQueueAddToSet(xTimerSemaphore, xQueueSet);
		xQueueAddToSet(xQueueUsartRx, xQueueSet);
	portEXIT_CRITICAL();
}


void MainTask::InitHardware() {
GPIO_InitTypeDef GPIO_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = SERIAL_USB_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SERIAL_USB_RX_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SERIAL_USB_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SERIAL_USB_TX_PORT, &GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_2;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(m_pUSART, &USART_InitStructure);

	USART_ITConfig(m_pUSART, USART_IT_RXNE, ENABLE);
	USART_Cmd(m_pUSART, ENABLE);
}


void USART1_IRQHandler() {
BaseType_t xHigherPriorityTaskWoken = pdFALSE;
MainTask &mt = MainTask::Instance();

	if (USART_GetITStatus(mt.m_pUSART, USART_IT_RXNE) != RESET) {
		uint8_t rx_data = mt.m_pUSART->DR;
		xQueueSendFromISR(mt.xQueueUsartRx, &rx_data, &xHigherPriorityTaskWoken);
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
