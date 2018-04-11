/**
  ******************************************************************************
  * @file    src/Wiegand.cpp
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    12 марта 2018
  * @brief   Реализация класса Wiegand
  ******************************************************************************
  * @attention
  * Всем очко!!
  *
  * <h2><center>&copy; 2018 ООО "РПС"</center></h2>
  ******************************************************************************
  */
#include <string.h>
#include "SEGGER_RTT.h"
#include "hardware.h"
#include "Wiegand.h"


Wiegand *wiegand_ch1 = NULL;
Wiegand *wiegand_ch2 = NULL;


static void vTimerCallback(TimerHandle_t pxTimer) {
SemaphoreHandle_t sem = static_cast<SemaphoreHandle_t>(pvTimerGetTimerID(pxTimer));
	xSemaphoreGive(sem);
}


Wiegand::Wiegand(WiegandChannel channel, QueueHandle_t result_queue) :
	xWiegandQueue(result_queue), m_eChannel(channel)
{
	InitHardware(channel);

	if (channel == WiegandChannel::Channel_1) {
		xTaskCreate(task_wiegand, "Wieg_1", configTASK_WIEGAND_STACK, this, configTASK_WIEGAND_PRIORITY, &handle);
		wiegand_ch1 = this;
	}
	else if (channel == WiegandChannel::Channel_2) {
		xTaskCreate(task_wiegand, "Wieg_2", configTASK_WIEGAND_STACK, this, configTASK_WIEGAND_PRIORITY, &handle);
		wiegand_ch2 = this;
	}
	else {
		_log("Select Channel!!!\n");
		assert_param(0);
	}

	xSemaphoreIRQ = xSemaphoreCreateBinary();
	assert_param(xSemaphoreIRQ);
	xSemaphoreTake(xSemaphoreIRQ, 0);

	xSemaphoreTimerTimeout = xSemaphoreCreateBinary();
	assert_param(xSemaphoreTimerTimeout);
	xSemaphoreTake(xSemaphoreTimerTimeout, 0);

	xTimer = xTimerCreate("TimerWieg", 20, pdFALSE, xSemaphoreTimerTimeout, vTimerCallback);
	assert_param(xTimer);

	xQueueSet = xQueueCreateSet(1 + 1);
	assert_param(xQueueSet);

	xQueueAddToSet(xSemaphoreIRQ, xQueueSet);
	xQueueAddToSet(xSemaphoreTimerTimeout, xQueueSet);
}


void Wiegand::task() {
	for (;;) {
		QueueSetMemberHandle_t event = xQueueSelectFromSet(xQueueSet, 5000);

		if (event == xSemaphoreIRQ) {
			xSemaphoreTake(event, 0);
			xTimerStart(xTimer, 0);
			data[len / BITS_IN_BYTE] = (data[len / BITS_IN_BYTE] << 1) | *data_1_pin;
			len++;

		} else if (event == xSemaphoreTimerTimeout) {
			xSemaphoreTake(event, 0);
			NVIC_DisableIRQ(IrqNumber);

			/// max_bits - округление количества бит в большую сторону до байта 24 -> 24, 26 -> 32
			uint8_t max_bits = (len / BITS_IN_BYTE + ((len % BITS_IN_BYTE) ? 1 : 0)) * BITS_IN_BYTE;
			data[len / BITS_IN_BYTE] <<= max_bits - len;	// Сдвигаем младшие биты в старшие
			_log("Receive(%3d bits):  0x%02X%02X%02X%02X\n", len, data[0], data[1], data[2], data[3]);

			if (xWiegandQueue != NULL) {
				WiegandStruct d;
				d.WiegandLen = len;
				d.Channel = m_eChannel;
				memcpy(d.Data, data, WIEGAND_BUFFER_SIZE * sizeof(data[0]));
				xQueueSend(xWiegandQueue, &d, 5);
			}

			len = 0;
			memset(data, 0, sizeof(data));
			vTaskDelay(150);

			EXTI_ClearITPendingBit(ExtiLine);
			NVIC_ClearPendingIRQ(IrqNumber);
			NVIC_EnableIRQ(IrqNumber);
		} else {
//			_log("PING\n");
		}
	}
}



void Wiegand::InitHardware(WiegandChannel channel) {
GPIO_InitTypeDef GPIO_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;

	if (channel == WiegandChannel::Channel_1) {
		data_1_pin = &WIEGAND1_DATA_IN;
		IrqNumber = EXTI0_IRQn;
		ExtiLine = EXTI_Line0;

		GPIO_InitStructure.GPIO_Pin = WIEGAND1_DATA_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(WIEGAND1_DATA_PORT, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = WIEGAND1_IRQ_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(WIEGAND1_IRQ_PORT, &GPIO_InitStructure);

		GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
		EXTI_InitStructure.EXTI_Line = EXTI_Line0;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

	} else if (channel == WiegandChannel::Channel_2) {
		data_1_pin = &WIEGAND2_DATA_IN;
		IrqNumber = EXTI1_IRQn;
		ExtiLine = EXTI_Line1;

		GPIO_InitStructure.GPIO_Pin = WIEGAND2_DATA_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(WIEGAND2_DATA_PORT, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = WIEGAND2_IRQ_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(WIEGAND2_IRQ_PORT, &GPIO_InitStructure);

		GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);
		EXTI_InitStructure.EXTI_Line = EXTI_Line1;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	} else {
		_log("Channel is invalid!!!\n");
		assert_param(0);
	}
}


BaseType_t Wiegand::ProcessISR() {
static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreIRQ, &xHigherPriorityTaskWoken);
	return xHigherPriorityTaskWoken;
}


void EXTI0_IRQHandler() {
	if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line0);
		if (wiegand_ch1 != NULL) {
			portYIELD_FROM_ISR(wiegand_ch1->ProcessISR());
		}
	}
}


void EXTI1_IRQHandler() {
	if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line1);
		if (wiegand_ch2 != NULL) {
			portYIELD_FROM_ISR(wiegand_ch2->ProcessISR());
		}
	}
}
