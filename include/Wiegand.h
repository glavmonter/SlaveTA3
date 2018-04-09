/**
  ******************************************************************************
  * @file    include/Wiegand.h
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    12 марта 2018
  * @brief   Заголовок от Wiegand.cpp
  ******************************************************************************
  * @attention
  * Всем очко!!
  *
  * <h2><center>&copy; 2018 ООО "РПС"</center></h2>
  ******************************************************************************
  */


#ifndef WIEGAND_H_
#define WIEGAND_H_

#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include <semphr.h>
#include <queue.h>
#include <stm32f10x.h>
#include "common.h"

#define WIEGAND_MAX_SIZE_IN_BITS		128
#define WIEGAND_BUFFER_SIZE				16
#define BITS_IN_BYTE		8


class Wiegand : public TaskBase {
public:
	enum WiegandChannel {
		Channel_1,
		Channel_2
	};

	Wiegand(WiegandChannel channel, QueueHandle_t result_queue);

	void task();
	static void task_wiegand(void *param) {
		static_cast<Wiegand *>(param)->task();
		while (1)
			vTaskDelay(portMAX_DELAY);
	}

	BaseType_t ProcessISR();

private:
	void InitHardware(WiegandChannel channel);

	TimerHandle_t xTimer;
	SemaphoreHandle_t xSemaphoreIRQ;
	SemaphoreHandle_t xSemaphoreTimerTimeout;
	QueueSetHandle_t xQueueSet;
	QueueHandle_t xWiegandQueue = NULL;

	uint8_t len = 0;
	uint8_t data[WIEGAND_BUFFER_SIZE];

	__IO uint32_t *data_1_pin;
	IRQn_Type IrqNumber;
	uint32_t ExtiLine;

	WiegandChannel m_eChannel;
};


typedef struct WiegandStruct_ {
	Wiegand::WiegandChannel Channel;
	uint8_t WiegandLen;
	uint8_t Data[WIEGAND_BUFFER_SIZE];
} WiegandStruct;


#ifdef __cplusplus
extern "C" {
#endif

void EXTI0_IRQHandler();
void EXTI1_IRQHandler();


#ifdef __cplusplus
}
#endif
#endif /* WIEGAND_H_ */
