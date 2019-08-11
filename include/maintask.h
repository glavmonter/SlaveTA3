/**
  ******************************************************************************
  * @file    include/maintask.h
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    21 сентября 2016
  * @brief   Описание класса главного потока
  ******************************************************************************
  * @attention
  * Всем очко!!
  *
  * <h2><center>&copy; 2018 ООО "РПС"</center></h2>
  ******************************************************************************
  */


#ifndef MAINTASK_H_
#define MAINTASK_H_

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include "common.h"
#include "Wake.h"
#include "Wiegand.h"
#include "trcRecorder.h"


uint32_t StartExternalApp(uint32_t address);

#define MAX_PULSE_PINS			5

typedef struct PulseStruct_ {
	uint32_t pin;
	uint32_t width; // ms
	uint32_t delay;
	TimerHandle_t timer;
} PulseStruct;


/**
 * @class MainTask
 */
class MainTask : public TaskBase {
public:
	static MainTask& Instance() {
		static MainTask s;
		return s;
	}

	int run();

	void task();
	static void task_main(void *param) {
		static_cast<MainTask *>(param)->task();
		while (1)
			vTaskDelay(portMAX_DELAY);
	}

private:
	Wiegand *m_pWiegandCh1;
	Wiegand *m_pWiegandCh2;

	traceString event;
	traceString actorPulse;

public:
	USART_TypeDef *m_pUSART = SERIAL_USB_USART;
	QueueHandle_t xQueueUsartRx;
	QueueHandle_t xQueuePulseTimer;

private:
	MainTask();
	~MainTask() {}

	MainTask(MainTask const &) = delete;
	MainTask& operator= (MainTask const &) = delete;


	void InitHardware();
	void Init();

	// RTOS Objects
	QueueSetHandle_t xQueueSet;
	TimerHandle_t xTimer;
	SemaphoreHandle_t xTimerSemaphore;
	QueueHandle_t xQueueWiegand;

	PulseStruct xPulseStruct[MAX_PULSE_PINS];

	// External Classes
	Wake *m_pWake;

	WiegandStruct sWiegand1;
	WiegandStruct sWiegand2;

	inline uint8_t PORTA_IDR() {return (~GPIOD->IDR) & 0x0F;}
	inline uint8_t PORTB_IDR() {return (~GPIOE->IDR) & 0x0F;}
	inline uint8_t PORTA_ODR() {return ((~GPIOD->ODR) >> 4) & 0x0F;}
	inline uint8_t PORTB_ODR() {return ((~GPIOE->ODR) >> 4) & 0x0F;}

	/* Process commands */
	void ProcessCmdInfo();
	void ProcessBoot();
	void ProcessWiegand(Command cmd, const WiegandStruct &wig);
	void ProcessPORTs(Command cmd);
	void PORTA_Write(uint8_t odr);
	void PORTB_Write(uint8_t odr);
	void PORTA_Toggle(uint8_t toggle);
	void PORTB_Toggle(uint8_t toggle);

	void ProcessClimate(Command cmd);
	void ProcessReadAll(Command cmd);
	void ProcessPulse(Command cmd);

	bool TogglePin(uint8_t pin);

	void ProcessWriteAll(Command cmd);
};



#ifdef __cplusplus
extern "C" {
#endif

void USART1_IRQHandler();

#ifdef __cplusplus
}
#endif


#endif /* MAINTASK_H_ */
