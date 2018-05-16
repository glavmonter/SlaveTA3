/**
  ******************************************************************************
  * @file    include/Climate.h
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    12 марта 2018
  * @brief   Заголовок от Climate.cpp
  ******************************************************************************
  * @attention
  * Всем очко!!
  *
  * <h2><center>&copy; 2018 ООО "РПС"</center></h2>
  ******************************************************************************
  */

#ifndef CLIMATE_H_
#define CLIMATE_H_

#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include <stm32f10x.h>
#include "common.h"
#include "I2CDriver.h"
#include "SHT30.h"
#include "SA56004.h"


typedef struct ClimateStruct_ {
	float TemperatureLocal;			/// Температура локального датчика
	float TemperatureExternal;		/// Температура внешнего датчика
	float TemperatureLocalAlt;		/// Температура в датчике влажности
	float Humidity;					/// Влажность относительная

	bool Heater;					/// Состояние печки
	bool Cooler;					/// Состояние вентилятора
	bool AutomaticMode;				/// Режим работы (автоматический или ручной)
} ClimateStruct;


class Climate : public TaskBase {
public:
	static Climate &Instance() {
		static Climate s;
		return s;
	}
	void task();
	static void task_climate(void *param) {
		static_cast<Climate *>(param)->task();
		while (1)
			vTaskDelay(portMAX_DELAY);
	}

	QueueHandle_t xQueueData;
	void PrintClimateData(const ClimateStruct &c);

private:
	I2CDriver *m_pI2CDriver = NULL;
	SHT30 *m_pSHT30 = NULL;
	SA56004 *m_pSA56 = NULL;

private:
	Climate();
	~Climate() {}
	Climate(Climate const &) = delete;
	Climate& operator= (Climate const &) = delete;

	TimerHandle_t xTimer60s;
	SemaphoreHandle_t xSemaphoreTimer60s;

	TimerHandle_t xTimer1s;
	SemaphoreHandle_t xSemaphoreTimer1s;

	QueueSetHandle_t xQueueSet;
	ClimateStruct m_xClimateData;

	bool m_bAutomaticMode = false;
	void ProcessAutomaticRegulation();
};


#endif /* CLIMATE_H_ */
