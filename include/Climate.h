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
#include <stm32f10x.h>
#include "common.h"
#include "I2CDriver.h"
#include "SHT30.h"
#include "SA56004.h"

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

private:
	I2CDriver *m_pI2CDriver = NULL;
	SHT30 *m_pSHT30 = NULL;
//	SA56004 *m_pSA56 = NULL;

private:
	Climate();
	~Climate() {}
	Climate(Climate const &) = delete;
	Climate& operator= (Climate const &) = delete;

};


#endif /* CLIMATE_H_ */
