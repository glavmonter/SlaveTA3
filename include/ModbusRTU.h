/**
  ******************************************************************************
  * @file    include/ModbusRTU.р
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    27 августа 2018
  * @brief   Task ModbusRTU
  ******************************************************************************
  * @attention
  * Всем очко!!
  *
  * <h2><center>&copy; 2018 ООО "РПС"</center></h2>
  ******************************************************************************
  */

#ifndef MODBUSRTU_H_
#define MODBUSRTU_H_

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <timers.h>
#include <stm32f10x.h>


#include "common.h"


class ModbusRTU : public TaskBase {
public:
	static ModbusRTU &Instance() {
		static ModbusRTU s;
		return s;
	}

	void task();
	static void task_modbusrtu(void *param) {
		static_cast<ModbusRTU *>(param)->task();
		while (1)
			vTaskDelay(portMAX_DELAY);
	}

private:
	ModbusRTU();
	~ModbusRTU() {}
	ModbusRTU(ModbusRTU const &) = delete;
	ModbusRTU& operator= (ModbusRTU const &) = delete;
};



#endif /* MODBUSRTU_H_ */
