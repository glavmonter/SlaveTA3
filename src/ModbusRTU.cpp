/**
  ******************************************************************************
  * @file    src/ModbusRTU.cpp
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    27 августа 2018
  * @brief   Реализация ModbusRTU
  ******************************************************************************
  * @attention
  * Всем очко!!
  *
  * <h2><center>&copy; 2018 ООО "РПС"</center></h2>
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
#include "ModbusRTU.h"


ModbusRTU::ModbusRTU() {

}


void ModbusRTU::task() {


	for (;;) {
		vTaskDelay(100);
	}
}
