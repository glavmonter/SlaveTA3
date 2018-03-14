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
#include "Climate.h"
#include "IOExpanders.h"


/**
 * @brief Конструктор.
 *
 * Создание таска ОСРВ с минимальным приоритетом (tskIDLE_PRIORITY)
 */
MainTask::MainTask() {
	xTaskCreate(task_main, "Main", configTASK_MAIN_STACK, this, configTASK_MAIN_PRIORITY, &handle);
}

/**
 * \brief Включение планировщика ОСРВ
 */
int MainTask::run() {
	vTaskStartScheduler();
	return 0;
}

/**
 * \brief Рабочие цикл таска.
 *
 * Рабочий цикл MainTask. Создание дочерних тасков и обслуживание связей между ними
 */
void MainTask::task() {

	portENTER_CRITICAL();
		Climate &clim = Climate::Instance();
		UNUSED(clim);

		IOExpanders &ioe = IOExpanders::Instance();
		UNUSED(ioe);
	portEXIT_CRITICAL();

	for (;;) {
		vTaskDelay(portMAX_DELAY);
	}
}
