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
  * <h2><center>&copy; 2016 ФГУП "18 ЦНИИ" МО РФ</center></h2>
  ******************************************************************************
  */


#ifndef MAINTASK_H_
#define MAINTASK_H_

#include <FreeRTOS.h>
#include <task.h>
#include "common.h"


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
	MainTask();
	~MainTask() {}

	MainTask(MainTask const &) = delete;
	MainTask& operator= (MainTask const &) = delete;
};


#endif /* MAINTASK_H_ */
