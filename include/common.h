/**
  ******************************************************************************
  * @file    include/common.h
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    10 января 2018
  * @brief   Заголовок от common.cpp
  ******************************************************************************
  * @attention
  * Всем очко!!
  *
  * <h2><center>&copy; 2018 ООО "РПС"</center></h2>
  ******************************************************************************
  */

#ifndef COMMON_H_
#define COMMON_H_

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <new>
#include <stm32f10x.h>

#define BKP_DOMAIN_DATA				0x35F8

#define BKP_DOMAIN_GAUGE_DATA		0x8649
#define BKP_DOMAIN_GAUGE_ADDR		RTC_BKP_DR1

class TaskBase {
public:
	TaskHandle_t handle;
	virtual ~TaskBase() {
#if INCLUDE_vTaskDelete
		vTaskDelete(handle);
#endif
		return;
	}
};

template<typename T> T min(T a, T b) {
	return (a < b) ? a : b;
}

template<typename T> T max(T a, T b) {
	return (a > b) ? a : b;
}



#define __SPI_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->SR) & (__FLAG__)) == (__FLAG__))

/** @brief  Clear the SPI OVR pending flag.
  * @param  __HANDLE__: specifies the SPI handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
#define __SPI_CLEAR_OVRFLAG(__HANDLE__) do{(__HANDLE__)->DR;\
                                               (__HANDLE__)->SR;}while(0)



class MutexLocker {
public:
	inline explicit MutexLocker(SemaphoreHandle_t mutex) {
		m = mutex;
		xSemaphoreTake(m, portMAX_DELAY);
	}
	inline ~MutexLocker() {
		xSemaphoreGive(m);
	}
	SemaphoreHandle_t m;
};


char *FloatToString(char *outstr, double value, int places, int minwidth = 0, bool rightjustify = false);

#ifdef __cplusplus
extern "C" {
#endif

void vApplicationTickHook();

#ifdef __cplusplus
}
#endif

#endif /* COMMON_H_ */
