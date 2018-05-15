/**
  ******************************************************************************
  * @file    include/main.h
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    10 января 2018
  * @brief   Заголовок от main.cpp
  ******************************************************************************
  * @attention
  * Всем очко!!
  *
  * <h2><center>&copy; 2018 ООО "РПС"</center></h2>
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include <stm32f10x.h>
#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
extern "C" {
#endif

#if configGENERATE_RUN_TIME_STATS == 1
	void TIM4_IRQHandler();
#endif


void vApplicationStackOverflowHook(TaskHandle_t handle, char *name);

#ifdef __cplusplus
}
#endif


#endif /* __MAIN_H */
