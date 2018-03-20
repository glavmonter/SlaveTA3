/**
  ******************************************************************************
  * @file    src/Climate.cpp
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    12 марта 2018
  * @brief   Реализация класса Climate
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
#include "Climate.h"


Climate::Climate() {
	m_pI2CDriver = new I2CDriver(I2C2);
	assert_param(m_pI2CDriver);

	m_pSHT30 = new SHT30(m_pI2CDriver, 0x88);
	assert_param(m_pSHT30);

	m_pSA56 = new SA56004(m_pI2CDriver, 0x98);
	assert_param(m_pSA56);

	xTaskCreate(task_climate, "Climate", configTASK_CLIMATE_STACK, this, configTASK_CLIMATE_PRIORITY, &handle);
}


void Climate::task() {
	_log("Start\n");

char str[16];
TickType_t timeout = xTaskGetTickCount();

	m_pSA56->Init();
	for (;;) {
		m_pSA56->UpdateData();
		float T = m_pSA56->GetTemperature(SA56004::Sensor_Internal);
		FloatToString(str, T, 3, 3);
		_log("Temperature Local: %s C\n", str);

		T = m_pSA56->GetTemperature(SA56004::Sensor_Remote);
		FloatToString(str, T, 3, 3);
		_log("Temperature Remote: %s C\n", str);

		m_pSHT30->UpdateData();
//		_log("Update: %d\n", ret);
		T = m_pSHT30->GetTemperature();
		FloatToString(str, T, 3, 3);
		_log("Temperature SHT30: %s C\n", str);

		vTaskDelayUntil(&timeout, 2000);
	}

#if 0
	for (;;) {
		uint16_t status;
		bool ret = m_pSHT30->ReadStatus(status);
		_log("Status %d: 0x%04X\n", ret, status);
		m_pSHT30->ClearStatus();

		ret = m_pSHT30->UpdateData();
		_log("Update: %d\n", ret);

		float T = m_pSHT30->GetTemperature();
		FloatToString(str, T, 3, 3);
		_log("Temperature: %s C\n", str);

		float H = m_pSHT30->GetHumidityRel();
		FloatToString(str, H, 3, 3);
		_log("Humidity: %s%%\n\n", str);
		vTaskDelayUntil(&timeout, 2000);
	}
#endif
}


