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
//#include <SEGGER_RTT.h>
#include "hardware.h"
#include "Climate.h"


static void TimerCallback(TimerHandle_t xTimer) {
SemaphoreHandle_t sem = static_cast<SemaphoreHandle_t>(pvTimerGetTimerID(xTimer));
	xSemaphoreGive(sem);
}


Climate::Climate() {
	m_xClimateData.TemperatureLocal = 24.0f;
	m_xClimateData.TemperatureLocalAlt = 24.0f;
	m_xClimateData.TemperatureExternal = 24.0f;
	m_xClimateData.Humidity = 24.0f;
	m_xClimateData.Cooler = false;
	m_xClimateData.Heater = false;
	m_xClimateData.AutomaticMode = false;

	m_pI2CDriver = new I2CDriver(I2C2);
	assert_param(m_pI2CDriver);

	m_pSHT30 = new SHT30(m_pI2CDriver, 0x88);
	assert_param(m_pSHT30);

	m_pSA56 = new SA56004(m_pI2CDriver, 0x98);
	assert_param(m_pSA56);

	xSemaphoreTimer1s = xSemaphoreCreateBinary();
	assert_param(xSemaphoreTimer1s);
	vTraceSetSemaphoreName(xSemaphoreTimer1s, "Climate_1s");
	xSemaphoreTake(xSemaphoreTimer1s, 0);

	xSemaphoreTimer60s = xSemaphoreCreateBinary();
	assert_param(xSemaphoreTimer60s);
	vTraceSetSemaphoreName(xSemaphoreTimer60s, "Climate_60s");
	xSemaphoreTake(xSemaphoreTimer60s, 0);

	xQueueSet = xQueueCreateSet(1 + 1);
	assert_param(xQueueSet);
	vTraceSetQueueName(xQueueSet, "ClimateQueueSet");

	xQueueAddToSet(xSemaphoreTimer1s, xQueueSet);
	xQueueAddToSet(xSemaphoreTimer60s, xQueueSet);


	xTimer1s = xTimerCreate("Timer1s", 1 * 1000, pdTRUE, xSemaphoreTimer1s, TimerCallback);
	assert_param(xTimer1s);

	xTimer60s = xTimerCreate("Timer60s", 60 * 1000, pdTRUE, xSemaphoreTimer60s, TimerCallback);
	assert_param(xTimer60s);

	xQueueData = xQueueCreate(1, sizeof(ClimateStruct));
	assert_param(xQueueData);
	vTraceSetQueueName(xQueueData, "ClimateData");
	xQueueOverwrite(xQueueData, &m_xClimateData);

	xTaskCreate(task_climate, "Climate", configTASK_CLIMATE_STACK, this, configTASK_CLIMATE_PRIORITY, &handle);
	assert_param(handle);
}


void Climate::task() {

	m_pSA56->Init();

	xTimerStart(xTimer1s, 1);
	xTimerStart(xTimer60s, 1);

ClimateStruct lc;

	for (;;) {
		QueueSetMemberHandle_t event = xQueueSelectFromSet(xQueueSet, 1000);
		if (event == xSemaphoreTimer1s) {
			xSemaphoreTake(event, 0);
			xQueuePeek(xQueueData, &lc, 0);
			if (lc.AutomaticMode != m_bAutomaticMode) {
				m_bAutomaticMode = lc.AutomaticMode;
				_log("Automatic Mode changed to %s\n", m_bAutomaticMode ? "True" : "False");
				if (m_bAutomaticMode == true) {
					CLIMATE_HEATER_EN = 0;
					CLIMATE_COOLER_EN = 0;
				}
			}

			if (m_bAutomaticMode == false) {
				CLIMATE_COOLER_EN = lc.Cooler ? 1 : 0;
				CLIMATE_HEATER_EN = lc.Heater ? 1 : 0;
			}

			if (m_pSA56->UpdateData() == false) {
				_log("SA56 Err\n");
				_log("SA56 reset hardware: %s\n", m_pI2CDriver->ResetHardware() ? "OK" : "Err");
			}

			if (m_pSHT30->UpdateData() == false) {
				_log("SHT Err\n");
				_log("SHT Reset hardware: %s\n", m_pI2CDriver->ResetHardware() ? "OK" : "Err");
			}

			m_xClimateData.TemperatureLocal = m_pSA56->GetTemperature(SA56004::Sensor_Internal);
			m_xClimateData.TemperatureExternal = m_pSA56->GetTemperature(SA56004::Sensor_Remote);
			m_xClimateData.TemperatureLocalAlt = m_pSHT30->GetTemperature();
			m_xClimateData.Humidity = m_pSHT30->GetHumidityRel();

			m_xClimateData.Cooler = (CLIMATE_COOLER_EN == 1);
			m_xClimateData.Heater = (CLIMATE_HEATER_EN == 1);

			m_xClimateData.AutomaticMode = m_bAutomaticMode;
			xQueueOverwrite(xQueueData, &m_xClimateData);

		} else if (event == xSemaphoreTimer60s) {
			xSemaphoreTake(event, 0);

			if (m_bAutomaticMode)
				ProcessAutomaticRegulation();

			m_xClimateData.Cooler = (CLIMATE_COOLER_EN == 1);
			m_xClimateData.Heater = (CLIMATE_HEATER_EN == 1);
			m_xClimateData.AutomaticMode = m_bAutomaticMode;

			xQueueOverwrite(xQueueData, &m_xClimateData);

		} else {
			__NOP();
		}
	}
}


void Climate::ProcessAutomaticRegulation() {
	_log("%s\n", __FUNCTION__);
}


void Climate::PrintClimateData(const ClimateStruct &c) {
char str[16];
	FloatToString(str, c.TemperatureLocal - 273.15f, 3, 3);
	_log("Temperature Local: %s C\n", str);

	FloatToString(str, c.TemperatureExternal - 273.15f, 3, 3);
	_log("Temperature Remote: %s C\n", str);

	FloatToString(str, c.TemperatureLocalAlt - 273.15f, 3, 3);
	_log("Temperature SHT30: %s C\n", str);

	FloatToString(str, c.Humidity, 3, 3);
	_log("Humidity: %s%%\n", str);

	_log("Heater: %s\n", c.Heater ? "ON" : "OFF");
	_log("Cooler: %s\n\n", c.Cooler ? "ON" : "OFF");
}


