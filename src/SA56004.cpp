/**
  ******************************************************************************
  * @file    include/SA56004.cpp
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    14 марта 2018
  * @brief   Реализация  SA56004
  ******************************************************************************
  * @attention
  * Всем очко!!
  *
  * <h2><center>&copy; 2018 ООО "РПС"</center></h2>
  ******************************************************************************
  */

#include "SEGGER_RTT.h"
#include "SA56004.h"

SA56004::SA56004(I2CDriver *driver, uint8_t address) :
	m_pDriver(driver), m_iAddress(address)
{
	assert_param(driver);
}

bool SA56004::Init() {
bool ret = false;

uint8_t cmd[] = {0xFE};
	ret = m_pDriver->read(m_iAddress, cmd, 1, false);
	_log("Manufacture ID (%d): 0x%02X\n", ret, cmd[0]);

	return ret;
}


float SA56004::GetTemperature(SA56004::SensorType type) {
	switch (type) {
	case Sensor_Internal:
		return m_fTemperatureLocal;
		break;
	case Sensor_Remote:
		return m_fTemperatureRemote;
		break;
	}

	return 0.0f;
}


bool SA56004::UpdateData() {
bool ret = false;
uint8_t cmd[] = {0x00};
uint8_t temp_h, temp_l;

	ret = m_pDriver->write(m_iAddress, cmd, 1, false);
	ret = m_pDriver->read(m_iAddress, &temp_h, 1, false);
	cmd[0] = 0x22;
	ret = m_pDriver->write(m_iAddress, cmd, 1, false);
	ret = m_pDriver->read(m_iAddress, &temp_l, 1, false);

int16_t local = (temp_h << 8) | temp_l;
	m_fTemperatureLocal = local * 0.125f / 32.0f;

	cmd[0] = 0x01;
	ret = m_pDriver->write(m_iAddress, cmd, 1, false);
	ret = m_pDriver->read(m_iAddress, &temp_h, 1, false);
	cmd[0] = 0x10;
	ret = m_pDriver->write(m_iAddress, cmd, 1, false);
	ret= m_pDriver->read(m_iAddress, &temp_l, 1, false);
int16_t remote = (temp_h << 8) | temp_l;
	m_fTemperatureRemote = remote * 0.125f / 32.0f;
 	return ret;
}

