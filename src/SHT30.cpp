/**
  ******************************************************************************
  * @file    include/SHT30.cpp
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    14 марта 2018
  * @brief   Реализация SHT30
  ******************************************************************************
  * @attention
  * Всем очко!!
  *
  * <h2><center>&copy; 2018 ООО "РПС"</center></h2>
  ******************************************************************************
  */

#include "SHT30.h"


SHT30::SHT30(I2CDriver *driver, uint8_t address) :
	m_pDriver(driver), m_iAddress(address)
{
	assert_param(m_pDriver);
}


bool SHT30::ReadStatus(uint16_t &status) {
uint8_t cmd[2] = {0xF3, 0x2D};

	m_pDriver->write(m_iAddress, cmd, 2, true);

uint8_t rx_data[3];
	m_pDriver->read(m_iAddress, rx_data, 3, false);

	status = (rx_data[0] << 8) | rx_data[1];
	return CRC8(rx_data[0], rx_data[1], rx_data[2]);
}

bool SHT30::ClearStatus() {
uint8_t cmd[2] = {0x30, 0x41};
bool ret = m_pDriver->write(m_iAddress, cmd, 2, false);
	vTaskDelay(1);
	return ret;
}


bool SHT30::UpdateData() {
uint8_t cmd[2] = {0x24, 0x0B};
bool ret = true;

	ret &= m_pDriver->write(m_iAddress, cmd, 2, false);
	vTaskDelay(35);	// Conversion time

uint8_t responce[6];
	ret &= m_pDriver->read(m_iAddress, responce, 6, false);

bool crc;
	crc = CRC8(responce[0], responce[1], responce[2]) and
		  CRC8(responce[3], responce[4], responce[5]);

	if (crc == false)
		return false;

	uint16_t TempatureRaw = (responce[0] << 8) | responce[1];
	uint16_t HumidityRaw = (responce[3] << 8) | responce[4];

	m_fTemperature = ((float)TempatureRaw) * 0.00267033f - 45.0f + 273.15f;
	m_fHumidity = ((float)HumidityRaw) * 0.0015259f;
	return ret;
}


float SHT30::GetHumidityAbs() {
	float millikelvin = (m_fTemperature + 273.15);
	return 0.0f;
}



bool SHT30::CRC8(uint8_t MSB, uint8_t LSB, uint8_t crc) {
	/*
	*	Name  : CRC-8
	*	Poly  : 0x31	x^8 + x^5 + x^4 + 1
	*	Init  : 0xFF
	*	Revert: false
	*	XorOut: 0x00
	*	Check : for 0xBE,0xEF CRC is 0x92
	*/
uint8_t tmp = 0xFF;
	tmp ^= MSB;

	for (int i = 0; i < 8; i++)
		tmp = tmp & 0x80 ? (tmp << 1) ^ 0x31 : tmp << 1;

	tmp ^= LSB;
	for (int i = 0; i < 8; i++)
		tmp = tmp & 0x80 ? (tmp << 1) ^ 0x31 : tmp << 1;

	return tmp == crc;
}
