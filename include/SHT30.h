/**
  ******************************************************************************
  * @file    include/SHT30.h
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    14 марта 2018
  * @brief   Заголовок от SHT30.cpp
  ******************************************************************************
  * @attention
  * Всем очко!!
  *
  * <h2><center>&copy; 2018 ООО "РПС"</center></h2>
  ******************************************************************************
  */

#ifndef SHT30_H_
#define SHT30_H_

#include <stdint.h>
#include "I2CDriver.h"

class SHT30 {
public:
	SHT30(I2CDriver *driver, uint8_t address = 0x44);
	bool ReadStatus(uint16_t &status);
	bool ClearStatus();
	bool UpdateData();

	float GetTemperature() {return m_fTemperature;}
	float GetHumidityRel() {return m_fHumidity;}
	float GetHumidityAbs();

protected:
	bool CRC8(uint8_t MSB, uint8_t LSB, uint8_t crc);

	I2CDriver *m_pDriver;
	uint8_t m_iAddress;

private:
	float m_fTemperature = 0.0f;
	float m_fHumidity = 0.0f;
	float _AbsHumPoly[6] = {-157.004f, 3158.0474f, -25482.532f, 103180.197f, -209805.497f, 171539.883f};
};


#endif /* SHT30_H_ */
