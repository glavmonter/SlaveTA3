/**
  ******************************************************************************
  * @file    include/SA56004.h
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    14 марта 2018
  * @brief   Заголовок от SA56004.cpp
  ******************************************************************************
  * @attention
  * Всем очко!!
  *
  * <h2><center>&copy; 2018 ООО "РПС"</center></h2>
  ******************************************************************************
  */

#ifndef SA56004_H_
#define SA56004_H_

#include "I2CDriver.h"

class SA56004 {
public:
	enum SensorType {
		Sensor_Internal,
		Sensor_Remote
	};

	SA56004(I2CDriver *driver, uint8_t address = 0x98);

	bool Init();
	bool UpdateData();
	float GetTemperature(SensorType type = Sensor_Internal);

protected:
	I2CDriver *m_pDriver;
	uint8_t m_iAddress;

private:
	float m_fTemperatureLocal = 0.0f;
	float m_fTemperatureRemote = 0.0f;
};


#endif /* SA56004_H_ */
