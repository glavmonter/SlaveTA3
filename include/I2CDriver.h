/**
  ******************************************************************************
  * @file    include/I2CDriver.h
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    14 марта 2018
  * @brief   Заголовок от I2CDriver.cpp
  ******************************************************************************
  * @attention
  * Всем очко!!
  *
  * <h2><center>&copy; 2018 ООО "РПС"</center></h2>
  ******************************************************************************
  */

#ifndef I2CDRIVER_H_
#define I2CDRIVER_H_


#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <stm32f10x.h>

#include "SEGGER_RTT.h"
#include "hardware.h"

class I2CDriver {
public:
	I2CDriver(I2C_TypeDef *i2c);
	I2C_TypeDef *m_pI2C;

	bool write(uint8_t address, const uint8_t *pData, uint8_t lenght, bool repeated = false);
	bool read(uint8_t address, uint8_t *pData, uint8_t lenght, bool repeated = false);

	bool ResetHardware();
	void InitHardware();

protected:

};



#endif /* I2CDRIVER_H_ */
