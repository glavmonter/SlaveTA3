/**
  ******************************************************************************
  * @file    include/I2CDriver.cpp
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    14 марта 2018
  * @brief   Реализация  I2CDriver
  ******************************************************************************
  * @attention
  * Всем очко!!
  *
  * <h2><center>&copy; 2018 ООО "РПС"</center></h2>
  ******************************************************************************
  */

#include <FreeRTOS.h>
#include "I2CDriver.h"

#define I2C_TIMEOUT						5		// Timeout ms
#define CHECK_TIMEOUT(str, ret)		do {	\
										if (timeout <= xTaskGetTickCount()) { \
											_log(str); \
											return (ret); \
										} \
									} while (0);

#define WAIT_EVENT_WITH_TO(i2c, event, err_str, err_ret)	do { \
																timeout = xTaskGetTickCount() + I2C_TIMEOUT; \
																while ((!I2C_CheckEvent(i2c, event)) && (timeout > xTaskGetTickCount())) {} \
																if (timeout <= xTaskGetTickCount()) { \
																	_log(err_str); \
																	return (err_ret); \
																} \
															} while (0);


I2CDriver::I2CDriver(I2C_TypeDef *i2c) :
	m_pI2C(i2c) {

	InitHardware();
}


void I2CDriver::InitHardware() {
RCC_APB1PeriphClockCmd(I2C2_CLK, ENABLE);
	RCC_APB1PeriphResetCmd(I2C2_CLK, ENABLE);
	RCC_APB1PeriphResetCmd(I2C2_CLK, DISABLE);

GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = I2C2_SCL_PIN | I2C2_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(I2C2_PORT, &GPIO_InitStructure);

I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_DeInit(m_pI2C);
	I2C_Init(m_pI2C, &I2C_InitStructure);
}


bool I2CDriver::ResetHardware() {
	I2C_DeInit(m_pI2C);
	RCC_APB1PeriphClockCmd(I2C2_CLK, DISABLE);

GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = I2C2_SCL_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(I2C2_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = I2C2_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(I2C2_PORT, &GPIO_InitStructure);

bool ret = false;

TickType_t timeout = xTaskGetTickCount() + 50;
	while (timeout > xTaskGetTickCount()) {
		vTaskDelay(1);
		GPIO_WriteBit(I2C2_PORT, I2C2_SCL_PIN, Bit_RESET);
		if (GPIO_ReadInputDataBit(I2C2_PORT, I2C2_SDA_PIN) == Bit_SET) {
			ret = true;
			break;
		}

		vTaskDelay(1);
		GPIO_WriteBit(I2C2_PORT, I2C2_SCL_PIN, Bit_SET);
		if (GPIO_ReadInputDataBit(I2C2_PORT, I2C2_SDA_PIN) == Bit_SET) {
			ret = true;
			break;
		}
	}

	if (ret == true)
		InitHardware();
	return ret;
}


bool I2CDriver::write(uint8_t address, const uint8_t *pData, uint8_t lenght, bool repeated) {
uint8_t writed = 0;
TickType_t timeout;

	I2C_GenerateSTART(m_pI2C, ENABLE);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_MODE_SELECT, "START failed\n", false);

	I2C_Send7bitAddress(m_pI2C, address, I2C_Direction_Transmitter);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, "Master failed\n", false);

	if (lenght == 1) {
		I2C_SendData(m_pI2C, pData[0]);
		WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED, "Transmitted failed\n", false);
		writed++;

	} else {
		for (uint8_t i = 0; i < lenght - 1; i++) {
			I2C_SendData(m_pI2C, pData[i]);
			WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING, "Transmitting failed\n", false);
			writed++;
		}
		I2C_SendData(m_pI2C, pData[lenght - 1]);
		WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED, "Transmitter many failed\n", false);
		writed++;
	}

	if (!repeated) {
		I2C_GenerateSTOP(m_pI2C, ENABLE);
		while (m_pI2C->SR2 & I2C_SR2_MSL) {}
	}

	return lenght == writed;
}

bool I2CDriver::read(uint8_t address, uint8_t *pData, uint8_t lenght, bool repeated) {
uint8_t readed = 0;
TickType_t timeout;

	I2C_GenerateSTART(m_pI2C, ENABLE);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_MODE_SELECT, "START error\n", false);

	I2C_Send7bitAddress(m_pI2C, address, I2C_Direction_Receiver);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, "Receiver error\n", false);

	if (lenght > 1) {
		for (uint8_t i = 0; i < lenght - 1; i++) {
			WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_RECEIVED, "Received error\n", false);
			*pData = m_pI2C->DR;
			pData++;
			readed++;
		}
	}

	if (!repeated) {
		I2C_AcknowledgeConfig(m_pI2C, DISABLE);
		I2C_GenerateSTOP(m_pI2C, ENABLE);
		WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_RECEIVED, "Received error many\n", false);
		*pData = m_pI2C->DR;
		readed++;
		I2C_AcknowledgeConfig(m_pI2C, ENABLE);
	} else {
		WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_RECEIVED, "Receiver mmm\n", false);
		*pData = m_pI2C->DR;
		readed++;
	}

	return readed == lenght;
}
