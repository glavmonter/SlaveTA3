/**
  ******************************************************************************
  * @file    src/IOExpanders.cpp
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    12 марта 2018
  * @brief   Реализация класса IOExpanders
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
#include "IOExpanders.h"


#define I2C_TIMEOUT 				5
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

#define IOE_TX_DATA_SIZE		32
uint8_t tx_data[IOE_TX_DATA_SIZE] __attribute__((aligned(8)));


IOExpanders::IOExpanders() {
	memset(tx_data, 0, sizeof(tx_data));
	InitHardware();
	xTaskCreate(task_expanders, "IOExp", configMINIMAL_STACK_SIZE*2, this, configTASK_IOE_PRIORITY, &handle);
}



void IOExpanders::task() {
	_log("Start\n");

	ConfigureExpanders();
	vTaskDelay(5);
	StartDMA();

	for (int i = 0; i <= IOE_TX_DATA_SIZE; i++) {
		vTaskDelay(100);
		CalcPWMData(tx_data, 0, i);
	}
	vTaskDelay(100);

	CalcPWMData(tx_data, 0, 0);
	vTaskDelay(100);

	CalcPWMData(tx_data, 0, IOE_TX_DATA_SIZE);
	vTaskDelay(100);

	StopDMA();

	vTaskDelay(100);
	WriteRegister(IOE_ADDR_RELAYS, MCP23017<BANK_1>::GPIOXA(), 0x00);

	for (;;) {
		uint16_t r16;
		ReadRegister(IOE_ADDR_INPUTS, MCP23017<BANK_0>::GPIOXA(), r16);
		_log("Input: 0x%04X\n", r16);
		vTaskDelay(250);
	}

}


void IOExpanders::ConfigureExpanders() {
bool ret;
	/* Включаем режим двух банков и отключаем режим инкрементирования адреса */
	ret = WriteRegister(IOE_ADDR_RELAYS, MCP23017<BANK_0>::IOCONA(),
			MCP23017_IOCON_BANK(1) | MCP23017_IOCON_SEQOP(1));
	_log("%d\n", ret);
	if (ret == false) {
		ret = ResetHardware();
		ret = WriteRegister(IOE_ADDR_RELAYS, MCP23017<BANK_0>::IOCONA(),
				MCP23017_IOCON_BANK(1) | MCP23017_IOCON_SEQOP(1));
	}
	_log("%d\n", ret);

	ret = WriteRegister(IOE_ADDR_POWER,  MCP23017<BANK_0>::IOCONA(),
			MCP23017_IOCON_BANK(1) | MCP23017_IOCON_SEQOP(1));

	WriteRegister(IOE_ADDR_INPUTS, MCP23017<BANK_0>::IOCONA(),
			MCP23017_IOCON_MIRROR(1));
	WriteRegister(IOE_ADDR_INPUTS, MCP23017<BANK_0>::IOCONB(),
			MCP23017_IOCON_MIRROR(1));

	// Change input polarity
	WriteRegister(IOE_ADDR_INPUTS, MCP23017<BANK_0>::IPOLA(), 0xFF);
	WriteRegister(IOE_ADDR_INPUTS, MCP23017<BANK_0>::IPOLB(), 0xFF);

	// Enable internal PullUp resistor
	WriteRegister(IOE_ADDR_INPUTS, MCP23017<BANK_0>::GPPUA(), 0xFF);
	WriteRegister(IOE_ADDR_INPUTS, MCP23017<BANK_0>::GPPUB(), 0xFF);

	/* Enable interrupt on discret inputs */
	WriteRegister(IOE_ADDR_INPUTS, MCP23017<BANK_0>::GPINTENA(), 0xFF);
	WriteRegister(IOE_ADDR_INPUTS, MCP23017<BANK_0>::GPINTENB(), 0xFF);


	WriteRegister(IOE_ADDR_RELAYS, MCP23017<BANK_1>::GPIOXA(), 0x00);
	WriteRegister(IOE_ADDR_RELAYS, MCP23017<BANK_1>::IODIRA(), 0x00);

	WriteRegister(IOE_ADDR_RELAYS, MCP23017<BANK_1>::GPIOXB(), 0x00);
	WriteRegister(IOE_ADDR_RELAYS, MCP23017<BANK_1>::IODIRB(), 0x00);
}


/**
 * @brief Запись значения регистра в расширитель
 * @param dev_addr - Адрес устройства на шине
 * @param reg_addr - Адрес регистра
 * @param data - Данные для записи
 * @retval true в случае успешности
 * @retval false во всех остальных
 *
 * Функция осуществляет запись данных data в регистр с адресом reg_addr на расширители с адресом dev_addr
 */
bool IOExpanders::WriteRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
bool ret = false;
TickType_t timeout;

	I2C_GenerateSTART(m_pI2C, ENABLE);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_MODE_SELECT, "START failed\n", false);

	I2C_Send7bitAddress(m_pI2C, dev_addr, I2C_Direction_Transmitter);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, "Master failed\n", false);

	I2C_SendData(m_pI2C, reg_addr);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING, "Transmitting failed\n", false);

	I2C_SendData(m_pI2C, data);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED, "Transmitted failed\n", false);

	I2C_GenerateSTOP(m_pI2C, ENABLE);
	while (m_pI2C->SR2 & I2C_SR2_MSL) {}
	return !ret;
}


/**
 * @brief Запись серии значений в регистр
 * @param dev_addr - Адрес устройства на шине
 * @param reg_addr - Адрес регистра
 * @param data - Буфер данных для записи
 * @param len - Размер буфера данных
 * @retval true в случае успешности
 * @retval false во всех остальных
 *
 * Функция осуществляет запись данных data в регистр с адресом reg_addr на расширители с адресом dev_addr
 */
bool IOExpanders::WriteRegister(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, size_t len) {
TickType_t timeout;

	I2C_GenerateSTART(m_pI2C, ENABLE);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_MODE_SELECT, "START failed\n", false);

	I2C_Send7bitAddress(m_pI2C, dev_addr, I2C_Direction_Transmitter);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, "Master failed\n", false);

	I2C_SendData(m_pI2C, reg_addr);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING, "Transmitting failed\n", false);

	if (len == 1) {
		I2C_SendData(m_pI2C, data[0]);
		WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED, "Transmitting failed\n", false);
	} else {
		for (size_t i = 0; i < len - 1; i++) {
			I2C_SendData(m_pI2C, data[i]);
			WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING, "Transmitting failed\n", false);
		}
		I2C_SendData(m_pI2C, data[len - 1]);
		WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED, "Transmitter many failed\n", false);
	}

	I2C_GenerateSTOP(m_pI2C, ENABLE);
	while (m_pI2C->SR2 & I2C_SR2_MSL) {}

	return true;
}

/**
 * @brief Чтение значения регистра из расширителя
 * @param dev_addr - Адрес устройства на шине
 * @param reg_addr - Адрес регистра
 * @param readed - Считанное значение
 * @retval true в случае успешности
 * @retval false во всех остальных
 *
 * Функция осуществляет чтение данных из регистра с адресом reg_addr на расширители с адресом dev_addr
 */
bool IOExpanders::ReadRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t &readed) {
TickType_t timeout;
	I2C_GenerateSTART(m_pI2C, ENABLE);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_MODE_SELECT, "START error\n", false);

	I2C_Send7bitAddress(m_pI2C, dev_addr, I2C_Direction_Transmitter);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, "I2C Address error\n", false);

	I2C_SendData(m_pI2C, reg_addr);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING, "Transmitting failed\n", false);

	I2C_GenerateSTOP(m_pI2C, ENABLE);
	while (m_pI2C->SR2 & I2C_SR2_MSL) {}

	I2C_GenerateSTART(m_pI2C, ENABLE);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_MODE_SELECT, "ReSTART error\n", false);

	I2C_Send7bitAddress(m_pI2C, dev_addr, I2C_Direction_Receiver);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, "I2C Address rcv error\n", false);

	I2C_AcknowledgeConfig(m_pI2C, DISABLE);

	(void)m_pI2C->SR2;
	I2C_GenerateSTOP(m_pI2C, ENABLE);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_RECEIVED, "Received error\n", false);

	readed = I2C_ReceiveData(m_pI2C);

	return true;
}

/**
 * @brief Чтение значения пары регистров из расширителя
 * @param dev_addr - Адрес устройства на шине
 * @param reg_addr - Адрес стартового регистра
 * @param readed - Считанное значение
 * @retval true в случае успешности
 * @retval false во всех остальных
 *
 * Функция осуществляет чтение пары регистров с начальным адресом @a reg_addr на расширителе с адресом @a dev_addr
 */
bool IOExpanders::ReadRegister(uint8_t dev_addr, uint8_t reg_addr, uint16_t &readed) {
TickType_t timeout;
	I2C_GenerateSTART(m_pI2C, ENABLE);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_MODE_SELECT, "START error\n", false);

	I2C_Send7bitAddress(m_pI2C, dev_addr, I2C_Direction_Transmitter);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, "I2C Address error\n", false);

	I2C_SendData(m_pI2C, reg_addr);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING, "Transmitting failed\n", false);

	I2C_GenerateSTOP(m_pI2C, ENABLE);
	while (m_pI2C->SR2 & I2C_SR2_MSL) {}

	/* Адрес регистра установлен */
	I2C_GenerateSTART(m_pI2C, ENABLE);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_MODE_SELECT, "ReSTART error\n", false);

	I2C_Send7bitAddress(m_pI2C, dev_addr, I2C_Direction_Receiver);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, "I2C Address rcv error\n", false);

	I2C_AcknowledgeConfig(m_pI2C, ENABLE);

uint8_t data[2];
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_RECEIVED, "Received error\n", false);
	data[0] = m_pI2C->DR;

	I2C_AcknowledgeConfig(m_pI2C, DISABLE);
	I2C_GenerateSTOP(m_pI2C, ENABLE);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_RECEIVED, "Received error many\n", false);
	data[1] = m_pI2C->DR;

	readed = (data[1] << 8) | data[0];
	return true;
}

bool IOExpanders::Test() {
TickType_t timeout;
	I2C_AcknowledgeConfig(m_pI2C, ENABLE);

	I2C_GenerateSTART(m_pI2C, ENABLE);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_MODE_SELECT, "START failed\n", false);

	I2C_Send7bitAddress(m_pI2C, IOE_ADDR_RELAYS, I2C_Direction_Transmitter);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, "Master failed\n", false);

	I2C_SendData(m_pI2C, MCP23017<BANK_1>::GPIOXA());
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING, "Transmitting failed\n", false);

	I2C_SendData(m_pI2C, 0x00);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING, "Transmitting failed\n", false);
	I2C_SendData(m_pI2C, 0x00);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING, "Transmitting failed\n", false);

	I2C_SendData(m_pI2C, 0xFF);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING, "Transmitting failed\n", false);
	I2C_SendData(m_pI2C, 0xFF);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING, "Transmitting failed\n", false);


	I2C_SendData(m_pI2C, 0x00);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING, "Transmitting failed\n", false);
	I2C_SendData(m_pI2C, 0x00);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING, "Transmitting failed\n", false);

	I2C_SendData(m_pI2C, 0xFF);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING, "Transmitting failed\n", false);
	I2C_SendData(m_pI2C, 0xFF);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED, "Transmitted failed\n", false);

	I2C_GenerateSTOP(m_pI2C, ENABLE);
	while (m_pI2C->SR2 & I2C_SR2_MSL) {}
	return true;
}

/**
 * @brief Настройка DMA
 *
 * Настройка DMA1 Channel 6 на передачу данных из памяти на I2C.
 * Режим циклический, что обеспечивает ШИМ на порту A расширителя (5 бит)
 */
void IOExpanders::ConfigureDMA() {
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&m_pI2C->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)tx_data;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = IOE_TX_DATA_SIZE * sizeof(tx_data[0]);

	DMA_DeInit(DMA1_Channel6);
	DMA_Init(DMA1_Channel6, &DMA_InitStructure);
}

/**
 * @brief Вычисление регистра ШИМ
 * @param pData - Массив циклических данных, что передается через DMA
 * @param channel - Номер канала ШИМ (0..5)
 * @param value - Величина ШИМ (0..IOE_TX_DATA_SIZE)
 *
 * В функции вычисляется значения элементов массива для обеспечения корректных данных для реализации ШИМ.
 * В соответствии с каналом выставляется необходимое количество единиц в элементах массива pData.
 */
void IOExpanders::CalcPWMData(uint8_t *pData, uint8_t channel, uint8_t value) {
int32_t tpwm = value;
	for (int i = 0; i < IOE_TX_DATA_SIZE; i++) {
		if (tpwm > 0) {
			pData[i] |= 1 << channel;
		} else {
			pData[i] &= ~(1 << channel);
		}
		tpwm--;
	}
}


/**
 * @brief Запуск передачи DMA
 * @retval true - успех, в противном случае false
 *
 * Конфигурирование расширителя портов для реле, выбор регистра (GPIOA) и запуск циклической передачи DMA.
 */
bool IOExpanders::StartDMA() {
TickType_t timeout;

	ConfigureDMA();
	I2C_DMALastTransferCmd(m_pI2C, DISABLE);

	I2C_GenerateSTART(m_pI2C, ENABLE);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_MODE_SELECT, "START failed\n", false);

	I2C_Send7bitAddress(m_pI2C, IOE_ADDR_RELAYS, I2C_Direction_Transmitter);
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, "Master failed\n", false);

	I2C_SendData(m_pI2C, MCP23017<BANK_1>::GPIOXA());
	WAIT_EVENT_WITH_TO(m_pI2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING, "Transmitting failed\n", false);

	I2C_DMACmd(m_pI2C, ENABLE);
	DMA_Cmd(DMA1_Channel6, ENABLE);
	return true;
}


/**
 * @brief Остановка DMA
 *
 * Остановка DMA и выставление на шину STOP.
 */
void IOExpanders::StopDMA() {
	DMA_Cmd(DMA1_Channel6, DISABLE);
	I2C_DMACmd(m_pI2C, DISABLE);

	I2C_GenerateSTOP(m_pI2C, ENABLE);
	while (m_pI2C->SR2 & I2C_SR2_MSL) {}
}


void IOExpanders::InitHardware() {
	RCC_APB1PeriphClockCmd(I2C1_CLK, ENABLE);
	RCC_APB1PeriphResetCmd(I2C1_CLK, ENABLE);
	RCC_APB1PeriphResetCmd(I2C1_CLK, DISABLE);

GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = I2C1_SCL_PIN | I2C1_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(I2C1_PORT, &GPIO_InitStructure);

I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 400000;
	I2C_DeInit(m_pI2C);
	I2C_Init(m_pI2C, &I2C_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource9);
EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line9;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


bool IOExpanders::ResetHardware() {
	I2C_DeInit(m_pI2C);
	RCC_APB1PeriphClockCmd(I2C1_CLK, DISABLE);

GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = I2C1_SCL_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(I2C1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = I2C1_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(I2C1_PORT, &GPIO_InitStructure);

bool ret = false;

TickType_t timeout = xTaskGetTickCount() + 50;
	while (timeout > xTaskGetTickCount()) {
		vTaskDelay(1);
		GPIO_WriteBit(I2C1_PORT, I2C1_SCL_PIN, Bit_RESET);
		if (GPIO_ReadInputDataBit(I2C1_PORT, I2C1_SDA_PIN) == Bit_SET) {
			ret = true;
			break;
		}

		vTaskDelay(1);
		GPIO_WriteBit(I2C1_PORT, I2C1_SCL_PIN, Bit_SET);
		if (GPIO_ReadInputDataBit(I2C1_PORT, I2C1_SDA_PIN) == Bit_SET) {
			ret = true;
			break;
		}
	}

	if (ret == true)
		InitHardware();
	return ret;
}


void EXTI9_5_IRQHandler() {
	if (EXTI_GetITStatus(EXTI_Line9) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line9);
	}
}

