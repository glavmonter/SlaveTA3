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

#define MAX_PWM_DATA		32
uint8_t pwm_data[MAX_PWM_DATA] __attribute__((aligned(8)));


static void TimerCallback(TimerHandle_t xTimer) {
	xSemaphoreGive(static_cast<SemaphoreHandle_t>(pvTimerGetTimerID(xTimer)));
}


IOExpanders::IOExpanders() {
	memset(pwm_data, 0, sizeof(pwm_data));
	InitHardware();

	xSemaphoreDiscretIrq = xSemaphoreCreateBinary();
	assert_param(xSemaphoreDiscretIrq);
#if (configUSE_TRACE_FACILITY == 1)
	vTraceSetSemaphoreName(xSemaphoreDiscretIrq, "DiscretIRQ");
#endif
	xSemaphoreTake(xSemaphoreDiscretIrq, 0);

	xSemaphoreTimer = xSemaphoreCreateBinary();
	assert_param(xSemaphoreTimer);
#if (configUSE_TRACE_FACILITY == 1)
	vTraceSetSemaphoreName(xSemaphoreTimer, "DiscretTimer");
#endif
	xSemaphoreTake(xSemaphoreTimer, 0);

	xQueueCommands = xQueueCreate(5, sizeof(IOECommand));
	assert_param(xQueueCommands);
#if (configUSE_TRACE_FACILITY == 1)
	vTraceSetQueueName(xQueueCommands, "DiscretCommands");
#endif

	xQueueSet = xQueueCreateSet(1 + 5 + 2);
	assert_param(xQueueSet);
#if (configUSE_TRACE_FACILITY == 1)
	vTraceSetQueueName(xQueueSet, "QueueSet");
#endif
	xQueueAddToSet(xSemaphoreDiscretIrq, xQueueSet);
	xQueueAddToSet(xSemaphoreTimer, xQueueSet);
	xQueueAddToSet(xQueueCommands, xQueueSet);

	xQueueResponce = xQueueCreate(1, sizeof(uint16_t));
	assert_param(xQueueResponce);
#if (configUSE_TRACE_FACILITY == 1)
	vTraceSetQueueName(xQueueResponce, "DiscretResponce");
#endif

	xTimer = xTimerCreate("IOE", 5, pdTRUE, xSemaphoreTimer, TimerCallback);
	assert_param(xTimer);

	xTaskCreate(task_expanders, "IOExp", configMINIMAL_STACK_SIZE*2, this, configTASK_IOE_PRIORITY, &handle);
	assert_param(handle);
}


void IOExpanders::task() {
	_log("Start\n");

	ResetHardware();

	ConfigureExpanders();
	vTaskDelay(5);

uint16_t idr = 0;
uint16_t idr_last = 0;

uint16_t resp = false;

//	xTimerStart(xTimer, 0);
	xSemaphoreGive(xSemaphoreTimer);

	for (;;) {
		QueueSetMemberHandle_t event = xQueueSelectFromSet(xQueueSet, portMAX_DELAY);
		if (event == xSemaphoreDiscretIrq) {
			xSemaphoreTake(event, 0);
			vTaskDelay(5);
			idr = RelaysReadIDR();
			if (idr != idr_last) {
				_log("IDRi: 0x%04X\n", idr);
				idr_last = idr;
			}

		} else if (event == xQueueCommands) {
			IOECommand cmd;
			xQueueReceive(event, &cmd, 0);

			switch (cmd.cmd) {
			case CMD_RELAYS_ODRR:
//				resp = RelaysRead();
				resp = relays_odr_last;
				xQueueOverwrite(xQueueResponce, &resp);
				break;

			case CMD_RELAYS_ODRW:
				resp = RelaysWrite((cmd.data[0] << 8) | cmd.data[1]);
				xQueueOverwrite(xQueueResponce, &resp);
				break;

			case CMD_RELAYS_RESET:
				resp = RelaysWriteZeros((cmd.data[0] << 8) | cmd.data[1]);
				xQueueOverwrite(xQueueResponce, &resp);
				break;

			case CMD_RELAYS_SET:
				resp = RelaysWriteOnes((cmd.data[0] << 8) | cmd.data[1]);
				xQueueOverwrite(xQueueResponce, &resp);
				break;

			case CMD_RELAYS_IDR:
				xQueueOverwrite(xQueueResponce, &idr_last);
				break;

			case CMD_POWERS_IDR:
				resp = 0;
				xQueueOverwrite(xQueueResponce, &resp);
				break;

			case CMD_POWERS_ODRR:
				resp = PowerRead();
				xQueueOverwrite(xQueueResponce, &resp);
				break;

			case CMD_POWERS_ODRW:
				resp = PowerWrite((cmd.data[0] << 8) | cmd.data[1]);
				xQueueOverwrite(xQueueResponce, &resp);
				break;

			case CMD_POWERS_SET:
				resp = PowerWriteOnes((cmd.data[0] << 8) | cmd.data[1]);
				xQueueOverwrite(xQueueResponce, &resp);
				break;

			case CMD_POWERS_RESET:
				resp = PowerWriteZeros((cmd.data[0] << 8) | cmd.data[1]);
				xQueueOverwrite(xQueueResponce, &resp);
				break;

			default:
				_log("Unknown Command\n");
				break;
			}

		} else if (event == xSemaphoreTimer){
			xSemaphoreTake(event, 0);

			idr = RelaysReadIDR();
			if (idr != idr_last) {
				_log("IDR: 0x%04X\n", idr);
				idr_last = idr;
			}

		} else {
			_log("Hui\n");
		}
	}
}


uint16_t IOExpanders::PowerRead() {
bool ret;
uint8_t retries;
uint8_t gpioa, gpiob;

	retries = 10;
	do {
		ret = ReadRegister(IOE_ADDR_POWER, MCP23017<BANK_1>::GPIOXA(), gpioa);
		if (ret == false)
			_log("Read Power1 Err {%d}\n", retries);
		retries--;
	} while ((ret == false) and (retries > 0));

	retries = 10;
	do {
		ret = ReadRegister(IOE_ADDR_POWER, MCP23017<BANK_1>::GPIOXB(), gpiob);
		if (ret == false)
			_log("Read Power2 Err {%d}\n", retries);
		retries--;
	} while ((ret == false) and (retries > 0));

uint16_t out   = ((gpiob & 0x10) >> 4) |
				 ((gpiob & 0x08) >> 2) |
				 ((gpiob & 0x04) >> 0) |
				 ((gpiob & 0x02) << 2) |
				 ((gpiob & 0x01) << 4);

uint16_t gpioa_ = ((gpioa & 0x80) >> 7) |
				 ((gpioa & 0x40) >> 5) |
				 ((gpioa & 0x20) >> 3) |
				 ((gpioa & 0x10) >> 1) |
				 ((gpioa & 0x08) << 1);

	return out | (gpioa_ << 5);
}


bool IOExpanders::PowerWrite(uint8_t gpioa, uint8_t gpiob) {
bool ret;
uint8_t retries;
	retries = 10;
	do {
		ret = WriteRegister(IOE_ADDR_POWER, MCP23017<BANK_1>::GPIOXA(), gpioa);
		if (ret == false)
			_log("Write1 Power Err {%d}\n", retries);
		retries--;
	} while ((ret == false) and (retries > 0));
	if (retries == 0)
		return false;

	retries = 10;
	do {
		ret = WriteRegister(IOE_ADDR_POWER, MCP23017<BANK_1>::GPIOXB(), gpiob);
		if (ret == false)
			_log("Write2 Power Err {%d}\n", retries);
	} while ((ret == false) and (retries > 0));
	if (retries == 0)
		return false;

	return true;
}


bool IOExpanders::PowerWrite(uint16_t data) {
uint8_t gpioa = ((data & 0x0020) << 2) |
				((data & 0x0040) << 0) |
				((data & 0x0080) >> 2) |
				((data & 0x0100) >> 4) |
				((data & 0x0200) >> 6);

uint8_t gpiob = ((data & 0x0001) << 4) |
				((data & 0x0002) << 2) |
				((data & 0x0004) << 0) |
				((data & 0x0008) >> 2) |
				((data & 0x0010) >> 4);

	return PowerWrite(gpioa, gpiob);
}

bool IOExpanders::PowerWriteOnes(uint16_t data) {
uint16_t odr = PowerRead();
	odr |= data;
	return PowerWrite(data);
}

bool IOExpanders::PowerWriteZeros(uint16_t data) {
uint16_t odr = PowerRead();
	odr &= ~data;
	return PowerWrite(odr);
}


/**
 * @brief Записать данные в регистры выходных реле
 * @param data_l - младшие 5 выходов
 * @param data_h - старшие 5 выходов
 * @retval true - в случае успешности
 */
bool IOExpanders::RelaysWrite(uint8_t data_l, uint8_t data_h) {
bool ret;
uint8_t retries;

	retries = 10;
	do {
		ret = WriteRegister(IOE_ADDR_RELAYS, MCP23017<BANK_1>::GPIOXA(), data_l);
		if (ret == false)
			_log("Write Relay1 Err {%d}\n", retries);
		retries--;
	} while ((ret == false) and (retries > 0));
	if (retries == 0)
		return false;

	retries = 10;
	do {
		ret = WriteRegister(IOE_ADDR_RELAYS, MCP23017<BANK_1>::GPIOXB(), data_h);
		if (ret == false)
			_log("Write Relay2 Err {%d}\n", retries);
		retries--;
	} while ((ret == false) and (retries > 0));
	if (retries == 0)
		return false;

	return true;
}


/**
 * @brief Записать данные в регистры выходных реле
 * @param data - 10 бит данных выходных реле
 * @retval true - в случае успешности
 */
bool IOExpanders::RelaysWrite(uint16_t data) {
uint8_t data_l, data_h;
	data_l = data & 0x1F;
	data_h = (data & 0x3E0) >> 5;
	relays_odr_last = data;
	return RelaysWrite(data_l, data_h);
}


bool IOExpanders::RelaysWriteOnes(uint16_t data) {
uint16_t odr = RelaysRead();
	odr |= data;
	relays_odr_last = odr;
	return RelaysWrite(odr);
}

bool IOExpanders::RelaysWriteZeros(uint16_t data) {
uint16_t odr = RelaysRead();
	odr &= ~data;
	relays_odr_last = odr;
	return RelaysWrite(odr);
}


uint16_t IOExpanders::RelaysRead() {
uint8_t data_l, data_h;

bool ret;
uint8_t retries;
	_log("RelaysRead\n");
	retries = 10;
	do {
		ret = ReadRegister(IOE_ADDR_RELAYS, MCP23017<BANK_1>::GPIOXA(), data_l);
		if (ret == false)
			_log("Read1 {%d}: Err\n", retries);
		retries--;
	} while ((ret == false) and (retries > 0));

	retries = 10;
	do {
		ret = ReadRegister(IOE_ADDR_RELAYS, MCP23017<BANK_1>::GPIOXB(), data_h);
		if (ret == false)
			_log("Read2 {%d}: Err\n", retries);
		retries--;
	} while ((ret == false) and (retries > 0));

	return (data_h << 5) | data_l;
}


uint16_t IOExpanders::RelaysReadIDR() {
bool ret;
uint8_t retries;
uint16_t idr;

	retries = 10;
	do {
		ret = ReadRegister(IOE_ADDR_INPUTS, MCP23017<BANK_0>::GPIOXA(), idr);
		if (ret == false)
			_log("Read16 {%d}: Err\n", retries);
		retries--;
	} while ((ret == false) and (retries > 0));

uint8_t low = ((idr & 0x80) >> 7) |
			  ((idr & 0x40) >> 5) |
			  ((idr & 0x20) >> 3) |
			  ((idr & 0x10) >> 1) |
			  ((idr & 0x08) << 1);

	low |=	  ((idr & 0x0100) >> 3) |
			  ((idr & 0x0200) >> 3) |
			  ((idr & 0x0400) >> 3);

uint8_t high = ((idr & 0x0800) >> 11) |
			   ((idr & 0x1000) >> 11);

	return (high << 8) | low;
}


void IOExpanders::SelfTest() {
	StartDMA();

uint8_t ch = 4;

	for (int i = 0; i <= MAX_PWM_DATA; i++) {
		vTaskDelay(100);
		CalcPWMData(pwm_data, ch, i);
	}
	vTaskDelay(100);

	CalcPWMData(pwm_data, ch, 0);
	vTaskDelay(100);

	CalcPWMData(pwm_data, ch, MAX_PWM_DATA);
	vTaskDelay(100);

	StopDMA();

	vTaskDelay(100);
	WriteRegister(IOE_ADDR_RELAYS, MCP23017<BANK_1>::GPIOXA(), 0x00);

	vTaskDelay(100);
	WriteRegister(IOE_ADDR_RELAYS, MCP23017<BANK_1>::GPIOXA(), 0xFF);
	vTaskDelay(100);
	WriteRegister(IOE_ADDR_RELAYS, MCP23017<BANK_1>::GPIOXA(), 0x00);

	vTaskDelay(100);
	WriteRegister(IOE_ADDR_RELAYS, MCP23017<BANK_1>::GPIOXB(), 0xFF);
	WriteRegister(IOE_ADDR_RELAYS, MCP23017<BANK_1>::GPIOXB(), 0x00);

TickType_t stoptime = xTaskGetTickCount() + 10000;
	for (;;) {
		uint16_t r16;
		ReadRegister(IOE_ADDR_INPUTS, MCP23017<BANK_0>::GPIOXA(), r16);
		_log("Input: 0x%04X\n", r16);
		vTaskDelay(250);

		if (xTaskGetTickCount() > stoptime)
			break;
	}
}


void IOExpanders::ConfigureExpanders() {
bool ret;
	/* Включаем режим двух банков и отключаем режим инкрементирования адреса */
	ret = WriteRegister(IOE_ADDR_RELAYS, MCP23017<BANK_0>::IOCONA(),
			MCP23017_IOCON_BANK(1) | MCP23017_IOCON_SEQOP(1));

	if (ret == false) {
		ret = ResetHardware();
		ret = WriteRegister(IOE_ADDR_RELAYS, MCP23017<BANK_0>::IOCONA(),
				MCP23017_IOCON_BANK(1) | MCP23017_IOCON_SEQOP(1));
	}

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


	// Set Powers to Output zero
	WriteRegister(IOE_ADDR_POWER, MCP23017<BANK_1>::GPIOXA(), 0x00);
	WriteRegister(IOE_ADDR_POWER, MCP23017<BANK_1>::IODIRA(), 0x00);
	WriteRegister(IOE_ADDR_POWER, MCP23017<BANK_1>::GPIOXB(), 0x00);
	WriteRegister(IOE_ADDR_POWER, MCP23017<BANK_1>::IODIRB(), 0x00);
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
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pwm_data;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = MAX_PWM_DATA * sizeof(pwm_data[0]);

	DMA_DeInit(DMA1_Channel6);
	DMA_Init(DMA1_Channel6, &DMA_InitStructure);
}

/**
 * @brief Вычисление регистра ШИМ
 * @param pData - Массив циклических данных, что передается через DMA
 * @param channel - Номер канала ШИМ [0..4]
 * @param value - Величина ШИМ [0..MAX_PWM_DATA]
 *
 * В функции вычисляется значения элементов массива для обеспечения корректных данных для реализации ШИМ.
 * В соответствии с каналом выставляется необходимое количество единиц в элементах массива pData.
 */
void IOExpanders::CalcPWMData(uint8_t *pData, uint8_t channel, uint8_t value) {
int32_t tpwm = value;
	if (channel > 4)
		return;

	for (int i = 0; i < MAX_PWM_DATA; i++) {
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
BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (EXTI_GetITStatus(EXTI_Line9) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line9);
		xSemaphoreGiveFromISR(IOExpanders::Instance().xSemaphoreDiscretIrq, &xHigherPriorityTaskWoken);
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

