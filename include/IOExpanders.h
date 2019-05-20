/**
  ******************************************************************************
  * @file    include/IOExpanders.h
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    12 марта 2018
  * @brief   Заголовок от IOExpanders.cpp
  ******************************************************************************
  * @attention
  * Всем очко!!
  *
  * <h2><center>&copy; 2018 ООО "РПС"</center></h2>
  ******************************************************************************
  */


#ifndef IOEXPANDERS_H_
#define IOEXPANDERS_H_

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <timers.h>
#include <stm32f10x.h>
#include "common.h"
#include "Wake.h"

#define BANK_0		false
#define BANK_1		!BANK_0


typedef struct IOECommand_ {
	Command cmd;
	uint8_t data[2];
} IOECommand;


class IOExpanders : public TaskBase {
public:
	static IOExpanders &Instance() {
		static IOExpanders s;
		return s;
	}

	void task();
	static void task_expanders(void *param) {
		static_cast<IOExpanders *>(param)->task();
		while (1)
			vTaskDelay(portMAX_DELAY);
	}

	SemaphoreHandle_t xSemaphoreDiscretIrq;
	SemaphoreHandle_t xSemaphoreTimer;
	QueueHandle_t xQueueCommands;
	QueueHandle_t xQueueResponce;

	QueueSetHandle_t xQueueSet;

private:
	IOExpanders();
	~IOExpanders() {}
	IOExpanders(IOExpanders const &) = delete;
	IOExpanders& operator= (IOExpanders const &) = delete;

protected:
	void CalcPWMData(uint8_t *pData, uint8_t channel, uint8_t value);
	bool StartDMA();
	void StopDMA();

	void ConfigureExpanders();
	bool WriteRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
	bool WriteRegister(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, size_t len);
	bool ReadRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t &readed);
	bool ReadRegister(uint8_t dev_addr, uint8_t reg_addr, uint16_t &readed);

	bool ResetHardware();

	TimerHandle_t xTimer;

	bool RelaysWrite(uint8_t data_l, uint8_t data_h);
	bool RelaysWrite(uint16_t data);
	bool RelaysWriteOnes(uint16_t data);
	bool RelaysWriteZeros(uint16_t data);

	uint16_t RelaysRead();
	uint16_t RelaysReadIDR();

	uint16_t PowerRead();
	bool PowerWrite(uint16_t data);
	bool PowerWrite(uint8_t gpioa, uint8_t gpiob);
	bool PowerWriteOnes(uint16_t data);
	bool PowerWriteZeros(uint16_t data);

private:
	I2C_TypeDef *m_pI2C = I2C1;
	uint16_t relays_odr_last = 0;

	void ConfigureDMA();
	void InitHardware();

	void SelfTest();
	bool Test();
};


/// Расширитель управления питанием
#define IOE_ADDR_POWER				0x40
#define IOE_ADDR_STATUS				0x42
#define IOE_ADDR_RELAYS				0x44
#define IOE_ADDR_INPUTS				0x46

template <bool bank>
class MCP23017 {
public:
	static uint8_t IODIRA() 	{return (bank) ? 0x00 : 0x00;}
	static uint8_t IPOLA() 		{return (bank) ? 0x01 : 0x02;}
	static uint8_t GPINTENA() 	{return (bank) ? 0x02 : 0x04;}
	static uint8_t DEFVALA()	{return (bank) ? 0x03 : 0x06;}
	static uint8_t INTCONA()	{return (bank) ? 0x04 : 0x08;}
	static uint8_t IOCONA()		{return (bank) ? 0x05 : 0x0A;}
	static uint8_t GPPUA()		{return (bank) ? 0x06 : 0x0C;}
	static uint8_t INTFA()		{return (bank) ? 0x07 : 0x0E;}
	static uint8_t INTCAPA()	{return (bank) ? 0x08 : 0x10;}
	static uint8_t GPIOXA()		{return (bank) ? 0x09 : 0x12;}
	static uint8_t OLATA()		{return (bank) ? 0x0A : 0x14;}

	static uint8_t IODIRB() 	{return (bank) ? 0x10 : 0x01;}
	static uint8_t IPOLB() 		{return (bank) ? 0x11 : 0x03;}
	static uint8_t GPINTENB() 	{return (bank) ? 0x12 : 0x05;}
	static uint8_t DEFVALB()	{return (bank) ? 0x13 : 0x07;}
	static uint8_t INTCONB()	{return (bank) ? 0x14 : 0x09;}
	static uint8_t IOCONB()		{return (bank) ? 0x15 : 0x0B;}
	static uint8_t GPPUB()		{return (bank) ? 0x16 : 0x0D;}
	static uint8_t INTFB()		{return (bank) ? 0x17 : 0x0F;}
	static uint8_t INTCAPB()	{return (bank) ? 0x18 : 0x11;}
	static uint8_t GPIOXB()		{return (bank) ? 0x19 : 0x13;}
	static uint8_t OLATB()		{return (bank) ? 0x1A : 0x15;}
};

#define MCP23017_IOCON_BANK(b)					(b << 7)
#define MCP23017_IOCON_MIRROR(b)				(b << 6)
#define MCP23017_IOCON_SEQOP(b)					(b << 5)
#define MCP23017_IOCON_DISSLW(b)				(b << 4)
#define MCP23017_IOCON_HAEN(b)					(b << 3)
#define MCP23017_IOCON_ODR(b)					(b << 2)
#define MCP23017_IOCON_INTPOL(b)				(b << 1)


#ifdef __cplusplus
extern "C" {
#endif

void EXTI9_5_IRQHandler();

#ifdef __cplusplus
}
#endif

#endif /* IOEXPANDERS_H_ */
