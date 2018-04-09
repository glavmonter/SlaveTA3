/**
  ******************************************************************************
  * @file    include/Wake.cpp
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    21 марта 2018
  * @brief   Реализация класса Wake
  ******************************************************************************
  * @attention
  * Всем очко!!
  *
  * <h2><center>&copy; 2016 ФГУП "18 ЦНИИ" МО РФ</center></h2>
  ******************************************************************************
  */

#include "SEGGER_RTT.h"
#include "Wake.h"


/// Frame End
#define WAKE_CODE_FEND			0xC0
/// Frame Escape
#define WAKE_CODE_FESC			0xDB
/// Transposed Frame End
#define WAKE_CODE_TFEND			0xDC
/// Transposed Frame Escape
#define WAKE_CODE_TFESC			0xDD

#define CRC_INIT 0x00 			//Innitial CRC value



static void Do_Crc8(uint8_t b, uint8_t *crc);

Wake::Wake(uint8_t address, USART_TypeDef *usart) :
	m_iAddress(address),
	m_pUSART(usart) {
}


Wake::Status Wake::ProcessInByte(uint8_t data_byte) {
Wake::Status ret = Wake::STA_INIT;

	if (data_byte == WAKE_CODE_FEND) {
		Rx_Pre = data_byte;
		Rx_Crc = CRC_INIT;
		Rx_FSM = WAIT_ADDR;
		Do_Crc8(data_byte, &Rx_Crc);
		return Wake::STA_INIT;
	}

	if (Rx_FSM == WAIT_FEND)
		return Wake::STA_INIT;

	uint8_t Pre = Rx_Pre;
	Rx_Pre = data_byte;
	if (Pre == WAKE_CODE_FESC) {
		if (data_byte == WAKE_CODE_TFESC)
			data_byte = WAKE_CODE_FESC;
		else if (data_byte == WAKE_CODE_TFEND)
			data_byte = WAKE_CODE_FEND;
		else {
			Rx_FSM = WAIT_FEND;
			// CMD
			return Wake::STA_INIT;
		}
	} else {
		if (data_byte == WAKE_CODE_FESC)
			return Wake::STA_INIT;
	}

	switch (Rx_FSM) {
	case WAIT_ADDR: {
		if (data_byte & 0x80) {	// Адрес
			// Address valid. Upper bit is High
			data_byte = data_byte & 0x7F;
			if (!data_byte || data_byte == m_iAddress) {	// Свой адрес
				Do_Crc8(data_byte, &Rx_Crc);
				Rx_FSM = WAIT_CMD;
				break;
			}
			Rx_FSM = WAIT_FEND;
			break;
		}
		Rx_FSM = WAIT_CMD;
	}

	case WAIT_CMD: {
		if (data_byte & 0x80) {
			// CMD not valid. Upper bit is High
			Rx_FSM = WAIT_FEND;
			break;
		}
		Rx_Cmd = data_byte;
		Do_Crc8(data_byte, &Rx_Crc);
		Rx_FSM = WAIT_NBT;
		break;
	}

	case WAIT_NBT: {
		if (data_byte > FRAME_SIZE) {
			Rx_FSM = WAIT_FEND;
			break;
		}

		Rx_Nbt = data_byte;
		Do_Crc8(data_byte, &Rx_Crc);
		Rx_Ptr = 0;
		Rx_FSM = WAIT_DATA;
		break;
	}

	case WAIT_DATA: {
		if (Rx_Ptr < Rx_Nbt) {
			RxData[Rx_Ptr++] = data_byte;
			Do_Crc8(data_byte, &Rx_Crc);
			break;
		}
		if (data_byte != Rx_Crc) {
			Rx_FSM = WAIT_FEND;
			break;
		}
		Rx_FSM = WAIT_FEND;
		ret = Wake::STA_READY;
		break;
	}
	}
	return ret;
}


Wake::Status Wake::ProcessTx(uint8_t adr, uint8_t cmd, uint8_t len) {
uint8_t crc = CRC_INIT;

	Do_Crc8(WAKE_CODE_FEND, &crc);
	m_pUSART->DR = WAKE_CODE_FEND;
	while (USART_GetFlagStatus(m_pUSART, USART_FLAG_TC) == RESET) {}


	Do_Crc8(adr, &crc);
	adr |= 0x80;
	StuffTx(adr);

	Do_Crc8(cmd, &crc);
	StuffTx(cmd);

	Do_Crc8(len, &crc);
	StuffTx(len);

	for (uint8_t i = 0; i < len; i++) {
		Do_Crc8(TxData[i], &crc);
		StuffTx(TxData[i]);
	}

	StuffTx(crc);
	_log("Crc: %02x\n", crc);

	return STA_READY;
}

void Wake::StuffTx(uint8_t data) {
	if (data == WAKE_CODE_FEND) {
		m_pUSART->DR = WAKE_CODE_FESC;
		while (USART_GetFlagStatus(m_pUSART, USART_FLAG_TC) == RESET) {}

		m_pUSART->DR = WAKE_CODE_TFEND;
		while (USART_GetFlagStatus(m_pUSART, USART_FLAG_TC) == RESET) {}
		return;

	} else if (data == WAKE_CODE_FESC) {
		m_pUSART->DR = WAKE_CODE_FESC;
		while (USART_GetFlagStatus(m_pUSART, USART_FLAG_TC) == RESET) {}

		m_pUSART->DR = WAKE_CODE_TFESC;
		while (USART_GetFlagStatus(m_pUSART, USART_FLAG_TC) == RESET) {}
		return;

	}

	m_pUSART->DR = data;
	while (USART_GetFlagStatus(m_pUSART, USART_FLAG_TC) == RESET) {}
}

static void Do_Crc8(uint8_t b, uint8_t *crc) {
	for (uint8_t i = 0; i < 8; b = b >> 1, i++)
        if ((b ^ *crc) & 1)
            *crc = ((*crc ^ 0x18) >> 1) | 0x80;
        else
            *crc = (*crc >> 1) & ~0x80;
}
