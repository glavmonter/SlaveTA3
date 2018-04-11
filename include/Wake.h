/**
  ******************************************************************************
  * @file    include/Wake.h
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    21 марта 2018
  * @brief   Описание класса Wake
  ******************************************************************************
  * @attention
  * Всем очко!!
  *
  * <h2><center>&copy; 2016 ФГУП "18 ЦНИИ" МО РФ</center></h2>
  ******************************************************************************
  */

#ifndef WAKE_H_
#define WAKE_H_

#include <stm32f10x.h>
#include <stdint.h>

#define FRAME_SIZE				32

typedef enum Command_ {
	CMD_NOP 						= 0x00,
	CMD_ERR 						= 0x01,
	CMD_ECHO 						= 0x02,
	CMD_INFO 						= 0x03, /// Информация об устройстве
	CMD_BOOT						= 0x04,	/// Запуск загрузчика для прошивки.

	CMD_PORTS_IDR					= 0x06,	/// Считывание состояния PORTA и PORTB
	CMD_PORTS_ODRR					= 0x07,	/// Считать значение выходов PORTA и PORTB
	CMD_PORTS_ODRW					= 0x08, /// Записать значение на выход PORTA и PORTB
	CMD_PORTS_SET					= 0x09, /// Установить биты PORTA и PORTB
	CMD_PORTS_RESET 				= 0x0A,	/// Сбросить биты PORTA и PORTB

	CMD_WIEGAND						= 0x0B,	/// Считывание Wiegand

	CMD_RELAYS_IDR					= 0x0C,	/// Дискретные входы
	CMD_RELAYS_ODRR					= 0x0D, /// Считывание текущего состояния выходных реле
	CMD_RELAYS_ODRW					= 0x0E,	/// Запись данных в выходные реле
	CMD_RELAYS_SET					= 0x0F, /// Установить биты данных в выходных реле
	CMD_RELAYS_RESET				= 0x10, /// Сбросить биты данных выходных реле

	CMD_POWERS_ODRR					= 0x11,	/// Считывание значения управляемых нагрузок
	CMD_POWERS_ORRW					= 0x12, /// Записать значение управляемых нагрузок
	CMD_POWERS_SET					= 0x13, /// Включить нагрузки
	CMD_POWERS_RESET				= 0x14,	/// Отключить нагрузки

} Command;

class Wake {
public:
	typedef enum Status_ {
		STA_INIT,
		STA_RECEIVING,
		STA_READY
	} Status;

public:
	Wake(uint8_t address, USART_TypeDef *usart);

	Status ProcessInByte(uint8_t data_byte);
	Command GetCommand() {return (Command)Rx_Cmd;}

	uint8_t RxData[FRAME_SIZE];

	uint8_t TxData[FRAME_SIZE];
	Status ProcessTx(uint8_t adr, uint8_t cmd, uint8_t len);

private:
	enum FsmRxState {
		WAIT_FEND,     /// ожидание приема FEND
		WAIT_ADDR,     /// ожидание приема адреса
		WAIT_CMD,      /// ожидание приема команды
		WAIT_NBT,      /// ожидание приема количества байт в пакете
		WAIT_DATA,     /// прием данных
		WAIT_CRC,      /// ожидание окончания приема CRC
		WAIT_CARR 	   /// ожидание несущей
	};

	enum FsmTxState {
		SEND_IDLE,		/// Бездействие
		SEND_ADDR,		/// Отправка адреса
		SEND_CMD,		/// Отправка команды
		SEND_NBT,		/// Отправка количества байт в пакете
		SEND_DATA,		/// Отправка данных
		SEND_CRC,		/// Отправка CRC
		SEND_END		/// Окончание передачи
	};

	uint8_t m_iAddress;
	uint8_t Rx_FSM = WAIT_FEND;
	uint8_t Rx_Pre;	/// Предыдущий принятый байт
	uint8_t Rx_Cmd;
	uint8_t Rx_Nbt;
	uint8_t Rx_Crc;
	uint8_t Rx_Ptr = 0;

	void StuffTx(uint8_t data);
	USART_TypeDef *m_pUSART;
};


#endif /* WAKE_H_ */
