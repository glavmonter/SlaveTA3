/**
  ******************************************************************************
  * @file    include/Commands.h
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    15 мая 2018
  * @brief   Команды Wake
  ******************************************************************************
  * @attention
  * Всем очко!!
  *
  * <h2><center>&copy; 2016 ФГУП "18 ЦНИИ" МО РФ</center></h2>
  ******************************************************************************
  */

#ifndef COMMANDS_H_
#define COMMANDS_H_


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

	CMD_POWERS_IDR					= 0x11, /// Считать значение статуса
	CMD_POWERS_ODRR					= 0x12,	/// Считывание значения управляемых нагрузок
	CMD_POWERS_ODRW					= 0x13, /// Записать значение управляемых нагрузок
	CMD_POWERS_SET					= 0x14, /// Включить нагрузки
	CMD_POWERS_RESET				= 0x15,	/// Отключить нагрузки

	CMD_CLIMATE_GET					= 0x16, /// Информация о климатике
	CMD_CLIMATE_SET					= 0x17, /// Выбор режима, включение

	CMD_RS485_INFO					= 0x20, /// RS485 Информация
} Command;


#endif /* COMMANDS_H_ */
