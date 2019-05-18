/**
  ******************************************************************************
  * @file    include/maintask.h
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V1.0.0
  * @date    21 сентября 2016
  * @brief   Реализация класса главного потока
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
#include "maintask.h"
#include "IOExpanders.h"
#include "Climate.h"

#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "slave.pb.h"


#define DEVICE_NAME			"Slave-arm"

__attribute__ ((section(".vars_version_sect")))
__IO const uint32_t device_version = 0x00000001;

__attribute__ ((section(".vars_key_sect")))
__IO const uint8_t key[] = {0x64, 0xCA, 0x56, 0xBA, 0x15, 0x57, 0x63, 0x39, 0xDA, 0x57, 0x40, 0x21};



static void TimerCallback(TimerHandle_t xTimer) {
SemaphoreHandle_t sem = static_cast<SemaphoreHandle_t>(pvTimerGetTimerID(xTimer));
	xSemaphoreGive(sem);
}


typedef void (*pFunction)(void);
#define BOOTLOADER_ADDRESS				0x1FFFF000


uint32_t StartExternalApp(uint32_t address) {
	if (((*(__IO uint32_t *)address) & 0x2FFC0000) == 0x20000000) {
		__IO uint32_t JumpAddress = *(__IO uint32_t *)(address + 4);
		pFunction Jump_to_app = (pFunction)JumpAddress;
		__set_MSP(*(__IO uint32_t *)address);
		Jump_to_app();
		return 0;
	} else {
		return 1;
	}
}


/**
 * @brief Конструктор.
 *
 * Создание таска ОСРВ с минимальным приоритетом (tskIDLE_PRIORITY)
 */
MainTask::MainTask() {
	SEGGER_RTT_printf(0, "Free: %d\n", xPortGetFreeHeapSize());
	xTaskCreate(task_main, "Main", configTASK_MAIN_STACK, this, configTASK_MAIN_PRIORITY, &handle);
	assert_param(handle);
}

/**
 * \brief Включение планировщика ОСРВ
 */
int MainTask::run() {
	vTaskStartScheduler();
	return 0;
}

const uint8_t wake_in[] = {0xC0, 0x85, 0x06, 0x07, 0x20, 0xDB, 0xDC, 0x22, 0x23, 0xF5, 0xDB, 0xDD, 0x05, 0x29};

/**
 * \brief Рабочие цикл таска.
 *
 * Рабочий цикл MainTask. Создание дочерних тасков и обслуживание связей между ними
 */
void MainTask::task() {
	_log("FreeHeap: %d\n", xPortGetFreeHeapSize());
	Init();
	xTimerStart(xTimer, 5);

	sWiegand1.ValidTime = 0;
	sWiegand2.ValidTime = 0;

uint8_t received_byte = 0;
WiegandStruct wiegand;
IOECommand ioe_cmd;
uint16_t ioe_responce;

	for (;;) {
		QueueSetHandle_t event = xQueueSelectFromSet(xQueueSet, portMAX_DELAY);
		if (event == xTimerSemaphore) {
			xSemaphoreTake(event, 0);

		} else if (event == xQueueUsartRx) {
			xQueueReceive(event, &received_byte, 0);

			if (m_pWake->ProcessInByte(received_byte) == Wake::STA_READY) {
				Command cmd = m_pWake->GetCommand();
				_log("Cmd: 0x%02X\n", cmd);

				switch (cmd) {
				case CMD_INFO:
					ProcessCmdInfo();
					break;

				case CMD_BOOT:
					ProcessBoot();
					break;

				case CMD_PORTS_IDR:
				case CMD_PORTS_ODRR:
				case CMD_PORTS_ODRW:
				case CMD_PORTS_SET:
				case CMD_PORTS_RESET:
					ProcessPORTs(cmd);
					break;

				case CMD_WIEGAND:
					if (xQueueReceive(xQueueWiegand, &wiegand, 1) == pdTRUE)
						ProcessWiegand(CMD_WIEGAND, wiegand);
					else
						m_pWake->ProcessTx(0x01, CMD_ERR, 0);
					break;

				case CMD_RELAYS_ODRW:
				case CMD_RELAYS_SET:
				case CMD_RELAYS_RESET:
					ioe_cmd.cmd = cmd;
					ioe_cmd.data[0] = m_pWake->RxData[0];
					ioe_cmd.data[1] = m_pWake->RxData[1];
					xQueueSend(IOExpanders::Instance().xQueueCommands, &ioe_cmd, 1);

					if (xQueueReceive(IOExpanders::Instance().xQueueResponce, &ioe_responce, 10) == pdTRUE) {
						_log("Set OK\n");
						m_pWake->ProcessTx(0x01, cmd, 0);
					} else {
						_log("Set Err\n");
						m_pWake->ProcessTx(0x01, CMD_ERR, 0);
					}
					break;

				case CMD_RELAYS_IDR:
				case CMD_RELAYS_ODRR:
					ioe_cmd.cmd = cmd;
					xQueueSend(IOExpanders::Instance().xQueueCommands, &ioe_cmd, 1);
					if (xQueueReceive(IOExpanders::Instance().xQueueResponce, &ioe_responce, 10) == pdTRUE) {
						_log("RI Ok\n");
						m_pWake->TxData[0] = (ioe_responce >> 8);
						m_pWake->TxData[1] = (ioe_responce & 0x00FF);
						m_pWake->ProcessTx(0x01, cmd, 2);
					} else {
						_log("Relays RI Err\n");
						m_pWake->ProcessTx(0x01, CMD_ERR, 0);
					}
					break;

				case CMD_POWERS_IDR:
				case CMD_POWERS_ODRR:
					ioe_cmd.cmd = cmd;
					xQueueSend(IOExpanders::Instance().xQueueCommands, &ioe_cmd, 1);
					if (xQueueReceive(IOExpanders::Instance().xQueueResponce, &ioe_responce, 10) == pdTRUE) {
						_log("Power RI Ok\n");
						m_pWake->TxData[0] = ioe_responce >> 8;
						m_pWake->TxData[1] = ioe_responce & 0x00FF;
						m_pWake->ProcessTx(0x01, cmd, 2);
					} else {
						_log("Power RI Err\n");
						m_pWake->ProcessTx(0x01, CMD_ERR, 0);
					}
					break;

				case CMD_POWERS_ODRW:
				case CMD_POWERS_RESET:
				case CMD_POWERS_SET:
					ioe_cmd.cmd = cmd;
					ioe_cmd.data[0] = m_pWake->RxData[0];
					ioe_cmd.data[1] = m_pWake->RxData[1];
					xQueueSend(IOExpanders::Instance().xQueueCommands, &ioe_cmd, 1);
					if (xQueueReceive(IOExpanders::Instance().xQueueResponce, &ioe_responce, 10) == pdTRUE) {
						_log("Power set ok\n");
						m_pWake->ProcessTx(0x01, cmd, 0);
					} else {
						_log("Power set err\n");
						m_pWake->ProcessTx(0x01, CMD_ERR, 0);
					}
					break;

				case CMD_CLIMATE_GET:
				case CMD_CLIMATE_SET:
					ProcessClimate(m_pWake->GetCommand());
					break;

				case CMD_READ_ALL:

					break;

				default:
					_log("Default: 0x%02X\n", m_pWake->GetCommand());
					m_pWake->ProcessTx(0x01, CMD_ERR, 0);
					break;
				}
			}

		} else if (event == xQueueWiegand) {
			xQueueReceive(event, &wiegand, 0);
			if (wiegand.Channel == Wiegand::WiegandChannel::Channel_1) {
				memcpy(&sWiegand1, &wiegand, sizeof(WiegandStruct));
			}
			if (wiegand.Channel == Wiegand::WiegandChannel::Channel_2) {
				memcpy(&sWiegand2, &wiegand, sizeof(WiegandStruct));
			}
			// ProcessWiegand(CMD_WIEGAND, wiegand);

		} else {
			_log("!!!Pizda!!!\n");
		}
	}
}


#define DEVICE_ID_BASE_ADDR		0x1FFFF7E8
#define DEVICE_FLASH_SIZE_REG	0x1FFFF7E0

/**
 * @brief Подготавливает ответ на команду INFO
 *
 * Функция отправляет на вычислитель следующую информацию:
 * Первые 2 байта - размер флеша в кБ (0х80 0х00)
 * 12 байт - уникальный ID
 * Последующие байты - человеко читаемое название
 */
void MainTask::ProcessCmdInfo() {
uint8_t len = 0;

#if 1
	memcpy(&m_pWake->TxData[len++], (uint16_t *)(DEVICE_FLASH_SIZE_REG), 2);
	len += 1;

	memcpy(&m_pWake->TxData[len++], (uint16_t *)(DEVICE_ID_BASE_ADDR), 2);
	len += 1;

	memcpy(&m_pWake->TxData[len++], (uint16_t *)(DEVICE_ID_BASE_ADDR + 0x02), 2);
	len += 1;

	memcpy(&m_pWake->TxData[len++], (uint32_t *)(DEVICE_ID_BASE_ADDR + 0x04), 4);
	len += 3;

	memcpy(&m_pWake->TxData[len++], (uint32_t *)(DEVICE_ID_BASE_ADDR + 0x08), 4);
	len += 3;

	_log("DeviceVersion Address: 0x%08X\n", (uint32_t)&device_version);
	memcpy(&m_pWake->TxData[len++], (void *)&device_version, 4);
	len += 3;

	strncpy((char *)&m_pWake->TxData[len], DEVICE_NAME, FRAME_SIZE - len);
	len += sizeof(DEVICE_NAME) - 1;

	len = (len > FRAME_SIZE) ? FRAME_SIZE : len;
	m_pWake->ProcessTx(0x01, CMD_INFO, len);
#else

	m_pWake->TxData[0] = 0x49;
	m_pWake->TxData[1] = 0xC0;
	m_pWake->TxData[2] = 0x64;
	m_pWake->TxData[3] = 0xDB;
	m_pWake->TxData[4] = 0xDB;
	m_pWake->TxData[5] = 0xFF;
	m_pWake->TxData[6] = 0xC0;
	m_pWake->TxData[7] = 0xDB;

	m_pWake->ProcessTx(0x01, CMD_INFO, 8);
#endif
}


/**
 * @brief Запуск процесса загрузчика
 * Функция осуществляет защищенную загрущку кода с хоста.
 *
 */
void MainTask::ProcessBoot() {
	_log("Process Boot\n");

#define KEY_SIZE	12
uint8_t uid[KEY_SIZE];
uint8_t cipher[KEY_SIZE];

	memcpy(&uid[0], (uint16_t *)(DEVICE_ID_BASE_ADDR       ), 2);
	memcpy(&uid[2], (uint16_t *)(DEVICE_ID_BASE_ADDR + 0x02), 2);
	memcpy(&uid[4], (uint32_t *)(DEVICE_ID_BASE_ADDR + 0x04), 4);
	memcpy(&uid[8], (uint32_t *)(DEVICE_ID_BASE_ADDR + 0x08), 4);

	memcpy(cipher, &m_pWake->RxData[0], KEY_SIZE);

	for (int i = 0; i < KEY_SIZE; i++) {
		cipher[i] = cipher[i] ^ key[i];
	}

	bool Valid = true;
	for (int i = 0; i < KEY_SIZE; i++) {
		Valid &= (cipher[i] == uid[i]);
	}
	memset(uid, 0, KEY_SIZE);
	memset(cipher, 0, KEY_SIZE);

	if (Valid) {
		_log("Change to Boot\n");
		m_pWake->ProcessTx(0x01, CMD_BOOT, 0);
		BKP_WriteBackupRegister(BKP_DR1, 0x1234);
		BKP_WriteBackupRegister(BKP_DR5, 0xCE94);

		vTaskDelay(100);
		NVIC_SystemReset();
	} else {
		m_pWake->ProcessTx(0x01, CMD_ERR, 0);
	}
}


void MainTask::ProcessClimate(Command cmd) {
ClimateStruct climate;
uint8_t len = 0;

	if (cmd == CMD_CLIMATE_GET) {
		xQueuePeek(Climate::Instance().xQueueData, &climate, 0);
		Climate::Instance().PrintClimateData(climate);

		memcpy(&m_pWake->TxData[len], (uint32_t *)&climate.TemperatureLocal, 4);
		len += 4;

		memcpy(&m_pWake->TxData[len], (uint32_t *)&climate.TemperatureExternal, 4);
		len += 4;

		memcpy(&m_pWake->TxData[len], (uint32_t *)&climate.TemperatureLocalAlt, 4);
		len += 4;

		memcpy(&m_pWake->TxData[len], (uint32_t *)&climate.Humidity, 4);
		len += 4;

		m_pWake->TxData[len] = 	(climate.Heater ? 0x01 : 0x00) |
								(climate.Cooler ? 0x02 : 0x00) |
								(climate.AutomaticMode ? 0x04 : 0x00);
		len += 1;

		assert_param(len < FRAME_SIZE);
		m_pWake->ProcessTx(0x01, CMD_CLIMATE_GET, len);
		return;

	} else if (cmd == CMD_CLIMATE_SET) {
		bool heater = m_pWake->RxData[0] & 0x01;
		bool cooler = m_pWake->RxData[0] & 0x02;
		bool automatic = m_pWake->RxData[0] & 0x04;
		_log("Automatic set to %s\n", automatic ? "True" : "False");
		_log("Heater set to %s\n", heater ? "True" : "False");
		_log("Cooler set to %s\n", cooler ? "True" : "False");

		climate.AutomaticMode = automatic;
		climate.Cooler = cooler;
		climate.Heater = heater;
		xQueueOverwrite(Climate::Instance().xQueueData, &climate);

		m_pWake->ProcessTx(0x01, CMD_CLIMATE_SET, 0);
		return;
	}

	m_pWake->ProcessTx(0x01, CMD_ERR, 0);
}

void MainTask::ProcessReadAll(Command cmd) {
uint8_t porta_idr = PORTA_IDR();
uint8_t porta_odr = PORTA_ODR();
uint8_t portb_idr = PORTB_IDR();
uint8_t portb_odr = PORTB_ODR();

ResponceAll responce;
	memset(&responce, 0, sizeof(responce));

uint8_t subcommand = m_pWake->RxData[0];

	if (subcommand & SCMD_PORTS_IDR) {
		responce.has_PORTA_IDR = responce.has_PORTB_IDR = true;
		responce.PORTA_IDR = porta_idr;
		responce.PORTB_IDR = portb_idr;
	}

	if (subcommand & SCMD_PORTS_ODR) {
		responce.has_PORTA_ODR = responce.has_PORTB_ODR = true;
		responce.PORTA_ODR = porta_odr;
		responce.PORTB_ODR = portb_odr;
	}

	uint16_t ioe_responce;
	IOECommand ioe_cmd;
	const IOExpanders &expanders = IOExpanders::Instance();

	if (subcommand & SCMD_RELAYS_IDR) {
		ioe_cmd.cmd = CMD_RELAYS_IDR;
		if (xQueueSend(expanders.xQueueCommands, &ioe_cmd, 5) == pdTRUE) {
			if (xQueueReceive(expanders.xQueueResponce, &ioe_responce, 10) == pdTRUE) {
				responce.has_RELAYS_IDR = true;
				responce.RELAYS_IDR = ioe_responce;
			}
		}
	}

	if (subcommand & SCMD_RELAYS_ODR) {
		ioe_cmd.cmd = CMD_RELAYS_ODRR;
		if (xQueueSend(expanders.xQueueCommands, &ioe_cmd, 5) == pdTRUE) {
			if (xQueueSend(expanders.xQueueResponce, &ioe_responce, 10) == pdTRUE) {
				responce.has_RELAYS_ODR = true;
				responce.RELAYS_ODR = ioe_responce;
			}
		}
	}

	TickType_t currentTick = xTaskGetTickCount();
	if (subcommand & SCMD_WIEGAND_1) {
		if ((sWiegand1.ValidTime > 0) && (sWiegand1.ValidTime < currentTick)) {
			responce.WiegandCh1.size = sWiegand1.WiegandLen;
			uint8_t max_bytes = GetWiegandSizeInBytes(sWiegand1);
			responce.WiegandCh1.data.size = max_bytes;
			memcpy(responce.WiegandCh1.data.bytes, sWiegand1.Data, max_bytes);
			responce.has_WiegandCh1 = true;
			sWiegand1.ValidTime = 0;
		}
	}

	if (subcommand & SCMD_WIEGAND_2) {
		if ((sWiegand2.ValidTime > 0) && (sWiegand2.ValidTime < currentTick)) {
			responce.WiegandCh2.size = sWiegand2.WiegandLen;
			uint8_t max_bytes = GetWiegandSizeInBytes(sWiegand2);
			responce.WiegandCh2.data.size = max_bytes;
			memcpy(responce.WiegandCh2.data.bytes, sWiegand2.Data, max_bytes);
			responce.has_WiegandCh2 = true;
			sWiegand2.ValidTime = 0;
		}
	}

	pb_ostream_t nanopb_ostream;
	nanopb_ostream = pb_ostream_from_buffer(m_pWake->TxData, FRAME_SIZE);
	if (pb_encode(&nanopb_ostream, ResponceAll_fields, &responce) == true) {
		m_pWake->ProcessTx(0x01, cmd, nanopb_ostream.bytes_written);
	} else {
		m_pWake->ProcessTx(0x01, CMD_ERR, 0);
	}
}


void MainTask::ProcessWiegand(Command cmd, const WiegandStruct &wig) {

uint8_t max_bytes = wig.WiegandLen / BITS_IN_BYTE + ((wig.WiegandLen % BITS_IN_BYTE) ? 1 : 0);
	m_pWake->TxData[0] = wig.Channel;
	m_pWake->TxData[1] = wig.WiegandLen;
	memcpy(&m_pWake->TxData[2], wig.Data, max_bytes);
	m_pWake->ProcessTx(0x01, cmd, max_bytes + 2);
}


void MainTask::ProcessPORTs(Command cmd) {
uint8_t porta_idr = PORTA_IDR();
uint8_t portb_idr = PORTB_IDR();

uint8_t porta_odr = PORTA_ODR();
uint8_t portb_odr = PORTB_ODR();

	_log("porta_idr: %02X\n", porta_idr);
	_log("portb_idr: %02X\n", portb_idr);

	_log("porta_odr: %02X\n", porta_odr);
	_log("portb_odr: %02X\n", portb_odr);

	if (cmd == CMD_PORTS_IDR) {
		m_pWake->TxData[0] = porta_idr;
		m_pWake->TxData[1] = portb_idr;
		m_pWake->ProcessTx(0x01, CMD_PORTS_IDR, 2);
		return;

	} else if (cmd == CMD_PORTS_ODRR) {
		m_pWake->TxData[0] = porta_odr;
		m_pWake->TxData[1] = portb_odr;
		m_pWake->ProcessTx(0x01, CMD_PORTS_ODRR, 2);
		return;

	} else if (cmd == CMD_PORTS_ODRW) {
		uint8_t odr = m_pWake->RxData[0];
		PORTA_DATA4_OUT = (odr & 0x01) ? 0 : 1;
		PORTA_DATA5_OUT = (odr & 0x02) ? 0 : 1;
		PORTA_DATA6_OUT = (odr & 0x04) ? 0 : 1;
		PORTA_DATA7_OUT = (odr & 0x08) ? 0 : 1;

		odr = m_pWake->RxData[1];
		PORTB_DATA4_OUT = (odr & 0x01) ? 0 : 1;
		PORTB_DATA5_OUT = (odr & 0x02) ? 0 : 1;
		PORTB_DATA6_OUT = (odr & 0x04) ? 0 : 1;
		PORTB_DATA7_OUT = (odr & 0x08) ? 0 : 1;
		m_pWake->ProcessTx(0x01, CMD_PORTS_ODRW, 0);
		return;

	} else if (cmd == CMD_PORTS_SET) {
		uint8_t odr = (m_pWake->RxData[0] & 0x0F) << 4;
		GPIOD->BRR = odr;
		odr = (m_pWake->RxData[1] & 0x0F) << 4;
		GPIOE->BRR = odr;
		m_pWake->ProcessTx(0x01, CMD_PORTS_SET, 0);
		return;

	} else if (cmd == CMD_PORTS_RESET) {
		uint8_t odr = (m_pWake->RxData[0] & 0x0F) << 4;
		GPIOD->BSRR = odr;
		odr = (m_pWake->RxData[1] & 0x0F) << 4;
		GPIOE->BSRR = odr;
		m_pWake->ProcessTx(0x01, CMD_PORTS_RESET, 0);
		return;
	}

	_log("Unknown command: 0x%02X\n", cmd);
	m_pWake->ProcessTx(0x01, CMD_ERR, 0);
}



void MainTask::Init() {
	portENTER_CRITICAL();
		InitHardware();

		m_pWake = new Wake(0x02, m_pUSART);
		assert_param(m_pWake);

		xTimerSemaphore = xSemaphoreCreateBinary();
		assert_param(xTimerSemaphore);
#if (configUSE_TRACE_FACILITY == 1)
		vTraceSetSemaphoreName(xTimerSemaphore, "sTimer");
#endif
		xSemaphoreTake(xTimerSemaphore, 0);

		xTimer = xTimerCreate("Main", 1000, pdTRUE, xTimerSemaphore, TimerCallback);
		assert_param(xTimer);

		xQueueUsartRx = xQueueCreate(20, sizeof(uint8_t));
		assert_param(xQueueUsartRx);
#if (configUSE_TRACE_FACILITY == 1)
		vTraceSetQueueName(xQueueUsartRx, "qUsartRx");
#endif

		xQueueWiegand = xQueueCreate(2, sizeof(WiegandStruct));
		assert_param(xQueueWiegand);
#if (configUSE_TRACE_FACILITY == 1)
		vTraceSetQueueName(xQueueWiegand, "qWiegData");
#endif

		xQueueSet = xQueueCreateSet(1 + 20 + 0);
		assert_param(xQueueSet);

		xQueueAddToSet(xTimerSemaphore, xQueueSet);
		xQueueAddToSet(xQueueUsartRx, xQueueSet);
		xQueueAddToSet(xQueueWiegand, xQueueSet);

		m_pWiegandCh1 = new Wiegand(Wiegand::Channel_1, xQueueWiegand);
		assert_param(m_pWiegandCh1);

		m_pWiegandCh2 = new Wiegand(Wiegand::Channel_2, xQueueWiegand);
		assert_param(m_pWiegandCh2);

		IOExpanders &ioe = IOExpanders::Instance();
		UNUSED(ioe);

		Climate &climate = Climate::Instance();
		UNUSED(climate);
	portEXIT_CRITICAL();
}


void MainTask::InitHardware() {
GPIO_InitTypeDef GPIO_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = SERIAL_USB_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SERIAL_USB_RX_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SERIAL_USB_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SERIAL_USB_TX_PORT, &GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	USART_InitStructure.USART_BaudRate = 256000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_2;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(m_pUSART, &USART_InitStructure);

	USART_ITConfig(m_pUSART, USART_IT_RXNE, ENABLE);
	USART_Cmd(m_pUSART, ENABLE);
}


void USART1_IRQHandler() {
BaseType_t xHigherPriorityTaskWoken = pdFALSE;
MainTask &mt = MainTask::Instance();

	if (USART_GetITStatus(mt.m_pUSART, USART_IT_RXNE) != RESET) {
		uint8_t rx_data = mt.m_pUSART->DR;
		xQueueSendFromISR(mt.xQueueUsartRx, &rx_data, &xHigherPriorityTaskWoken);
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
