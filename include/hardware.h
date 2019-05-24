/**
  ******************************************************************************
  * @file    include/hardware.h
  * @author  Владимир Мешков <vld.meshkov@gmail.com>
  * @version V2.0.0
  * @date    16 июня 2017
  * @brief   Описание аппаратной части проекта
  ******************************************************************************
  * @attention
  * Всем очко!!
  *
  * <h2><center>&copy; 2018 ООО "РПС"</center></h2>
  ******************************************************************************
  */

#ifndef HARDWARE_H_
#define HARDWARE_H_
#include <bitbanding.h>
#include "stm32f10x.h"

/**
 * \addtogroup Hardware
 * @{
 */

/**
 * \defgroup Serial  Serial
 * \brief Постедовательные интерфейсы.
 *
 * \addtogroup Serial
 * @{
 */


/**
 * \name Serial interface 1 (USB)
 * \brief Последовательный интерфейс через преобразователь USB-Serial
 *
 * Периферия - USART1, шина - APB2, 72 MHz.
 *
 * Подключение выводов:
 * 		- SERIAL_USB_TX - USART1_TX - PA9 - Альтернативная функция
 * 		- SERIAL_USB_RX - USART1_RX - PA10 - Альтернативная функция
 */
///@{
#define SERIAL_USB_USART						USART1

#define SERIAL_USB_TX_PORT						GPIOA
#define SERIAL_USB_TX_PIN						GPIO_Pin_9

#define SERIAL_USB_RX_PORT						GPIOA
#define SERIAL_USB_RX_PIN						GPIO_Pin_10
///@}


/**
 * \name Serial interface 2 (RS-232)
 * \brief Классический последовательный интерфейс RS-232
 *
 * Периферия - USART2, шина - APB1, 36 MHz.
 *
 * Подключение выводов:
 * 		- SERIAL_RS232_TX - USART2_TX - PA2 - Альтернативная функция
 * 		- SERIAL_RS232_RX - USART2_RX - PA3 - Альтернативная функция
 */
///@{
#define SERIAL_RS232_USART						USART2

#define SERIAL_RS232_TX_PORT					GPIOA
#define SERIAL_RS232_TX_PIN						GPIO_Pin_2

#define SERIAL_RS232_RX_PORT					GPIOA
#define SERIAL_RS232_RX_PIN						GPIO_Pin_3
///@}

/**
 * \name Serial interface 3 (RS-485)
 * \brief Интерфейс RS-485
 *
 * Периферия - USART3, шина - APB1, 36 MHz.
 *
 * Подключение выводов:
 * 		- SERIAL_RS485_TX - USART3_TX - PC10 - Альтернативная функция
 * 		- SERIAL_RS485_RX - USART3_RX - PC11 - Альтернативная функция
 * 		- SERIAL_RS485_RE - PC4 - Выход общего назначения, включение приемника
 * 		- SERIAL_RS485_DE - PC5 - Выход общего назначения, включение передатчика
 */
///@{
#define SERIAL_RS485_USART						USART3

#define SERIAL_RS485_TX_PORT					GPIOC
#define SERIAL_RS485_TX_PIN						GPIO_Pin_10

#define SERIAL_RS485_RX_PORT					GPIOC
#define SERIAL_RS485_RX_PIN						GPIO_Pin_11

#define SERIAL_RS485_RE_PORT					GPIOC
#define SERIAL_RS485_RE_PIN						GPIO_Pin_4
/// Управление приемником RS-485 (0 - включен, 1 - отключен)
#define SERIAL_RS485_RE_OUT						TO_BIT_BAND_PER(SERIAL_RS485_RE_PORT->ODR, SERIAL_RS485_RE_PIN)

#define SERIAL_RS485_DE_PORT					GPIOC
#define SERIAL_RS485_DE_PIN						GPIO_Pin_5
/// Управление передатчиком RS-485 (0 - отключен, 1 - включен)
#define SERIAL_RS485_DE_OUT						TO_BIT_BAND_PER(SERIAL_RS485_DE_PORT->ODR, SERIAL_RS485_DE_PIN)
///@}

/**
 * Group Serial
 * @}
 */


/**
 * \defgroup I2C I2C
 * \brief Интерфес I2C.
 *
 * \addtogroup I2C
 * @{
 */

/**
 * \name I2C1 периферия
 * \brief Описание периферии I2C1.
 *
 *	К интерфейсу I2C1 подключены микросхемы расширителей портов MCP23017T.
 *
 *	I2C1 расположен на шине APB1 с тактовой частотой 36 МГц.
 *
 *	Соотношение выводов:
 *		- I2C1_SCL - PB6 - Альтернативная функция
 *		- I2C1_SDA - PB7 - Альтернативная функция
 */
///@{
#define I2C1_CLK									RCC_APB1Periph_I2C1
#define I2C1_PORT_CLK								RCC_AHB1Periph_GPIOB
#define I2C1_PORT									GPIOB

#define I2C1_SCL_PIN								GPIO_Pin_6
#define I2C1_SCL_PIN_SOURCE							GPIO_PinSource6
#define I2C1_SCL_PIN_AF								GPIO_AF_I2C1

#define I2C1_SDA_PIN								GPIO_Pin_7
#define I2C1_SDA_PIN_SOURCE							GPIO_PinSource7
#define I2C1_SDA_PIN_AF								GPIO_AF_I2C1

#define I2C1_DUTYCYCLE								I2C_DutyCycle_2
#define I2C1_SPEED									1000000

// TODO Проверить правильность настройки DMA для I2C1
#define I2C_DMA_CHANNEL								DMA_Channel_1
#define I2C_DMA_STREAM_RX							DMA1_Stream5
#define I2C_DMA_STREAM_TX							DMA1_Stream6

#define I2C_DMA_RX_FLAGS							(DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_DMEIF5 | DMA_FLAG_FEIF5)
#define I2C_DMA_TX_FLAGS							(DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6 | DMA_FLAG_TEIF6 | DMA_FLAG_DMEIF6 | DMA_FLAG_FEIF6)

#define I2C_DMA_RX_IRQ								DMA1_Stream5_IRQn
#define I2C_DMA_TX_IRQ								DMA1_Stream6_IRQn
///@}



/**
 * \name I2C2 периферия
 * \brief Описание периферии I2C2.
 *
 *	К интерфейсу I2C2 подключены микросхема термометра SA56004 и микросхема гигрометра SHT30-DIS.
 *
 *	I2C2 расположен на шине APB1 с тактовой частотой 36 МГц.
 *
 *	Соотношение выводов:
 *		- I2C2_SCL - PB10 - Альтернативная функция
 *		- I2C2_SDA - PB11 - Альтернативная функция
 */
///@{
#define I2C2_CLK									RCC_APB1Periph_I2C2
#define I2C2_PORT_CLK								RCC_AHB1Periph_GPIOB
#define I2C2_PORT									GPIOB

#define I2C2_SCL_PIN								GPIO_Pin_10
#define I2C2_SCL_PIN_SOURCE							GPIO_PinSource10
#define I2C2_SCL_PIN_AF								GPIO_AF_I2C2

#define I2C2_SDA_PIN								GPIO_Pin_11
#define I2C2_SDA_PIN_SOURCE							GPIO_PinSource11
#define I2C2_SDA_PIN_AF								GPIO_AF_I2C2
///@}

/**
 * Group I2C
 * @}
 */


/**
 * \defgroup Wiegand Wiegand
 * \brief Интерфейс Wiegand
 *
 * \addtogroup Wiegand
 * @{
 */

/**
 * \name Wiegand
 * \brief Описание периферии для работы с Wiegand (2 канала)
 *
 * Для работы с Wiegand нужно 2 вывода: прерывание и вход данных
 *
 * Соотношение выводов Wiegand_1:
 * 		- WIEGAND1_DATA - PA12 - Цифровой вход данных (единицы или нули)
 * 		- WIEGAND1_IRQ - PB0 - Цифровой вход, прерывание по фронту
 *
 * Соотношение выводов Wiegand_2:
 * 		- WIEGAND2_DATA - PA11 - Цифровой вход данных (единицы или нули)
 * 		- WIEGAND2_IRQ - PB1 - Цифровой вход, прерывание по фронту
 */
///@{
#define WIEGAND1_DATA_PORT							GPIOA
#define WIEGAND1_DATA_PIN							GPIO_Pin_12
#define WIEGAND1_DATA_IN							TO_BIT_BAND_PER(WIEGAND1_DATA_PORT->IDR, WIEGAND1_DATA_PIN)

#define WIEGAND1_IRQ_PORT							GPIOB
#define WIEGAND1_IRQ_PIN							GPIO_Pin_0
/*************************************************************************************************************************/

#define WIEGAND2_DATA_PORT							GPIOA
#define WIEGAND2_DATA_PIN							GPIO_Pin_11
#define WIEGAND2_DATA_IN							TO_BIT_BAND_PER(WIEGAND2_DATA_PORT->IDR, WIEGAND2_DATA_PIN)

#define WIEGAND2_IRQ_PORT							GPIOB
#define WIEGAND2_IRQ_PIN							GPIO_Pin_1
///@}

/**
 * Group Wiegand
 * @}
 */

/**
 * \defgroup DiscretIO DiscretIO
 * \brief Дискретные входы - выходы
 *
 * Дискретные входы и выходы, аналогичные выводам на старом Slave_TA.
 * Подключение к МК напрямую, минуя расширитель портов.
 *
 * \addtogroup DiscretIO
 * @{
 */

/**
 * \name PORTA
 * \brief Эмуляция дискретного канала PORTA
 *
 * Эмуляция дискретного канала PORTA из старого Slave_TA: 4 входа через оптопары и 4 выхода через твердотельные реле.
 *
 * Соотношение выводов PORTA:
 * 		- PORTA_IN[0:3] - PD0:PD3 - Цифровой вход
 * 		- PORTA_OUT[0:3] - PD4:PD7 - Цифровой выход, открытый коллектор
 */
///@{
#define PORTA_DATA_PORT								GPIOD
#define PORTA_DATA_OUT_PINS							(GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7)
#define PORTA_DATA_IN_PINS							(GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3)
#define PORTA_DATA4_OUT								TO_BIT_BAND_PER(PORTA_DATA_PORT->ODR, GPIO_Pin_4)
#define PORTA_DATA5_OUT								TO_BIT_BAND_PER(PORTA_DATA_PORT->ODR, GPIO_Pin_5)
#define PORTA_DATA6_OUT								TO_BIT_BAND_PER(PORTA_DATA_PORT->ODR, GPIO_Pin_6)
#define PORTA_DATA7_OUT								TO_BIT_BAND_PER(PORTA_DATA_PORT->ODR, GPIO_Pin_7)
#define PORTA_DATA4_IN
///@}


/**
 * \name PORTB
 * \brief Эмуляция дискретного канала PORTB
 *
 * Эмуляция дискретного канала PORTB из старого Slave_TA: 4 входа через оптопары и 4 выхода через твердотельные реле.
 *
 * Соотношение выводов PORTB:
 * 		- PORTB_IN[0:3] - PE0:PE3 - Цифровой вход
 * 		- PORTB_OUT[0:3] - PE4:PE7 - Цифровой выход, открытый коллектор
 */
///@{
#define PORTB_DATA_PORT								GPIOE
#define PORTB_DATA_OUT_PINS							(GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7)
#define PORTB_DATA_IN_PINS							(GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3)
#define PORTB_DATA4_OUT								TO_BIT_BAND_PER(PORTB_DATA_PORT->ODR, GPIO_Pin_4)
#define PORTB_DATA5_OUT								TO_BIT_BAND_PER(PORTB_DATA_PORT->ODR, GPIO_Pin_5)
#define PORTB_DATA6_OUT								TO_BIT_BAND_PER(PORTB_DATA_PORT->ODR, GPIO_Pin_6)
#define PORTB_DATA7_OUT								TO_BIT_BAND_PER(PORTB_DATA_PORT->ODR, GPIO_Pin_7)
///@}

/**
 * Group DiscretIO
 * @}
 */

/**
 * \defgroup Climate Climate
 * \brief Выводы, относящиеся к климат-контролю
 *
 * \addtogroup Climate
 * @{
 */

/**
 * \name Climate
 * \brief Управление климатом
 *
 * Входы температурных аварий и выходы управления вентиляторм и пречкой
 *
 * Соотношение выводов:
 * 		- CLIMATE_TCRIT - PA6 - Цифровой вход
 * 		- CLIMATE_ALERT - PA7 - Цифровой вход
 * 		- CLIMATE_HEATER - PB2 - Цифровой выход
 * 		- CLIMATE_COOLER - PE8 - Цифровой выход
 *
 */
///@{
#define CLIMATE_TCRIT_PORT							GPIOA
#define CLIMATE_TCRIT_PIN							GPIO_Pin_6

#define CLIMATE_ALERT_PORT							GPIOA
#define CLIMATE_ALERT_PIN							GPIO_Pin_7

#define CLIMATE_HEATER_PORT							GPIOB
#define CLIMATE_HEATER_PIN							GPIO_Pin_2
/// Включение печки
#define CLIMATE_HEATER_EN							TO_BIT_BAND_PER(CLIMATE_HEATER_PORT->ODR, CLIMATE_HEATER_PIN)
/*************************************************************************************************************************/

#define CLIMATE_COOLER_PORT							GPIOE
#define CLIMATE_COOLER_PIN							GPIO_Pin_8
/// Включение вентилятора
#define CLIMATE_COOLER_EN							TO_BIT_BAND_PER(CLIMATE_COOLER_PORT->ODR, CLIMATE_COOLER_PIN)
///@}

/**
 * Group Climate
 * @}
 */

/**
 * \defgroup Miscellaneous Misc
 * \brief Не вошедшие никуда
 *
 * \addtogroup Miscellaneous
 * @{
 */

/**
 * \name Misc
 * \brief Прочие выводы
 *
 * Соотношение выводов:
 * 		- MISC_STAT_IRQ - PD10 - Цифровой вход, генерация прерывания
 * 		- MISC_DISCR_IRQ - PD9 - Цифровой вход, генерация прерывания
 */
///@{

#define MISC_STAT_IRQ_PORT							GPIOD
/// Прерывание от системы управления питанием
#define MISC_STAT_IRQ_PIN							GPIO_Pin_10

#define MISC_DISCR_IRQ_PORT							GPIOD
/// Прерывание от дискретных входов на расширителе портов
#define MISC_DISCR_IRQ_PIN							GPIO_Pin_9
///@}

/**
 * Group Additional
 * @}
 */


/**
 * Group Hardware
 * @}
 */

#endif /* HARDWARE_H_ */
