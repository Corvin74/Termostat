/*
 * termostat.h
 *
 *  Created on: 3 окт. 2018 г.
 *      Author: corvin
 */

#ifndef TERMOSTAT_H_
#define TERMOSTAT_H_

// Для DS18B20
#define PIN_DS18B20 8				// Номер пина Arduino с подключенным датчиком
#define TEMPERATURE_PRECISION 12	//
#define TEMP_UPDATE_TIME 10000		// Определяем периодичность проверок
#define SEND_UPDATE_TIME 20000		// Определяем периодичность отправки через RS485

// Для RS485
#define SerialTxControl 9			// RS485 указываем номер вывода arduino, к которому подключены выводы RE и DE конвертирующего модуля
#define RS485Transmit    HIGH
#define RS485Receive     LOW
char buffer[100];					// Буфер приема информации через RS485

// Для LCD_I2C
#define LCD_LIGHT 7

#include <OneWire.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>
#include <EEPROM.h>

void printAddress(DeviceAddress deviceAddress);
void printTemperature(DeviceAddress deviceAddress);
void printResolution(DeviceAddress deviceAddress);
void printData(DeviceAddress deviceAddress);
float getTemperature(DeviceAddress deviceAddress);

typedef struct Message {
	byte DeviceID;		// ID устройства, может принимать значения 1-254
	byte DestinationID;	// Номер устройства-получателя, может принимать значения 0-254
						// (0 - броадкаст)
	uint8_t PacketID;	// Идентификатор пакета.
	byte ActuatorID;	// ID исполнительного устройства, может принимать значения 1-254
	byte CommandID;		// Название параметра (feedTemperature, returnTemperature, relayState...)
	int DataValue;		// Значение параметра типа int
};

uint8_t stateWork = 0b00000000; // Битовое поле статусов задач
/*
 *	0 - отправлен запрос на измерение температуры (0-нет, 1-да)
 *	1 - считана температура с датчика после последнего запроса (0-нет, 1-да)
 *	2 - статус первого тена (0-выключен, 1-включен)
 *	3 - статус второго тена (0-выключен, 1-включен)
 *	4 - статус третьего тена (0-выключен, 1-включен)
 *	5 - режим работы
 *	6 - режим работы
 *	7 - изменение температуры более чем на 1 градус
 */

#endif /* TERMOSTAT_H_ */
