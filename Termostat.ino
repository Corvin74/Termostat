#include "termostat.h"

// Для LCD_I2C
LiquidCrystal_I2C lcd(0x3F, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display

// Создаем объект OneWire
OneWire oneWire(PIN_DS18B20);

// Создаем объект DallasTemperature для работы с сенсорами,
// передавая ему ссылку на объект для работы с 1-Wire.
DallasTemperature sensors(&oneWire);

// Адреса устройств на шине OneWire
DeviceAddress tempFeedAddr = { 0x28, 0xFF, 0xBC, 0xB4, 0xB2, 0x16, 0x05, 0x33 };  // 28 FF BC B4 B2 16 5 33
DeviceAddress tempReturnAddr = { 0x28, 0xFF, 0x89, 0x57, 0x6D, 0x14, 0x04, 0x2D };  // 28 FF 89 57 6D 14 4 2D

// Переменные для хранения текущих значений температуры с датчиков DS18B20
float tempFeed = 0.0;		// Подача с котла
float tempFeedOld = 0.0;	// Предидущее значение подачи с котла
float tempReturn = 0.0;		// Обратка в котел
float tempReturnOld = 0.0;	// Предидущее значение обратки в котел

long lastUpdateTime = 0;	// Переменная для хранения времени последнего считывания с датчика
long lastSendTime = 0;		// Переменная для хранения времени последнего считывания с датчика

Message dataMessage[4];		// Массив структур для хранения данных с датчиков и исполнительных
							// устройств

void setup() {
	// Датчик подачи
	dataMessage[0].DeviceID = 1;
	dataMessage[0].DestinationID = 254;
	dataMessage[0].PacketID = 1;
	dataMessage[0].ActuatorID = 1;
	dataMessage[0].CommandID = 0;
	dataMessage[0].DataValue = 0;
	// Датчик обратки
	dataMessage[1].DeviceID = 1;
	dataMessage[1].DestinationID = 254;
	dataMessage[1].PacketID = 1;
	dataMessage[1].ActuatorID = 2;
	dataMessage[1].CommandID = 0;
	dataMessage[1].DataValue = 0;
	// Датчик на улице
	dataMessage[2].DeviceID = 1;
	dataMessage[2].DestinationID = 254;
	dataMessage[2].PacketID = 1;
	dataMessage[2].ActuatorID = 3;
	dataMessage[2].CommandID = 0;
	dataMessage[2].DataValue = 0;
	// Датчик в погребе
	dataMessage[3].DeviceID = 1;
	dataMessage[3].DestinationID = 254;
	dataMessage[3].PacketID = 1;
	dataMessage[3].ActuatorID = 4;
	dataMessage[3].CommandID = 0;
	dataMessage[3].DataValue = 0;
	// Реле бойлера
	dataMessage[4].DeviceID = 1;
	dataMessage[4].DestinationID = 254;
	dataMessage[4].PacketID = 1;
	dataMessage[4].ActuatorID = 5;
	dataMessage[4].CommandID = 0;
	dataMessage[4].DataValue = 0;
	// Реле насоса в погребе
/*	dataMessage[5].DeviceID = 1;
	dataMessage[5].DestinationID = 254;
	dataMessage[5].PacketID = 1;
	dataMessage[5].ActuatorID = 6;
	dataMessage[5].CommandID = 0;
	dataMessage[5].DataValue = 0;
	// Реле насоса в скважене
	dataMessage[6].DeviceID = 1;
	dataMessage[6].DestinationID = 254;
	dataMessage[6].PacketID = 1;
	dataMessage[6].ActuatorID = 7;
	dataMessage[6].CommandID = 0;
	dataMessage[6].DataValue = 0;
	*/

	Serial.begin(38400);
	// Инициализируем шину RS485
	pinMode(SerialTxControl, OUTPUT);				// устанавливаем режим работы вывода
													// PIN_direction_TX_RX, как "выход"
	digitalWrite(SerialTxControl, RS485Transmit);	// устанавливаем уровень логической «1» на
													// выводе SerialTxControl (переводим модуль
													// в режим передачи данных)
	Serial.println("DeviceID 1 ready");				// отправляем тестовое сообщение
	delay(20);
	digitalWrite(SerialTxControl, RS485Receive);	// устанавливаем уровень логического «0» на
													// выводе SerialTxControl (переводим модуль
													// в режим приема данных)

	// Инициализируем дисплей
	lcd.init();
	lcd.backlight();
	lcd.setCursor(0, 0);
	lcd.print("I2CLCDInit complite!");

	// Инициализируем шину OneWire
	sensors.begin();
	delay(5);

	// Выполняем поиск устройств на шине OneWire
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Locating devices");
	for(uint8_t i = 0 ; i < 3 ; i++){
		delay(1000);
		lcd.print(".");
	}
	lcd.setCursor(0, 1);
	lcd.print("Found ");
	lcd.print(sensors.getDeviceCount());
	lcd.print(" devices.");
	delay(5000);

	// Устанавливаем разрешение датчиков в соответствии с настройкой
	sensors.setResolution(tempFeedAddr, TEMPERATURE_PRECISION);
	sensors.setResolution(tempReturnAddr, TEMPERATURE_PRECISION);

	// Гасим подсветку экрана
	lcd.noBacklight();

	// Выполняем запрос на измерение температур
	sensors.requestTemperatures();
	stateWork |= (1<<0);
	// Обновляем время последнего замера температуры
	lastUpdateTime = millis();
}

void loop() {
/*
	if(stateWork & (1<<0)){
		digitalWrite(SerialTxControl, RS485Transmit);
		Serial.println(stateWork, BIN);
		delay(20);
		digitalWrite(SerialTxControl, RS485Receive);
	}
*/
	// Для включения подсветки экрана нажать и удерживать кнопку
	if (digitalRead(LCD_LIGHT) == LOW) {
		lcd.backlight();
	} else {
		lcd.noBacklight();
	}

	// Выполняем опрос датчиков температуры в соответствии с настройкой TEMP_UPDATE_TIME
	if ((millis() - lastUpdateTime > TEMP_UPDATE_TIME) && (stateWork & (1<<0))) {
		// Обновляем время последнего считывания и замера температуры
		lastUpdateTime = millis();
		// Сохраняем предидущие значения и считываем текущие значения с датчиков по их адресам
		tempFeedOld = tempFeed;
 		tempFeed = getTemperature(tempFeedAddr);
		dataMessage[0].DataValue = tempFeed * 100;
		dataMessage[0].PacketID++;

		tempReturnOld = tempReturn;
		tempReturn = getTemperature(tempReturnAddr);
		dataMessage[1].DataValue = tempReturn * 100;
		dataMessage[1].PacketID++;

		stateWork |= (1<<0);
		// И в случае изменения более чем на 1 градус выводим их на экран и выставляем флаг немедленной отправки на сервер
		if((max(tempFeed, tempFeedOld)-min(tempFeed, tempFeedOld))>1){
			lcd.setCursor(0, 0);
			lcd.print("t1 = ");
			lcd.print(tempFeed);
			stateWork |= (1<<7);
		}
		if((max(tempReturn, tempReturnOld)-min(tempReturn, tempReturnOld))>1){
			lcd.setCursor(0, 1);
			lcd.print("t2 = ");
			lcd.print(tempReturn);
			stateWork |= (1<<7);
		}
		// Выполняем запрос на измерение температур
		sensors.requestTemperatures();
	}

	// Выполняем отправку данных на сервер через интерфейс RS485 в соответствии с настройкой SEND_UPDATE_TIME
	if ((millis() - lastSendTime > SEND_UPDATE_TIME) || (stateWork & (1<<7))) {
		lastSendTime = millis();
		stateWork &= ~(1<<7);		// Сбрасываем сигнал изменения температуры на градус

		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("t1 = ");
		lcd.print(tempFeed);
		lcd.setCursor(0, 1);
		lcd.print("t2 = ");
		lcd.print(tempReturn);
		lcd.setCursor(0, 2);
		lcd.print("Size = ");
		lcd.print(sizeof(dataMessage));
		digitalWrite(SerialTxControl, RS485Transmit);
		delay(50);
		Serial.write((uint8_t*) &dataMessage, sizeof(dataMessage));
		delay(150);
		digitalWrite(SerialTxControl, RS485Receive);
		delay(50);

/*
		if (tempFeed != 1000) {
			lcd.setCursor(0, 0);
			lcd.print("t1 = ");
			lcd.print(tempFeed);
			digitalWrite(SerialTxControl, RS485Transmit);
			delay(50);
			Serial.write((uint8_t*) &dataMessage[0], sizeof(dataMessage[0]));
			delay(150);
			digitalWrite(SerialTxControl, RS485Receive);
		}
		delay(50);
		if (tempReturn != 1000) {
			lcd.setCursor(0, 1);
			lcd.print("t2 = ");
			lcd.print(tempReturn);
			digitalWrite(SerialTxControl, RS485Transmit);
			delay(50);
			Serial.write((uint8_t*) &dataMessage[1], sizeof(dataMessage[1]));
			delay(150);
			digitalWrite(SerialTxControl, RS485Receive);
		}
*/
	}
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress) {
	for (uint8_t i = 0; i < 8; i++) {
// zero pad the address if necessary
		if (deviceAddress[i] < 16)
			Serial.print("0");
		Serial.print(deviceAddress[i], HEX);
	}
}

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress) {
	float tempC = sensors.getTempC(deviceAddress);
	Serial.print("Temp C: ");
	Serial.print(tempC);
}

// function to print the temperature for a device
float getTemperature(DeviceAddress deviceAddress) {
	float tempC = sensors.getTempC(deviceAddress);
	return tempC;
}

// function to print a device's resolution
void printResolution(DeviceAddress deviceAddress) {
	Serial.print("Resolution: ");
	Serial.print(sensors.getResolution(deviceAddress));
	Serial.println();
}

// main function to print information about a device
void printData(DeviceAddress deviceAddress) {
	Serial.print("Device Address: ");
	printAddress(deviceAddress);
	Serial.print(" ");
	printTemperature(deviceAddress);
	Serial.println();
}
