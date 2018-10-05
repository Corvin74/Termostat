#include <OneWire.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>

// Для DS18B20
#define PIN_DS18B20 8				// Номер пина Arduino с подключенным датчиком
#define TEMPERATURE_PRECISION 12	//
#define TEMP_UPDATE_TIME 10000 // Определяем периодичность проверок
#define PRINT_UPDATE_TIME 10000 // Определяем периодичность проверок

// Для RS485
#define SerialTxControl 9			// RS485 указываем номер вывода arduino, к которому подключены выводы RE и DE конвертирующего модуля
#define RS485Transmit    HIGH
#define RS485Receive     LOW
char buffer[100];					// Буфер приема информации через RS485

// Для LCD_I2C
LiquidCrystal_I2C lcd(0x3F,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// Создаем объект OneWire
OneWire oneWire(PIN_DS18B20);
// Создаем объект DallasTemperature для работы с сенсорами, передавая ему ссылку на объект для работы с 1-Wire.
DallasTemperature sensors(&oneWire);
// Адреса устройств на шине OneWire
DeviceAddress insideThermometer = { 0x28, 0xFF, 0xBC, 0xB4, 0xB2, 0x16, 0x05, 0x33 };  // 28 FF BC B4 B2 16 5 33
DeviceAddress outsideThermometer = { 0x28, 0xFF, 0x89, 0x57, 0x6D, 0x14, 0x04, 0x2D };  // 28 FF 89 57 6D 14 4 2D

float insideTemperature = 1000; // Глобальная переменная для хранения значение температуры с датчика DS18B20
float outsideTemperature = 1000; // Глобальная переменная для хранения значение температуры с датчика DS18B20

long lastUpdateTime = 0; // Переменная для хранения времени последнего считывания с датчика
long lastPrintTime = 0; // Переменная для хранения времени последнего считывания с датчика

void printAddress(DeviceAddress deviceAddress);
void printTemperature(DeviceAddress deviceAddress);
void printResolution(DeviceAddress deviceAddress);
void printData(DeviceAddress deviceAddress);
float getTemperature(DeviceAddress deviceAddress);

typedef struct Message
{
	byte DeviceID;		// ID устройства, может принимать значения 1-254
	byte DestinationID;	// Номер устройства-получателя, может принимать значения 0-254 (0 - броадкаст)
	uint8_t PacketID;	// Идентификатор пакета.
	byte ActuatorID;	// ID исполнительного устройства, может принимать значения 1-254
	byte CommandID;		// Название параметра (feedTemperature, returnTemperature, relayState...)
	int DataValue;	// Значение параметра типа int
};

Message dataMessage;

void setup() {
	dataMessage.DeviceID = 1;
	dataMessage.DestinationID = 254;
	dataMessage.PacketID = 1;
	dataMessage.ActuatorID = 1;
	dataMessage.CommandID = 0;
	dataMessage.DataValue = 0;

	Serial.begin(38400);
	pinMode(SerialTxControl, OUTPUT);				// устанавливаем режим работы вывода PIN_direction_TX_RX, как "выход"
	digitalWrite(SerialTxControl, RS485Transmit);	// устанавливаем уровень логической «1» на выводе SerialTxControl (переводим модуль в режим передачи данных)
	Serial.println("Termostat 1 ready");			// отправляем тестовое сообщение
	delay(20);
	digitalWrite(SerialTxControl, RS485Receive);	// устанавливаем уровень логического «0» на выводе SerialTxControl (переводим модуль в режим приема данных)
	
	lcd.init();
	lcd.backlight();
	lcd.setCursor(0,0);
	lcd.print("I2CLCDInit complite!");
	
	sensors.begin();
	delay(5);
	// locate devices on the bus
	lcd.clear();
	lcd.setCursor(0,0);
	lcd.print("Locating devices...");
	//Serial.print("Locating devices...");
	lcd.setCursor(0,1);
	//Serial.print("Found ");
	lcd.print("Found ");
	//Serial.print(sensors.getDeviceCount(), DEC);
	lcd.print(sensors.getDeviceCount());
	delay(500);
	//Serial.println(" devices.");
	lcd.print(" devices.");

  oneWire.reset_search();
  if (!oneWire.search(insideThermometer)) //Serial.println("Unable to find address for insideThermometer");
  if (!oneWire.search(outsideThermometer)) //Serial.println("Unable to find address for outsideThermometer");

  // set the resolution to TEMPERATURE_PRECISION bit per device
  sensors.setResolution(insideThermometer, TEMPERATURE_PRECISION);
  sensors.setResolution(outsideThermometer, TEMPERATURE_PRECISION);
}

void loop() {
  if (millis() - lastUpdateTime > TEMP_UPDATE_TIME) {
    lastUpdateTime = millis();
    //Serial.print("Requesting temperatures...");
    sensors.requestTemperatures();
    //Serial.println("DONE");
    delay(100);
    insideTemperature = getTemperature(insideThermometer);
    outsideTemperature = getTemperature(outsideThermometer);
  }
  if (millis() - lastPrintTime > PRINT_UPDATE_TIME) {
    lastPrintTime = millis();
    lcd.clear();
    if (insideTemperature != 1000) {
	  dataMessage.ActuatorID = 1;
	  dataMessage.DataValue = insideTemperature*100;
      lcd.setCursor(0,0);
      lcd.print("t1 = ");
      lcd.print(insideTemperature);
      digitalWrite(SerialTxControl, RS485Transmit);
      delay(50);
      dataMessage.PacketID++;
      Serial.write((uint8_t*)&dataMessage, sizeof(dataMessage));
      delay(100);
      digitalWrite(SerialTxControl, RS485Receive);
    }
    delay(10);
    if (outsideTemperature != 1000) {
	  dataMessage.ActuatorID = 2;
	  dataMessage.DataValue = outsideTemperature*100;
      lcd.setCursor(0,1);
      lcd.print("t2 = ");
      lcd.print(outsideTemperature);
      digitalWrite(SerialTxControl, RS485Transmit);
      delay(50);
      dataMessage.PacketID++;
      Serial.write((uint8_t*)&dataMessage, sizeof(dataMessage));
      delay(100);
      digitalWrite(SerialTxControl, RS485Receive);
    }
  }
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  Serial.print("Temp C: ");
  Serial.print(tempC);
}

// function to print the temperature for a device
float getTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  return tempC;
}

// function to print a device's resolution
void printResolution(DeviceAddress deviceAddress)
{
  Serial.print("Resolution: ");
  Serial.print(sensors.getResolution(deviceAddress));
  Serial.println();
}

// main function to print information about a device
void printData(DeviceAddress deviceAddress)
{
  Serial.print("Device Address: ");
  printAddress(deviceAddress);
  Serial.print(" ");
  printTemperature(deviceAddress);
  Serial.println();
}
