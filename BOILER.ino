#include "TM1637.h"
#include <EEPROM.h>
#include <OneWire.h>

#define MEASUREMENT_DELAY 1000
#define DEFAULT_TRIGGER_TEMPERATURE 15.5
#define INVALID_TEMPERATURE -273.
#define CLK_PIN 6
#define DIO_PIN 7
#define RELAY_PIN 4
#define SENSOR_PIN 2
#define LED_POWER_PIN 8
#define LED_OPENED_PIN 3
#define LED_CLOSED_PIN 5
#define BUTTON_PIN 10

TM1637 display(CLK_PIN, DIO_PIN);
OneWire oneWire(SENSOR_PIN);

float triggerTemperature;
byte firstSensorAddress[8];
byte secondSensorAddress[8];

bool opened = true;
bool lastShowedTriggerTemp = false;
float temperature;

void setup() {
  Serial.begin(9600);

  display.clearDisplay();
  display.setBrightness(3);
  display.showPoints(true);

  pinMode(LED_POWER_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_OPENED_PIN, OUTPUT);
  pinMode(LED_CLOSED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);

  digitalWrite(BUTTON_PIN, HIGH);
  digitalWrite(LED_POWER_PIN, HIGH);

  triggerTemperature = readTempFromEEPROM();
  findSensorsAddresses();

  //  changeResolution(firstSensorAddress);
  //  changeResolution(secondSensorAddress);

  gateOpen();
}

void changeResolution(byte address[]) {
  oneWire.reset();
  oneWire.select(address);
  oneWire.write(0x4E);
  oneWire.write(0x00);
  oneWire.write(0x00);
  oneWire.write(0x5f); // 0x1f - 9 bit, 0x3f - 10 bit, 0x5f - 11 bit, 0x7f - 12 bit
  oneWire.reset();

  oneWire.select(address);
  oneWire.write(0x48);
  delay(15);
}

void findSensorsAddresses() {
  oneWire.search(firstSensorAddress);
  printSensorAddress(firstSensorAddress, 1);
  bool valid = validateSensorAddressCRC(firstSensorAddress, 1);
  oneWire.search(secondSensorAddress);
  printSensorAddress(secondSensorAddress, 2);
  valid = valid && validateSensorAddressCRC(secondSensorAddress, 2);
}

void printSensorAddress(byte sensorAddress[], byte index) {
  Serial.print("Found sensor ");
  Serial.print(index);
  Serial.print(": ");
  for (int i = 0; i < 8; i++) {
    Serial.print(sensorAddress[i], HEX);
  }
  Serial.println();
}

bool validateSensorAddressCRC(byte sensorAddress[], byte index) {
  if (OneWire::crc8(sensorAddress, 7) != sensorAddress[7]) {
    Serial.print("CRC of Sensor ");
    Serial.print(index);
    Serial.println(" is not valid!");
    return false;
  }
  return true;
}

float readTempFromEEPROM() {
  int integralPart = EEPROM.read(0);
  int fraction = EEPROM.read(1);
  float temperature = (float)integralPart + (float)fraction / 100.;
  if (temperatureIsValid(temperature)) {
    Serial.print("*** Temperature read from EEPROM: ");
    Serial.println(temperature);
    return temperature;
  }
  Serial.print("*** Couldn't read temperature from EEPROP, using default: ");
  Serial.println(DEFAULT_TRIGGER_TEMPERATURE);
  return DEFAULT_TRIGGER_TEMPERATURE;
}

void saveTempToEEPROM(float temperature) {
  int tempValue = (int)(temperature * 100.);
  int fraction = tempValue % 100;
  int integralPart = (tempValue - fraction) / 100;
  EEPROM.update(0, integralPart);
  EEPROM.update(1, fraction);
  Serial.print("*** Temperature saved to EEPROM: ");
  Serial.println(temperature);
}

bool temperatureIsValid(float temp) {
  return temp >= 0. && temp <= 125.;
}

float getTemperature() {
  startSensorTemperatureConversion(firstSensorAddress);
  startSensorTemperatureConversion(secondSensorAddress);

  operationalLoop(MEASUREMENT_DELAY);

  float firstTemperature = readTemperatureFromSensor(firstSensorAddress, 1);
  float secondTemperature = readTemperatureFromSensor(secondSensorAddress, 2);

  bool firstTempIsValid = temperatureIsValid(firstTemperature);
  bool secondTempIsValid = temperatureIsValid(secondTemperature);
  if (!firstTempIsValid && !secondTempIsValid) {
    return INVALID_TEMPERATURE;
  } else if (!firstTempIsValid) {
    return secondTemperature;
  } else if (!secondTempIsValid) {
    return firstTemperature;
  }
  return min(firstTemperature, secondTemperature);
}

void startSensorTemperatureConversion(byte sensorAddress[]) {
  oneWire.reset();
  oneWire.select(sensorAddress);
  oneWire.write(0x44, 1);
}

float readTemperatureFromSensor(byte sensorAddress[], byte index) {
  byte sensorData[12];

  oneWire.reset();
  oneWire.select(sensorAddress);
  oneWire.write(0xBE);
  for (byte i = 0; i < 9; i++) {
    sensorData[i] = oneWire.read();
  }

  int16_t raw = (sensorData[1] << 8) | sensorData[0];
  byte cfg = (sensorData[4] & 0x60);
  if (cfg == 0x00) raw = raw & ~7;
  else if (cfg == 0x20) raw = raw & ~3;
  else if (cfg == 0x40) raw = raw & ~1;

  float temp = (float)raw / 16.0;

  Serial.print("Sensor ");
  Serial.print(index);
  Serial.print(": ");
  Serial.println(temp);

  return temp;
}

void operationalLoop(int delayMs) {
  unsigned long start = millis();
  while ((unsigned long)(millis() - start) < delayMs) {
    showCurrentOrTriggerTemp(temperature);
    checkSerialCommands();
    delay(100);
  }
}

void checkSerialCommands() {
  if (Serial.available()) {
    char str[10];
    String input = Serial.readString();
    if (input.substring(0, 4) == "set ") {
      float newTemperature = input.substring(4).toFloat();
      if (temperatureIsValid(newTemperature) && newTemperature != 0.) {
        triggerTemperature = newTemperature;
        saveTempToEEPROM(triggerTemperature);
        Serial.print("CMD => Trigger temperature set to: ");
        Serial.println(triggerTemperature);
      } else {
        Serial.println("CMD => Invalid temperature");
      }
    } else {
      Serial.println("CMD => Invalid command");
    }
  }
}

void loop() {
  temperature = getTemperature();
  if (!temperatureIsValid(temperature)) {
    errorState();
  }
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" => ");
  Serial.println(triggerTemperature);

  if (opened) {
    if (temperature < triggerTemperature - 0.1) {
      gateClose();
      opened = false;
    }
  } else {
    if (temperature > triggerTemperature + 0.1) {
      gateOpen();
      opened = true;
    }
  }
  Serial.println("=====");
}

void gateOpen() {
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_OPENED_PIN, HIGH);
  digitalWrite(LED_CLOSED_PIN, LOW);
  Serial.println("Gate OPEN");
}

void gateClose() {
  digitalWrite(RELAY_PIN, HIGH);
  digitalWrite(LED_OPENED_PIN, LOW);
  digitalWrite(LED_CLOSED_PIN, HIGH);
  Serial.println("Gate CLOSE");
}

void showCurrentOrTriggerTemp(float temperature) {
  float numberToDisplay;

  bool buttonPressed = digitalRead(BUTTON_PIN) == LOW;
  if (buttonPressed) {
    numberToDisplay = triggerTemperature;
    lastShowedTriggerTemp = true;
    digitalWrite(LED_OPENED_PIN, HIGH);
    digitalWrite(LED_CLOSED_PIN, HIGH);
  } else {
    numberToDisplay = temperature;
    if (lastShowedTriggerTemp) {
      digitalWrite(LED_OPENED_PIN, opened ? HIGH : LOW);
      digitalWrite(LED_CLOSED_PIN, opened ? LOW : HIGH);
    }
    lastShowedTriggerTemp = false;
  }

  display.displayTwoNumbers((int)numberToDisplay, (int)(numberToDisplay * 100.) % 100);
}

void errorState() {
  int counter = 0;
  while (true) {
    digitalWrite(LED_POWER_PIN, HIGH);
    digitalWrite(LED_OPENED_PIN, HIGH);
    digitalWrite(LED_CLOSED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_POWER_PIN, LOW);
    digitalWrite(LED_OPENED_PIN, LOW);
    digitalWrite(LED_CLOSED_PIN, LOW);
    delay(500);
    counter++;
    if (counter == 7) {
      counter = 0;
      Serial.println("ERROR: got bad temperature result. Is there at least one DS18B20 connected?");
    }
  }
}

