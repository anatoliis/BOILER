#include "TM1637.h"
#include <EEPROM.h>
#include <OneWire.h>

#define DEFAULT_TRIGGER_TEMPERATURE 22.5
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

  gateOpen();
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

float triggerTemperature = readTempFromEEPROM();
byte currentSensorAddress[8];
bool opened = true;
bool lastShowedTriggerTemp = false;
float temperature;

float getTemperatureFromSensors() {
  float minTemperature = 10000;
  bool gotAtLeastOneValidResult = false;

  int sensorIndex = 0;
  while (oneWire.search(currentSensorAddress)) {
    float temp = getTemperatureFromNextSensor();
    Serial.print("Sensor ");
    Serial.print(sensorIndex);
    Serial.print(": ");
    if (temperatureIsValid(temp)) {
      minTemperature = min(minTemperature, temp);
      Serial.println(temp);
      gotAtLeastOneValidResult = true;
    } else {
      Serial.println("Invalid");
    }
    sensorIndex++;
  }
  oneWire.reset_search();
  
  if (gotAtLeastOneValidResult) return minTemperature;
  return INVALID_TEMPERATURE;
}

bool temperatureIsValid(float temp) {
  return temp >= -30. && temp <= 125.;
}

float getTemperatureFromNextSensor() {
  byte i;
  byte data[12];

  if (OneWire::crc8(currentSensorAddress, 7) != currentSensorAddress[7]) {
    Serial.println("CRC is not valid!");
    return INVALID_TEMPERATURE;
  }

  oneWire.reset();
  oneWire.select(currentSensorAddress);
  oneWire.write(0x44, 1);

  operationalLoop(850);

  oneWire.reset();
  oneWire.select(currentSensorAddress);
  oneWire.write(0xBE);

  for ( i = 0; i < 9; i++) {
    data[i] = oneWire.read();
  }

  int16_t raw = (data[1] << 8) | data[0];
  byte cfg = (data[4] & 0x60);
  if (cfg == 0x00) raw = raw & ~7;
  else if (cfg == 0x20) raw = raw & ~3;
  else if (cfg == 0x40) raw = raw & ~1;
  
  return (float)raw / 16.0;
}

void operationalLoop(int delayMs) {
  unsigned long start = millis();
  unsigned long finish = start + delayMs;
  while (millis() < finish) {
    showCurrentOrTriggerTemp(temperature);
    checkSerialCommands();
    delay(50);
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
  temperature = getTemperatureFromSensors();
  if (!temperatureIsValid(temperature)) {
    errorState();
  }
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" => ");
  Serial.println(triggerTemperature);

  if (opened) {
    if (temperature < triggerTemperature - 0.05) {
      gateClose();
      opened = false;
    }
  } else {
    if (temperature > triggerTemperature + 0.05) {
      gateOpen();
      opened = true;
    }
  }
  Serial.println("==========");
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

