#include <Arduino.h>
#include "TM1637.h"

static uint8_t predefinedChars[] = {
  0x3f, // 0
  0x06, // 1
  0x5b, // 2
  0x4f, // 3
  0x66, // 4
  0x6d, // 5
  0x7d, // 6
  0x07, // 7
  0x7f, // 8
  0x6f, // 9
  0x77, // 10
  0x7c, // 11
  0x39, // 12
  0x5e, // 13
  0x79, // 14
  0x71  // 15
};

static uint8_t specialChars[] = {
  0x76, // H
  0x79, // E
  0x38  // L 
};

TM1637::TM1637(uint8_t clk, uint8_t data) {
  clk_pin = clk;
  data_pin = data;
  _brightness = DEFAULT_BRIGHTNESS;
  pinMode(clk_pin, OUTPUT);
  pinMode(data_pin, OUTPUT);
}

void TM1637::clearDisplay(void) {
  display(0x00, EMPTY_ALIAS);
  display(0x01, EMPTY_ALIAS);
  display(0x02, EMPTY_ALIAS);
  display(0x03, EMPTY_ALIAS);
}

void TM1637::display(uint8_t bit_address, uint8_t data) {
  int8_t segment_data = convertDataToSegments(data);
  displayRawData(bit_address, segment_data);
}

int8_t TM1637::convertDataToSegments(uint8_t data) {
  if (data == EMPTY_ALIAS) {
    return 0x00 + _point_data;
  };
  return predefinedChars[data] + _point_data;
}

void TM1637::displayRawData(uint8_t bit_address, int8_t data) {
  // TODO: what is this for?
  startTransmission();
  writeByte(ADDR_FIXED);
  stopTransmission();

  // Transfering main data
  startTransmission();
  writeByte(bit_address|STARTADDR);
  writeByte(data);
  stopTransmission();

  // Setting brightness
  startTransmission();
  writeByte(BRGHT_ADDR + _brightness);
  stopTransmission();
}

void TM1637::displayHell() {
  displayRawData(0, specialChars[0]);
  displayRawData(1, specialChars[1]);
  displayRawData(2, specialChars[2]);
  displayRawData(3, specialChars[2]);
}

void TM1637::startTransmission(void) {
  // The order is important
  digitalWrite(clk_pin, HIGH);
  digitalWrite(data_pin, HIGH);
  digitalWrite(data_pin, LOW);
  digitalWrite(clk_pin, LOW);
}

void TM1637::stopTransmission(void) {
  // The order is important
  digitalWrite(clk_pin, LOW);
  digitalWrite(data_pin, LOW);
  digitalWrite(clk_pin, HIGH);
  digitalWrite(data_pin, HIGH);
}

void TM1637::writeByte(uint8_t data) {
  uint8_t count;
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(clk_pin, LOW);
    if (data & 0x01) {
      digitalWrite(data_pin, HIGH);
    } else {
      digitalWrite(data_pin, LOW);
    }
    data >>= 1;
    digitalWrite(clk_pin, HIGH);
  }
  digitalWrite(clk_pin, LOW);
  digitalWrite(data_pin, HIGH);
  digitalWrite(clk_pin, HIGH);

  pinMode(data_pin, INPUT);
  while(digitalRead(data_pin)) {
    count++;
    if (count == 200) {
      pinMode(data_pin, OUTPUT);
      digitalWrite(data_pin, LOW);
      count = 0;
    }
    pinMode(data_pin, INPUT);
  }
  pinMode(data_pin, OUTPUT);
}

void TM1637::display(uint8_t display_data[]) {
  uint8_t segment_data[4];
  for (uint8_t i = 0; i < 4; i++) {
    segment_data[i] = display_data[i];
  }
  convertDataArrayToSegments(segment_data);

  startTransmission();
  writeByte(ADDR_AUTO);
  stopTransmission();

  startTransmission();
  writeByte(STARTADDR);
  for (uint8_t i = 0; i < 4; i++) {
    writeByte(segment_data[i]);
  }
  stopTransmission();

  startTransmission();
  writeByte(BRGHT_ADDR + _brightness);
  stopTransmission();
}

void TM1637::convertDataArrayToSegments(uint8_t data[]) {
  for (uint8_t i = 0; i < 4; i++) {
    if (data[i] == EMPTY_ALIAS) {
      data[i] = 0x00 + _point_data;
    } else {
      data[i] = predefinedChars[data[i]] + _point_data;
    }
  }
}

void TM1637::displayNumber(uint16_t number, uint8_t length, uint8_t start_index) {
  uint8_t digit;
  for (int8_t i = start_index + length - 1; i >= start_index; i--) {
    digit = number % 10;
    number = number / 10;
    display(i, digit);
  }
}

void TM1637::displayTwoNumbers(uint8_t n1, uint8_t n2) {
  displayNumber(n1, 2, 0);
  displayNumber(n2, 2, 2);
}

void TM1637::setBrightness(uint8_t brightness) {
  _brightness = brightness;
}

void TM1637::showPoints(bool use_points) {
  if (use_points) {
    _point_data = POINT_ADDR;
  } else {
    _point_data = 0x00;
  }
}

void TM1637::showLeadingZeros(bool leading_zeros) {
  _leading_zeros = leading_zeros;
}
