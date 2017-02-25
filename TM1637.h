#ifndef TM1637_h
#define TM1637_h
/********** TM1637 specific definitions **********/
#define ADDR_AUTO  0x40
#define ADDR_FIXED 0x44
#define STARTADDR  0xc0
#define POINT_ADDR 0x80
#define BRGHT_ADDR 0x88
/*************************************************/
#define EMPTY_ALIAS 0x7f
#define DEFAULT_BRIGHTNESS 2

class TM1637 {
  public:
    TM1637(uint8_t, uint8_t);
    void clearDisplay(void);
    void display(uint8_t display_data[]);
    void display(uint8_t bit_address, uint8_t data);
    void displayNumber(uint16_t number, uint8_t length = 4, uint8_t start_index = 0);
    void displayTwoNumbers(uint8_t n1, uint8_t n2);
    void setBrightness(uint8_t brightness = DEFAULT_BRIGHTNESS);
    void showPoints(bool use_points);
    void showLeadingZeros(bool leading_zeros);
    void displayHell();
  private:
    uint8_t clk_pin;
    uint8_t data_pin;
    uint8_t _point_data;
    uint8_t _brightness;
    bool _leading_zeros;
    int8_t convertDataToSegments(uint8_t data);
    void convertDataArrayToSegments(uint8_t data[]);
    void displayRawData(uint8_t bit_address, int8_t data);
    void startTransmission(void);
    void stopTransmission(void);
    void writeByte(uint8_t data);
};

#endif
