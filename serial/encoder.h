#include <SPI.h>

const double ADC_MAX = double(0b1111111111);
const double AS5045_MAX = double(0b111111111111);

class Encoder
{
public:
  virtual double read();
};

class ADCEncoder: public Encoder
{
public:
  int adcPin;

public:
  ADCEncoder(int adc): adcPin(adc)
  {
    pinMode(adcPin, INPUT_PULLUP);
  }

  double read()
  {
    return double(analogRead(adcPin)) / ADC_MAX;
  }
};

class AS5045Encoder: public Encoder
{
private:
  struct AS5045
  {
    unsigned int position : 14;
  };

public:
  int csPin;

public:
  AS5045Encoder(int cs): csPin(cs)
  {
    SPI.begin();
    SPI.beginTransaction(SPISettings(
      1e6,
      MSBFIRST,
      SPI_MODE0
    ));

    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
  }

  ~AS5045Encoder()
  {
    SPI.endTransaction();
    SPI.end();
  }

public:
  double read()
  {
    // Select
    digitalWrite(csPin, LOW);
    delayMicroseconds(8);

    // Read
    AS5045 data;
    *((uint16_t*)&data) = SPI.transfer16(0x0);

    // Deselect
    digitalWrite(csPin, HIGH);

    // Convert
    return double(data.position) / double(AS5045_MAX);
  }
};
