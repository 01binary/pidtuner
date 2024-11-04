/*
    encoder.h
    Gets absolute encoder readings.
    Supports SSI/SPI and analog/PWM interfaces.
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <SPI.h>

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

const float ADC_MAX = float(0b1111111111);
const float AS5045_MAX = float(0b111111111111);

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class Encoder
{
public:
  virtual float read();
};

class ADCEncoder: public Encoder
{
public:
  // Analog pin the potentiometer or encoder is connected to
  int adcPin;

public:
  ADCEncoder(int adc): adcPin(adc)
  {
    pinMode(adcPin, INPUT_PULLUP);
  }

  float read()
  {
    return float(analogRead(adcPin)) / ADC_MAX;
  }
};

// AS5045 (SPI)    | Arduino Mega (SPI)
// ------------------------------------
// GND (black)     | GND
// 5V (red)        | 5V
// SS (green)      | CS (53)
// SCK (yellow)    | SCK (52)
// MISO (white)    | MOSI (51)
// MOSI (orange)   | MISO (50)

class AS5045Encoder: public Encoder
{
public:
  int csPin;

public:
  AS5045Encoder(int cs): csPin(cs)
  {
    SPI.begin();

    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
  }

  ~AS5045Encoder()
  {
    SPI.end();
  }

public:
  float read()
  {
    // Select
    digitalWrite(csPin, LOW);
    delayMicroseconds(1);

    // Read
    SPI.beginTransaction(SPISettings(
      1e6,
      MSBFIRST,
      SPI_MODE0
    ));

    unsigned int raw = SPI.transfer16(0);

    SPI.endTransaction();

    // Deselect
    digitalWrite(csPin, HIGH);

    // Convert
    unsigned int reading = (raw >> 3) & 0x1FFF;
    return float(reading) / float(AS5045_MAX);
  }
};
