#include <SPI.h>

const int chipSelectPin = 7;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, HIGH);
  SPI.begin();
  //frequency of the SCLK to be between 1 MHz and 4 MHz
  //MSB first
  //The bits of information, placed on the falling edge of the SCLK and valid on the subsequent rising edge of SCLK
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0)); 
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(chipSelectPin, LOW);
  uint16_t rawData = SPI.transfer16(0x0000); //ADC081S021 delivers a single reading in 16 SCLK clock cycles.
  digitalWrite(chipSelectPin, HIGH);
  uint8_t data = (rawData >> 5) & 0xFF; //three leading zeroes, the eight bits of information, and four trailing zeroes.
  Serial.println(rawData, HEX);
  Serial.println(data);
  delay(100);
}