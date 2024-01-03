
#include"Soft_SPI.h"

#define cs 10
#define mosi 11
#define miso 12
#define sck 13

SoftSPI_Dev _spi(cs,mosi,miso,sck);

void setup() 
{  
  Serial.begin(115200);
  Serial.println("Testing Firmware for the BME680 By BOSCH!");
  Serial.println("Initializing Chip for SPI use.");
  //Configure our pins
  _spi.begin(MSBFIRST);
  
}

void loop() 
{
  delay(3000);
  Serial.println("Reading");
}
