
#include"Soft_SPI.h"
#include "bme68x.h"
#define cs 10
#define mosi 11
#define miso 12
#define sck 13

SoftSPI_Dev _spi(cs,mosi,miso,sck);

static void delay_usec(uint32_t us, void* intf_ptr);
static int8_t spi_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* interface);
static int8_t spi_write(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* interface);
bme68x_dev gas_sensor;

void setup() 
{  
  Serial.begin(115200);
  Serial.println("Testing Firmware for the BME680 By BOSCH!");
  Serial.println("Initializing Chip for SPI use.");
  //Configure our pins
  _spi.begin(MSBFIRST);
  //Setup our structure
  gas_sensor.chip_id = 0;
  gas_sensor.intf = BME68X_SPI_INTF;
  gas_sensor.intf_ptr = &_spi;
  //It appears that we are missing our read and write methods
  gas_sensor.read = 
  gas_sensor.amb_temp = 25;
  gas_sensor.delay_us = delay_usec;
  int8_t r = bme68x_init(&gas_sensor);
  Serial.println("SPI INITIALIZED and CHIP SETUP");
  Serial.println("Result Status: " + String(r));
}

void loop() 
{
  delay(3000);
  //Serial.println("Reading");
}

static void delay_usec(uint32_t us, void* intf_ptr)
{
  delayMicroseconds(us);
  yield();
}

static int8_t spi_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* interface)
{

}

static int8_t spi_write(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* interface)
{
  SoftSPI* _dev = (SoftSPI*) intf_ptr;
  if(!)
}
