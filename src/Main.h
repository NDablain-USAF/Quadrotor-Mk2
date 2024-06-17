#ifndef MAIN_H
#define MAIN_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include <HardwareSerial.h>

// Transmit: Writes one byte of data into a slave device register
// returns 4 if successful, lower numbers indicate stage of error 
uint8_t Transmit(uint8_t Slave_Address, uint8_t Register_Address, uint8_t Register_Data);
// Read: Reads up to 256 bytes of data from the specified slave address
// IMPORTANT, is looking for an array of data, pass address if only reading a single value
// Returns 1 if successful
uint8_t Read(uint8_t Slave_Address, uint8_t Register_Address, uint8_t data[]);
// A reproduction of the delay() function, prevents overwhelming the serial monitor
void Detain(uint16_t length);
// Use to group Transmit calls when sensor is being set up 
uint8_t Initialize_BMP390(uint32_t PAR_T[3]);
// Use this to read the Temperature data off the sensor
uint16_t Read_BMP390(uint32_t PAR_T[3]);
// Use to group MCU one time register setups prior to running while loop
void Initialize_ATMEGA328P();

#define BMTR_ADRS 0x77 // BMP 390 Address
#define BMTR_PWR_CTRL 0x1B // Power control register
#define BMTR_TEMP_DATA 0x07 // First bit of temperature data
#define BMTR_OSR 0x1C // Output sample rate register
#define BMTR_ODR 0x1D // Output data rate register
#define BMTR_CON_START 0x32 // Start register of factory calibration data

#endif
