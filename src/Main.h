#ifndef MAIN_H
#define MAIN_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include <math.h>
#include <HardwareSerial.h>

// Transmit: Writes one byte of data into a slave device register
// returns 4 if successful, lower numbers indicate stage of error 
uint8_t Transmit(uint8_t Slave_Address, uint8_t Register_Address, uint8_t Register_Data);
// Read: Reads up to 256 bytes of data from the specified slave address
// IMPORTANT, is looking for an array of data, pass address if only reading a single value
// Returns 8 if successful, lower numbers indicate stage of error
uint8_t Read(uint8_t Slave_Address, uint8_t Register_Address, uint8_t *data, uint8_t length);
// A reproduction of the delay() function, prevents overwhelming the serial monitor
void Detain(uint16_t length);
// Use to group Transmit calls when sensor is being set up 
// Returns 2 if successful, lower numbers indicate stage of error
uint8_t Initialize_BMP390(float *PAR_T, float *PAR_P);
// Use this to read the Temperature data off the sensor
float Read_BMP390(float *PAR_T, float *PAR_P);
// Use to group MCU one time register setups prior to running while loop
void Initialize_ATMEGA328P();

#define BMTR_ADRS 0x77 // BMP 390 Address
#define BMTR_PWR_CTRL 0x1B // Power control register
#define BMTR_TEMP_DATA 0x07 // First bit of temperature data
#define BMTR_PRES_DATA 0x04 // First bit of pressure data
#define BMTR_OSR 0x1C // Output sample rate register
#define BMTR_ODR 0x1D // Output data rate register
#define BMTR_IIR 0x1F // IIR filter register
#define BMTR_INT 0x19 // Interrupt Control register
#define BMTR_CON_START 0x31 // Start register of factory calibration data
// Below constants are scaled to use integers instead of floats 
// Temperature calibration constants
#define BMTR_cT1 pow(2,-8) // 2^-8 
#define BMTR_cT2 pow(2,30) // 2^30 
#define BMTR_cT3 pow(2,48) // 2^48 
// Pressure calibration constants
#define BMTR_cP0 pow(2,14) // 2^14
#define BMTR_cP1 pow(2,20) // 2^20 
#define BMTR_cP2 pow(2,29) // 2^29 
#define BMTR_cP3 pow(2,32) // 2^32 
#define BMTR_cP4 pow(2,37)// 2^37 
#define BMTR_cP5 pow(2,-3) // 2^-3 
#define BMTR_cP6 pow(2,6) // 2^6
#define BMTR_cP7 pow(2,8) // 2^8
#define BMTR_cP8 pow(2,15) // 2^15
#define BMTR_cP9 pow(2,48) // 2^48 
#define BMTR_cP10 pow(2,48) //281474976710656 // 2^48 
#define BMTR_cP11 pow(2,65)//3689348814741910323 // 2^65 scaled by 1e-1
// Height calculation constants
#define BMTR_Tb 288.15 
#define BMTR_Lb -0.0065
#define BMTR_Pb 101325
#define BMTR_R 8.31432
#define BMTR_g 9.80665
#define BMTR_M 0.0289644

volatile uint8_t bmtr_flag;

#endif
