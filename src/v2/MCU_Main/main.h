#ifndef MAIN_H
#define MAIN_H

#include <xc.h>

#define SS_BAR 2 // Control bit in adr is 0 for a write and 1 for a read
#define BAR_PORT 
#define BAR_CTRL_REG1 0x10 // Sets the ODR and OSR
#define BAR_CTRL_REG2 0x11 // Sets IIR
#define BAR_CTRL_REG4 0x13 // Sets int
#define BAR_RPDS 0x1A // Start of reference pressure, 16 bit
#define BAR_DATA 0x28 // Start of pressure data, 24 bit
#define BAR_SCALE_FACTOR 4096.0f // Divide pressure by this to arrive at hPa (100 Pa)
#define BAR_TB 288.15f // Standard temperature at sea level (K)
#define BAR_LB -0.0065f // Standard temperature lapse rate (K/m)
#define BAR_PB 101325f // Standard static pressure at sea level (Pa)
#define BAR_R 8.31432f // Universal gas constant (N-m)/(mol-K)
#define BAR_G 9.80665f // Acceleration due to gravity (m/s^2)
#define BAR_M 0.0289644f // Molar mass of air (kg/mol)

#define SS_MAG 3
#define SS_DGW 4
#define SS_IMU 2
#define SS_LORA 3

typedef struct {
	unsigned long Reference_Pressure;
	float Measured_Pressure;
} BAR;

void AVR_Setup(){
	// Setup SPI SS pins
	PORTF_DIRSET |= (1<<SS_BAR) | (1<<SS_MAG) | (1<<SS_DGW);
	PORTF_OUTSET |= (1<<SS_BAR) | (1<<SS_MAG) | (1<<SS_DGW);
	PORTA_DIRSET |= (1<<SS_IMU) | (1<<SS_LORA) | (1<<4) | (1<<6); // PA4  (MOSI), PA6 (SCK)
	PORTA_OUTSET |= (1<<SS_IMU) | (1<<SS_LORA);
	// Setup SPI interface
	SPI0_CTRLA |= (1<<5) | (1<<1) | (1<<0); // Set master mode, /16 prescaler, enable spi
}

void SPI_Transfer(char port, unsigned char pin, unsigned char address, unsigned char* data, unsigned char length){
	switch (port) {
		case 97:
			PORTA_OUTCLR |= (1<<pin);
			break;
		case 98:
			PORTB_OUTCLR |= (1<<pin);
			break;
		case 99:
			PORTC_OUTCLR |= (1<<pin);
			break; 
		case 100:
			PORTD_OUTCLR |= (1<<pin);
			break;
		case 101:
			PORTE_OUTCLR |= (1<<pin);
			break;
		case 102:
			PORTF_OUTCLR |= (1<<pin);
			break; 
	}

	SPI0_DATA = address;
	while (SPI0_INTFLAGS & ~(1<<7));

	unsigned char i = 0;
	unsigned char temp = 0;
	while (i++<length){
		temp = SPI0_DATA;
		SPI0_DATA = *data;
		*data++ = temp;
		while (SPI0_INTFLAGS & ~(1<<7));
	}

	switch (port) {
	case 97:
		PORTA_OUTSET |= (1<<pin);
		break;
	case 98:
		PORTB_OUTSET |= (1<<pin);
		break;
	case 99:
		PORTC_OUTSET |= (1<<pin);
		break; 
	case 100:
		PORTD_OUTSET |= (1<<pin);
		break;
	case 101:
		PORTE_OUTSET |= (1<<pin);
		break;
	case 102:
		PORTF_OUTSET |= (1<<pin);
		break; 
}
}

void BAR_SETUP(BAR* sensor){
	unsigned char temp = 0b00011111;
	SPI_Transfer(BAR_PORT,SS_BAR,BAR_CTRL_REG1,&temp,1); // Sets 10Hz ODR, 512 OSR (max)
	temp = 0b00100000;
	SPI_Transfer(BAR_PORT,SS_BAR,BAR_CTRL_REG2,&temp,1); // Sets ODR/9 for IIR
	temp = 0b01100000;
	SPI_Transfer(BAR_PORT,SS_BAR,BAR_CTRL_REG4,&temp,1); // Enables interrupt on INT pin
	unsigned char data[2];
	SPI_Transfer(BAR_PORT,SS_BAR,BAR_RPDS,data,2); // Pulls reference pressure
	sensor->Reference_Pressure = (data[1]*256) + data[0];
}

void BAR_READ(BAR* sensor){
	unsigned char data[3];
	float temp = (data[2]*65536) + (data[1]*256) + data[0];
	sensor->Measured_Pressure = temp/BAR_SCALE_FACTOR;
}

#endif
