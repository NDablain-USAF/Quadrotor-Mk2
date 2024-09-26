#ifndef MAIN_H
#define MAIN_H

#include <xc.h>

#define SS_BAR 2
#define SS_MAG 3
#define SS_DGW 4
#define SS_IMU 2
#define SS_LORA 3

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
	
}

#endif
