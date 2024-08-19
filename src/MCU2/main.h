#ifndef MAIN_H
#define MAIN_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/twi.h>
#include "spi.h"

#define SS_ICE_PIN PINB2
#define SS_LORA_PIN PINB1

#define LORA_REG_OP_MODE 0x01
#define LORA_REG_F_MSB 0x06
#define LORA_REG_F_MIDB 0x07
#define LORA_REG_F_LSB 0x08
#define LORA_REG_RX_N_BYTES 0x13
#define LORA_REG_FIFO_ADR_PTR 0x0D
#define LORA_REG_RX_ADR 0x10
#define LORA_REG_FIFO 0x00
#define LORA_REG_PAYLOAD_LENGTH 0x22

int pow10(unsigned char in){
	if (in==0){return 0;}
	int out = 1;
	for (unsigned char i=0;i<in;i++){
		out *= 10;
	}
	return out;
}

void LoRa_Init(){
	(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_OP_MODE,0b10000000); // Set LoRa mode, still in standby
	(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_F_MSB,0b11100100); // Set frequency to 915 MHz
	(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_F_MIDB,0b11000000);
	(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_F_LSB,0b00000000);
	(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_OP_MODE,0b10000101); // Set LoRa mode into RXCONTINUOUS
}

void LoRa_Read(){
	unsigned char data_available;
	SPI_Read_Register(1,SS_LORA_PIN,LORA_REG_RX_N_BYTES,&data_available,1);
	if (data_available>0){
		unsigned char buffer[data_available];
		unsigned char RX_Adrs;
		SPI_Read_Register(1,SS_LORA_PIN,LORA_REG_RX_ADR,&RX_Adrs,1);
		(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_FIFO_ADR_PTR,RX_Adrs); // Set FIFO ptr to current FIFO RX address
		SPI_Read_Register(1,SS_LORA_PIN,LORA_REG_FIFO,buffer,data_available);
		int check_sum = 0;
		int sum = 0;
		for (unsigned char i=0;i<data_available;i++){
			if (i<12){check_sum+=buffer[i];}
			else if (i>=12){sum+=((buffer[i]-48)*pow10(data_available-i-1));}
		}
		SPI_Transmit_Long_Data_Visualizer((long)sum,10);
	}

}

void I2C_Init(){
	sei();
	TWAR |= (0x10<<1); // Slave address
	TWCR |= (1<<TWEA) | (1<<TWEN);
}

unsigned char I2C_Slave_Read(unsigned char data){
// returns 2 if successful, lower numbers indicate stage of error
	if ((TWSR & 0xF8) != 0xA8){return 0;} // SLA+R received
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT))){asm("");}
	if ((TWSR & 0xF8) != 0xC0){return 1;}
	TWCR = (1<<TWINT) | (1<<TWEA);

	return 2;
}

#endif
