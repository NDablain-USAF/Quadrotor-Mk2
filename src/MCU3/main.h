#ifndef MAIN_H
#define MAIN_H

#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 16000000UL // Change me
#endif

#define SS_LORA_PIN PINB1
#define LORA_REG_OP_MODE 0x01
#define LORA_REG_F_MSB 0x06
#define LORA_REG_F_MIDB 0x07
#define LORA_REG_F_LSB 0x08
#define LORA_REG_FIFO_ADR_PTR 0x0D
#define LORA_REG_FIFO 0x00
#define LORA_REG_TX_BASE 0x80
#define LORA_REG_PAYLOAD_LENGTH 0x22
#define LORA_TRANSMIT 0b10000011
#define LORA_REG_IRQ_MASK 0x11
#define LORA_REG_IRQ_FLAGS 0x12

void Detain(unsigned long duration){
	volatile unsigned long i;
	while(++i<duration){}
}

void SPI_Read_Register(char Port, char Pin, char Register, unsigned char* buffer, unsigned char length){
	if (Port==1){PORTB &= ~(1<<Pin);}
	else if (Port==3){PORTD &= ~(1<<Pin);}
	else if (Port==2){PORTC &= ~(1<<Pin);}

	SPDR = Register; // Send register to read from, 0 in 7th bit signifies read command
	while (!(SPSR & (1<<SPIF))){asm("");} // Wait for flag to be set in status register to continue
	unsigned char i = 0;
	while(i++<length){
		SPDR = 1; // Must perform a dummy write even during read
		while (!(SPSR & (1<<SPIF))){asm("");} // Wait for flag to be set in status register to continue
		*buffer = SPDR;
		++buffer;
	}
	
	if (Port==1){PORTB |= (1<<Pin);}
	else if (Port==3){PORTD |= (1<<Pin);}
	else if (Port==2){PORTC |= (1<<Pin);}
}

unsigned char SPI_Write_Register(char Port, char Pin, char Register, unsigned char Data){
	if (Port==1){PORTB &= ~(1<<Pin);}
	else if (Port==2){PORTC &= ~(1<<Pin);}
	else if (Port==3){PORTD &= ~(1<<Pin);}

	SPDR = (Register|(1<<7)); // Send register to write to, 7th bit must be 1 for write
	while (!(SPSR & (1<<SPIF))){asm("");} // Wait for flag to be set in status register to continue
	SPDR = Data; // Send new value
	while (!(SPSR & (1<<SPIF))){asm("");}
	unsigned char inbound = SPDR;
	//if (inbound!=Data){return 0;}
	
	if (Port==1){PORTB |= (1<<Pin);}
	else if (Port==2){PORTC |= (1<<Pin);}
	else if (Port==3){PORTD |= (1<<Pin);}
	
	return inbound;
}

void ATMega328P_Init(){
	SPCR |= (1<<SPE) | (1<<MSTR) | (1<<SPR0);
}

void LoRa_Init(){
	DDRB |= (1<<DDB5) | (1<<DDB3) | (1<<DDB1);
	PORTB |= (1<<PORTB1); // Set SS high
	(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_OP_MODE,0b10000000); // Set to LoRa mode
	(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_F_MSB,0b11100100); // Set frequency to 915 MHz
	(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_F_MIDB,0b11000000);
	(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_F_LSB,0b00000000);
	(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_IRQ_MASK,0b11110111); // Only keep TX done bit unmasked
}

void LoRa_Transmit(unsigned char data){
	//(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_FIFO_ADR_PTR,LORA_REG_TX_BASE);
	(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_FIFO,data);	
	(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_OP_MODE,LORA_TRANSMIT);
	unsigned char status = 0;
	while (status!=8){
		(void)SPI_Read_Register(1,SS_LORA_PIN,LORA_REG_IRQ_FLAGS,&status,1);
	}
	
	
}
#endif