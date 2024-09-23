#ifndef MAIN_H
#define MAIN_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

#ifndef F_CPU
#define F_CPU 20000000UL 
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
#define LORA_REG_PA_CONFIG 0x09
#define LORA_REG_OCP 0x0B

#define BUTTON_DELAY 60000
#define START 5000
#define STOP 7000
#define START_LB (unsigned char)START
#define START_MB START>>8
#define STOP_LB (unsigned char)STOP
#define STOP_MB STOP>>8
#define MUX_MASK 0b11110000
#define POT_MUX1 0b00000011 // ADC3
#define POT_MUX2 0b00000100 // ADC4
#define POT_MUX3 0b00000101 // ADC5

struct Potentiometer {
	char Mux;
	char Led;
	int Voltage;
	};

void Detain(unsigned long duration){
	volatile unsigned long i;
	while(++i<duration);
}

void SPI_Transmit_Long_Data_Visualizer(long outbound, unsigned char length, unsigned char option){
	char buffer[length];
	unsigned char n = snprintf(buffer,length,"%ld",outbound);
	PORTB &= ~(1<<PINB2); // Set B2 LOW
	for (unsigned char i=0;i<n;i++){
		SPDR = buffer[i]; // Put data into data register
		while (!(SPSR & (1<<SPIF))){asm("");} // Wait for flag to be set in status register to continue
	}
	if (option==2){SPDR = 10;} // Line feed
	else {SPDR = 44;} // ,
	while (!(SPSR & (1<<SPIF))){asm("");}
	PORTB |= (1<<PINB2); // Set B2 HIGH
}

void SPI_Read_Register(char Port, char Pin, char Register, unsigned char* buffer, unsigned char length){
	if (Port==1){PORTB &= ~(1<<Pin);}
	else if (Port==3){PORTD &= ~(1<<Pin);}
	else if (Port==2){PORTC &= ~(1<<Pin);}

	SPDR = Register; // Send register to read from, 0 in 7th bit signifies read command
	while (!(SPSR & (1<<SPIF))); // Wait for flag to be set in status register to continue
	
	unsigned char i = 0;
	while(i<length){
		SPDR = 1; // Must perform a dummy write even during read
		while (!(SPSR & (1<<SPIF))); // Wait for flag to be set in status register to continue
		*buffer = SPDR;
		++i;
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
	while (!(SPSR & (1<<SPIF))); // Wait for flag to be set in status register to continue
	SPDR = Data; // Send new value
	while (!(SPSR & (1<<SPIF)));
	unsigned char inbound = SPDR;
	
	if (Port==1){PORTB |= (1<<Pin);}
	else if (Port==2){PORTC |= (1<<Pin);}
	else if (Port==3){PORTD |= (1<<Pin);}
	return inbound;
}

void ATMega328P_Init(){
	sei();
	// ADC Setup
	ADMUX |= (1<<REFS0); // Use Vcc as reference voltage for ADC
	ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // Enable ADC, set /128 prescaler
	// SPI Setup
	DDRB |= (1<<DDB5) | (1<<DDB3) | (1<<DDB2);
	SPCR |= (1<<SPE) | (1<<MSTR) | (1<<SPR0);
	// LED Setup
	DDRD |= (1<<DDD7) | (1<<DDD6) | (1<<DDD5) | (1<<DDD4);
	PORTD &= ~((1<<PORTD7) | (1<<PORTD6) | (1<<PORTD5) | (1<<PORTD4));
	// Button Setup
	PCICR |= (1<<2);
	PCMSK2 |= (1<<3);
}

void LoRa_Init(){
	DDRB |= (1<<DDB1);
	(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_OP_MODE,0b10000000); // Set to LoRa mode
	(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_F_MSB,0b11100100); // Set frequency to 915 MHz
	(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_F_MIDB,0b11000000);
	(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_F_LSB,0b00000000);
	(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_IRQ_MASK,0b11110111); // Only keep TX done bit unmasked
	(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_OCP,0b00111111);
	(void)SPI_Write_Register(1,SS_LORA_PIN,0x0C,0b00100011);
	(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_PA_CONFIG,0b11110010); // more..MORE...MORE!!!
}

void LoRa_Transmit(unsigned char data[14]){
	unsigned char i = 0;
	(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_FIFO_ADR_PTR,LORA_REG_TX_BASE);
	(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_PAYLOAD_LENGTH,14);
	while (i<14){
		(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_FIFO,data[i]);
		++i;
		//++data;	
	}
	(void)SPI_Write_Register(1,SS_LORA_PIN,LORA_REG_OP_MODE,LORA_TRANSMIT);
	unsigned char status = 0;
	while (status!=8){
		(void)SPI_Read_Register(1,SS_LORA_PIN,LORA_REG_IRQ_FLAGS,&status,1);
	}
}

void Sample_Pot(struct Potentiometer** Pot){
	ADMUX &= MUX_MASK;
	ADMUX |= (*Pot)->Mux;
	ADCSRA |= (1<<ADSC); // Begin conversion
	while (!(ADCSRA&(1<<ADIF))); // Wait for conversion to finish
	(*Pot)->Voltage = ADC;
	if (((*Pot)->Voltage<550)&&((*Pot)->Voltage>450)){PORTD |= (1<<(*Pot)->Led);}
	else {PORTD &= ~(1<<(*Pot)->Led);}	
}

void Measure_Pot(struct Potentiometer* Pot1, struct Potentiometer* Pot2, struct Potentiometer* Pot3){
	Sample_Pot(&Pot1);
	Sample_Pot(&Pot2);
	Sample_Pot(&Pot3);
}

void Run_Radio(struct Potentiometer* Pot1,struct Potentiometer* Pot2,struct Potentiometer* Pot3){
	int height = Pot1->Voltage>>1;
	unsigned char height_LB = (unsigned char)height;
	unsigned char height_MB = height>>8;
	unsigned char message_height[14] = {74,117,115,116,32,105,110,32,116,105,109,101,height_LB,height_MB}; // Luckily the message arrived just in time
	unsigned char message_start[14] = {74,117,115,116,32,105,110,32,116,105,109,101,START_LB,START_MB};
	unsigned char message_stop[14] = {74,117,115,116,32,105,110,32,116,105,109,101,STOP_LB,STOP_MB};
	static char onetime_start = 0;
	if (PIND&(1<<PIND5)){
		if (onetime_start){LoRa_Transmit(message_height);}
		else{
			LoRa_Transmit(message_start);
			++onetime_start;
		}
	}
	else {LoRa_Transmit(message_stop);}
}

ISR (PCINT2_vect){
	if (!(PIND&(1<<PIND3))){
		Detain(10000UL);
		PORTD ^= (1<<PORTD5);
	}
}
#endif
