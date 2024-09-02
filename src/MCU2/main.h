#ifndef MAIN_H
#define MAIN_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

#define SS_ICE_PIN PINB2
#define SS_LORA_PIN PIND6

#define LORA_REG_OP_MODE 0x01
#define LORA_REG_F_MSB 0x06
#define LORA_REG_F_MIDB 0x07
#define LORA_REG_F_LSB 0x08
#define LORA_REG_RX_N_BYTES 0x13
#define LORA_REG_FIFO_ADR_PTR 0x0D
#define LORA_REG_RX_ADR 0x10
#define LORA_REG_FIFO 0x00
#define LORA_REG_PAYLOAD_LENGTH 0x22

#define BMP_ID 0x00
#define BMP_PWR 0x1B
#define BMP_OSR 0x1C
#define BMP_ODR 0x1D
#define BMP_INT 0x19
#define BMP_CONFIG 0x1F
#define BMP_CAL_DATA 0x31

volatile unsigned char g_start_rst_flag = 0;

int pow10(unsigned char in){
	if (in==0){return 0;}
	int out = 1;
	for (unsigned char i=0;i<in;i++){
		out *= 10;
	}
	return out;
}

void SPI_Init(){
	DDRB |= (1<<PINB5) | (1<<PINB3) | (1<<PINB2); // Set SCK, MOSI, and SS as outputs
	PORTB |= (1<<PINB2); // Set SS high
	SPCR |= (1<<SPE) | (1<<MSTR) | (1<<SPR0); // Enables SPI, sets AVR as master, sets clock to f_osc/16
}

void Detain(unsigned long duration){
	volatile unsigned long i = 0;
	while (++i<duration){}
}

void SPI_Transmit_Char_Data_Visualizer(unsigned char* outbound, unsigned char length){
	PORTB &= ~(1<<PINB2); // Set B2 LOW
	unsigned char i = 0;
	while (i++<length){
		SPDR = *outbound++; // Put data into data register
		while (!(SPSR & (1<<SPIF))){asm("");} // Wait for flag to be set in status register to continue
	}
	SPDR = 10; // Line feed
	while (!(SPSR & (1<<SPIF))){asm("");}
	PORTB |= (1<<PINB2); // Set B2 HIGH
}

void SPI_Transmit_Long_Data_Visualizer(long outbound, unsigned char length){
	char buffer[length];
	unsigned char n = snprintf(buffer,length,"%ld",outbound);
	PORTB &= ~(1<<PINB2); // Set B2 LOW
	for (unsigned char i=0;i<n;i++){
		SPDR = buffer[i]; // Put data into data register
		while (!(SPSR & (1<<SPIF))){asm("");} // Wait for flag to be set in status register to continue
	}
	SPDR = 10; // Line feed
	while (!(SPSR & (1<<SPIF))){asm("");}
	PORTB |= (1<<PINB2); // Set B2 HIGH
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


void ATMega328P_Init(){
	sei();
	DDRD |= (1<<DDD4);
	PORTD |= (1<<PORTD7);
	PORTD &= ~(1<<PORTD4);
}

void LoRa_Init(){
	DDRD |= (1<<PIND6); // Set LoRa SS as output
	PORTD |= (1<<PIND6);
	(void)SPI_Write_Register(3,SS_LORA_PIN,LORA_REG_OP_MODE,0b10000000); // Set LoRa mode, still in standby
	(void)SPI_Write_Register(3,SS_LORA_PIN,LORA_REG_F_MSB,0b11100100); // Set frequency to 915 MHz
	(void)SPI_Write_Register(3,SS_LORA_PIN,LORA_REG_F_MIDB,0b11000000);
	(void)SPI_Write_Register(3,SS_LORA_PIN,LORA_REG_F_LSB,0b00000000);
	(void)SPI_Write_Register(3,SS_LORA_PIN,LORA_REG_OP_MODE,0b10000101); // Set LoRa mode into RXCONTINUOUS
}

void BMP_Init(){
	unsigned char data[21];
	(void)SPI_Write_Register(1,1,BMP_PWR,0b00110011);
	(void)SPI_Write_Register(1,1,BMP_OSR,0b00010101);
	(void)SPI_Write_Register(1,1,BMP_ODR,0x04);
	(void)SPI_Write_Register(1,1,BMP_INT,0b01000000);
	(void)SPI_Write_Register(1,1,BMP_CONFIG,0b0001000);
	(void)SPI_Read_Register(1,1,BMP_CAL_DATA,data,21);
	for (int i=0;i<21;i++){
		SPI_Transmit_Long_Data_Visualizer((long)data[i],5);
	}
	Detain(5000UL);
}

void LoRa_Read(int* value){
	unsigned char data_available;
	SPI_Read_Register(3,SS_LORA_PIN,LORA_REG_RX_N_BYTES,&data_available,1);
	if (data_available>0){
		unsigned char buffer[data_available];
		unsigned char RX_Adrs;
		SPI_Read_Register(3,SS_LORA_PIN,LORA_REG_RX_ADR,&RX_Adrs,1);
		(void)SPI_Write_Register(3,SS_LORA_PIN,LORA_REG_FIFO_ADR_PTR,RX_Adrs); // Set FIFO ptr to current FIFO RX address
		SPI_Read_Register(3,SS_LORA_PIN,LORA_REG_FIFO,buffer,data_available);
		int check_sum = 0;
		int sum = 0;
		for (unsigned char i=0;i<data_available;i++){
			if (i<12){check_sum+=buffer[i];}
			else if (i>=12){sum+=((buffer[i]-48)*pow10(data_available-i-1));}
		}
		if (sum==5000){
			if (g_start_rst_flag==0){
				PORTD |= (1<<PORTD4);
				g_start_rst_flag = 1;
			}
			else{
				DDRD |= (1<<DDD7);
				PORTD &= ~((1<<PORTD7) | (1<<PORTD4));
				Detain(1000ULL);
				DDRD &= ~(1<<DDD7);
				PORTD |= (1<<PORTD7);
				g_start_rst_flag = 0;
			}
			Detain(200000UL);
		}
		else{
			*value = sum;
		}
	}
}

void I2C_Init(){
	TWAR = (0x10<<1); // Slave address
	TWCR |= (1<<TWEA) | (1<<TWEN);
}

unsigned char I2C_Slave_Transmit(unsigned char* data, unsigned char length){
	// returns 3 if successful, lower numbers indicate stage of error
	if ((TWSR & 0xF8) != 0xA8){return 0;} // SLA+R received
	unsigned char j = 0;
	while (++j<length){
		TWDR = *data++;
		TWCR |= (1<<TWINT) | (1<<TWEA);
		while(!(TWCR & (1<<TWINT))){asm("");}
		if ((TWSR & 0xF8) != 0xB8){return 1;}
	}
	TWDR = *data;
	data++;
	TWCR |= (1<<TWINT);
	TWCR ^= (1<<TWEA);
	while(!(TWCR & (1<<TWINT))){asm("");}
	if ((TWSR & 0xF8) != 0xC8){return 2;}
	TWCR |= (1<<TWINT) | (1<<TWEA);

	return 3;
}

#endif
