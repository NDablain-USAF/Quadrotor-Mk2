#ifndef SPI_H
#define SPI_H

#include <avr/io.h>
#include <stdio.h>

#ifndef F_CPU
#define F_CPU 20000000UL
#endif

void SPI_Init(){
	DDRB |= (1<<PINB5) | (1<<PINB3) | (1<<PINB2) | (1<<PINB1); // Set SCK, MOSI, and SS as outputs
	PORTB |= (1<<PINB2) | (1<<PINB1); // Set SS high
	SPCR |= (1<<SPE) | (1<<MSTR) | (1<<SPR0); // Enables SPI, sets AVR as master, sets clock to f_osc/16
}

void Detain(long duration){
	volatile long i = 0;
	while (++i<duration){asm("");}
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

#endif
