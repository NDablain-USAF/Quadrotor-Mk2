#include "spi.h"

int main(void){
	SPI_Init();
	while (1) {
		int x = -1234;
		Transmit((long)x);
	}
}

void SPI_Init(){
	DDRB |= (1<<PINB5) | (1<<PINB3) | (1<<PINB2); // Set SCK, MOSI, and SS as outputs
	PORTB |= (1<<PINB2); // Set SS high
	SPCR |= (1<<SPE) | (1<<MSTR) | (1<<SPR0); // Enables SPI, sets AVR as master, sets clock to f_osc/16
}

void Transmit(long outbound){
	char buffer[10];
	int n = sprintf(buffer,"%ld",outbound);
	PORTB &= ~(1<<PINB2); // Set B2 LOW
	for (int i=0;i<n;i++){
		SPDR = buffer[i]; // Put data into data register
		while (!(SPSR & (1<<SPIF))){asm("");} // Wait for flag to be set in status register to continue
	}
	SPDR = 10; // Line feed
	while (!(SPSR & (1<<SPIF))){asm("");}
	PORTB |= (1<<PINB2); // Set B2 HIGH
}
