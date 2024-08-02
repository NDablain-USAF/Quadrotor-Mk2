#include <avr/io.h>

#define F_CPU 16000000UL

void Transmit(unsigned char outbound);

int main(void){
	DDRB |= (1<<PINB5) | (1<<PINB3) | (1<<PINB2); // Set SCK, MOSI, and SS as outputs
	PORTB |= (1<<PINB2);
	SPCR |= (1<<SPE) | (1<<MSTR) | (1<<SPR1) | (1<<SPR0); // Enables SPI, sets AVR as master, sets clock to f_osc/128
	while (1) {
		unsigned char x = 1;
		Transmit(x);
		//volatile int i = 0;
		//while(i<60000){i++;}
	}
}

void Transmit(unsigned char outbound){
	PORTB &= ~(1<<PINB2); // Set B2 LOW
	SPDR = (outbound+48); // Put data into data register
	while (!(SPSR & (1<<SPIF))){asm("");} // Wait for flag to be set in status register to continue
	PORTB |= (1<<PINB2); // Set B2 HIGH
}
