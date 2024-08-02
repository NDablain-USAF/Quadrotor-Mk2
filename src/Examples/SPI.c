#include <avr/io.h>
#include <cstdio.h>

#define F_CPU 16000000UL

void Transmit(long outbound);

int main(void){
	DDRB |= (1<<PINB5) | (1<<PINB3) | (1<<PINB2); // Set SCK, MOSI, and SS as outputs
	PORTB |= (1<<PINB2); // Set SS high
	SPCR |= (1<<SPE) | (1<<MSTR) | (1<<SPR0); // Enables SPI, sets AVR as master, sets clock to f_osc/16
	while (1) {
		int16_t x = 1000;
		Transmit((long)x);
	}
}

void Transmit(long outbound){
	char buffer[20];
	uint8_t n = sprintf(buffer,%d,outbound);
	PORTB &= ~(1<<PINB2); // Set B2 LOW
	for (uint8_t i=0;i<n;i++){	
		SPDR = (buffer[i]+48); // Put data into data register
		while (!(SPSR & (1<<SPIF))){asm("");} // Wait for flag to be set in status register to continue
	}
	PORTB |= (1<<PINB2); // Set B2 HIGH
}
