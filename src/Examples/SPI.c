#include <avr/io.h>

volatile char g_Timer_Flag = 0; 

void Transmit(char outbound);

int main(void){
    DDRB |= (1<<DDB5) | (1<<DDB3); // Set SCK and MOSI as outputs
    SPCR |= (1<<SPE) | (1<<MSTR) | (1<<SPR0); // Enables SPI, sets AVR as master, sets clock to f_osc/16
    while (1) {
	if (g_Timer_Flag==1){Transmit(5);}
    }
}

void Transmit(char outbound){
	SPDR = outbound; // Put data into data register
	while (!(SPSR & (1<<SPIF))); // Wait for flag to be set in status register to continue
	SPSR &= 0b01111111; // Ensure SPIF flag is reset
	g_Timer_Flag = 0;
}
