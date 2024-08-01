#include <avr/io.h>

volatile char g_Timer_Flag = 0; 

void Transmit(int outbound);

int main(void){
    DDRB |= (1<<DDB5) | (1<<DDB3); // Set SCK and MOSI as outputs
	SPCR |= (1<<SPE) | (1<<MSTR) | (1<<SPR0); // Enables SPI, sets AVR as master, sets clock to f_osc/16
    while (1) {
		if (g_Timer_Flag==1){Transmit(5);}

    }
}

void Transmit(int outbound){
	SPDR = outbound; // Put data into data register
	while (!(SPSR & (1<<SPIF))); // Wait for flag to be set in status register to continue
	g_Timer_Flag = 0;
}
