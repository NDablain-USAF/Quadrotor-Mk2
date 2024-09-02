#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef F_CPU
#define F_CPU 20000000UL
#endif

void UART_Transmit(unsigned char* outboud, unsigned char length);
void UART_Flush();
void UART_Init(unsigned long baud);
void UART_Stop();
void PA1616_Init();
void Detain(unsigned long long duration){
	volatile unsigned long long i;
	while (++i<duration){}
}

int main(void){
	//DDRD |= (1<<PIND1);
	PA1616_Init();
    //UART_Init(9600UL);
	while (1) {
		unsigned char data[80];
		unsigned char j = 0;
		while(j<80){
			while (!(UCSR0A & (1<<RXC0))){asm("");}
			data[j++] = UDR0;	
		}
		Detain(1000ULL);
    }
}

void UART_Receive(unsigned char* data){
	
}

void UART_Stop(){
	UART_Flush();
	UCSR0B ^= ((1<<RXEN0) | (1<<TXEN0));
}

void UART_Flush(){
	unsigned char fudge;
	while (UCSR0A & (1<<RXC0)){fudge = UDR0;}
}

void UART_Transmit(unsigned char* outbound, unsigned char length){
	volatile unsigned char j = 0;
	while (j<length){
		while(!(UCSR0A & (1<<UDRE0))){asm("");}
		UDR0 = *outbound;
		outbound++;
		j++;
	}
}

void UART_Init(unsigned long baud){
	long ubrr = (F_CPU/(16*baud))-1; // Set Baud rate using formula UBRR = (f_osc/(16*BAUD))-1
	if (baud==115200){ubrr = 10;}
	UBRR0L = (char)ubrr; 
	UBRR0H = ubrr>>8;
	UCSR0B |= (1<<RXEN0) | (1<<TXEN0); // Enable RX and TX lines
}

void PA1616_Init(){
	unsigned char setBaud115200[20] = {36,80,77,84,75,50,53,49,44,49,49,53,50,48,48,42,49,70,13,10}; //115200 baud rate
	/////////////////////////////$  P  M  T  K  2  5  1  ,  1  1  5  2  0  0  *  1  F  CR LF
	unsigned char setUpdate10[25] = {36,80,77,84,75,51,48,48,44,49,48,48,44,48,44,48,44,48,44,48,42,50,67,13,10}; // 10Hz data rate
	///////////////////////////$  P  M  T  K  5  0  0  ,  1  0  0  ,  0  ,  0  ,  0  ,  0  *  2  C  CR LF
	unsigned char setOutput[51] = {36,80,77,84,75,51,49,52,44,48,44,48,44,48,44,49,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,42,50,57,13,10}; // Only enable GGA
	UART_Init(9600UL);
	UART_Transmit(setBaud115200,20);
	Detain(20000000ULL);
	UART_Stop();
	Detain(20000000ULL);
	UART_Init(115200UL);
	Detain(20000000);
	UART_Transmit(setUpdate10,25);
	Detain(20000000ULL);
	UART_Transmit(setOutput,51);
	Detain(20000000ULL);
}