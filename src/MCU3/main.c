#include "main.h"

int main(void){
	ATMega328P_Init();
	LoRa_Init();
    while(1){
		unsigned char message = 20;
        LoRa_Transmit(message);
		Detain(100000UL);
    }
	return 0;
}