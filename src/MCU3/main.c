#include "main.h"

int main(void){
	ATMega328P_Init();
	LoRa_Init();
	struct Potentiometer Pot1 = {POT_MUX1,PORTD6,0};
	struct Potentiometer Pot2 = {POT_MUX2,PORTD7,0};
	struct Potentiometer Pot3 = {POT_MUX3,PORTD4,0};
    while(1){
		Measure_Pot(&Pot1,&Pot2,&Pot3);
		Run_Radio(&Pot1,&Pot2,&Pot3);
		
		Detain(100000UL);
    }
	return 0;
}
