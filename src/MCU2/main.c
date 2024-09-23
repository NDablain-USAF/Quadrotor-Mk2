#include "main.h"

int main(void){
	ATMega328P_Init();
	SPI_Init();
	//I2C_Init();
	LoRa_Init();
	struct BMP barometer = {0};
	BMP_Init(&barometer);
	int passed_variable = 0;
    while(1){
		LoRa_Read(&passed_variable);
        BMP_Read(&barometer);
		//unsigned char data[2] = {0,0};
		//data[0] = (unsigned char)passed_variable;
		//data[1] = passed_variable>>8;
		//unsigned char status = I2C_Slave_Transmit(data,2);
		SPI_Transmit_Long_Data_Visualizer((long)passed_variable,10);
        SPI_Transmit_Float_Data_Visualizer(barometer.height,10);
		Detain(10000UL);
    }
	return 0;
}
