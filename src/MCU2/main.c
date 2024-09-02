#include "main.h"

int main(void){
	ATMega328P_Init();
	SPI_Init();
	I2C_Init();
	LoRa_Init();
	//BMP_Init();
	int passed_variable = 0;
    while(1){
		LoRa_Read(&passed_variable);
		unsigned char data[2] = {0,0};
		data[0] = (unsigned char)passed_variable;
		data[1] = passed_variable>>8;
		unsigned char status = I2C_Slave_Transmit(data,2);
		//unsigned char data;
		//SPI_Read_Register(1,1,BMP_ID,&data,1);
		//SPI_Transmit_Long_Data_Visualizer((long)passed_variable,5);
    }
}
