#include "main.h"
// check number of dummy reads, for BMM350 2 are called out, perhaps only one for BMP which leads to "temp" variable? 
int main(){
	struct BMI088 IMU = {0};
	struct BMM350 MAG = {0};
	struct Motor Motor1 = {0,0,0,ref,MOTOR1_MUX};
	struct Motor Motor2 = {0,0,0,ref,MOTOR2_MUX};
	struct Motor Motor3 = {0,0,0,ref,MOTOR3_MUX};
	struct Motor Motor4 = {0,0,0,ref,MOTOR4_MUX};
	ATMega328P_Int();
	//char BMI_Status = BMI088_Int(&IMU);
	//char BMM_Status = BMM350_Int();
	while(1){
		while (g_Start_Flag==1){
			unsigned char data[3];
			unsigned char status = I2C_Read(MCU2_ADRS,1,data,2);
			unsigned int passed_variable = data[2]*256 + data[1];
			Motor1.r = passed_variable;
			//Run_BMI(BMI_Status, &IMU);
			//Run_BMM(BMM_Status, &MAG);
			Run_Motors(&Motor1,&Motor2,&Motor3,&Motor4);
		}
	}

	return 0;
}
