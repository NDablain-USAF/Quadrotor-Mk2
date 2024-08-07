#include "main.h"

int main(){
	Initialize_ATMEGA328P();
	struct BMI088 IMU = {0};
	char BMI_Status = Initialize_BMI088(&IMU);
	while(1){
		if (BMI_Status==9){
			if (g_Acc_Flag==1){
				g_Acc_Flag = 0;
				Read_Acc(&IMU);
			}	
			if (g_Gyr_Flag==1){
				g_Gyr_Flag = 0;
				Read_Gyr(&IMU);
			}
		}
	}

	return 0;
}
