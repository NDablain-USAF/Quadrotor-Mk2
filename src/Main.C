#include "Main.h"

int main(){
  float PAR_T[3];
  float PAR_P[11];
  Initialize_ATMEGA328P();
  unsigned char Status = Initialize_BMP390(PAR_T, PAR_P); // Group sensor setup in this function
  while(1){
    if (bmtr_flag==1){
      if (Status==2){ // If initialization was successful
        volatile float Height = Read_BMP390(PAR_T, PAR_P);
      }
    }

  }

  return 0;
}
