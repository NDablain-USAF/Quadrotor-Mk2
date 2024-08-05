// Example to show how to transmit and receive data as Master using ATMega328P Two Wire Interface (TWI)
// Test sensor is BMP 390 Barometer, internal pullup resistors are utilized on SDA/SCL pins of MCU
// Can be changed to use any sensor, just change the addresses defined at the start as well as the
// Register Data being written and length of the data array to be read
// Created by Nathan Dablain
#include "I2C.h"

int main(){
  Initialize();
  unsigned char Status = Initialize_BMP(); // Group sensor setup in this function
  while(1){
    if (Status==1){ // If initialization was successful
      unsigned char data[21]; 
      unsigned char length = sizeof(data);
      // Again if data is a scaler, add an & before it in the function call below
      unsigned char result = Read(BMTR_ADRS,BMTR_CON_START,data,length); 
      Detain(65000);
      Detain(65000);
    }
  }
  return 0;
}
