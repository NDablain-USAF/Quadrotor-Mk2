#include "Main.h"

int main(){
  Serial.begin(115200);
  uint32_t PAR_T[3];
  Initialize_ATMEGA328P();
  uint8_t Status = Initialize_BMP390(PAR_T); // Group sensor setup in this function
  Serial.println(PAR_T[0]);
  Serial.println(PAR_T[1]);
  Serial.println(PAR_T[2]);
  while(1){
    if (Status==1){ // If initialization was successful
      uint16_t Temperature = Read_BMP390();
    }
    Detain(65000);
  }

  return 0;
}

uint16_t Read_BMP390(uint32_t PAR_T[3]){
    uint8_t data[3];
    uint8_t result = Read(BMTR_ADRS,BMTR_TEMP_DATA,data);
    if (result==1){
        uint32_t Temperature = data[2];
        Temperature = Temperature<<16;
        uint16_t MRSFudge = data[1];
        MRSFudge = MRSFudge<<8;
        MRSFudge += data[0];
        Temperature += MRSFudge;
        /*
        int32_t t_partial0 = Temperature-PAR_T[0];
        int32_t t_partial1 = t_partial0*PAR_T[2];
        uint32_t t_partial2 = t_partial0*t_partial0*PAR_T[2];
        t_partial2/=1000000000; //1e9
        int32_t TC = t_partial1+t_partial2;
        TC/=10000000; //1e7
        uint16_t Temperature_Compensated = TC;
        */
    }
    
    return Temperature_Compensated;
}

void Initialize_ATMEGA328P(){
  sei(); 
  // f_SCL = 16000000/(16+2*(TWBR*Prescaler))
  TWBR = 1;
  TWSR = (1<<TWPS1); // Sets prescaler to 16
  // f_SCL = 400kHz with above settings, limit of ATMega328P with internal pullup resistors
}

uint8_t Initialize_BMP390(uint32_t PAR_T[3]){
  // Throw initial register setting on sensor in this function, results will ==4 if all checks passed
  uint8_t Register_Data0 = (1<<5) | (1<<4) | (1<<1) | (1<<0);
  uint8_t result0 = Transmit(BMTR_ADRS,BMTR_PWR_CTRL,Register_Data0);
  uint8_t Register_Data1 = (1<<4) | (1<<2) | (1<<0);
  uint8_t result1 = Transmit(BMTR_ADRS,BMTR_OSR,Register_Data1);
  uint8_t Register_Data2 = (1<<2);
  uint8_t result2 = Transmit(BMTR_ADRS,BMTR_ODR,Register_Data2);
  if ((result0!=4)||(result1!=4)||(result2!=4)){return 1;}
  // Establish factory constants
  uint8_t data[21];
  // Read factory calibration data
  // Addresses, sizes, and sign data located on page 28 of BST-BMP390-DS001.pdf
  uint8_t result = Read(BMTR_ADRS,BMTR_CON_START,data);
  if (result!=1){return 1;}
  uint16_t NVM_PAR_T1 = data[1]<<8;
  NVM_PAR_T1 += data[0];
  uint16_t NVM_PAR_T2 = data[3]<<8;
  NVM_PAR_T2 += data[2];
  int8_t NVM_PAR_T3 = data[0];
  int16_t NVM_PAR_P1 = data[2]<<8;
  NVM_PAR_P1 += data[1];
  int16_t NVM_PAR_P2 = data[4]<<8;
  NVM_PAR_P2 += data[3];
  int8_t NVM_PAR_P3 = data[5];
  int8_t NVM_PAR_P4 = data[6];
  uint16_t NVM_PAR_P5 = data[1]<<8;
  NVM_PAR_P5 += data[0];
  uint16_t NVM_PAR_P6 = data[3]<<8;
  NVM_PAR_P6 += data[2];
  int8_t NVM_PAR_P7 = data[0];
  int8_t NVM_PAR_P8 = data[1];
  int16_t NVM_PAR_P9 = data[3]<<8;
  NVM_PAR_P9 += data[2];
  int8_t NVM_PAR_P10 = data[4];
  int8_t NVM_PAR_P11 = data[5];
  Serial.println(NVM_PAR_T1);
  Serial.println(NVM_PAR_T2);
  Serial.println(NVM_PAR_T3);
  // Figure out wtf to do with these floats
  PAR_T[0] = NVM_PART_T1*10000; // Scale to divide by whole number
  PAR_T[0]/=39; 
  PAR_T[1] = NVM_PAR_T2/107; // Check value of NVM_PAR_T2, this is scaled by 1e7
  PAR_T[2] = NVM_PAR_T3*100000; // Scaled by 1e16
  PAR_T[2]/=2815;
  return 0;
}

void Detain(uint16_t length){
      for (uint16_t i=0;i<length;i++){asm("");}
}

uint8_t Transmit (uint8_t Slave_Address, uint8_t Register_Address, uint8_t Register_Data){
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // TWINT starts operation of TWI, TWSTA sends START condition, TWEN enables TWI
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set indicating START condition set
  if (((TWSR & 0xF8) != TW_REP_START)&&((TWSR & 0xF8) != TW_START)){return 0;} // Mask prescaler bits in TWSR, check if not equal to start condition to trigger error
  TWDR = Slave_Address<<1; // Write slave address to data register as write
  TWCR = (1<<TWINT) | (1<<TWEN); // Start transmission
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
  if ((TWSR & 0xF8) != TW_MT_SLA_ACK){return 1;} // Mask prescaler bits in TWSR, check if slave acknowledge
  TWDR = Register_Address; // Send data (address to write to)
  TWCR = (1<<TWINT) | (1<<TWEN); // Start transmission
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
  if ((TWSR & 0xF8) != TW_MT_DATA_ACK){return 2;} // Mask prescaler bits in TWSR, check if data sent
  TWDR = Register_Data; // Send data (new value of address)
  TWCR = (1<<TWINT) | (1<<TWEN); // Start transmission
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
  if ((TWSR & 0xF8) != TW_MT_DATA_ACK){return 3;} // Mask prescaler bits in TWSR, check if data sent
  TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN); // Send Stop
  return 4; // Success
}

uint8_t Read(uint8_t Slave_Address, uint8_t Register_Address, uint8_t data[]){
  volatile uint8_t i = 0;
  uint8_t length = sizeof(data)-1;
  while(i<1){  
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // TWINT starts operation of TWI, TWSTA sends START condition, TWEN enables TWI
    while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set indicating START condition set
    if (((TWSR & 0xF8) != TW_REP_START)&&((TWSR & 0xF8) != TW_START)){return 0;} // Mask prescaler bits in TWSR, check if not equal to start condition to trigger error
    TWDR = Slave_Address<<1; // Write slave address to data register as write
    TWCR = (1<<TWINT) | (1<<TWEN); // Start transmission
    while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
    if ((TWSR & 0xF8) != TW_MT_SLA_ACK){return 0;} // Mask prescaler bits in TWSR, check if slave acknowledge
    TWDR = Register_Address; // Send data (address device looks at)
    TWCR = (1<<TWINT) | (1<<TWEN); // Start transmission
    while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
    if ((TWSR & 0xF8) != TW_MT_DATA_ACK){return 0;} // Mask prescaler bits in TWSR, check if data sent
    TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN); // Send Stop
    i++;
  }
  if (i>0){

    volatile uint8_t j = 0;
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // TWINT starts operation of TWI, TWSTA sends START condition, TWEN enables TWI
    while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set indicating START condition set
    if (((TWSR & 0xF8) != TW_REP_START)&&((TWSR & 0xF8) != TW_START)){return 0;} // Mask prescaler bits in TWSR, check if not equal to start condition to trigger error
    TWDR = (Slave_Address<<1) | (1<<0); // Write slave address to data register as read
    TWCR = (1<<TWINT) | (1<<TWEN); // Start transmission
    while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
    if ((TWSR & 0xF8) != TW_MR_SLA_ACK){return 0;} // Mask prescaler bits in TWSR, check if slave acknowledge

    while(j<length){
      TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA); // Data byte received and ACK transmitted back to slave
      while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
      if ((TWSR & 0xF8) != TW_MR_DATA_ACK){return 0;} // Mask prescaler bits in TWSR, send acknowledge to slave   
      data[j] = TWDR; // Read data sent from slave
      ++j;
    }

    TWCR = (1<<TWINT) | (1<<TWEN) | (0<<TWEA); // Data byte received and NACK transmitted back to slave
    while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
    if ((TWSR & 0xF8) != TW_MR_DATA_NACK){return 0;} // Mask prescaler bits in TWSR, send not acknowledge to slave   
    data[j] = TWDR; // Read data sent from slave
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO); // Send STOP
  }
  return 1;
}
