#include "Main.h"

int main(){
  Serial.begin(115200);
  float PAR_T[3];
  float PAR_P[11];
  Initialize_ATMEGA328P();
  uint8_t Status = Initialize_BMP390(PAR_T, PAR_P); // Group sensor setup in this function
  while(1){
    if (bmtr_flag==1){
      if (Status==2){ // If initialization was successful
        volatile float Height = Read_BMP390(PAR_T, PAR_P);
        Serial.println(Height);
      }
    }

  }

  return 0;
}

float Read_BMP390(float *PAR_T, float *PAR_P){
  static volatile float Height_Bias;
  static volatile uint8_t calWait;
  const float c1 = BMTR_Tb/BMTR_Lb;
  const float c2 = (-BMTR_R*BMTR_Lb)/(BMTR_g*BMTR_M);
  uint8_t data[6];
  uint8_t length = sizeof(data);
  uint8_t result = Read(BMTR_ADRS,BMTR_PRES_DATA,data,length);
  if (result==8){
    // Read Data
    uint32_t Pressure = data[2];
    Pressure = Pressure<<16;
    uint16_t MRFudge = data[1];
    MRFudge = MRFudge<<8;
    MRFudge += data[0];
    Pressure += MRFudge;
    uint32_t Temperature = data[5];
    Temperature = Temperature<<16;
    uint16_t MRSFudge = data[4];
    MRSFudge = MRSFudge<<8;
    MRSFudge += data[3];
    Temperature += MRSFudge;

    // Calibrate Data
    float tpartial_data[2];
    float ppartial_data[4];
    float ppartial_out[2];

    tpartial_data[0] = (float)(Temperature-*PAR_T);
    ++PAR_T; // Move to PAR_T2
    tpartial_data[1] = (float)(tpartial_data[0]**PAR_T);
    ++PAR_T; // Move to PAR_T3
    float Temperature_Comp = tpartial_data[1]+(tpartial_data[0]*tpartial_data[0]**PAR_T);
    float temp1 = Temperature_Comp*Temperature_Comp;
    float temp2 = temp1*Temperature_Comp;
    PAR_P += 5; // Move to PAR_P6
    ppartial_data[0] = *PAR_P*Temperature_Comp;
    ++PAR_P; // Move to PAR_P7
    ppartial_data[1] = *PAR_P*temp1;
    ++PAR_P; // Move to PAR_P8
    ppartial_data[2] = *PAR_P*temp2;
    PAR_P -= 3; // Move to PAR_P5
    ppartial_out[0] = *PAR_P+ppartial_data[0]+ppartial_data[1]+ppartial_data[2];
    PAR_P -= 3; // Move to PAR_P2
    ppartial_data[0] = *PAR_P*Temperature_Comp;
    ++PAR_P; // Move to PAR_P3
    ppartial_data[1] = *PAR_P*temp1;
    ++PAR_P; // Move to PAR_P4
    ppartial_data[2] = *PAR_P*temp2;
    PAR_P -= 3; // Move to PAR_P1
    ppartial_out[1] = (float)Pressure*(*PAR_P+ppartial_data[0]+ppartial_data[1]+ppartial_data[2]);
    ppartial_data[0] = Pressure*Pressure;
    PAR_P += 8; // Move to PAR_P9
    ppartial_data[1] = *PAR_P;
    ++PAR_P; // Move to PAR_P10
    ppartial_data[1] += (*PAR_P*Temperature_Comp);
    ppartial_data[2] = ppartial_data[0]*ppartial_data[1];
    ++PAR_P; // Move to PAR_P11
    ppartial_data[3] = ppartial_data[2] + (ppartial_data[0]*Pressure**PAR_P);
    float Pressure_Comp = ppartial_out[0]+ppartial_out[1]+ppartial_data[3];
    float Height_Compensated = c1*(pow((Pressure_Comp/BMTR_Pb),c2)-1); // Height in m
    if (calWait<50){
      ++calWait;
      Height_Bias = Height_Compensated;
    }
    else if (calWait==50){
      float Height_Out = Height_Compensated-Height_Bias;
      return Height_Out;
    }
    
  }
  
  return 0;
}

void Initialize_ATMEGA328P(){
  sei(); 
  // f_SCL = 16000000/(16+2*(TWBR*Prescaler))
  TWBR = 1;
  TWSR = (1<<TWPS1); // Sets prescaler to 16
  // f_SCL = 400kHz with above settings, limit of ATMega328P with internal pullup resistors
  PCICR |= (1<<2); // Enable PC interrupts on port D
  PCMSK2 |= (1<<3); // Enable D3 for PC interrupts
}

uint8_t Initialize_BMP390(float *PAR_T, float *PAR_P){
  // Throw initial register setting on sensor in this function, results will ==4 if all checks passed
  uint8_t Register_Data0 = (1<<5) | (1<<4) | (1<<1) | (1<<0);
  uint8_t result0 = Transmit(BMTR_ADRS,BMTR_PWR_CTRL,Register_Data0);
  uint8_t Register_Data1 = (1<<4) | (1<<2) | (1<<0);
  uint8_t result1 = Transmit(BMTR_ADRS,BMTR_OSR,Register_Data1);
  uint8_t Register_Data2 = (1<<2);
  uint8_t result2 = Transmit(BMTR_ADRS,BMTR_ODR,Register_Data2);
  uint8_t Register_Data3 = (1<<3);
  uint8_t result3 = Transmit(BMTR_ADRS,BMTR_IIR,Register_Data3);
  uint8_t Register_Data4 = (1<<6);
  uint8_t result4 = Transmit(BMTR_ADRS,BMTR_INT,Register_Data4);
  if ((result0!=4)||(result1!=4)||(result2!=4)||(result3!=4)||(result4!=4)){return 0;}
  // Establish factory constants
  uint8_t data[21];
  uint8_t length = sizeof(data);
  // Read factory calibration data
  // Addresses, sizes, and sign data located on page 28 of BST-BMP390-DS001.pdf
  uint8_t result = Read(BMTR_ADRS,BMTR_CON_START,data,length);
  if (result!=8){return 1;}
  uint16_t NVM_PAR_T1 = data[1]<<8;
  NVM_PAR_T1 += data[0];
  uint16_t NVM_PAR_T2 = data[3]<<8;
  NVM_PAR_T2 += data[2];
  int8_t NVM_PAR_T3 = data[4];
  int16_t NVM_PAR_P1 = data[6]<<8;
  NVM_PAR_P1 += data[5];
  int16_t NVM_PAR_P2 = data[8]<<8;
  NVM_PAR_P2 += data[7];
  int8_t NVM_PAR_P3 = data[9];
  int8_t NVM_PAR_P4 = data[10];
  uint16_t NVM_PAR_P5 = data[12]<<8;
  NVM_PAR_P5 += data[11];
  uint16_t NVM_PAR_P6 = data[14]<<8;
  NVM_PAR_P6 += data[13];
  int8_t NVM_PAR_P7 = data[15];
  int8_t NVM_PAR_P8 = data[16];
  int16_t NVM_PAR_P9 = data[18]<<8;
  NVM_PAR_P9 += data[17];
  int8_t NVM_PAR_P10 = data[19];
  int8_t NVM_PAR_P11 = data[20];

  *PAR_T = NVM_PAR_T1/BMTR_cT1; // PAR_T1
  ++PAR_T;
  *PAR_T = NVM_PAR_T2/BMTR_cT2; // PAR_T2
  ++PAR_T;
  *PAR_T = NVM_PAR_T3/BMTR_cT3; // PAR_T3

  *PAR_P = (NVM_PAR_P1-BMTR_cP0);
  *PAR_P /= BMTR_cP1; // PAR_P1
  ++PAR_P;
  *PAR_P = (NVM_PAR_P2-BMTR_cP0);
  *PAR_P /= BMTR_cP2; // PAR_P2
  ++PAR_P;
  *PAR_P = NVM_PAR_P3/BMTR_cP3; // PAR_P3
  ++PAR_P;
  *PAR_P = NVM_PAR_P4/BMTR_cP4; // PAR_P4
  ++PAR_P;
  *PAR_P = NVM_PAR_P5/BMTR_cP5; // PAR_P5
  ++PAR_P;
  *PAR_P = NVM_PAR_P6/BMTR_cP6; // PAR_P6
  ++PAR_P;
  *PAR_P = NVM_PAR_P7/BMTR_cP7; // PAR_P7
  ++PAR_P;
  *PAR_P = NVM_PAR_P8/BMTR_cP8; // PAR_P8
  ++PAR_P;
  *PAR_P = NVM_PAR_P9/BMTR_cP9; // PAR_P9
  ++PAR_P;
  *PAR_P = NVM_PAR_P10/BMTR_cP10; // PAR_P10
  ++PAR_P;
  *PAR_P = NVM_PAR_P11/BMTR_cP11; // PAR_P11
  return 2;
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

uint8_t Read(uint8_t Slave_Address, uint8_t Register_Address, uint8_t *data, uint8_t length){
  // start write write start write/read read ack read ack... stop
  uint8_t j = 0;
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // TWINT starts operation of TWI, TWSTA sends START condition, TWEN enables TWI
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set indicating START condition set
  if (((TWSR & 0xF8) != TW_REP_START)&&((TWSR & 0xF8) != TW_START)){return 0;} // Mask prescaler bits in TWSR, check if not equal to start condition to trigger error
  TWDR = Slave_Address<<1; // Write slave address to data register as write
  TWCR = (1<<TWINT) | (1<<TWEN); // Start transmission
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
  if ((TWSR & 0xF8) != TW_MT_SLA_ACK){return 1;} // Mask prescaler bits in TWSR, check if slave acknowledge
  TWDR = Register_Address; // Send data (address device looks at)
  TWCR = (1<<TWINT) | (1<<TWEN); // Start transmission
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
  if ((TWSR & 0xF8) != TW_MT_DATA_ACK){return 2;} // Mask prescaler bits in TWSR, check if data sent
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // TWINT starts operation of TWI, TWSTA sends START condition, TWEN enables TWI
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set indicating START condition set
  if ((TWSR & 0xF8) != TW_REP_START){return 3;} // Mask prescaler bits in TWSR, check if not equal to start condition to trigger error

  TWDR = (Slave_Address<<1) | (1<<0); // Write slave address to data register as read
  TWCR = (1<<TWINT) | (1<<TWEN); // Start transmission
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
  if ((TWSR & 0xF8) != TW_MR_SLA_ACK){return 4;} // Mask prescaler bits in TWSR, check if slave acknowledge

  uint8_t temp = TWDR; // Throw away first value
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA); // Data byte received and ACK transmitted back to slave
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
  if ((TWSR & 0xF8) != TW_MR_DATA_ACK){return 5;} // Mask prescaler bits in TWSR, send acknowledge to slave  

  while(j<length){
    *data = TWDR; // Read data sent from slave
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA); // Data byte received and ACK transmitted back to slave
    while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
    if ((TWSR & 0xF8) != TW_MR_DATA_ACK){return 6;} // Mask prescaler bits in TWSR, send acknowledge to slave   
    ++data; // Increment pointer
    ++j;
  }
 
  *data = TWDR; // Read last byte of data sent from slave
  TWCR = (1<<TWINT) | (1<<TWEN) | (0<<TWEA); // Data byte received and NACK transmitted back to slave
  while(!(TWCR & (1<<TWINT))){asm("");} // Wait for TWINT flag to be set
  if ((TWSR & 0xF8) != TW_MR_DATA_NACK){return 7;} // Mask prescaler bits in TWSR, send not acknowledge to slave 
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO); // Send STOP

  return 8; // Success
}

ISR (PCINT2_vect){
  bmtr_flag ^= 1;
}
