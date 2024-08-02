#include "IMU.h"
#include "matrix_math.h"
#include <Wire.h>

IMU::IMU(){
}

void IMU::update(bool *calStatus){
  read();
  int16_t x = DataAG[0]<<8;
  x += DataAG[1];
  int32_t X = x*b2mg;
  accel[0] = X-accelBias[0];
  accelGrav[0] = accelGrav[0]*c1AccelGrav + (X-accelGravBias[0])*c2AccelGrav;
  int16_t y = DataAG[2]<<8;
  y += DataAG[3];
  int32_t Y = y*b2mg;
  accel[1] = Y-accelBias[1];
  accelGrav[1] = accelGrav[1]*c1AccelGrav + (Y-accelGravBias[1])*c2AccelGrav;
  int16_t z = DataAG[4]<<8;
  z += DataAG[5];
  int32_t Z = z*b2mg;
  accel[2] = Z-accelBias[2];
  accelGrav[2] = accelGrav[2]*c1AccelGrav + (Z-accelGravBias[2])*c2AccelGrav;
  float q = DataAG[8]<<8;
  q += (DataAG[9]/131);
  q = (q/gyroRange)-gyroBias[1];
  float p = DataAG[10]<<8;
  p += (DataAG[11]/131);
  p = (p/gyroRange)-gyroBias[0];
  float r = DataAG[12]<<8;
  r += (DataAG[13]/131);
  r = (r/gyroRange)-gyroBias[2];
  angularRates[0] = p;
  angularRates[1] = q;
  angularRates[2] = r;

  if (calWait==1000){
    for (uint8_t i=0;i<3;i++){
      gyroBias[i] = angularRates[i];
      accelBias[i] = accel[i];
      if (i<2){
        accelGravBias[i] = accelGrav[i];
      }
      else {
        accelGravBias[i] = accelGrav[i]-9810;
      }
    }
    ++calWait;
    *calStatus = true;
  }
  else if (calWait<1000){
    ++calWait;
    *calStatus = false;
  }
}

void IMU::read(){
  uint8_t i = 0;
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x3B); // Accel x Higher Bit Address
  Wire.endTransmission();
  Wire.requestFrom(IMUAddress,14);
  while(Wire.available()){
    DataAG[i++] = Wire.read();
  }
}

void IMU::Initialize(){
  uint8_t data[3];
  Wire.begin();
  delay(10);
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x37);
  Wire.write(0x02);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x1C); // Accel config
  Wire.write(0b00001000); // Set range to +- 4g
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x1B); // Gyro config
  Wire.write(0b00000000); // Set range to 250 dps
  Wire.endTransmission();
  delay(10);
}

void IMU::Kalman_filter_attitude(float Quaternion[4]){
  // Here the system states are the four components of a unit quaternion, with the first state being the vector magnitude
  // We measure the gravity vector that is obtained by normalizing accelerometer outputs and then running their outputs through a low pass filter (IIR)
  // Convert deg/s to rad/s
  for (uint8_t i=0;i<3;i++){
    w_rad[i] = angularRates[i]*d2r; 
    accelGrav_g[i] = accelGrav[i]*0.001;
  }

  // Update time step
  float dt = (micros()-timelast)/1e6;
  if (dt>0.02){
    dt = 0.01;
  }
  timelast = micros();
  // Random for testing
  /*
  dt = 0.01;
  w_rad[0] = 1;
  w_rad[1] = 2;
  w_rad[2] = 3;
  accelGrav_g[0] = 0;
  accelGrav_g[1] = 0;
  accelGrav_g[2] = 9.81;
  Quaternion[0] = 4;
  Quaternion[1] = 5;
  Quaternion[2] = 6;
  Quaternion[3] = 7;
  */
  // Linearize state (F) and measurement (H) matrices
  float c1 = (dt*w_rad[0])/2;
  float c2 = (dt*w_rad[1])/2;
  float c3 = (dt*w_rad[2])/2;
  float c4 = 2*g*Quaternion[0];
  float c5 = 2*g*Quaternion[1];
  float c6 = 2*g*Quaternion[2];
  float c7 = 2*g*Quaternion[3];
  float F[16] = {1,c1,c2,c3,-c1,1,c3,-c2,-c2,-c3,1,c1,-c3,c2,-c1,1}; // 4x4
  float H[12] = {c6,c7,c4,c5,-c5,-c4,c7,c6,c4,-c5,-c6,c7}; // 3x4
  // Predict, using nonlinear equations for measurement and state transition
  float x_hat_k1_k0[4] = {0,0,0,0};
  x_hat_k1_k0[0] = Quaternion[0] + dt*((Quaternion[1]*w_rad[0])/2 + (Quaternion[2]*w_rad[1])/2 + (Quaternion[3]*w_rad[2])/2);
  x_hat_k1_k0[1] = Quaternion[1] - dt*((Quaternion[0]*w_rad[0])/2 + (Quaternion[3]*w_rad[1])/2 - (Quaternion[2]*w_rad[2])/2);
  x_hat_k1_k0[2] = Quaternion[2] - dt*((Quaternion[0]*w_rad[1])/2 - (Quaternion[3]*w_rad[0])/2 + (Quaternion[1]*w_rad[2])/2);
  x_hat_k1_k0[3] = Quaternion[3] - dt*((Quaternion[2]*w_rad[0])/2 - (Quaternion[1]*w_rad[1])/2 + (Quaternion[0]*w_rad[2])/2);
  float y_hat[3] = {0,0,0};
  y_hat[0] = 2*g*(x_hat_k1_k0[0]*x_hat_k1_k0[2] + x_hat_k1_k0[1]*x_hat_k1_k0[3]);
  y_hat[1] = 2*g*(x_hat_k1_k0[2]*x_hat_k1_k0[3] - x_hat_k1_k0[0]*x_hat_k1_k0[1]);
  y_hat[2] = g*(pow(x_hat_k1_k0[0],2)-pow(x_hat_k1_k0[1],2)-pow(x_hat_k1_k0[2],2)+pow(x_hat_k1_k0[3],2));
  // Update
  // P_k1_k0 = F*P_k0_k0*F' + Q;
  float P_k1_k0[16]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 
  float temp1[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  float temp2[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  float temp3[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  Matrix_mult(F,P_k0_k0,temp1,4,4,4); // F*P_k0_k0
  Matrix_trans(F,temp2,4,4);
  Matrix_mult(temp1,temp2,temp3,4,4,4); // F*P_k0_k0*F'
  Matrix_add(temp3,Q,P_k1_k0,4,4);

  // ybar = y-yhat;
  float y_bar[3] = {0,0,0};
  Matrix_sub(accelGrav_g,y_hat,y_bar,3,1);

  // S = H*P_k1_k0*H' + R;
  float S[9]= {0,0,0,0,0,0,0,0,0};
  float temp4[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
  float temp5[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
  float temp6[9] = {0,0,0,0,0,0,0,0,0};
  Matrix_trans(H,temp5,3,4); // H'
  Matrix_mult(H,P_k1_k0,temp4,3,4,4); // H*P_k1_k0
  Matrix_mult(temp4,temp5,temp6,3,4,3); // H*P_k1_k0*H'
  Matrix_add(temp6,R,S,3,3); // H*P_k1_k0*H' + R

  // L = P_k1_k0*H'*inv(S);
  float L[12] = {0,0,0,0,0,0,0,0,0,0,0,0}; // 4x3
  float temp7[9] = {0,0,0,0,0,0,0,0,0};
  Matrix_inv3(S,temp7);
  Matrix_mult(temp4,temp7,L,4,3,3);

  // xhat_k1_k1 = xhat_k1_k0 + L*ybar + dt.*((1-dot(xhat_k1_k0,xhat_k1_k0))*xhat_k1_k0);
  float temp8 = 0;
  Vector_dot(x_hat_k1_k0,x_hat_k1_k0,&temp8,4); // dot(xhat_k1_k0,xhat_k1_k0)
  float temp9 = dt*(1-temp8); // dt.*((1-dot(xhat_k1_k0,xhat_k1_k0))
  float temp10[4] = {0,0,0,0};
  Vector_con(x_hat_k1_k0,&temp9,temp10,4); // dt.*((1-dot(xhat_k1_k0,xhat_k1_k0))*xhat_k1_k0)
  float temp11[4] = {0,0,0,0}; 
  Matrix_mult(L,y_bar,temp11,4,3,1); // L*ybar
  float temp12[4] = {0,0,0,0};
  Matrix_add(temp11,temp10,temp12,4,1); // L*ybar + dt.*((1-dot(xhat_k1_k0,xhat_k1_k0))*xhat_k1_k0)
  float x_hat_k1_k1[4] = {0,0,0,0};
  Matrix_add(x_hat_k1_k0,temp12,x_hat_k1_k1,4,1);

  // P_k1_k1 = (eye(4)-L*H)*P_k1_k0;
  float temp13[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  Matrix_mult(L,H,temp13,4,3,4); // L*H
  float temp14[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  Matrix_sub(EYE_4,temp13,temp14,4,4); // (eye(4)-L*H)
  float P_k1_k1[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  Matrix_mult(temp14,P_k1_k0,P_k0_k0,4,4,4);
  
  static uint8_t counter;
  if (++counter>0){
    for (uint8_t i=0;i<9;i++){
      Serial.print(temp7[i],3);
      Serial.print(",");
    }
    Serial.println();
    counter = 0;
  }
}

void IMU::quaternion_mult(float *Q1, float *Q2, float *Q_out){
  float ptr[4][4];

  for (uint8_t i=0;i<4;i++){
    for (uint8_t j=0;j<4;j++){
      ptr[i][j] = *Q1**Q2;
      ++Q2;
    }
    Q2 -= 4;
    ++Q1;
  }
  
  *Q_out = ptr[0][0] - ptr[1][1] - ptr[2][2] - ptr[3][3];
  ++Q_out;
  *Q_out = ptr[0][1] + ptr[1][0] + ptr[2][3] - ptr[3][2];
  ++Q_out;
  *Q_out = ptr[0][2] - ptr[1][3] + ptr[2][0] + ptr[3][1];
  ++Q_out;
  *Q_out = ptr[0][3] + ptr[1][2] - ptr[2][1] + ptr[3][0];
}

void IMU::quaternion_rotate(float *Q, float *v_in, float *v_out){
  float temp[4];
  quaternion_mult(Q,v_in,temp);
  float Q_conj[4] = {*Q, -1.0**(++Q), -1.0**(++Q), -1.0**(++Q)};
  quaternion_mult(temp,Q_conj,v_out);
}