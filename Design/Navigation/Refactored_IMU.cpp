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
  // Linearize state (F) and measurement (H) matrices
  float c1 = (dt*w_rad[0])/2;
  float c2 = (dt*w_rad[1])/2;
  float c3 = (dt*w_rad[2])/2;
  float c4 = 2*g*Quaternion[0];
  float c5 = 2*g*Quaternion[1];
  float c6 = 2*g*Quaternion[2];
  float c7 = 2*g*Quaternion[3];
  // Predict, using nonlinear equations for measurement and state transition
  float x_hat_k1_k00 = Quaternion[0] + dt*((Quaternion[1]*w_rad[0])/2 + (Quaternion[2]*w_rad[1])/2 + (Quaternion[3]*w_rad[2])/2);
  float x_hat_k1_k01 = Quaternion[1] - dt*((Quaternion[0]*w_rad[0])/2 + (Quaternion[3]*w_rad[1])/2 - (Quaternion[2]*w_rad[2])/2);
  float x_hat_k1_k02 = Quaternion[2] - dt*((Quaternion[0]*w_rad[1])/2 - (Quaternion[3]*w_rad[0])/2 + (Quaternion[1]*w_rad[2])/2);
  float x_hat_k1_k03 = Quaternion[3] - dt*((Quaternion[2]*w_rad[0])/2 - (Quaternion[1]*w_rad[1])/2 + (Quaternion[0]*w_rad[2])/2);
  
  float y_hat0 = 2*g*(x_hat_k1_k00*x_hat_k1_k02 + x_hat_k1_k01*x_hat_k1_k03);
  float y_hat1 = 2*g*(x_hat_k1_k02*x_hat_k1_k03 - x_hat_k1_k00*x_hat_k1_k01);
  float y_hat2 = g*(pow(x_hat_k1_k00,2)-pow(x_hat_k1_k01,2)-pow(x_hat_k1_k02,2)+pow(x_hat_k1_k03,2));
  // Update
  // P_k1_k0 = F*P_k0_k0*F' + Q;
  float P_k1_k00 = P_k0_k04*c1*c1 + 2*P_k0_k05*c1*c2 + 2*P_k0_k06*c1*c3 + 2*P_k0_k01*c1 + P_k0_k07*c2*c2 + 2*P_k0_k08*c2*c3 + 2*P_k0_k02*c2 + P_k0_k09*c3*c3 + 2*P_k0_k03*c3 + P_k0_k00 + Q;
  float P_k1_k01 = P_k0_k01 + P_k0_k04*c1 + P_k0_k05*c2 + P_k0_k06*c3 - c1*(P_k0_k00 + P_k0_k01*c1 + P_k0_k02*c2 + P_k0_k03*c3) + c3*(P_k0_k02 + P_k0_k05*c1 + P_k0_k07*c2 + P_k0_k08*c3) - c2*(P_k0_k03 + P_k0_k06*c1 + P_k0_k08*c2 + P_k0_k09*c3);
  float P_k1_k02 = P_k0_k02 + P_k0_k05*c1 + P_k0_k07*c2 + P_k0_k08*c3 - c2*(P_k0_k00 + P_k0_k01*c1 + P_k0_k02*c2 + P_k0_k03*c3) - c3*(P_k0_k01 + P_k0_k04*c1 + P_k0_k05*c2 + P_k0_k06*c3) + c1*(P_k0_k03 + P_k0_k06*c1 + P_k0_k08*c2 + P_k0_k09*c3);
  float P_k1_k03 = P_k0_k03 + P_k0_k06*c1 + P_k0_k08*c2 + P_k0_k09*c3 - c3*(P_k0_k00 + P_k0_k01*c1 + P_k0_k02*c2 + P_k0_k03*c3) + c2*(P_k0_k01 + P_k0_k04*c1 + P_k0_k05*c2 + P_k0_k06*c3) - c1*(P_k0_k02 + P_k0_k05*c1 + P_k0_k07*c2 + P_k0_k08*c3);
  float P_k1_k04 = P_k0_k00*c1*c1 + 2*P_k0_k03*c1*c2 - 2*P_k0_k02*c1*c3 - 2*P_k0_k01*c1 + P_k0_k09*c2*c2 - 2*P_k0_k08*c2*c3 - 2*P_k0_k06*c2 + P_k0_k07*c3*c3 + 2*P_k0_k05*c3 + P_k0_k04 + Q;
  float P_k1_k05 = P_k0_k05 - P_k0_k02*c1 + P_k0_k07*c3 - P_k0_k08*c2 - c2*(P_k0_k01 - P_k0_k00*c1 + P_k0_k02*c3 - P_k0_k03*c2) - c3*(P_k0_k04 - P_k0_k01*c1 + P_k0_k05*c3 - P_k0_k06*c2) + c1*(P_k0_k06 - P_k0_k03*c1 + P_k0_k08*c3 - P_k0_k09*c2);
  float P_k1_k06 = P_k0_k06 - P_k0_k03*c1 + P_k0_k08*c3 - P_k0_k09*c2 - c3*(P_k0_k01 - P_k0_k00*c1 + P_k0_k02*c3 - P_k0_k03*c2) + c2*(P_k0_k04 - P_k0_k01*c1 + P_k0_k05*c3 - P_k0_k06*c2) - c1*(P_k0_k05 - P_k0_k02*c1 + P_k0_k07*c3 - P_k0_k08*c2);
  float P_k1_k07 = P_k0_k09*c1*c1 - 2*P_k0_k03*c1*c2 - 2*P_k0_k06*c1*c3 + 2*P_k0_k08*c1 + P_k0_k00*c2*c2 + 2*P_k0_k01*c2*c3 - 2*P_k0_k02*c2 + P_k0_k04*c3*c3 - 2*P_k0_k05*c3 + P_k0_k07 + Q;
  float P_k1_k08 = P_k0_k08 - P_k0_k03*c2 - P_k0_k06*c3 + P_k0_k09*c1 - c3*(P_k0_k02 - P_k0_k00*c2 - P_k0_k01*c3 + P_k0_k03*c1) + c2*(P_k0_k05 - P_k0_k01*c2 - P_k0_k04*c3 + P_k0_k06*c1) - c1*(P_k0_k07 - P_k0_k02*c2 - P_k0_k05*c3 + P_k0_k08*c1);
  float P_k1_k09 = P_k0_k07*c1*c1 - 2*P_k0_k05*c1*c2 + 2*P_k0_k02*c1*c3 - 2*P_k0_k08*c1 + P_k0_k04*c2*c2 - 2*P_k0_k01*c2*c3 + 2*P_k0_k06*c2 + P_k0_k00*c3*c3 - 2*P_k0_k03*c3 + P_k0_k09 + Q;
  // ybar = y-yhat;
  float y_bar0 = accelGrav_g[0]-y_hat0;
  float y_bar1 = accelGrav_g[1]-y_hat1;
  float y_bar2 = accelGrav_g[2]-y_hat2;
  // S = H*P_k1_k0*H' + R;
  float S0 = P_k1_k07*c4*c4 + 2*P_k1_k08*c4*c5 + 2*P_k1_k02*c4*c6 + 2*P_k1_k05*c4*c7 + P_k1_k09*c5*c5 + 2*P_k1_k03*c5*c6 + 2*P_k1_k06*c5*c7 + P_k1_k00*c6*c6 + 2*P_k1_k01*c6*c7 + P_k1_k04*c7*c7 + R;
  float S1 = -c5*(P_k1_k00*c6 + P_k1_k02*c4 + P_k1_k01*c7 + P_k1_k03*c5) - c4*(P_k1_k01*c6 + P_k1_k05*c4 + P_k1_k04*c7 + P_k1_k06*c5) + c7*(P_k1_k02*c6 + P_k1_k07*c4 + P_k1_k05*c7 + P_k1_k08*c5) + c6*(P_k1_k03*c6 + P_k1_k08*c4 + P_k1_k06*c7 + P_k1_k09*c5);
  float S2 = c4*(P_k1_k00*c6 + P_k1_k02*c4 + P_k1_k01*c7 + P_k1_k03*c5) - c5*(P_k1_k01*c6 + P_k1_k05*c4 + P_k1_k04*c7 + P_k1_k06*c5) - c6*(P_k1_k02*c6 + P_k1_k07*c4 + P_k1_k05*c7 + P_k1_k08*c5) + c7*(P_k1_k03*c6 + P_k1_k08*c4 + P_k1_k06*c7 + P_k1_k09*c5);
  float S3 = - c6*(P_k1_k00*c5 + P_k1_k01*c4 - P_k1_k02*c7 - P_k1_k03*c6) - c7*(P_k1_k01*c5 + P_k1_k04*c4 - P_k1_k05*c7 - P_k1_k06*c6) - c4*(P_k1_k02*c5 + P_k1_k05*c4 - P_k1_k07*c7 - P_k1_k08*c6) - c5*(P_k1_k03*c5 + P_k1_k06*c4 - P_k1_k08*c7 - P_k1_k09*c6);
  float S4 = P_k1_k04*c4*c4 + 2*P_k1_k01*c4*c5 - 2*P_k1_k06*c4*c6 - 2*P_k1_k05*c4*c7 + P_k1_k00*c5*c5 - 2*P_k1_k03*c5*c6 - 2*P_k1_k02*c5*c7 + P_k1_k09*c6*c6 + 2*P_k1_k08*c6*c7 + P_k1_k07*c7*c7 + R;
  float S5 = c5*(P_k1_k01*c5 + P_k1_k04*c4 - P_k1_k05*c7 - P_k1_k06*c6) - c4*(P_k1_k00*c5 + P_k1_k01*c4 - P_k1_k02*c7 - P_k1_k03*c6) + c6*(P_k1_k02*c5 + P_k1_k05*c4 - P_k1_k07*c7 - P_k1_k08*c6) - c7*(P_k1_k03*c5 + P_k1_k06*c4 - P_k1_k08*c7 - P_k1_k09*c6);
  float S6 = c6*(P_k1_k00*c4 - P_k1_k01*c5 - P_k1_k02*c6 + P_k1_k03*c7) + c7*(P_k1_k01*c4 - P_k1_k04*c5 - P_k1_k05*c6 + P_k1_k06*c7) + c4*(P_k1_k02*c4 - P_k1_k05*c5 - P_k1_k07*c6 + P_k1_k08*c7) + c5*(P_k1_k03*c4 - P_k1_k06*c5 - P_k1_k08*c6 + P_k1_k09*c7);
  float S7 = c7*(P_k1_k02*c4 - P_k1_k05*c5 - P_k1_k07*c6 + P_k1_k08*c7) - c4*(P_k1_k01*c4 - P_k1_k04*c5 - P_k1_k05*c6 + P_k1_k06*c7) - c5*(P_k1_k00*c4 - P_k1_k01*c5 - P_k1_k02*c6 + P_k1_k03*c7) + c6*(P_k1_k03*c4 - P_k1_k06*c5 - P_k1_k08*c6 + P_k1_k09*c7);
  float S8 = P_k1_k00*c4*c4 - 2*P_k1_k01*c4*c5 - 2*P_k1_k02*c4*c6 + 2*P_k1_k03*c4*c7 + P_k1_k04*c5*c5 + 2*P_k1_k05*c5*c6 - 2*P_k1_k06*c5*c7 + P_k1_k07*c6*c6 - 2*P_k1_k08*c6*c7 + P_k1_k09*c7*c7 + R;
  // L = P_k1_k0*H'*inv(S);
  float L0 = (P_k1_k00*S3*S7*c4 - P_k1_k00*S4*S6*c4 + P_k1_k00*S3*S8*c5 - P_k1_k00*S5*S6*c5 - P_k1_k01*S3*S7*c5 + P_k1_k01*S3*S8*c4 + P_k1_k01*S4*S6*c5 - P_k1_k01*S5*S6*c4 + P_k1_k00*S4*S8*c6 - P_k1_k00*S5*S7*c6 - P_k1_k02*S3*S7*c6 + P_k1_k02*S4*S6*c6 + P_k1_k02*S4*S8*c4 - P_k1_k02*S5*S7*c4 + P_k1_k01*S4*S8*c7 - P_k1_k01*S5*S7*c7 - P_k1_k02*S3*S8*c7 + P_k1_k02*S5*S6*c7 + P_k1_k03*S3*S7*c7 - P_k1_k03*S3*S8*c6 - P_k1_k03*S4*S6*c7 + P_k1_k03*S4*S8*c5 + P_k1_k03*S5*S6*c6 - P_k1_k03*S5*S7*c5)/(S0*S4*S8 - S0*S5*S7 - S1*S3*S8 + S1*S5*S6 + S2*S3*S7 - S2*S4*S6);
  float L1 = -(P_k1_k00*S0*S7*c4 - P_k1_k00*S1*S6*c4 + P_k1_k00*S0*S8*c5 - P_k1_k00*S2*S6*c5 - P_k1_k01*S0*S7*c5 + P_k1_k01*S0*S8*c4 + P_k1_k01*S1*S6*c5 - P_k1_k01*S2*S6*c4 + P_k1_k00*S1*S8*c6 - P_k1_k00*S2*S7*c6 - P_k1_k02*S0*S7*c6 + P_k1_k02*S1*S6*c6 + P_k1_k02*S1*S8*c4 - P_k1_k02*S2*S7*c4 + P_k1_k01*S1*S8*c7 - P_k1_k01*S2*S7*c7 - P_k1_k02*S0*S8*c7 + P_k1_k02*S2*S6*c7 + P_k1_k03*S0*S7*c7 - P_k1_k03*S0*S8*c6 - P_k1_k03*S1*S6*c7 + P_k1_k03*S1*S8*c5 + P_k1_k03*S2*S6*c6 - P_k1_k03*S2*S7*c5)/(S0*S4*S8 - S0*S5*S7 - S1*S3*S8 + S1*S5*S6 + S2*S3*S7 - S2*S4*S6);
  float L2 = (P_k1_k00*S0*S4*c4 - P_k1_k00*S1*S3*c4 + P_k1_k00*S0*S5*c5 - P_k1_k00*S2*S3*c5 - P_k1_k01*S0*S4*c5 + P_k1_k01*S0*S5*c4 + P_k1_k01*S1*S3*c5 - P_k1_k01*S2*S3*c4 + P_k1_k00*S1*S5*c6 - P_k1_k00*S2*S4*c6 - P_k1_k02*S0*S4*c6 + P_k1_k02*S1*S3*c6 + P_k1_k02*S1*S5*c4 - P_k1_k02*S2*S4*c4 + P_k1_k01*S1*S5*c7 - P_k1_k01*S2*S4*c7 - P_k1_k02*S0*S5*c7 + P_k1_k02*S2*S3*c7 + P_k1_k03*S0*S4*c7 - P_k1_k03*S0*S5*c6 - P_k1_k03*S1*S3*c7 + P_k1_k03*S1*S5*c5 + P_k1_k03*S2*S3*c6 - P_k1_k03*S2*S4*c5)/(S0*S4*S8 - S0*S5*S7 - S1*S3*S8 + S1*S5*S6 + S2*S3*S7 - S2*S4*S6);
  float L3 = (P_k1_k01*S3*S7*c4 - P_k1_k01*S4*S6*c4 + P_k1_k01*S3*S8*c5 - P_k1_k01*S5*S6*c5 + P_k1_k01*S4*S8*c6 - P_k1_k01*S5*S7*c6 - P_k1_k04*S3*S7*c5 + P_k1_k04*S3*S8*c4 + P_k1_k04*S4*S6*c5 - P_k1_k04*S5*S6*c4 - P_k1_k05*S3*S7*c6 + P_k1_k05*S4*S6*c6 + P_k1_k05*S4*S8*c4 - P_k1_k05*S5*S7*c4 + P_k1_k04*S4*S8*c7 - P_k1_k04*S5*S7*c7 - P_k1_k05*S3*S8*c7 + P_k1_k05*S5*S6*c7 + P_k1_k06*S3*S7*c7 - P_k1_k06*S3*S8*c6 - P_k1_k06*S4*S6*c7 + P_k1_k06*S4*S8*c5 + P_k1_k06*S5*S6*c6 - P_k1_k06*S5*S7*c5)/(S0*S4*S8 - S0*S5*S7 - S1*S3*S8 + S1*S5*S6 + S2*S3*S7 - S2*S4*S6);
  float L4 = -(P_k1_k01*S0*S7*c4 - P_k1_k01*S1*S6*c4 + P_k1_k01*S0*S8*c5 - P_k1_k01*S2*S6*c5 + P_k1_k01*S1*S8*c6 - P_k1_k01*S2*S7*c6 - P_k1_k04*S0*S7*c5 + P_k1_k04*S0*S8*c4 + P_k1_k04*S1*S6*c5 - P_k1_k04*S2*S6*c4 - P_k1_k05*S0*S7*c6 + P_k1_k05*S1*S6*c6 + P_k1_k05*S1*S8*c4 - P_k1_k05*S2*S7*c4 + P_k1_k04*S1*S8*c7 - P_k1_k04*S2*S7*c7 - P_k1_k05*S0*S8*c7 + P_k1_k05*S2*S6*c7 + P_k1_k06*S0*S7*c7 - P_k1_k06*S0*S8*c6 - P_k1_k06*S1*S6*c7 + P_k1_k06*S1*S8*c5 + P_k1_k06*S2*S6*c6 - P_k1_k06*S2*S7*c5)/(S0*S4*S8 - S0*S5*S7 - S1*S3*S8 + S1*S5*S6 + S2*S3*S7 - S2*S4*S6);
  float L5 = (P_k1_k01*S0*S4*c4 - P_k1_k01*S1*S3*c4 + P_k1_k01*S0*S5*c5 - P_k1_k01*S2*S3*c5 + P_k1_k01*S1*S5*c6 - P_k1_k01*S2*S4*c6 - P_k1_k04*S0*S4*c5 + P_k1_k04*S0*S5*c4 + P_k1_k04*S1*S3*c5 - P_k1_k04*S2*S3*c4 - P_k1_k05*S0*S4*c6 + P_k1_k05*S1*S3*c6 + P_k1_k05*S1*S5*c4 - P_k1_k05*S2*S4*c4 + P_k1_k04*S1*S5*c7 - P_k1_k04*S2*S4*c7 - P_k1_k05*S0*S5*c7 + P_k1_k05*S2*S3*c7 + P_k1_k06*S0*S4*c7 - P_k1_k06*S0*S5*c6 - P_k1_k06*S1*S3*c7 + P_k1_k06*S1*S5*c5 + P_k1_k06*S2*S3*c6 - P_k1_k06*S2*S4*c5)/(S0*S4*S8 - S0*S5*S7 - S1*S3*S8 + S1*S5*S6 + S2*S3*S7 - S2*S4*S6);
  float L6 = (P_k1_k02*S3*S7*c4 - P_k1_k02*S4*S6*c4 + P_k1_k02*S3*S8*c5 - P_k1_k02*S5*S6*c5 + P_k1_k02*S4*S8*c6 - P_k1_k02*S5*S7*c6 - P_k1_k05*S3*S7*c5 + P_k1_k05*S3*S8*c4 + P_k1_k05*S4*S6*c5 - P_k1_k05*S5*S6*c4 - P_k1_k07*S3*S7*c6 + P_k1_k07*S4*S6*c6 + P_k1_k07*S4*S8*c4 - P_k1_k07*S5*S7*c4 + P_k1_k05*S4*S8*c7 - P_k1_k05*S5*S7*c7 - P_k1_k07*S3*S8*c7 + P_k1_k07*S5*S6*c7 + P_k1_k08*S3*S7*c7 - P_k1_k08*S3*S8*c6 - P_k1_k08*S4*S6*c7 + P_k1_k08*S4*S8*c5 + P_k1_k08*S5*S6*c6 - P_k1_k08*S5*S7*c5)/(S0*S4*S8 - S0*S5*S7 - S1*S3*S8 + S1*S5*S6 + S2*S3*S7 - S2*S4*S6);
  float L7 = -(P_k1_k02*S0*S7*c4 - P_k1_k02*S1*S6*c4 + P_k1_k02*S0*S8*c5 - P_k1_k02*S2*S6*c5 + P_k1_k02*S1*S8*c6 - P_k1_k02*S2*S7*c6 - P_k1_k05*S0*S7*c5 + P_k1_k05*S0*S8*c4 + P_k1_k05*S1*S6*c5 - P_k1_k05*S2*S6*c4 - P_k1_k07*S0*S7*c6 + P_k1_k07*S1*S6*c6 + P_k1_k07*S1*S8*c4 - P_k1_k07*S2*S7*c4 + P_k1_k05*S1*S8*c7 - P_k1_k05*S2*S7*c7 - P_k1_k07*S0*S8*c7 + P_k1_k07*S2*S6*c7 + P_k1_k08*S0*S7*c7 - P_k1_k08*S0*S8*c6 - P_k1_k08*S1*S6*c7 + P_k1_k08*S1*S8*c5 + P_k1_k08*S2*S6*c6 - P_k1_k08*S2*S7*c5)/(S0*S4*S8 - S0*S5*S7 - S1*S3*S8 + S1*S5*S6 + S2*S3*S7 - S2*S4*S6);
  float L8 = (P_k1_k02*S0*S4*c4 - P_k1_k02*S1*S3*c4 + P_k1_k02*S0*S5*c5 - P_k1_k02*S2*S3*c5 + P_k1_k02*S1*S5*c6 - P_k1_k02*S2*S4*c6 - P_k1_k05*S0*S4*c5 + P_k1_k05*S0*S5*c4 + P_k1_k05*S1*S3*c5 - P_k1_k05*S2*S3*c4 - P_k1_k07*S0*S4*c6 + P_k1_k07*S1*S3*c6 + P_k1_k07*S1*S5*c4 - P_k1_k07*S2*S4*c4 + P_k1_k05*S1*S5*c7 - P_k1_k05*S2*S4*c7 - P_k1_k07*S0*S5*c7 + P_k1_k07*S2*S3*c7 + P_k1_k08*S0*S4*c7 - P_k1_k08*S0*S5*c6 - P_k1_k08*S1*S3*c7 + P_k1_k08*S1*S5*c5 + P_k1_k08*S2*S3*c6 - P_k1_k08*S2*S4*c5)/(S0*S4*S8 - S0*S5*S7 - S1*S3*S8 + S1*S5*S6 + S2*S3*S7 - S2*S4*S6);
  float L9 = (P_k1_k03*S3*S7*c4 - P_k1_k03*S4*S6*c4 + P_k1_k03*S3*S8*c5 - P_k1_k03*S5*S6*c5 + P_k1_k03*S4*S8*c6 - P_k1_k03*S5*S7*c6 - P_k1_k06*S3*S7*c5 + P_k1_k06*S3*S8*c4 + P_k1_k06*S4*S6*c5 - P_k1_k06*S5*S6*c4 - P_k1_k08*S3*S7*c6 + P_k1_k08*S4*S6*c6 + P_k1_k08*S4*S8*c4 - P_k1_k08*S5*S7*c4 + P_k1_k06*S4*S8*c7 - P_k1_k06*S5*S7*c7 - P_k1_k08*S3*S8*c7 + P_k1_k08*S5*S6*c7 + P_k1_k09*S3*S7*c7 - P_k1_k09*S3*S8*c6 - P_k1_k09*S4*S6*c7 + P_k1_k09*S4*S8*c5 + P_k1_k09*S5*S6*c6 - P_k1_k09*S5*S7*c5)/(S0*S4*S8 - S0*S5*S7 - S1*S3*S8 + S1*S5*S6 + S2*S3*S7 - S2*S4*S6);
  float L10 = -(P_k1_k03*S0*S7*c4 - P_k1_k03*S1*S6*c4 + P_k1_k03*S0*S8*c5 - P_k1_k03*S2*S6*c5 + P_k1_k03*S1*S8*c6 - P_k1_k03*S2*S7*c6 - P_k1_k06*S0*S7*c5 + P_k1_k06*S0*S8*c4 + P_k1_k06*S1*S6*c5 - P_k1_k06*S2*S6*c4 - P_k1_k08*S0*S7*c6 + P_k1_k08*S1*S6*c6 + P_k1_k08*S1*S8*c4 - P_k1_k08*S2*S7*c4 + P_k1_k06*S1*S8*c7 - P_k1_k06*S2*S7*c7 - P_k1_k08*S0*S8*c7 + P_k1_k08*S2*S6*c7 + P_k1_k09*S0*S7*c7 - P_k1_k09*S0*S8*c6 - P_k1_k09*S1*S6*c7 + P_k1_k09*S1*S8*c5 + P_k1_k09*S2*S6*c6 - P_k1_k09*S2*S7*c5)/(S0*S4*S8 - S0*S5*S7 - S1*S3*S8 + S1*S5*S6 + S2*S3*S7 - S2*S4*S6);
  float L11 = (P_k1_k03*S0*S4*c4 - P_k1_k03*S1*S3*c4 + P_k1_k03*S0*S5*c5 - P_k1_k03*S2*S3*c5 + P_k1_k03*S1*S5*c6 - P_k1_k03*S2*S4*c6 - P_k1_k06*S0*S4*c5 + P_k1_k06*S0*S5*c4 + P_k1_k06*S1*S3*c5 - P_k1_k06*S2*S3*c4 - P_k1_k08*S0*S4*c6 + P_k1_k08*S1*S3*c6 + P_k1_k08*S1*S5*c4 - P_k1_k08*S2*S4*c4 + P_k1_k06*S1*S5*c7 - P_k1_k06*S2*S4*c7 - P_k1_k08*S0*S5*c7 + P_k1_k08*S2*S3*c7 + P_k1_k09*S0*S4*c7 - P_k1_k09*S0*S5*c6 - P_k1_k09*S1*S3*c7 + P_k1_k09*S1*S5*c5 + P_k1_k09*S2*S3*c6 - P_k1_k09*S2*S4*c5)/(S0*S4*S8 - S0*S5*S7 - S1*S3*S8 + S1*S5*S6 + S2*S3*S7 - S2*S4*S6);
  // x_hat_k1_k1 = x_hat_k1_k0 + L*y_bar + dt.*((1-dot(x_hat_k1_k0,x_hat_k1_k0))*x_hat_k1_k0);
  float temp = x_hat_k1_k00*x_hat_k1_k00 + x_hat_k1_k01*x_hat_k1_k01 + x_hat_k1_k02*x_hat_k1_k02 + x_hat_k1_k03*x_hat_k1_k03 - 1;
  float x_hat_k1_k10 = x_hat_k1_k00 + L0*y_bar0 + L1*y_bar1 + L2*y_bar2 - dt*x_hat_k1_k00*temp;
  float x_hat_k1_k11 = x_hat_k1_k01 + L3*y_bar0 + L4*y_bar1 + L5*y_bar2 - dt*x_hat_k1_k01*temp;
  float x_hat_k1_k12 = x_hat_k1_k02 + L6*y_bar0 + L7*y_bar1 + L8*y_bar2 - dt*x_hat_k1_k02*temp;
  float x_hat_k1_k13 = x_hat_k1_k03 + L9*y_bar0 + L10*y_bar1 + L11*y_bar2 - dt*x_hat_k1_k03*temp;
  // P_k1_k1 = (eye(4)-L*H)*P_k1_k0;
  P_k0_k00 = P_k1_k01*(L1*c4 - L0*c7 + L2*c5) - P_k1_k00*(L0*c6 - L1*c5 + L2*c4 - 1) - P_k1_k02*(L0*c4 + L1*c7 - L2*c6) - P_k1_k03*(L0*c5 + L1*c6 + L2*c7);
  P_k0_k01 = P_k1_k01*(L1*c4 - L0*c7 + L2*c5) - P_k1_k00*(L0*c6 - L1*c5 + L2*c4 - 1) - P_k1_k02*(L0*c4 + L1*c7 - L2*c6) - P_k1_k03*(L0*c5 + L1*c6 + L2*c7);
  P_k0_k02 = P_k1_k05*(L1*c4 - L0*c7 + L2*c5) - P_k1_k02*(L0*c6 - L1*c5 + L2*c4 - 1) - P_k1_k07*(L0*c4 + L1*c7 - L2*c6) - P_k1_k08*(L0*c5 + L1*c6 + L2*c7);
  P_k0_k03 = P_k1_k06*(L1*c4 - L0*c7 + L2*c5) - P_k1_k03*(L0*c6 - L1*c5 + L2*c4 - 1) - P_k1_k08*(L0*c4 + L1*c7 - L2*c6) - P_k1_k09*(L0*c5 + L1*c6 + L2*c7);
  P_k0_k04 = P_k1_k04*(L4*c4 - L3*c7 + L5*c5 + 1) - P_k1_k01*(L3*c6 - L4*c5 + L5*c4) - P_k1_k05*(L3*c4 + L4*c7 - L5*c6) - P_k1_k06*(L3*c5 + L4*c6 + L5*c7);
  P_k0_k05 = P_k1_k05*(L4*c4 - L3*c7 + L5*c5 + 1) - P_k1_k02*(L3*c6 - L4*c5 + L5*c4) - P_k1_k07*(L3*c4 + L4*c7 - L5*c6) - P_k1_k08*(L3*c5 + L4*c6 + L5*c7);
  P_k0_k06 = P_k1_k06*(L4*c4 - L3*c7 + L5*c5 + 1) - P_k1_k03*(L3*c6 - L4*c5 + L5*c4) - P_k1_k08*(L3*c4 + L4*c7 - L5*c6) - P_k1_k09*(L3*c5 + L4*c6 + L5*c7);
  P_k0_k07 = P_k1_k05*(L7*c4 - L6*c7 + L8*c5) - P_k1_k02*(L6*c6 - L7*c5 + L8*c4) - P_k1_k07*(L6*c4 + L7*c7 - L8*c6 - 1) - P_k1_k08*(L6*c5 + L7*c6 + L8*c7);
  P_k0_k08 = P_k1_k06*(L7*c4 - L6*c7 + L8*c5) - P_k1_k03*(L6*c6 - L7*c5 + L8*c4) - P_k1_k08*(L6*c4 + L7*c7 - L8*c6 - 1) - P_k1_k09*(L6*c5 + L7*c6 + L8*c7);
  P_k0_k09 = P_k1_k06*(L10*c4 - L9*c7 + L11*c5) - P_k1_k03*(L9*c6 - L10*c5 + L11*c4) - P_k1_k08*(L9*c4 + L10*c7 - L11*c6) - P_k1_k09*(L9*c5 + L10*c6 + L11*c7 - 1);
  static uint8_t counter;
  uint8_t length = 3;
  float output[length] = {y_bar0,y_bar1,y_bar2};
  if (++counter>0){
    for (uint8_t i=0;i<length;i++) {
        Serial.print(output[i])
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
