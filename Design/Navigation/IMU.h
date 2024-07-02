#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

class IMU {
  private:
    float
      P_k0_k0[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      Q[16] = {0.000001,0,0,0,0,0.000001,0,0,0,0,0.000001,0,0,0,0,0.000001}, 
      EYE_4[16] = {1.0,0,0,0,0,1.0,0,0,0,0,1.0,0,0,0,0,1.0},
      R[9] = {1.0,0,0,0,1.0,0,0,0,1.0},
      gyroBias[3],
      w_rad[3],
      accelGrav[3],
      accelGrav_g[3],
      // Filter constants, 0-1, increase c1 to lower cutoff frequency
      c1AccelGrav = 0.99, // Use for gravity vector from accelerometers 0.998
      c2AccelGrav = 1-c1AccelGrav,
      // Random constants
      b2mg = 1.1975, // Converts bits to milli g's for acceleration, assume 8 g range and 16 bit precision
      d2r = PI/180, // Converts degrees to radians
      g = 9.81;
    uint32_t
      timelast;
    int32_t
      accel[3];
    uint16_t
      calWait;
    int16_t
      angularRates[3],
      accelBias[3],
      accelGravBias[3];
    uint8_t
      onetime_KF,
      gyroRange = 250;
    int8_t
      DataAG[14],// = {-6,-5,-4,-3,-2,-1,0,1,2,3,4,5,6,7}, // Initial values used in test function, if changed must change check in Test
      IMUAddress = 0x68;
    void
      read();
  public:
    IMU();
    void
      Initialize(),
      update(bool *calStatus),
      Kalman_filter_attitude(float Quaternion[4]),
      quaternion_mult(float* Q1,float* Q2, float* Q_out),
      quaternion_rotate(float *Q, float *v_in, float *v_out);
};

#endif