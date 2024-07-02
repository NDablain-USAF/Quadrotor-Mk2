#include <Wire.h>
#include "IMU.h"

// IMU Setup
float Quaternions[4] = {1,0,0,0};
bool calStatus;
IMU IMU;

void setup() {
  Initialize();
}

void loop() {
  IMU.update(&calStatus);
  
  if (calStatus==true){
    IMU.Kalman_filter_attitude(Quaternions);
    /*
    static uint8_t time;
    float v_in[4] = {0,0,0,1};
    float v_out[4];
    IMU.quaternion_rotate(Quaternions,v_in,v_out);
    if (time>100){
      for (uint8_t i=0;i<4;i++){
        Serial.print(v_out[i]);
        Serial.print(",");
      }
    Serial.println();
    time = 0;
    }
    time++;
    */
  }

}

void Initialize(){
  Serial.begin(115200);
  IMU.Initialize();
}