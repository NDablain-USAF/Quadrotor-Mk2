This serves as a repository for the second iteration of the autonmous quadrotor from: https://github.com/NDablain-USAF/Quadrotor
<br /> 
<br /> 
Currently in test and development
<br /> 
<br /> 
Table of Contents:<br /> 
1.0    Table of Changes<br /> 
2.0    Detailed Description of Changes         
<br /> 
<br /> 
<br /> 
1.0 Table of Changes
<br /> 
<br /> 
| Component | Was | Is |
|----------:|----------------|--------------|
| Frame | Pine Wood | 3D Printed ABS |
| Motor Speed Measurement | Motor Shaft Encoders | Back EMF & ADC |
| Motor Speed Control Hardware | NPN BJT | N-Channel MOSFET |
| Motor Speed Control Software | Lead/Lag Compensator| Proportional/Integral Controller |
| Electrical Connections | Wires,Solder,Perf-Board | PCB |
| Code | C++ | C |
| MCU | Arduino Uno R3 Board | ATMega328P |
| IDE | Arduino | Platform IO |
| Attitude Measurement Hardware - Accelerometer | MPU 9250 Breakout Board | BMI 088 |
| Attitude Measurement Hardware - Gyroscope | MPU 9250 Breakout Board | BMI 088 |
| Attitude Measurement Hardware - Magnetometer | MPU 9250 Breakout Board | None |
| Attitude Measurement Software | Float operations using Math.h trig. functions | Integers with polynomial trig function approximations |
| Attitude Representation | Euler Angles | Quaternions |
<br /> 
<br /> 
<br /> 
2.0 Detailed Description of Changes
<br /> 
<br /> 
  Frame: Pine wood was initially selected due to its low density, perceived ease of manipulation, and low cost. However, it was found in the Mk1 that the weight of the wood drove higher motor speed and propeller thrust requirements and it was difficult to produce multiple parts to similar dimensions by hand. The higher upfront cost of purchasing a 3D printer will allow for faster part production and replacement rates in the long term along with greater reliability in part tolerances.
  <br /> 
  <br /> 
  Motor Speed Measurement: Removing the requirement of having an integrated shaft encoder opens up a much broader range of motors to choose from, including lower priced ones. Moving from encoder counts triggered by an ISR to sampling the on board ADC allows improved control of measurement sample rate. On the ATMega328P the ADC conversion can be selected at a frequency desired by the designer, and implemented with a timer. An ISR indicates the conversion is complete, and triggers a flag for a measurement function. This greatly reduces the number of interrupts per measurement from potentially hundreds with the encoder to one with the ADC. Additionally the wiring leaving the motor is reduced from five to three. 
  <br /> 
  <br /> 
