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
| IDE | Arduino | Microchip Studio |
| Attitude Measurement Hardware - Accelerometer | MPU 9250 Breakout Board | BMI 088 |
| Attitude Measurement Hardware - Gyroscope | MPU 9250 Breakout Board | BMI 088 |
| Attitude Measurement Hardware - Magnetometer | MPU 9250 Breakout Board | BMM 350 |
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
  Motor Speed Measurement: Removing the requirement of having an integrated shaft encoder opens up a much broader range of motors to choose from, including lower priced ones. Moving from encoder counts triggered by an ISR to sampling the on board ADC allows improved control of measurement sample rate. On the ATMega328P the ADC conversion can be selected at a frequency desired by the designer, and implemented with a timer. An ISR indicates the conversion is complete, and triggers a flag for a measurement function. This greatly reduces the number of interrupts per measurement from potentially hundreds with the encoder to one with the ADC. Additionally the numbers of wires leaving the motor is reduced from five to three. 
  <br /> 
  <br /> 
  Motor Speed Control Hardware: Bipolar Junction Transistors (BJTs) require current through their base to drive current through their collector-emitter junction. This allows for a faster switching time as well as amplification as opposed to just binary saturation but requires additional components as in most cases the current supplied by an MCU IO pin is not enough to fully open the collector-emitter junction. On the other hand, a Metal Oxide Semiconductor Field Effect Transistor (MOSFET) only requires a voltage potential differenc on its gate, no current is required. This change will reduce the number of components required to control the motor speeds, decreasing cost and complexity.
  <br />
  <br />
   Motor Speed Control Software: The initial quadrotor used a lead/lag compensator to control motor speeds. This provided excellent performance, but had a higher computational complexity. First off, there were five terms to tune between the lead compensator gain, and the two poles and zeros. 4 integrals had to be taken and kept track of per motor resulting in a high RAM useage. Switching to a Proportional-Integral controller will reduce the instantaneous computation as well as the number of variables stored in RAM as only one integral will need to be maintained per motor.
   <br />
   <br />
   Electrical Connections: Hand-made connections were the primary method of attaching devices on the initial quadrotor. This allowed for rapid prototyping and the ability to easily swap damaged parts, but also drastically increased production time and quality errors. Most electrical components will be contained on a centralized PCB, several components such as the LoRa module and PA1616S GPS module will be kept on breakout boards and attached via headers to the main PCB. Hand-made connections to the motors will still have to be made with decoupling capacitors directly soldered across the motor terminals.
   <br />
   <br />
   Code: The decisions to switch from C++ to C is mainly based on the desire to learn more about the C language and perhaps reduce some of the technical baggage that can come with using C++. This means the code will take a functional approach rather than object oriented one. This will mean not using the Arduino.h libraries and instead programming primarily off of the MCU data sheet using direct register manipulation. This will allow for direct control over peripherals such as timers,counters,interrupts, and the analog to digital converter. This will likely increase development time but allow for a depper understanding of the working of the MCU.
   <br />
   <br />
![Screenshot (123)](https://github.com/user-attachments/assets/2ee20101-8639-47f4-a36a-29c0646cb2f9)
![Screenshot (124)](https://github.com/user-attachments/assets/0fb40c63-6411-4f93-b63e-e45e33841059)
![Screenshot (125)](https://github.com/user-attachments/assets/1e7cdf7a-865b-4834-a4fc-1415ada93190)
