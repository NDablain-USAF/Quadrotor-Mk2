This serves as a repository for the second iteration of the autonmous quadrotor from: https://github.com/NDablain-USAF/Quadrotor

Table of Contents:
1.0    Overview
2.0    Detailed Description of Changes



1.0    Overview
Areas of interest for this project include:
  - Moving from wooden pine frame to a 3D printed frame of mostly ABS
  - Monitoring motor speeds through back EMF rather than encoder counts




2.0 Detailed Description of Changes
  1.1
    - Much broader range of motors to choose from, including lower priced ones
    - Improved control of measurement sample rate. On the ATMega328P the ADC conversion can be selected at a frequency desired by the designer, and implemented with a timer. An ISR indicates the conversion is complete, and triggers a flag for a measurement function. This greatly reduces the number of interrupts per measurement from potentially hundreds with the encoder to one with the ADC.
    - Reduced wiring from five per motor to three.
