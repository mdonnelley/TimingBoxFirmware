WCH_Timing_Box
==============

Code for WCH timing box using Arduino Uno.

The timing box has 8 BNC outputs that are mapped to the Arduino pins as:

OUT1 PIN4
OUT2 PIN5
OUT3 PIN6
OUT4 PIN7
PWM1 PIN9
PWM2 PIN10
IN1  PIN2
IN2  PIN3
LED  PIN13

This allows use of hardware PWM on pins 9 and 10 and hardware interrupts on pins 2 and 3. Pin 13 is connected to an LED on the front of the box.