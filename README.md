# LIDAR-lite-v3-with-STM32F407
# AUTHER :
         JAGADISH JADHAV
# REQUIREMENT :-
	     1) System Workbence for STM32 ide
	     2) STM32 CUBE MX SOFTWARE.(optional)
# PIN DISCRIPTION:-
	i uesd the i2c1 interface of stm32 to communicate with lidar & uart 2 to send distace to monitor/screen.
	we can we any other i2c and uart. we just need to change respective handle type def.

 	 CONNECTION:
 			  STM32F407					   LIDAR LITR V3
 			     VCC						VCC
 			     GND						GND
  			I2C1-PB6(SCL)						SCL
  			I2C1-PBA(SDA)						SDA
  			    PA2-TX
  			    PA3-RX
