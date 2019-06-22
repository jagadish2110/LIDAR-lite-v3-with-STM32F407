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
# Execuation:
	1)  download zip file and extract the file to any folder.
	2)  open the system workbnce ide and click on file tab.
	3)  now click on open project from file system.
	4)  then select the your downloaded file and finish.
	5)  now your project is open in project explorar tab.
	6)  open src folder click on main.c file 
	7)  press ctrl b to build the project.
	
