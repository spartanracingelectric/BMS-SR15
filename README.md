# SR-15 Battery Management System Repository
<br/> A.k.a the AMS<br/>

## Environment Setup/ Tools Needed
- Install STM32CubeIDE
- Install STM32CubeProgrammer
- Install STM32CubeMX
- ST-LINK/V2

## Project Overview
Note: Lots of the code is prewritten after setting up the project
1. main.c and main.h
2. 6811.c and 6811.h: contains the methods mostly written by analog devices
3. balance.c and balance.h: conatins the cell balancing code
4. can.c and can.h: contains the code to send CAN bus messages
5. module.c and module.h: conatins the code to read voltages (Volt) and to read temperatures (Celsius)
6. print.c and print.h: contains the methods for printing over serial
7. safety.c and safety.h: conatins the code for cell summary, faults, warnings, and state

The #defines are in the files listed below, and the picture show what can be modified

main.h:

![image](https://github.com/spartanracingelectric/BMS-SR15/assets/95559518/2c7bda48-9fde-4fd8-8149-28e496742c8e)

6811.c: 

![image](https://github.com/spartanracingelectric/BMS-SR15/assets/95559518/d18eaa4f-ec60-435e-8436-ded5282d4fb4)

module.c:

![image](https://github.com/spartanracingelectric/BMS-SR15/assets/95559518/ff2355f2-a36c-4e8f-8915-326b2c3bf2f1)

safety.c:

![image](https://github.com/spartanracingelectric/BMS-SR15/assets/95559518/32291510-9fff-4c82-ae84-1914c4afdcc3)

can.c: 

![image](https://github.com/spartanracingelectric/BMS-SR15/assets/95559518/39d6e180-5b10-44e5-a95f-db393b4798fa)


## Resources 
1. https://www.analog.com/media/en/technical-documentation/data-sheets/ltc6811-1-6811-2.pdf
