## STM32 to Odrive connection
The stm32 is using UART 4 to communicate commands to the odrive  
TX: stm32 PA0 <--> Odrive GPIO 2  
RX: stm32 PA1 <--> Odrive GPIO 1  
