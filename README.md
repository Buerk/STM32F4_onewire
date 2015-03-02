STM32F4
=======

Template project for STM32F4xx with Debugging in RAM and FLASH support.
The project is set up for EM::BLOCKS (V1.45) with external GNU-ARM Compiler.
However it will also work with the integrated ARM-Compiler.  

This sample / template project will toggle GPIOG, GPIO_Pin_13, using Systick_Handler
and measures temperature using a DS18B20 OneWire sensor connected at Port G on a
STM32F4-Discovery board.
The Sensor is connected as follows:
PG7: power supply for DS18B20
PG5: OneWire bus


To use the external Compiler select Menu Settings->Tools->Global Compiler Settings
Select ARM GCC (generic) at the drop down menu
Select tab Toolchain executables and set the path to the compiler executable.
For example: C:\Program Files (x86)\GNU Tools ARM Embedded\4.8 2013q4\bin

Select tab Search directoroes and check the settings:  
- $(TARGET_COMPILER_DIR)\..\arm-none-eabi\include  
- $(TARGET_COMPILER_DIR)\..\arm-none-eabi  
If they're missing, add them.


Setting DEBUG -> Debug in RAM  
Setting RELEASE -> Debug or run in FLASH

To adapt the template project to your needs, edit

1. the HSE_VALUE with the oscillator frequency in file "stm32f4xx.h" example: 8000000 => 8MHz
2. system_stm32f4xx.c: typically only set PLL_M to the oscillator frequency in MHz (8 => 8MHz) eventually check and set PLL_N, PLL_P and PLL_Q and check SystemCoreClock
3. stm32f4xx_conf.h uncomment the peripheral files if additional peripheral drivers are needed
4. set STACK_SIZE in startup_stm32f4xx.c
