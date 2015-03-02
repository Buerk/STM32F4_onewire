/*
* Copyright (c) 2014, Richard Buerk
* All rights reserved.

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the <organization> nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL JENS NIELSEN BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


/* Power for OneWire Sensor provided at PG7
   One-Wire commmunication interface is connected to PG5
   Used Board: STM32F4-discovery (STM32F429)
   Just a very simple sample program: temperature is measured once and saved in variable "temperature"


   Please note: code for other controllers than STM32F4xx not yet tested!
*/

#include "stm32f4xx.h"
#include "driver/onewire.h"
#include "driver/ds18b20.h"

ONE_WIRE ow;

static void init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

	// Clock Enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

	// Config PG13 als Digital-Ausgang
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
}


void SysTick_Handler(void)
{
	static int i = 0;
	if (i > 999)
	{
		i = 0;
		GPIO_ToggleBits(GPIOG, GPIO_Pin_13);
	}
	else
		i++;
}

void OneWirePower(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOG, &GPIO_InitStructure);
    GPIO_SetBits(GPIOG, GPIO_Pin_7);
}

uint8_t crc_cal;
float temperature;
uint8_t data[8];


int main(void)
{
    SystemInit();
    SystemCoreClockUpdate();
    init();
    SysTick_Config(SystemCoreClock / 1000);     // 1kHz Systick

    OneWirePower();
    OneWireInit(&ow, GPIOG, GPIO_Pin_5);
    crc_cal = CalcCRC8(ow.ROM_NO, 7); // CRC is calculated but not yet compared


    OneWireReset(&ow);
    //OneWireSelect(&ow, ow.ROM_NO);    // Select specific device with given ID/address on the bus
    OneWireSkipRom(&ow);    // Select all devices on bus
    DS18B20_StartTemperatureConversion(&ow);

    //OneWireReset(&ow);
    //OneWireSkipRom(&ow);
    //DS18B20ScratchpadRead(&ow, data);

    OneWireReset(&ow);
    OneWireSkipRom(&ow);    // Select all devices on bus - only to be used if just one device is connected!
    temperature = DS18B20TempRead(&ow);

    while(1)
    {

    }
}
