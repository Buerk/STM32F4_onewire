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

#include "stm32f4xx.h"
#include "OneWire.h"
#include "DS18B20.h"



//*****************************************************************************
// Start temperature conversion
void DS18B20_StartTemperatureConversion(ONE_WIRE *owire)
{
    int i;
    OneWireWritePowerOn(owire, DS18B20_CONVERT);
    for (i = 0; i < 20;i++)
        Delay_us(1000000/20);
}


//*****************************************************************************
// Read Scratchpad of Sensor
// Read data is saved in pData, buffer has to be 8 bytes large
void DS18B20ScratchpadRead(ONE_WIRE *owire, uint8_t *pData)
{
    OneWireWrite(owire, DS18B20_READ_SCRATCHPAD);
    OneWireReadBytes(owire, pData, 8);
}

//*****************************************************************************
// Read Temperature from sensor
// Temperature is returned in degree Celcius
float DS18B20TempRead(ONE_WIRE *owire)
{
    uint16_t ulTemp = 0;
    float temp;
    OneWireWrite(owire, DS18B20_READ_SCRATCHPAD);
    ulTemp = OneWireRead(owire);
    ulTemp |= (OneWireRead(owire) << 8);
    if (ulTemp > 2097)
    {
        ulTemp = 65536 - ulTemp;
        temp = -(((ulTemp & 0x7F0) >> 4) * 1.0 + (ulTemp & 0xf)*0.0625);
    }
    else
    {
        temp = ((ulTemp & 0x7F0) >> 4) * 1.0 + (ulTemp & 0xf)*0.0625;
    }
    return(temp);
}

