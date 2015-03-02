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

#ifndef ONEWIRE_H
#define ONEWIRE_H


#define BOOL int
#define FALSE 0
#define TRUE  1


// You can exclude CRC checks altogether by defining this to 0
#ifndef ONEWIRE_CRC
#define ONEWIRE_CRC 1
#endif



#define ONEWIRE_CMD_SEARCH      0xF0
#define ONEWIRE_CMD_SELECT      0x55
#define ONEWIRE_CMD_SKIP        0xCC


typedef struct
{
    GPIO_TypeDef         *pPort;                // used port
    uint16_t              bPin;                 // pin (binary coded ex. 1000b => pin 3)

    __IO uint32_t        *pModeReg;             // Port mode register (input, output,...)
    uint32_t              modeRegMask;          // Mask for mode register to set input, output,...
    uint32_t              modeInputMask;        // mask to set pin as input (floating)  to be written to the port mode register
    uint32_t              modeOutputMaske;      // mask to set pin as open drain output to be written to the port mode register
} ONE_WIRE_PIN;

typedef struct
{
    ONE_WIRE_PIN pin;

    // global search state
    unsigned char ROM_NO[8];
    uint8_t LastDiscrepancy;
    uint8_t LastFamilyDiscrepancy;
    uint8_t LastDeviceFlag;
} ONE_WIRE;

void OneWireInit(ONE_WIRE *owire, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint8_t OneWireReset(ONE_WIRE *owire);

void OneWireResetSearch(ONE_WIRE *owire);
uint8_t OneWireSearch(ONE_WIRE *owire, uint8_t *newAddr);

void OneWireWrite(ONE_WIRE *owire, uint8_t val);
void OneWireWritePowerOn(ONE_WIRE *owire, uint8_t val);
uint8_t OneWireRead(ONE_WIRE *owire);
void OneWireReadBytes(ONE_WIRE *owire, uint8_t *buf, uint16_t count);
void OneWireWriteBytes(ONE_WIRE *owire, const uint8_t *buf, uint16_t count);
void OneWireSelect(ONE_WIRE *owire, uint8_t const *rom);
void OneWireSkipRom(ONE_WIRE *owire);
void Delay_us(uint16_t us); // uses Timer 3

#ifdef ONEWIRE_CRC
uint8_t CalcCRC8(uint8_t *addr, uint8_t len);

#endif

#endif

