/*
 * FileLed.c
 *
 *  Created on: 26.06.2018
 *      Author: Thomas Scherzer
 */

#include <FireIndicator.h>
#include "spi.h"
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern bool SpiInitDone;

//Codierung G8 : R 8 : B8
ui8 TransmitDataBuffer[BUFFERSIZE] = {0};
ui64 ColorLookupTable[COLORS] = {0};


//Functions

void GenerateLookupTable()
{
	static bool LookupTableGenerated = 0;

	if(!LookupTableGenerated)
	{
		const ui64 LOW_BIT = 0x80;//0xC0;
		const ui64 HIGH_BIT = 0xFC;//0xFC;

		for(ui32 i = 0; i < COLORS; i++)
		{
			register ui64 ColorValue = 0;
			ColorValue |= (i & 0x80 ? HIGH_BIT : LOW_BIT)<<0;//56;
			ColorValue |= (i & 0x40 ? HIGH_BIT : LOW_BIT)<<8;//48;
			ColorValue |= (i & 0x20 ? HIGH_BIT : LOW_BIT)<<16;//40;
			ColorValue |= (i & 0x10 ? HIGH_BIT : LOW_BIT)<<24;//32;
			ColorValue |= (i & 0x08 ? HIGH_BIT : LOW_BIT)<<32;//24;
			ColorValue |= (i & 0x04 ? HIGH_BIT : LOW_BIT)<<40;//16;
			ColorValue |= (i & 0x02 ? HIGH_BIT : LOW_BIT)<<48;//8;
			ColorValue |= (i & 0x01 ? HIGH_BIT : LOW_BIT)<<56;//0;

			ColorLookupTable[i] = ColorValue;
		}
		LookupTableGenerated = 1;
	}
}

void SetLedColor(ui32 LedNumber, ui32 LedColor)
{
	if(LedNumber < LEDs)
	{
		register ui64* Storage = (ui64*)&TransmitDataBuffer;
		Storage[LedNumber*3 + 1] = ColorLookupTable[(LedColor>>0) & 0xFF]; //red
		Storage[LedNumber*3 + 0] = ColorLookupTable[(LedColor>>8) & 0xFF]; //green
		Storage[LedNumber*3 + 2] = ColorLookupTable[(LedColor>>16) & 0xFF];//blue
	}
}


ui32 GetColor(ui32 ColorNumber)
{
	switch (ColorNumber)
	{
	case 0:  return White	; break;
	case 1:  return Silver	; break;
	case 2:  return Gray	; break;
	case 3:  return Black	; break;
	case 4:  return Red		; break;
	case 5:  return Maroon	; break;
	case 6:  return Yellow	; break;
	case 7:  return Olive	; break;
	case 8:  return Lime	; break;
	case 9:  return Green	; break;
	case 10: return Aqua	; break;
	case 11: return Teal	; break;
	case 12: return Blue	; break;
	case 13: return Navy	; break;
	case 14: return Fuchsia	; break;
	case 15: return Purple	; break;
	default : return White;
	}
}

inline void SetLedColorRGB(ui32 LedNumber, RGB color)
{
	SetLedColor(LedNumber, *((ui32*)&color));
}

void InitializeColors()
{
	GenerateLookupTable();
	for(ui32 i = 0; i < LEDs; i++)
	{
		SetLedColor(i,0xFFFFFF);
	}
}

void UpdateLeds(ui32 dT)
{
	const ui32 RefreshInterval = 20*ms;
	static ui32 Timer = 0;

	Timer = (Timer + dT) % RefreshInterval;
	if((Timer < dT) && SpiInitDone)
	{
	  HAL_SPI_Transmit_DMA(&hspi2,TransmitDataBuffer,sizeof(TransmitDataBuffer));
	}
}
