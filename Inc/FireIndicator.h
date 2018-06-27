/*
 * FireLed.h
 *
 *  Created on: 26.06.2018
 *      Author: Thomas Scherzer
 */

#ifndef FIREINDICATOR_H_
#define FIREINDICATOR_H_

#include "System.h"
#include "main.h"

#define LEDs							(60)
#define BUFFERSIZE						(3*8*(LEDs)+40)
#define COLORS							(256)

//0xC0         //0xFC
//28% low      //72% high
//2bit         //6bit
#define full 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC
#define zero 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0

typedef struct
{
	ui8 red;
	ui8 green;
	ui8 blue;
}RGB;

typedef enum
{
	White	= 0xFFFFFF,
	Silver	= 0xC0C0C0,
	Gray	= 0x808080,
	Black	= 0x000000,
	Red		= 0x0000FF,
	Maroon	= 0x000080,
	Yellow	= 0x00FFFF,
	Olive	= 0x008080,
	Lime	= 0x00FF00,
	Green	= 0x008000,
	Aqua	= 0xFFFF00,
	Teal	= 0x808000,
	Blue	= 0xFF0000,
	Navy	= 0x800000,
	Fuchsia	= 0xFF00FF,
	Purple	= 0x800080
}Color;


void UpdateLeds(ui32 dT);
void InitializeColors();
void SetLedColor(ui32 LedNumber, ui32 LedColor);
void SetLedColorRGB(ui32 LedNumber, RGB color);
ui32 GetColor(ui32 ColorNumber);


#endif /* FIREINDICATOR_H_ */
