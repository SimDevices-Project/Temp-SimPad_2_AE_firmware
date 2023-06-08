
#ifndef	__RGB_LED_H__
#define __RGB_LED_H__

#include "./Public/CH554.H"

void ResetLED(void);

void ShowLED();
void SetLED(UINT8 R, UINT8 G, UINT8 B);

#endif