#include "./Public/CH554.H"
#include <intrins.h>
#include "RGB_LIB.H"

#define LED_COUNT 4		//LED的数量

// @24Mhz
#define Delay0_3us {_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();}
#define Delay0_5us {_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();}

// #define TEST_VERSION 0
#ifdef TEST_VERSION
sbit SK = P1^1;				//信号输出引脚
#else
sbit SK = P1^2;				//信号输出引脚
#endif


unsigned char data tmp[LED_COUNT * 3] _at_ 0x20;
unsigned char data * data pointer;
void ResetLED()
{
	  pointer=tmp;
    SK = 0;
}

void SetLED(UINT8 R, UINT8 G, UINT8 B)
{
	*(pointer++)=G;
	*(pointer++)=R;
	*(pointer++)=B;
  
	*(pointer++)=G;
	*(pointer++)=R;
	*(pointer++)=B;
}


void ShowLED()
{
  unsigned char count=0;
	// EA=0; // 禁用中断
	pointer=tmp;
	do
	{
		SK = 1;
		_nop_();
		_nop_();
		SK = (*pointer & (1 << 7));
		Delay0_3us;
		SK = 0;
		Delay0_5us;
		
		SK = 1;
		_nop_();		_nop_();		_nop_();
		_nop_();
		SK = (*pointer & (1 << 6));
		Delay0_3us;
		SK = 0;
		Delay0_5us;
		
		SK = 1;
		_nop_();		_nop_();		_nop_();
		_nop_();		_nop_();
		SK = (*pointer & (1 << 5));
		Delay0_3us;
		SK = 0;
		Delay0_5us;
		
		SK = 1;
		_nop_();		_nop_();		_nop_();
		_nop_();		_nop_();
		SK = (*pointer & (1 << 4));
		Delay0_3us;
		SK = 0;
		Delay0_5us;
		
		SK = 1;
		_nop_();		_nop_();		_nop_();
		_nop_();		_nop_();
		SK = (*pointer & (1 << 3));
		Delay0_3us;
		SK = 0;
		Delay0_5us;
		
		SK = 1;
		_nop_();		_nop_();		_nop_();
		_nop_();		_nop_();
		SK = (*pointer & (1 << 2));
		Delay0_3us;
		SK = 0;
		Delay0_5us;
		
		SK = 1;
		_nop_();		_nop_();		_nop_();
		_nop_();		_nop_();
		SK = (*pointer & (1 << 1));
		Delay0_3us;
		SK = 0;
		Delay0_5us;
		
		SK = 1;
		_nop_();		_nop_();		_nop_();
		_nop_();		_nop_();		_nop_();
		SK = (*pointer & (1 << 0));
		Delay0_3us;
		SK = 0;
		_nop_();
		pointer++;
	}	while(++count != LED_COUNT * 3);
	
	
	  pointer=tmp;
    SK = 0;
		// EA = 1; // 启用中断
}
