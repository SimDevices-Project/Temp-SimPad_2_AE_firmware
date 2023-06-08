
/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.C
* Author             : WCH
* Version            : V1.1
* Date               : 2017/11/18
* Description        : GPIO ������ʹ�ú�GPIO�ж�ʹ��ʾ��   
*******************************************************************************/
#include "..\Public\CH554.H"                                                   
#include "..\Public\Debug.H"
#include "GPIO.H"
#include "stdio.h"
#include <string.h>

#pragma  NOAREGS

sbit LED0 = P1^6;
sbit LED1 = P1^7;

void main( ) 
{
    UINT16 j = 0;
    CfgFsys( );                                                                //CH554ʱ��ѡ������   
    mDelaymS(20);
    mInitSTDIO( );                                                             //����0��ʼ��
    printf("start ...\n"); 
    Port1Cfg(1,6);                                                             //P16��������ģʽ
    Port1Cfg(1,7);                                                             //P17��������ģʽ
    LED0 = 0;
    LED1 = 0;	

    GPIOInterruptCfg();                                                        //GPIO�ж����ú���	
    EA = 1;
    printf("Run"); 
    while(1){
      printf(".");
      LED0 = ~LED0;
      LED1 = ~LED1;			
      mDelaymS(100);			
    }
}