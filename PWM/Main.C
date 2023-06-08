
/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.C
* Author             : WCH
* Version            : V1.1
* Date               : 2017/02/27
* Description        : CH554 PWM��ʼ����ռ�ձ����ã�PWMĬ�ϵ�ƽ����
                       ֧���жϷ�ʽ�޸�PWMռ�ձ� 
*******************************************************************************/

#include "..\Public\CH554.H"                                                   
#include "..\Public\Debug.H"
#include "PWM.H"
#include "stdio.h"

#pragma  NOAREGS

main( ) 
{
    CfgFsys( );                                                                //CH554ʱ��ѡ������   
    mDelaymS(5);                                                               //����ʱ�Ӻ󣬽�����ʱ�ȶ�ʱ��
    mInitSTDIO( );                                                             //����0��ʼ��
    printf("start ...\n"); 

    P1_MOD_OC &= ~(bPWM1 | bPWM2);                                             //����PWM����Ϊ�������
    P1_DIR_PU |= bPWM1 | bPWM2;			
    
    SetPWMClk(4);                                                              //PWMʱ������	��Fsys/256/4��Ƶ
    ForceClearPWMFIFO( );                                                      //ǿ�����PWM FIFO��COUNT
    CancleClearPWMFIFO( );                                                     //ȡ�����PWM FIFO��COUNT
    PWM1OutEnable( );                                                          //����PWM1���                           
    PWM2OutEnable( );                                                          //����PWM2��� 	

    PWM1OutPolarHighAct( );                                                    //PWM1���Ĭ�ϵͣ�����Ч                                                   
    PWM2OutPolarLowAct( );                                                     //PWM2���Ĭ�ϸߣ�����Ч 

#if PWM_INTERRUPT
    PWMInterruptEnable();	
    EA = 1;
    SetPWM1Dat(0x10);                  
    SetPWM2Dat(0x40);	
    while(1);
#endif	
    SetPWM1Dat(0x10);                                                          //ռ�ձ�0x10/256                                                         
    SetPWM2Dat(0x40);	
    while(1){
      if(PWM_CTRL&bPWM_IF_END){
        PWM_CTRL |= bPWM_IF_END;                                               //���PWM�ж�				
        SetPWM1Dat(0x10);
        SetPWM2Dat(0x40);	
#ifdef DE_PRINTF
    printf("PWM_CYC_END  %02X\n",(UINT16)PWM_CTRL);
#endif	
      }
    }
}