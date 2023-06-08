
/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.C
* Author             : WCH
* Version            : V1.0
* Date               : 2017/01/20
* Description        : CH544 ADC��ʼ����ADC�жϺͲ�ѯ��ʽ�ɼ���ʾʾ��                        
*******************************************************************************/
#include "..\Public\CH554.H"                                                  
#include "..\Public\Debug.H"
#include "ADC.H"
#include "stdio.h"
#include <string.h>

#pragma  NOAREGS

void main( ) 
{
    UINT16 i;
    UINT16 j = 0;
    CfgFsys( );                                                                //CH554ʱ��ѡ������   
    mDelaymS(20);
    mInitSTDIO( );                                                             //����0��ʼ��
    printf("start ...\n"); 

    ADCInit( 0 );                                                              //ADCʱ�����ã�0(96clk) 1(384clk)��ADCģ�鿪��	

#if ADC_INTERRUPT                                                              //ADC�жϷ�ʽ
    EA = 1;
    while(1)                                                                   
    {
      for(i=0;i<4;i++){	
        printf("AIN%02x ",(UINT16)i);			                                     //ADC����ͨ����ӡ��ʾ��������	
        ADC_ChannelSelect( i );                                                //ADC������Դ������ͨ�����ã�i(0-3)��ʾ����ͨ��
        ADC_START = 1;                                                         //��ʼ������������ɽ����ж�
        mDelayuS(30);                                                          //�ȴ��ɼ���ɲ����л�����һͨ��
      }
    } 
#else	
    while(1)                                                                   //ADC ��ѯ��ʽ                                                                  
    {
      for(i=0;i<4;i++){				
        printf("AIN%02x ",(UINT16)i);		
        ADC_ChannelSelect( i );                                                //ADC������ʼ��
        ADC_START = 1;                                                         //��ʼ������������ɽ����ж�
        while(ADC_START);                                                      //ADC_START��Ϊ0ʱ����ʾ�������
        printf("DATA: %02x\n",(UINT16)ADC_DATA);
        mDelaymS(100);                                                         //ģ�ⵥƬ���������� 
      }	
    }		
#endif		
}