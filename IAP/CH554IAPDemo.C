
/********************************** (C) COPYRIGHT ******************************
* File Name          : CH554IAPDemo.C
* Author             : WCH
* Version            : V1.0
* Date               : 2017/01/20
* Description        : �ϵ����к�P17LED����˸������⡰EnableIAP����Ϊ�͵�ƽ�󣬽����û�������ת��BOOT��ͨ��BOOT�����û����� 
*******************************************************************************/
#include "./Public/CH554.H"                                                    
#include "./Public/Debug.H"

sbit EnableIAP  = P1^6;         
#define BOOT_ADDR  0x3800

#pragma NOAREGS

/*******************************************************************************
* Function Name  : main
* Description    : ������
*                ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
typedef void( *pTaskFn)( void );

pTaskFn tasksArr[1]; 
 
void main( void ) 
{
	UINT16 i=0;
    while(1){
    SCK = ~SCK;                                                              //P17��˸
    mDelaymS(50);
    if(EnableIAP == 0){                                                      //P16���ż�⵽�͵�ƽ��ת
      break;
    }
  }
  EA = 0;                                                                    //�ر����жϣ��ؼ�
	tasksArr[0] = BOOT_ADDR;
  mDelaymS( 100 ); 				
  (tasksArr[0])( );                                                          //����BOOT��������,ʹ��ISP��������	
  while(1); 
}