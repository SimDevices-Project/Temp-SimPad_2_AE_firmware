//描述符类说明 http://blog.chinaunix.net/uid-28458801-id-3977853.html
//00 00 1D 00 1B 01 15 00
//01 29 00 3B 00 00 00 00


/*
01 00 1D 00 00
02 00 1B 00 00
03 01 15 00 00
04 00 29 00 00
05 00 3B 00 00

*/
/********************************** (C) COPYRIGHT *******************************
* File Name          :CompositeKM.C
* Author             : WCH
* Version            : V1.2
* Date               : 2018/02/28
* Description        : CH559模拟USB复合设备，键鼠，支持类命令,支持唤醒
*******************************************************************************/

#include "./Public/CH554.H"
#include "./RGB_LIB.H"

//#include "./DataFlash/DataFlash.H"
#include <string.h>
#include <stdio.h>

#include "./Public/Debug.H"

#define Fullspeed               1
#ifdef  Fullspeed
//#define THIS_ENDP0_SIZE         64
//#else
#define THIS_ENDP0_SIZE         DEFAULT_ENDP0_SIZE
#endif

#define VERSION_YEAR 20
#define VERSION_YEAR_S 20
#define VERSION_MONTH 0x02
#define VERSION_DAY 0x1C

#define IAP_ProgrameStartAddr    (0x3000) //IAP程序位置
// #define UPF_DisableRd(SS)  (USB_C_CTRL = SS ? (USB_C_CTRL|bUCC1_PD_EN|bUCC2_PD_EN) : 0)

typedef void( *pTaskFn)( void );
pTaskFn tasksArr[1];

#define THIS_ENDP2_SIZE 8

UINT8X  Ep0Buffer[8>(THIS_ENDP0_SIZE+2)?8:(THIS_ENDP0_SIZE+2)] _at_ 0x0000;    //端点0 OUT&IN缓冲区，必须是偶地址
UINT8X  Ep1Buffer[64>(MAX_PACKET_SIZE+2)?64:(MAX_PACKET_SIZE+2)] _at_ 0x000a;  //端点1 IN缓冲区,必须是偶地址
//UINT8X  Ep2Buffer[64>(MAX_PACKET_SIZE+2)?64:(MAX_PACKET_SIZE+2)] _at_ 0x0050;  //端点2 IN缓冲区,必须是偶地址
UINT8X  Ep2Buffer[128>(2*MAX_PACKET_SIZE+4)?128:(2*MAX_PACKET_SIZE+4)] _at_ 0x0050;//端点2 IN&OUT缓冲区,必须是偶地址
UINT8X  Ep3Buffer[64>(2*MAX_PACKET_SIZE+4)?64:(2*MAX_PACKET_SIZE+4)] _at_ 0x00D4;//端点3 IN&OUT缓冲区,必须是偶地址
UINT8 SetupReq,SetupLen,Ready,Count,UsbConfig,FLAG;

PUINT8 xdata  pDescr;                                                          //USB配置标志
USB_SETUP_REQ   SetupReqBuf;                                                   //暂存Setup包

//sbit Ep2InKey = P1^5;

#define UsbSetupBuf     ((PUSB_SETUP_REQ)Ep0Buffer)

#define LED_CHANGE_DELAY 0x0A00

#define DEBUG 0
#pragma  NOAREGS

UINT16 data LED_RAINBOW_DELAY = 0x0A00;
UINT16 data LED_EASE_DELAY = 0x0A00;

sbit BT1 = P3^3;//默认设置Z
sbit BT2 = P3^2;//默认设置X

sbit BT3 = P3^4;//默认设置Esc
sbit BT4 = P3^1;//默认设置F1
sbit BT5 = P3^0;//默认设置F2



unsigned int data KEY_DELAY;
unsigned int data KEY_DELAY_MIN;

// unsigned char data cmdCode[] = {0x00,0x00};

unsigned char data activeKeyCode1[] = {0x00,0x00,0x00};
unsigned char data activeKeyCode2[] = {0x00,0x00,0x00};

unsigned char xdata keyCode1[] = {0x00,0x00,0x00};
unsigned char xdata keyCode2[] = {0x00,0x00,0x00};

unsigned char xdata keyCode3[] = {0x00,0x00,0x00, 0x00};
unsigned char xdata keyCode4[] = {0x00,0x00,0x00, 0x00};
unsigned char xdata keyCode5[] = {0x00,0x00,0x00, 0x00};

unsigned char xdata keyCode1G2[] = {0x00,0x00,0x00};
unsigned char xdata keyCode2G2[] = {0x00,0x00,0x00};
// unsigned char xdata keyCode1G3[] = {0x00,0x00,0x00};
// unsigned char xdata keyCode2G3[] = {0x00,0x00,0x00};


unsigned char data lightMode;

unsigned char data colorR0;
unsigned char data colorG0;
unsigned char data colorB0;

unsigned char data LED0_brightness;

unsigned char data colorR1;
unsigned char data colorG1;
unsigned char data colorB1;

unsigned char data LED1_brightness;

unsigned char data colorToShowR0;
unsigned char data colorToShowG0;
unsigned char data colorToShowB0;

unsigned char data colorToShowR1;
unsigned char data colorToShowG1;
unsigned char data colorToShowB1;


// unsigned char data keyCode_Roller[] = {0x00,0x00,0x00};

// unsigned char data lightMode;

unsigned int data BT1keyState = 0;
unsigned int data BT2keyState = 0;

unsigned int data BT3keyState = 0;
unsigned int data BT4keyState = 0;
unsigned int data BT5keyState = 0;

// unsigned int data BT_RollerkeyState = 0;

unsigned char hiSpeedMode = 0x00;
unsigned char isDisableOfflineLight = 0x00;

unsigned char IAPflag;

bit data sendingFlag = 0;
bit data sendingFlagMouse = 0;
bit data waitToSend = 0;
bit data waitToSendMouse = 0;

bit data led0LightSwitchFlag = 0;
bit data led1LightSwitchFlag = 0;

/*设备描述符*/
//低字节在前
UINT8C DevDesc[18] = {0x12,//描述符长度(18字节)
                      0x01,//描述符类型
                      0x10,0x01,//本设备所用USB版本
                      //0x20,0x00,//本设备所用USB版本
                      0x00,//类代码
                      0x00,//子类代码
                      0x00,//设备所用协议
                      THIS_ENDP0_SIZE,//端点0最大包长
                      //0x3d,0x41,//厂商ID
                      //0x07,0x21,//产品ID
                      0x88,0x80,//厂商ID
                      // 0x03,0x00,//产品ID
                      /*
                      * 00 01 SimPad v2
                      * 00 02 SimPad v2 - Extra
                      * 00 03 SimPad v2 - Lite
                      * 00 04 SimPad Nano
                      * 00 06 SimPad v2 - Year Edition
                      * 00 FF SimPad Boot
                      */


                      0x06,0x00,//产品ID

                      0x05, 0x01,//设备版本号

                      0x01,//描述厂商信息的字符串描述符的索引值
                      0x02,//描述产品信息的字串描述符的索引值
                      0x03,//描述设备序列号信息的字串描述符的索引值
                      0x01//可能的配置数
                     };

/*字符串描述符*/

// 语言描述符

/**function: CharToHex()
*** ACSII change to 16 hex
*** input:ACSII
***Return :Hex
**/
unsigned char CharToHex(unsigned char bHex)
{
	bHex = bHex & 0x0F;
	if((bHex>=0)&&(bHex<=9))
	{
		bHex += 0x30;
	}
	else if((bHex>=10)&&(bHex<=15))//Capital
	{
		bHex += 0x37;
	}
	else 
	{
		bHex = 0xff;
	}
	return bHex;
}

UINT8X SerialNumber[] = {
  0x12,0x03,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,
};

//UINT8C	LangDescr[] = { 0x04, 0x03, 0x04, 0x08 };//后两位为语言描述符，0x0804 简体中文 0x0409 英文
UINT8C	LangDescr[] = { 0x0C, 0x03,
                        0x09, 0x04,
                        0x04, 0x08,
                        0x04, 0x0C,
                        0x04, 0x04,
                        0x11, 0x04
                     };// 英文，简中，繁中HK，繁中TW，日语

// 厂家信息
UINT8C	ManuInfo[] = { 32,//长度
                       0x03,
                       'H', 0, 'a', 0, 'n', 0, 'd', 0, 'l', 0, 'e', 0, ' ', 0,
                       'B', 0, 'Y', 0, 'S', 0, 'B', 0, '.', 0, 'n', 0, 'e', 0, 't', 0
                    };
UINT8C ManuInfo_zh_CN[26]=
{
    0x1A,0x03,0x4B,0x62,0xC4,0x67,0x1B,0x54,0x20,0x00,0x42,0x00,0x59,0x00,0x53,0x00,
    0x42,0x00,0x2E,0x00,0x6E,0x00,0x65,0x00,0x74,0x00
};
// 产品信息
UINT8C	ProdInfo[52] = {
    0x34,0x03,0x53,0x00,0x69,0x00,0x6D,0x00,0x50,0x00,0x61,0x00,0x64,0x00,0x20,0x00,
    0x76,0x00,0x32,0x00,0x20,0x00,0x47,0x00,0x61,0x00,0x6D,0x00,0x69,0x00,0x6E,0x00,
    0x67,0x00,0x20,0x00,0x4B,0x00,0x65,0x00,0x79,0x00,0x62,0x00,0x6F,0x00,0x61,0x00,
    0x72,0x00,0x64,0x00
};
UINT8C ProdInfo_zh_CN[30]=
{
    0x1E,0x03,0x53,0x00,0x69,0x00,0x6D,0x00,0x50,0x00,0x61,0x00,0x64,0x00,0x20,0x00,0x76,0x00,
    0x32,0x00,0x20,0x00,0xF7,0x8F,0x60,0x4F,0x2E,0x95,0xD8,0x76
};
UINT8C ProdInfo_zh_TW_HK[32]=
{
    0x20,0x03,0x53,0x00,0x69,0x00,0x6D,0x00,0x50,0x00,0x61,0x00,0x64,0x00,0x20,0x00,
    0x76,0x00,0x32,0x00,0x20,0x00,0x38,0x6E,0x32,0x62,0x75,0x93,0xE4,0x76,0x00,0x00
};
UINT8C ProdInfo_ja_JP[41]=
{
    0x29,0x03,0x53,0x00,0x69,0x00,0x6D,0x00,0x50,0x00,0x61,0x00,0x64,0x00,0x20,0x00,
    0x76,0x00,0x32,0x00,0x20,0x00,0xB1,0x30,0x9E,0xFF,0xFC,0x30,0xE0,0x30,0xAD,0x30,
    0xFC,0x30,0xDB,0x30,0x9E,0xFF,0xFC,0x30,0xC8
};
//UINT8C	ProdInfo[] = { 12,//长度
//											0x03,
//											0x4f, 0, 0x6e, 0, 0x65, 0, 0xe6, 0x89, 0xd8, 0x76};//One触盘

/*HID类报表描述符*/
UINT8C KeyRepDesc[62] =
{
    0x05,0x01,0x09,0x06,0xA1,0x01,0x05,0x07,
    0x19,0xe0,0x29,0xe7,0x15,0x00,0x25,0x01,
    0x75,0x01,0x95,0x08,0x81,0x02,0x95,0x01,
    0x75,0x08,0x81,0x01,0x95,0x03,0x75,0x01,
    0x05,0x08,0x19,0x01,0x29,0x03,0x91,0x02,
    0x95,0x05,0x75,0x01,0x91,0x01,0x95,0x06,
    0x75,0x08,0x26,0xff,0x00,0x05,0x07,0x19,
    0x00,0x29,0x91,0x81,0x00,0xC0
};
UINT8C MouseRepDesc[52] =
{
    0x05,0x01,0x09,0x02,0xA1,0x01,0x09,0x01,
    0xA1,0x00,0x05,0x09,0x19,0x01,0x29,0x03,
    0x15,0x00,0x25,0x01,0x75,0x01,0x95,0x03,
    0x81,0x02,0x75,0x05,0x95,0x01,0x81,0x01,
    0x05,0x01,0x09,0x30,0x09,0x31,0x09,0x38,
    0x15,0x81,0x25,0x7f,0x75,0x08,0x95,0x03,
    0x81,0x06,0xC0,0xC0
};
/*HID类报表描述符*/
/*UINT8C HIDRepDesc[ ] =
{
    0x06, 0x00,0xff,
    0x09, 0x01,
    0xa1, 0x01,                                                   //集合开始
    0x09, 0x02,                                                   //Usage Page  用法
    0x15, 0x00,                                                   //Logical  Minimun
    0x26, 0x00,0xff,                                              //Logical  Maximun
    0x75, 0x08,                                                   //Report Size
    0x95, THIS_ENDP0_SIZE,                                        //Report Counet
    0x81, 0x06,                                                   //Input
    0x09, 0x02,                                                   //Usage Page  用法
    0x15, 0x00,                                                   //Logical  Minimun
    0x26, 0x00,0xff,                                              //Logical  Maximun
    0x75, 0x08,                                                   //Report Size
    0x95, THIS_ENDP0_SIZE,                                        //Report Counet
    0x91, 0x06,                                                   //Output
    0xC0
};*/


UINT8C  HIDRepDesc[]= {
    0x05,0x01,                                               //报表描述符，每个条目占一行
    0x09,0x00,
    0xa1,0x01,
    0x15,0x00,
    0x25,0xff,
    0x19,0x01,
    0x29,0x08,
    0x95,0x08,
    0x75,0x08,
    0x81,0x02,
    0x09,0x02,
    0x15,0x00,
    0x25,0xff,
    0x75,0x08,
    0x95,0x40,
    0x91,0x06,
    0xc0
};

// UINT8C CfgDesc[] =
UINT8 xdata CfgDesc[] =
{
    //配置描述符
    0x09,//配置描述符长度，固定9
    0x02,//配置描述符类型
    0x5B,0x00,//整个描述符数据的长度．指此配置返回的配置描述符，接口描述符以及端点描述符的全部大小
    0x03,//配置所支持的接口数．指该配置配备的接口数量，也表示该配置下接口描述符数量
    0x01,//当使用SetConfiguration和GetConfiguration请求时所指定的配置索引值
    0x00,//用于描述该配置字符串描述符的索引
    0xA0,//供电模式选择．D7:总线供电，D6:自供电，D5:远程唤醒，D4～D0:保留 0xA0== 0b 1010 0000

    //0x32,//总线供电的USB设备的最大消耗电流．以2mA为单位．例如0x32为50*2=100mA
    0x96,
    //0xFA,

    //描述符类说明 http://blog.chinaunix.net/uid-28458801-id-3977853.html

    //接口描述符,键盘

    0x09,//接口描述符长度
    0x04,//接口描述表类 常量 0x04 == 接口
    0x00,//接口号
    0x00,//可选设置的索引值
    0x01,//此接口用的端点数量
    0x03,//类值
    0x01,//子类码
    0x01,//协议码
    0x00,//描述此接口的字串描述表的索引值

    //HID类描述符

    0x09,//描述符类型
    0x21,//描述符类 常量 0x21==HID
    0x10,0x01,//HID规范版本号 BCD
    0x00,//硬件设备所在国家的国家代码（不说明为0）
    0x01,//类别描述符数目（至少有一个报表描述符）
    0x22,//类别描述符类型
    sizeof(KeyRepDesc),0x00,//报表描述符的总长度
    //byte 附加描述符类型，可选
    //word 附加描述符总长度，可选

    //端点描述符

    0x07,//长度
    0x05,//类型编号
    0x81,//端点地址以及输入输出属性
    0x03,//端点的传输类型属性
    0x08,0x00,//端点收发包的最大包大小
    0x01,     //主机查询端点的时间间隔

    //接口描述符,HID设备

    0x09,//接口描述符长度
    0x04,//接口描述表类 常量 0x04 == 接口
    0x01,//接口号
    0x00,//可选设置的索引值
    0x02,//此接口用的端点数量
    0x03,//类值
    //0x01,//子类码
    //0x02,//协议码
    //0x00,//描述此接口的字串描述表的索引值

    0x00,//子类码
    0x00,//协议码
    0x00,//描述此接口的字串描述表的索引值

    //HID类描述符

    0x09,//描述符类型
    0x21,//描述符类 常量 0x21==HID
    0x10,0x01,//HID规范版本号 BCD
    0x00,//硬件设备所在国家的国家代码（不说明为0）
    0x01,//类别描述符数目（至少有一个报表描述符）
    0x22,//类别描述符类型
    //0x34,0x00,//报表描述符的总长度

    sizeof(HIDRepDesc),0x00,//报表描述符的总长度
    //byte 附加描述符类型，可选
    //word 附加描述符总长度，可选

    //端点描述符
    0x07,//长度
    0x05,//类型编号
    0x82,//端点地址以及输入输出属性
    0x03,//端点的传输类型属性
    // 0x04,0x00,//端点收发包的最大包大小
    //0x05,     //主机查询端点的时间间隔

    THIS_ENDP2_SIZE,0x00,//端点收发包的最大包大小
    0x0a,     //主机查询端点的时间间隔

    //端点描述符
    0x07,//长度
    0x05,//类型编号
    0x02,//端点地址以及输入输出属性
    0x03,//端点的传输类型属性
    THIS_ENDP2_SIZE,0x00,//端点收发包的最大包大小
    0x02,     //主机查询端点的时间间隔
		
		//接口描述符,鼠标

    0x09,//接口描述符长度
    0x04,//接口描述表类 常量 0x04 == 接口
    0x02,//接口号
    0x00,//可选设置的索引值
    0x01,//此接口用的端点数量
    0x03,//类值
    0x01,//子类码
    0x02,//协议码
    0x00,//描述此接口的字串描述表的索引值

    //HID类描述符

    0x09,//描述符类型
    0x21,//描述符类 常量 0x21==HID
    0x10,0x01,//HID规范版本号 BCD
    0x00,//硬件设备所在国家的国家代码（不说明为0）
    0x01,//类别描述符数目（至少有一个报表描述符）
    0x22,//类别描述符类型
    //0x34,0x00,//报表描述符的总长度

    sizeof(MouseRepDesc),0x00,//报表描述符的总长度
    //byte 附加描述符类型，可选
    //word 附加描述符总长度，可选

    //端点描述符
    0x07,//长度
    0x05,//类型编号
    0x83,//端点地址以及输入输出属性
    0x03,//端点的传输类型属性
    0x04,0x00,//端点收发包的最大包大小

    //THIS_ENDP2_SIZE,0x00,//端点收发包的最大包大小
    0x01,     //主机查询端点的时间间隔

};

UINT8X UserEp2Buf[THIS_ENDP2_SIZE];                                            //用户数据定义
UINT8X DataToWrite[14]; //用于复写内存储
//bit dataFlag = 0;

/*鼠标数据*/
UINT8 data HIDMouse[4] = {0x0,0x0,0x0,0x0};
/*键盘数据*/
UINT8 data HIDKey[8] = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};

/*******************************************************************************
* Function Name  : TypeC_UPF_PDInit()
* Description    : Type-C UPF初始化
* Input          : None
* Output         : None
* Return         : None							 
*******************************************************************************/
/* void TypeC_UPF_PDInit( void )
{
   P1_MOD_OC &= ~(bUCC2|bUCC1);                                                   
   P1_DIR_PU &= ~(bUCC2|bUCC1);                                                   //UCC1 UCC2 设置浮空输入
	 UPF_DisableRd(1);                                                              //开启UCC下拉电阻
   // ADC_CFG = ADC_CFG & ~bADC_CLK | bADC_EN;											                  //ADC时钟配置,0(96clk) 1(384clk),ADC模块开启	
	 // P1_DIR_PU &= ~(bAIN0 | bAIN1);																									//配置UCC1和UCC2作为ADC引脚
   mDelayuS(2);                                                                   //等待上拉完全关闭和ADC电源稳定	
}
*/
UINT8 WriteDataFlash(UINT8,PUINT8,UINT8);
void refreshKeyCode();
void CheckCmdCode();

/*******************************************************************************
* Function Name  : CH554SoftReset()
* Description    : CH554软复位
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH554SoftReset( )
{
    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA;
    GLOBAL_CFG	|=bSW_RESET;
}

/*******************************************************************************
* Function Name  : CH554USBDevWakeup()
* Description    : CH554设备模式唤醒主机，发送K信号
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*void CH554USBDevWakeup( )
{
    UDEV_CTRL |= bUD_LOW_SPEED;
    mDelaymS(2);
    UDEV_CTRL &= ~bUD_LOW_SPEED;
}
*/
/*******************************************************************************
* Function Name  : USBDeviceInit()
* Description    : USB设备模式配置,设备模式启动，收发端点配置，中断开启
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBDeviceInit()
{
    IE_USB = 0;
    USB_CTRL = 0x00;                                                           // 先设定USB设备模式
    UDEV_CTRL = bUD_PD_DIS;                                                    // 禁止DP/DM下拉电阻

    UDEV_CTRL &= ~bUD_LOW_SPEED;                                               //选择全速12M模式，默认方式
    // USB_CTRL &= ~bUC_LOW_SPEED;

    UEP1_DMA = Ep1Buffer;                                                      //端点1数据传输地址
    UEP4_1_MOD = UEP4_1_MOD & ~bUEP1_BUF_MOD | bUEP1_TX_EN;                    //端点1发送使能 64字节缓冲区
    UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;                                 //端点1自动翻转同步标志位，IN事务返回NAK

    UEP2_DMA = Ep2Buffer;                                                      //端点2数据传输地址
    UEP3_DMA = Ep3Buffer;                                                      //端点3数据传输地址
    //UEP2_3_MOD = UEP2_3_MOD & ~bUEP2_BUF_MOD | bUEP2_TX_EN;                    //端点2发送使能 64字节缓冲区
    UEP2_3_MOD |= bUEP2_TX_EN | bUEP2_RX_EN | bUEP3_TX_EN ;                                   //端点2/3发送接收使能
    UEP2_3_MOD &= ~bUEP2_BUF_MOD & ~bUEP3_BUF_MOD;                                              //端点2/3收发各64字节缓冲区
    //UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;                                 //端点2自动翻转同步标志位，IN事务返回NAK
    UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;                 //端点2自动翻转同步标志位，IN事务返回NAK，OUT返回ACK
    UEP3_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;                 //端点3自动翻转同步标志位，IN事务返回NAK，OUT返回ACK

    UEP0_DMA = Ep0Buffer;                                                      //端点0数据传输地址
    UEP4_1_MOD &= ~(bUEP4_RX_EN | bUEP4_TX_EN);                                //端点0单64字节收发缓冲区
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;                                 //OUT事务返回ACK，IN事务返回NAK

    USB_DEV_AD = 0x00;
    //USB_CTRL = bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;                      // 启动USB设备及DMA，在中断期间中断标志未清除前自动返回NAK
    USB_CTRL |= bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;                      // 启动USB设备及DMA，在中断期间中断标志未清除前自动返回NAK
    UDEV_CTRL |= bUD_PORT_EN;                                                  // 允许USB端口
    USB_INT_FG = 0xFF;                                                         // 清中断标志
    USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
    IE_USB = 1;
}
/*******************************************************************************
* Function Name  : Enp1IntIn()
* Description    : USB设备模式端点1的中断上传
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void Enp1IntIn( )
{

    memcpy( Ep1Buffer, HIDKey, 0x08);                              //加载上传数据
    UEP1_T_LEN = 0x08;                                             //上传数据长度
    UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                //有数据时上传数据并应答ACK

}
/*******************************************************************************
* Function Name  : Enp3IntIn()
* Description    : USB设备模式端点1的中断上传
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void Enp3IntIn( )
{
    memcpy( Ep3Buffer, HIDMouse, 0x04);
    //memcpy( Ep3Buffer, HIDMouse, sizeof(HIDMouse));                              //加载上传数据
    // UEP3_T_LEN = sizeof(HIDMouse);                                             //上传数据长度
    UEP3_T_LEN = 0x04;
    UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                //有数据时上传数据并应答ACK

}
/*******************************************************************************
* Function Name  : Enp2IntIn()
* Description    : USB设备模式端点2的中断上传
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*void Enp2IntIn( )
{
    //memcpy( Ep2Buffer, HIDMouse, sizeof(HIDMouse));                              //加载上传数据
    //UEP2_T_LEN = sizeof(HIDMouse);                                              //上传数据长度
    memcpy( Ep2Buffer, UserEp2Buf, sizeof(UserEp2Buf));                              //加载上传数据
    UEP2_T_LEN = sizeof(UserEp2Buf);                                              //上传数据长度
    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                  //有数据时上传数据并应答ACK
}
*/
/*******************************************************************************
* Function Name  : Enp2BlukIn()
* Description    : USB设备模式端点2的批量上传
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Enp2BlukIn( )
{
    //memcpy( Ep2Buffer+MAX_PACKET_SIZE, UserEp2Buf, sizeof(UserEp2Buf));        //加载上传数据
    //UEP2_T_LEN = sizeof(UserEp2Buf);                                              //上传最大包长度
    memcpy( Ep2Buffer+MAX_PACKET_SIZE, UserEp2Buf, 8);        //加载上传数据
    UEP2_T_LEN = 8;                                              //上传最大包长度
    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                  //有数据时上传数据并应答ACK
    while(UEP2_CTRL&MASK_UEP_T_RES == UEP_T_RES_ACK);                          //等待传输完成
}

/*******************************************************************************
* Function Name  : DeviceInterrupt()
* Description    : CH559USB中断处理函数
*******************************************************************************/
bit getSms = 0;
void DeviceInterrupt( ) interrupt INT_NO_USB using 1                      //USB中断服务程序,使用寄存器组1
{
    UINT8 data len = 0;//,i;
    if(UIF_TRANSFER)                                                            //USB传输完成标志
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
        case UIS_TOKEN_IN | 2:                                                  //endpoint 2# 中断端点上传
            UEP2_T_LEN = 0;                                                     //预使用发送长度一定要清空
//            UEP1_CTRL ^= bUEP_T_TOG;                                          //如果不设置自动翻转则需要手动翻转
            UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //默认应答NAK
            // FLAG = 1;
            break;
        case UIS_TOKEN_OUT | 2:                                                 //endpoint 2# 端点批量下传
            if ( U_TOG_OK )                                                     // 不同步的数据包将丢弃
            {
                UEP2_CTRL &= ~UEP_R_RES_ACK;
                UEP2_CTRL |= UEP_R_RES_NAK;
                len = USB_RX_LEN;                                               //接收数据长度，数据从Ep2Buffer首地址开始存放

                // 写DataFlash
                if(Ep2Buffer[0]) {
                    DataToWrite[0] = Ep2Buffer[1];
                    DataToWrite[1] = Ep2Buffer[2];
                    DataToWrite[2] = Ep2Buffer[3];
                    DataToWrite[3] = Ep2Buffer[4];
                    if(Ep2Buffer[5]==(Ep2Buffer[1]^Ep2Buffer[2]^Ep2Buffer[3]^Ep2Buffer[4])) {
                        WriteDataFlash((Ep2Buffer[0]) * 4,DataToWrite,4);
                        refreshKeyCode();
                    }
                } else {
                    getSms = 1;
                }


                UEP2_T_LEN = 8;
                //UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;       // 允许上传
                UEP2_CTRL &= ~UEP_R_RES_NAK;
                UEP2_CTRL |= UEP_R_RES_ACK;
            }
            break;
        case UIS_TOKEN_IN | 1:                                                  //endpoint 1# 中断端点上传
            UEP1_T_LEN = 0;                                                     //预使用发送长度一定要清空
//            UEP2_CTRL ^= bUEP_T_TOG;                                          //如果不设置自动翻转则需要手动翻转
            UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //默认应答NAK
            // FLAG = 1;                                                           /*传输完成标志*/
            break;
        case UIS_TOKEN_IN | 3:                                                  //endpoint 3# 中断端点上传
            UEP3_T_LEN = 0;                                                     //预使用发送长度一定要清空
            //UEP3_CTRL ^= bUEP_T_TOG;                                          //如果不设置自动翻转则需要手动翻转
            UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //默认应答NAK
            // FLAG = 1;                                                           /*传输完成标志*/
            break;
        case UIS_TOKEN_SETUP | 0:                                                //SETUP事务
            len = USB_RX_LEN;
            if(len == (sizeof(USB_SETUP_REQ)))
            {
                SetupLen = UsbSetupBuf->wLengthL;

                if(UsbSetupBuf->wLengthH || SetupLen > 0x7F )
                {
                    SetupLen = 0x7F;    // 限制总长度
                }
                len = 0;                                                        // 默认为成功并且上传0长度
                SetupReq = UsbSetupBuf->bRequest;
                if ( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )/* HID类命令 */
                {
                    switch( SetupReq )
                    {
                    case 0x01://GetReport
                        break;
                    case 0x02://GetIdle
                        break;
                    case 0x03://GetProtocol
                        break;
                    case 0x09://SetReport
                        Ready = 1;
                        break;
                    case 0x0A://SetIdle
                        break;
                    case 0x0B://SetProtocol
                        break;
                    default:
                        len = 0xFF;  								 					            /*命令不支持*/
                        break;
                    }
                    if ( SetupLen > len )
                    {
                        SetupLen = len;    //限制总长度
                    }
                    len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen;//本次传输长度
                    memcpy(Ep0Buffer,pDescr,len);                            //加载上传数据
                    SetupLen -= len;
                    pDescr += len;
                }
                else
                {   //标准请求
                    switch(SetupReq)                                        //请求码
                    {
                    case USB_GET_DESCRIPTOR:
                        switch(UsbSetupBuf->wValueH)
                        {
                        case 1:                                             //设备描述符
                            pDescr = DevDesc;                               //把设备描述符送到要发送的缓冲区
                            len = sizeof(DevDesc);
                            break;
                        case 2:                                             //配置描述符
                            pDescr = CfgDesc;                               //把设备描述符送到要发送的缓冲区
                            len = sizeof(CfgDesc);
                            break;

                            //请求字符串描述符等
                        case 3:                                             //字符串描述符
                            switch( UsbSetupBuf->wValueL ) {
                            case 1:
                                if(UsbSetupBuf->wIndexL == 0x04 && UsbSetupBuf->wIndexH == 0x04) {
                                    pDescr = ManuInfo_zh_CN;
                                    len = sizeof( ManuInfo_zh_CN );
                                    break;
                                }
                                if(UsbSetupBuf->wIndexL == 0x04 && UsbSetupBuf->wIndexH == 0x08) {
                                    pDescr = ManuInfo_zh_CN;
                                    len = sizeof( ManuInfo_zh_CN );
                                    break;
                                }
                                if(UsbSetupBuf->wIndexL == 0x04 && UsbSetupBuf->wIndexH == 0x0C) {
                                    pDescr = ManuInfo_zh_CN;
                                    len = sizeof( ManuInfo_zh_CN );
                                    break;
                                }
                                pDescr = ManuInfo;
                                len = sizeof( ManuInfo );
                                break;
                            case 2:
                                if(UsbSetupBuf->wIndexL == 0x11 && UsbSetupBuf->wIndexH == 0x04) {
                                    pDescr = ProdInfo_ja_JP;
                                    len = sizeof( ProdInfo_ja_JP );
                                    break;
                                }
                                if(UsbSetupBuf->wIndexL == 0x04 && UsbSetupBuf->wIndexH == 0x08) {
                                    pDescr = ProdInfo_zh_CN;
                                    len = sizeof( ProdInfo_zh_CN );
                                    break;
                                }
                                if(UsbSetupBuf->wIndexL == 0x04 && UsbSetupBuf->wIndexH == 0x0C) {
                                    pDescr = ProdInfo_zh_TW_HK;
                                    len = sizeof( ProdInfo_zh_TW_HK );
                                    break;
                                }
                                if(UsbSetupBuf->wIndexL == 0x04 && UsbSetupBuf->wIndexH == 0x04) {
                                    pDescr = ProdInfo_zh_TW_HK;
                                    len = sizeof( ProdInfo_zh_TW_HK );
                                    break;
                                }
                                pDescr = ProdInfo;
                                len = sizeof( ProdInfo );
                                break;
														case 3:
														    pDescr = SerialNumber;
														    len = 18;
														    break;
                            case 0:
                                pDescr = LangDescr;
                                len = sizeof( LangDescr );
                                break;
                            default:
                                len = 0xFF;                               // 不支持的字符串描述符
                                break;
                            }
                            break;

                        case 0x22:                                          //报表描述符
                            if(UsbSetupBuf->wIndexL == 0)                   //报表0报表描述符
                            {
                                pDescr = KeyRepDesc;                        //数据准备上传
                                len = sizeof(KeyRepDesc);
                            }
                            else if(UsbSetupBuf->wIndexL == 1)              //报表1报表描述符
                            {
                                //pDescr = MouseRepDesc;                      //数据准备上传
                                //len = sizeof(MouseRepDesc);

                                pDescr = HIDRepDesc;                                 //数据准备上传
                                len = sizeof(HIDRepDesc);
                                //Ready = 1;                                  //如果有更多报表，该标准位应该在最后一个报表配置完成后有效
                            }
                            else if(UsbSetupBuf->wIndexL == 2)              //报表2报表描述符
                            {
                                pDescr = MouseRepDesc;                      //数据准备上传
                                len = sizeof(MouseRepDesc);
                                Ready = 1;                                  //如果有更多报表，该标准位应该在最后一个报表配置完成后有效
                            }
                            else
                            {
                                len = 0xff;                                 //本程序只有3个报表，这句话正常不可能执行
                            }
                            break;
                        default:
                            len = 0xff;                                     //不支持的命令或者出错
                            break;
                        }
                        if ( SetupLen > len )
                        {
                            SetupLen = len;    //限制总长度
                        }
                        len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen;                  //本次传输长度
                        memcpy(Ep0Buffer,pDescr,len);                        //加载上传数据
                        SetupLen -= len;
                        pDescr += len;
                        break;
                    case USB_SET_ADDRESS:
                        SetupLen = UsbSetupBuf->wValueL;                     //暂存USB设备地址
                        break;
                    case USB_GET_CONFIGURATION:
                        Ep0Buffer[0] = UsbConfig;
                        if ( SetupLen >= 1 )
                        {
                            len = 1;
                        }
                        break;
                    case USB_SET_CONFIGURATION:
                        UsbConfig = UsbSetupBuf->wValueL;
                        break;
                    case 0x0A:
                        break;
                    case USB_CLEAR_FEATURE:                                            //Clear Feature
                        if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )// 端点
                        {
                            switch( UsbSetupBuf->wIndexL )
                            {
                            case 0x82:
                                UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
                            case 0x81:
                                UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
                            case 0x83:
                                UEP3_CTRL = UEP3_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
                            case 0x01:
                                UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                break;
                            case 0x02:
                                UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                break;
                            case 0x03:
                                UEP3_CTRL = UEP3_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                break;
                            default:
                                len = 0xFF;                                            // 不支持的端点
                                break;
                            }
                        }
                        else if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )// 设备
                        {
                            break;
                        }
                        else
                        {
                            len = 0xFF;                                                // 不是端点不支持
                        }
                        break;
                    case USB_SET_FEATURE:                                              /* Set Feature */
                        if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x00 )             /* 设置设备 */
                        {
                            if( ( ( ( UINT16 )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
                            {
                                if( CfgDesc[ 7 ] & 0x20 )
                                {
                                    /* 设置唤醒使能标志 */
                                }
                                else
                                {
                                    len = 0xFF;                                        /* 操作失败 */
                                }
                            }
                            else
                            {
                                len = 0xFF;                                            /* 操作失败 */
                            }
                        }
                        else if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x02 )        /* 设置端点 */
                        {
                            if( ( ( ( UINT16 )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x00 )
                            {
                                switch( ( ( UINT16 )UsbSetupBuf->wIndexH << 8 ) | UsbSetupBuf->wIndexL )
                                {
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点2 IN STALL */
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点2 OUT Stall */
                                    break;
                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点1 IN STALL */
                                    break;
                                case 0x83:
                                    UEP3_CTRL = UEP3_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点3 IN STALL */
                                    break;
                                case 0x03:
                                    UEP3_CTRL = UEP3_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点3 OUT Stall */
                                    break;
                                default:
                                    len = 0xFF;                               //操作失败
                                    break;
                                }
                            }
                            else
                            {
                                len = 0xFF;                                   //操作失败
                            }
                        }
                        else
                        {
                            len = 0xFF;                                      //操作失败
                        }
                        break;
                    case USB_GET_STATUS:
                        Ep0Buffer[0] = 0x00;
                        Ep0Buffer[1] = 0x00;
                        if ( SetupLen >= 2 )
                        {
                            len = 2;
                        }
                        else
                        {
                            len = SetupLen;
                        }
                        break;
                    default:
                        len = 0xff;                                           //操作失败
                        break;
                    }
                }
            }
            else
            {
                len = 0xff;                                                   //包长度错误
            }
            if(len == 0xff)
            {
                SetupReq = 0xFF;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;//STALL
            }
            else if(len)                                                //上传数据或者状态阶段返回0长度包
            {
                UEP0_T_LEN = len;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1，返回应答ACK
            }
            else
            {
                UEP0_T_LEN = 0;  //虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1,返回应答ACK
            }
            break;
        case UIS_TOKEN_IN | 0:                                               //endpoint0 IN
            switch(SetupReq)
            {
            case USB_GET_DESCRIPTOR:
                len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen;                          //本次传输长度
                memcpy( Ep0Buffer, pDescr, len );                            //加载上传数据
                SetupLen -= len;
                pDescr += len;
                UEP0_T_LEN = len;
                UEP0_CTRL ^= bUEP_T_TOG;                                     //同步标志位翻转
                break;
            case USB_SET_ADDRESS:
                USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            default:
                UEP0_T_LEN = 0;                                              //状态阶段完成中断或者是强制上传0长度数据包结束控制传输
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            }
            break;
        case UIS_TOKEN_OUT | 0:  // endpoint0 OUT
            len = USB_RX_LEN;
            //  UEP0_T_LEN = 0;  //虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
            // UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA0,返回应答ACK
            if(SetupReq == 0x09)
            {
                if(Ep0Buffer[0])
                {
                    //printf("Light on Num Lock LED!\n");
                }
                else if(Ep0Buffer[0] == 0)
                {
                    //printf("Light off Num Lock LED!\n");
                }
            }
            UEP0_CTRL ^= bUEP_R_TOG;                                     //同步标志位翻转
            break;
        default:
            break;
        }
        UIF_TRANSFER = 0;                                                 //写0清空中断
    }
    if(UIF_BUS_RST)                                                       //设备模式USB总线复位中断
    {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP3_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0;                                                 //清中断标志
    }
    if (UIF_SUSPEND)                                                     //USB总线挂起/唤醒完成
    {
        UIF_SUSPEND = 0;
        if ( USB_MIS_ST & bUMS_SUSPEND )                                 //挂起
        {
//             while ( XBUS_AUX & bUART0_TX );                              //等待发送完成
//             SAFE_MOD = 0x55;
//             SAFE_MOD = 0xAA;
//             WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO;                      //USB或者RXD0有信号时可被唤醒
//             PCON |= PD;                                                  //睡眠
//             SAFE_MOD = 0x55;
//             SAFE_MOD = 0xAA;
//             WAKE_CTRL = 0x00;
        }
    }
    else {                                                               //意外的中断,不可能发生的情况
        USB_INT_FG = 0xFF;                                               //清中断标志
//      printf("UnknownInt  N");
    }

    //检查CMD代码
    if(getSms) {
        getSms= 0;
        CheckCmdCode();
    }
}

/*******************************************************************************
* Function Name  : WriteDataFlash(UINT8 data Addr,PUINT8 data buf,UINT8 data len)
* Description    : DataFlash写
* Input          : UINT8 data Addr，PUINT16 buf,UINT8 data len
* Output         : None
* Return         : UINT8 data i 返回写入长度
*******************************************************************************/
UINT8 WriteDataFlash(UINT8 data Addr,PUINT8 data buf,UINT8 data len) {
    UINT8 data i;
    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA;                                                           //进入安全模式
    GLOBAL_CFG |= bDATA_WE;                                                    //使能DataFlash写
    SAFE_MOD = 0;                                                              //退出安全模式
    ROM_ADDR_H = DATA_FLASH_ADDR >> 8;
    Addr <<= 1;
    for(i=0; i<len; i++)
    {
        ROM_ADDR_L = Addr + i*2;
        ROM_DATA_L = *(buf+i);
        if ( ROM_STATUS & bROM_ADDR_OK ) {                                     // 操作地址有效
            ROM_CTRL = ROM_CMD_WRITE;                                           // 写入
        }
        if((ROM_STATUS ^ bROM_ADDR_OK) > 0) return i;                          // 返回状态,0x00=success,  0x02=unknown command(bROM_CMD_ERR)
    }
    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA;                                                           //进入安全模式
    GLOBAL_CFG &= ~bDATA_WE;                                                   //开启DataFlash写保护
    SAFE_MOD = 0;                                                              //退出安全模式
    return i;
}

/*******************************************************************************
* Function Name  : ReadDataFlash(UINT8 data Addr,UINT8 data len,PUINT8 data buf)
* Description    : 读DataFlash
* Input          : UINT8 data Addr UINT8 data len PUINT8 data buf
* Output         : None
* Return         : UINT8 data i 返回写入长度
*******************************************************************************/
UINT8 ReadDataFlash(UINT8 data Addr,UINT8 data len,PUINT8 data buf) {
    UINT8 data i;
    ROM_ADDR_H = DATA_FLASH_ADDR >> 8;
    Addr <<= 1;
    for(i=0; i<len; i++) {
        ROM_ADDR_L = Addr + i*2;                                                   //Addr必须为偶地址
        ROM_CTRL = ROM_CMD_READ;
//     if ( ROM_STATUS & bROM_CMD_ERR ) return( 0xFF );                        // unknown command
        *(buf+i) = ROM_DATA_L;
    }
    return i;
}

UINT8 data i;

void CleanUserBuf2() {
    for(i=0; i<8; i++)                                                   //清空UserBuf2
    {
        UserEp2Buf[i] = 0;
    }
}

#define ROM_CHIP_ID_ADDR 0x3FFC

/*******************************************************************************
* Function Name  : GetChipID(void)
* Description    : 获取ID号和ID号和校验
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
UINT32 GetChipID( void )
{
    UINT8	d0, d1;
    UINT16	xl, xh;
    E_DIS = 1;                                                                  //避免进入中断
    d0 = *(PUINT8C)( ROM_CHIP_ID_ADDR + 0 );
    d1 = *(PUINT8C)( ROM_CHIP_ID_ADDR + 1 );                                    //ID号低字
    xl = ( d1 << 8 ) | d0;
    d0 = *(PUINT8C)( ROM_CHIP_ID_ADDR + 2 );
    d1 = *(PUINT8C)( ROM_CHIP_ID_ADDR + 3 );                                    //ID号高字
    xh = ( d1 << 8 ) | d0;
    E_DIS = 0;
    return( ( (UINT32)xh << 16 ) | xl );
}

bit data colorChangeFlag = 0;

bit invalidROM = 0;

UINT32 xdata chipId;
UINT32 xdata chipIdCheck;
UINT32 xdata invalidROMChecker;

void CheckCmdCode() {
    if(Ep2Buffer[1]) {
        CleanUserBuf2();
        switch (Ep2Buffer[1]) {
            //上位机请求flash
        case 0x01: {
            if(Ep2Buffer[2] == 0x1C) {
                Ep2Buffer[2] = 0x00;
            }
            ReadDataFlash(Ep2Buffer[2]*4,4,UserEp2Buf);
            UserEp2Buf[4] = Ep2Buffer[2];
            Enp2BlukIn();
            break;
        }
        //单片机自身上传指令
        case 0x02: {
            switch(Ep2Buffer[2]) {
            case 0x00: {
                // 发送固件版本
                UserEp2Buf[0] = VERSION_YEAR;
                UserEp2Buf[1] = VERSION_YEAR_S;
                UserEp2Buf[2] = VERSION_MONTH;
                UserEp2Buf[3] = VERSION_DAY;
                Enp2BlukIn();
                break;

            }
            case 0x01: {
                // 发送单片机ID
                UserEp2Buf[0] = (chipId >> 24);// & 0xFF;
                UserEp2Buf[1] = (chipId >> 16);// & 0xFF;
                UserEp2Buf[2] = (chipId >> 8);// & 0xFF;
                UserEp2Buf[3] = chipId;// & 0xFF;
                Enp2BlukIn();
                break;
            }
            }
            break;
        }
        // 直接操控灯光
        case 0x03: {
            switch(Ep2Buffer[2]) {
            case 0x01: {
                // 操作1号灯
                colorToShowR0 = Ep2Buffer[3];
                colorToShowG0 = Ep2Buffer[4];
                colorToShowB0 = Ep2Buffer[5];
                break;

            }
            case 0x02: {
                // 操作2号灯
                colorToShowR1 = Ep2Buffer[3];
                colorToShowG1 = Ep2Buffer[4];
                colorToShowB1 = Ep2Buffer[5];
                break;
            }
            case 0xFF: {
                // 操作所有灯
                colorToShowR0 = Ep2Buffer[3];
                colorToShowG0 = Ep2Buffer[4];
                colorToShowB0 = Ep2Buffer[5];

                colorToShowR1 = Ep2Buffer[3];
                colorToShowG1 = Ep2Buffer[4];
                colorToShowB1 = Ep2Buffer[5];
                break;
            }
            }
            colorChangeFlag = 1;
            break;
        }
        //跳转到IAP刷机模式
        case 0x0B: {
            DataToWrite[0]=hiSpeedMode;
            DataToWrite[1]=0xAA;
            DataToWrite[2]=0x00;
            DataToWrite[3]=0x00;
            WriteDataFlash(0x28,DataToWrite,4);
            //mDelaymS( 1000 );
            EA = 0; //禁用中断
            CH554SoftReset( );
            break;
        }
        }
    }
}

void SendKeyPack() {
    if(sendingFlag) {
        waitToSend = 1;
        return;
    }
    sendingFlag = 1;

    HIDKey[0] = 0x00;
    //HIDKey[1] = 0x00;

    HIDKey[2] = 0x00;
    HIDKey[3] = 0x00;
    HIDKey[4] = 0x00;
    HIDKey[5] = 0x00;
    HIDKey[6] = 0x00;
    //HIDKey[7] = 0x00;

    if(BT1keyState) {
        HIDKey[0] |= activeKeyCode1[0];
        HIDKey[2] = activeKeyCode1[1];
    }
    if(BT2keyState) {
        HIDKey[0] |= activeKeyCode2[0];
        HIDKey[3] = activeKeyCode2[1];
    }
    if(BT3keyState) {
        HIDKey[0] |= keyCode3[0];
        HIDKey[4] = keyCode3[1];
    }
    if(BT4keyState) {
        HIDKey[0] |= keyCode4[0];
        HIDKey[5] = keyCode4[1];
    }
    if(BT5keyState) {
        HIDKey[0] |= keyCode5[0];
        HIDKey[6] = keyCode5[1];
    }
    /*
    if(BT_RollerkeyState) {
    HIDKey[0] |= keyCode_Roller[0];
    HIDKey[6] = keyCode_Roller[1];
    }
    */
    Enp1IntIn();

    sendingFlag = 0;
    if(waitToSend) {
        waitToSend = 0;
        SendKeyPack();
    }
}

bit data MouseOn = 0;

void MouseClick() {
    if(MouseOn == 0) {
        return;
    }
    if(sendingFlagMouse) {
        waitToSendMouse = 1;
        return;
    }
    sendingFlagMouse = 1;

    HIDMouse[0] = 0x08;

    //HIDMouse[1] = 0x00;
    //HIDMouse[2] = 0x00;
    //HIDMouse[3] = 0x00;


    if(BT1keyState) {
        HIDMouse[0] |= activeKeyCode1[2];
    }
    if(BT2keyState) {
        HIDMouse[0] |= activeKeyCode2[2];
    }
    if(BT3keyState) {
        HIDMouse[0] |= keyCode3[2];
    }
    if(BT4keyState) {
        HIDMouse[0] |= keyCode4[2];
    }
    if(BT5keyState) {
        HIDMouse[0] |= keyCode5[2];
    }
    /* if(BT_RollerkeyState) {
        HIDMouse[0] |= keyCode_Roller[2];
    }*/
    Enp3IntIn();

    sendingFlagMouse = 0;
    if(waitToSendMouse) {
        waitToSendMouse = 0;
        MouseClick();
    }
}

void CheckMouseOn() {
    if(activeKeyCode1[2]||activeKeyCode2[2]||keyCode3[2]||keyCode4[2]||keyCode5[2]) {
        MouseOn = 1;
    }
    else {
        MouseOn = 0;
    }
}

//按下按键
/*
void PressKey() {
    SendKeyPack();
    MouseClick();
}
*/
//松开按键
void UpKey() {
    SendKeyPack();
    MouseClick();
}

unsigned int data isChangeColor = 0;

// 公用颜色模式标记
unsigned char data colorsChangeMode;
unsigned char data colorsChangeMode2;

unsigned char data easeOutChangeMode;
unsigned char data easeOutChangeMode2;

unsigned char xdata easeOutInColorLevel0[9];
unsigned char xdata easeOutInColorLevel1[9];

// 共用高速Temp变量
// 用于 渐显渐隐计算 以及 彩虹模式开机时状态计算
unsigned int data temp = 0;
unsigned int data temp2 = 0;

void countEaseInOut() {
    easeOutInColorLevel0[0] = (unsigned int)colorR0 *3 >> 2;
    easeOutInColorLevel0[1] = (unsigned int)colorG0 *3 >> 2;
    easeOutInColorLevel0[2] = (unsigned int)colorB0 *3 >> 2;

    easeOutInColorLevel0[3] = (unsigned int)colorR0 >> 1;
    easeOutInColorLevel0[4] = (unsigned int)colorG0 >> 1;
    easeOutInColorLevel0[5] = (unsigned int)colorB0 >> 1;

    easeOutInColorLevel0[6] = (unsigned int)colorR0    >> 2;
    easeOutInColorLevel0[7] = (unsigned int)colorG0    >> 2;
    easeOutInColorLevel0[8] = (unsigned int)colorB0    >> 2;

    easeOutInColorLevel1[0] = (unsigned int)colorR1 *3 >> 2;
    easeOutInColorLevel1[1] = (unsigned int)colorG1 *3 >> 2;
    easeOutInColorLevel1[2] = (unsigned int)colorB1 *3 >> 2;

    easeOutInColorLevel1[3] = (unsigned int)colorR1 >> 1;
    easeOutInColorLevel1[4] = (unsigned int)colorG1 >> 1;
    easeOutInColorLevel1[5] = (unsigned int)colorB1 >> 1;

    easeOutInColorLevel1[6] = (unsigned int)colorR1    >> 2;
    easeOutInColorLevel1[7] = (unsigned int)colorG1    >> 2;
    easeOutInColorLevel1[8] = (unsigned int)colorB1    >> 2;
}

// void countAndFlashLed() interrupt 5 using 2 {
void countAndFlashLed() {
    // 去抖计算
    if(BT1keyState) {
        BT1keyState--;
        if(BT1keyState == 0) {
            //keyNum = 1;
            UpKey();
        }
    }
    if(BT2keyState) {
        BT2keyState--;
        if(BT2keyState == 0) {
            //keyNum = 2;
            UpKey();
        }
    }
    if(BT3keyState) {
        BT3keyState--;
        if(BT3keyState == 0) {
            //keyNum = 3;
            UpKey();
        }
    }
		if(BT4keyState) {
        BT4keyState--;
        if(BT4keyState == 0) {
            //keyNum = 4;
            UpKey();
        }
    }
    if(BT5keyState) {
        BT5keyState--;
        if(BT5keyState == 0) {
            //keyNum = 5;
            UpKey();
        }
    }
    // 如果处在高速模式，直接返回
    if(hiSpeedMode == 0x01) {
        return;
    }
    switch(lightMode) {
        //彩虹渐变色
    default:
    case 9: // 彩虹，切换
    case 7:
    case 6: {
        isChangeColor++;
        if(isChangeColor == LED_RAINBOW_DELAY) {
            isChangeColor = 0;
            switch(colorsChangeMode) {
            case 0x00: {
                colorG0++;
                // rainbowColorFreqRecalc = 0x01;
                if(colorG0 == 0xFF)colorsChangeMode = 0x01;
                break;
            }
            case 0x01: {
                colorR0--;
                // rainbowColorFreqRecalc = 0x00;
                if(colorR0 == 0x00)colorsChangeMode = 0x02;
                break;
            }
            case 0x02: {
                colorB0++;
                // rainbowColorFreqRecalc = 0x02;
                if(colorB0 == 0xFF)colorsChangeMode = 0x03;
                break;
            }
            case 0x03: {
                colorG0--;
                // rainbowColorFreqRecalc = 0x01;
                if(colorG0 == 0x00)colorsChangeMode = 0x04;
                break;
            }
            case 0x04: {
                colorR0++;
                // rainbowColorFreqRecalc = 0x00;
                if(colorR0 == 0xFF)colorsChangeMode = 0x05;
                break;
            }
            case 0x05: {
                colorB0--;
                // rainbowColorFreqRecalc = 0x02;
                if(colorB0 == 0x00)colorsChangeMode = 0x00;
                break;
            }
            }
            // LED 2
            switch(colorsChangeMode2) {
            case 0x00: {
                colorG1++;
                // rainbowColorFreqRecalc = 0x01;
                if(colorG1 == 0xFF)colorsChangeMode2 = 0x01;
                break;
            }
            case 0x01: {
                colorR1--;
                // rainbowColorFreqRecalc = 0x00;
                if(colorR1 == 0x00)colorsChangeMode2 = 0x02;
                break;
            }
            case 0x02: {
                colorB1++;
                // rainbowColorFreqRecalc = 0x02;
                if(colorB1 == 0xFF)colorsChangeMode2 = 0x03;
                break;
            }
            case 0x03: {
                colorG1--;
                // rainbowColorFreqRecalc = 0x01;
                if(colorG1 == 0x00)colorsChangeMode2 = 0x04;
                break;
            }
            case 0x04: {
                colorR1++;
                // rainbowColorFreqRecalc = 0x00;
                if(colorR1 == 0xFF)colorsChangeMode2 = 0x05;
                break;
            }
            case 0x05: {
                colorB1--;
                // rainbowColorFreqRecalc = 0x02;
                if(colorB1 == 0x00)colorsChangeMode2 = 0x00;
                break;
            }
            }
            // 通知刷新灯光
            colorChangeFlag = 1;
            // 如果模式7 彩虹渐隐，则继续，否则为彩虹，拷贝颜色并显示
            if(lightMode == 0x07) {
                countEaseInOut();
            } else if(lightMode == 0x09) {
                if(led0LightSwitchFlag) {
                    colorToShowR0 = colorR0;
                    colorToShowG0 = colorG0;
                    colorToShowB0 = colorB0;
                } else {
                    colorToShowR0 = 0;
                    colorToShowG0 = 0;
                    colorToShowB0 = 0;
                }
                if(led1LightSwitchFlag) {
                    colorToShowR1 = colorR1;
                    colorToShowG1 = colorG1;
                    colorToShowB1 = colorB1;
                } else {
                    colorToShowR1 = 0;
                    colorToShowG1 = 0;
                    colorToShowB1 = 0;
                }
                // break;
            } else {
                colorToShowR0 = colorR0;
                colorToShowG0 = colorG0;
                colorToShowB0 = colorB0;
                colorToShowR1 = colorR1;
                colorToShowG1 = colorG1;
                colorToShowB1 = colorB1;
                // break;
            }
        }
        if(lightMode == 0x06 || lightMode == 0x09) {
            // 如果模式6 彩虹，则跳出 switch
            break;
        }
    }
    // 按下熄灭，然后渐显
    // case 1:
    // 按下点亮，然后渐隐
    case 0: {
        /*
        if(lightMode != 0x07) {
        isChangeColor++;
        if(isChangeColor == LED_CHANGE_DELAY) {
            colorChangeFlag = 1;
            isChangeColor = 0;
        }
        }
        */
        if(BT1keyState) {
            if(easeOutChangeMode != 0x10) {
                easeOutChangeMode = 0x10;
                colorChangeFlag = 1;
                colorToShowR0 = colorR0;
                colorToShowG0 = colorG0;
                colorToShowB0 = colorB0;
            }
            else if(colorChangeFlag) {
                colorToShowR0 = colorR0;
                colorToShowG0 = colorG0;
                colorToShowB0 = colorB0;
            }
        }
        else if(easeOutChangeMode && BT1keyState == 0) {
            temp++;
            if(easeOutChangeMode >= 0x9F) {
                easeOutChangeMode = 0;
            }
            if(temp == LED_EASE_DELAY) {
                temp = 0;
                easeOutChangeMode += 0x01;
                if(easeOutChangeMode & 0x0F == 0x0F) {
                    easeOutChangeMode &= 0xF0;
                    easeOutChangeMode += 0x10;
                    switch(easeOutChangeMode >> 4) {
                    case 0x02: {
                        colorToShowR0 = ((unsigned int)colorR0 + (unsigned int)easeOutInColorLevel0[0]) >> 1;
                        colorToShowG0 = ((unsigned int)colorG0 + (unsigned int)easeOutInColorLevel0[1]) >> 1;
                        colorToShowB0 = ((unsigned int)colorB0 + (unsigned int)easeOutInColorLevel0[2]) >> 1;
                        break;
                    }
                    case 0x03: {
                        colorToShowR0 = easeOutInColorLevel0[0];
                        colorToShowG0 = easeOutInColorLevel0[1];
                        colorToShowB0 = easeOutInColorLevel0[2];
                        break;
                    }
                    case 0x04: {
                        colorToShowR0 = ((unsigned int)easeOutInColorLevel0[0] + (unsigned int)easeOutInColorLevel0[3]) >> 1;
                        colorToShowG0 = ((unsigned int)easeOutInColorLevel0[1] + (unsigned int)easeOutInColorLevel0[4]) >> 1;
                        colorToShowB0 = ((unsigned int)easeOutInColorLevel0[2] + (unsigned int)easeOutInColorLevel0[5]) >> 1;
                        break;
                    }
                    case 0x05: {
                        colorToShowR0 = easeOutInColorLevel0[3];
                        colorToShowG0 = easeOutInColorLevel0[4];
                        colorToShowB0 = easeOutInColorLevel0[5];
                        break;
                    }
                    case 0x06: {
                        colorToShowR0 = ((unsigned int)easeOutInColorLevel0[3] + (unsigned int)easeOutInColorLevel0[6]) >> 1;
                        colorToShowG0 = ((unsigned int)easeOutInColorLevel0[4] + (unsigned int)easeOutInColorLevel0[7]) >> 1;
                        colorToShowB0 = ((unsigned int)easeOutInColorLevel0[5] + (unsigned int)easeOutInColorLevel0[8]) >> 1;
                        break;
                    }
                    case 0x07: {
                        colorToShowR0 = easeOutInColorLevel0[6];
                        colorToShowG0 = easeOutInColorLevel0[7];
                        colorToShowB0 = easeOutInColorLevel0[8];
                        break;
                    }
                    case 0x08: {
                        colorToShowR0 = easeOutInColorLevel0[6] >> 1;
                        colorToShowG0 = easeOutInColorLevel0[7] >> 1;
                        colorToShowB0 = easeOutInColorLevel0[8] >> 1;
                        break;
                    }
                    case 0x09: {
                        colorToShowR0 = 0;
                        colorToShowG0 = 0;
                        colorToShowB0 = 0;
                        break;
                    }
                    }
                    colorChangeFlag = 1;
                }
            }
        }
        if(BT2keyState) {
            if(easeOutChangeMode2 != 0x10) {
                easeOutChangeMode2 = 0x10;
                colorChangeFlag = 1;
                colorToShowR1 = colorR1;
                colorToShowG1 = colorG1;
                colorToShowB1 = colorB1;
            } else if(colorChangeFlag) {
                colorToShowR1 = colorR1;
                colorToShowG1 = colorG1;
                colorToShowB1 = colorB1;
            }
        } else if(easeOutChangeMode2 && BT2keyState == 0) {
            temp2++;
            if(easeOutChangeMode2 >= 0x9F) {
                easeOutChangeMode2 = 0;
            }
            if(temp2 == LED_EASE_DELAY) {
                temp2 = 0;
                easeOutChangeMode2 += 0x01;
                if(easeOutChangeMode2 & 0x0F == 0x0F) {
                    easeOutChangeMode2 &= 0xF0;
                    easeOutChangeMode2 += 0x10;
                    switch(easeOutChangeMode2 >> 4) {
                    case 0x02: {
                        colorToShowR1 = ((unsigned int)colorR1 + (unsigned int)easeOutInColorLevel1[0]) >> 1;
                        colorToShowG1 = ((unsigned int)colorG1 + (unsigned int)easeOutInColorLevel1[1]) >> 1;
                        colorToShowB1 = ((unsigned int)colorB1 + (unsigned int)easeOutInColorLevel1[2]) >> 1;
                        break;
                    }
                    case 0x03: {
                        colorToShowR1 = easeOutInColorLevel1[0];
                        colorToShowG1 = easeOutInColorLevel1[1];
                        colorToShowB1 = easeOutInColorLevel1[2];
                        break;
                    }
                    case 0x04: {
                        colorToShowR1 = ((unsigned int)easeOutInColorLevel1[0] + (unsigned int)easeOutInColorLevel1[3]) >> 1;
                        colorToShowG1 = ((unsigned int)easeOutInColorLevel1[1] + (unsigned int)easeOutInColorLevel1[4]) >> 1;
                        colorToShowB1 = ((unsigned int)easeOutInColorLevel1[2] + (unsigned int)easeOutInColorLevel1[5]) >> 1;
                        break;
                    }
                    case 0x05: {
                        colorToShowR1 = easeOutInColorLevel1[3];
                        colorToShowG1 = easeOutInColorLevel1[4];
                        colorToShowB1 = easeOutInColorLevel1[5];
                        break;
                    }
                    case 0x06: {
                        colorToShowR1 = ((unsigned int)easeOutInColorLevel1[3] + (unsigned int)easeOutInColorLevel1[6]) >> 1;
                        colorToShowG1 = ((unsigned int)easeOutInColorLevel1[4] + (unsigned int)easeOutInColorLevel1[7]) >> 1;
                        colorToShowB1 = ((unsigned int)easeOutInColorLevel1[5] + (unsigned int)easeOutInColorLevel1[8]) >> 1;
                        break;
                    }
                    case 0x07: {
                        colorToShowR1 = easeOutInColorLevel1[6];
                        colorToShowG1 = easeOutInColorLevel1[7];
                        colorToShowB1 = easeOutInColorLevel1[8];
                        break;
                    }
                    case 0x08: {
                        colorToShowR1 = easeOutInColorLevel1[6] >> 1;
                        colorToShowG1 = easeOutInColorLevel1[7] >> 1;
                        colorToShowB1 = easeOutInColorLevel1[8] >> 1;
                        break;
                    }
                    case 0x09: {
                        colorToShowR1 = 0;
                        colorToShowG1 = 0;
                        colorToShowB1 = 0;
                        break;
                    }
                    }
                    colorChangeFlag = 1;
                }
            }
        }
        break;
    }
    //按下时点亮
    case 5: {
        isChangeColor++;
        if(isChangeColor == LED_CHANGE_DELAY) {
            colorChangeFlag = 1;
        }
        if(BT1keyState && colorsChangeMode == 0) {
            colorsChangeMode = 1;
            colorChangeFlag = 1;
            colorToShowR0 = colorR0;
            colorToShowG0 = colorG0;
            colorToShowB0 = colorB0;
            isChangeColor = 0;
        } else if(colorsChangeMode && BT1keyState == 0) {
            colorsChangeMode = 0;
            colorChangeFlag = 1;
            colorToShowR0 = 0;
            colorToShowG0 = 0;
            colorToShowB0 = 0;
            isChangeColor = 0;
        }
        if(BT2keyState && colorsChangeMode2 == 0) {
            colorsChangeMode2 = 1;
            colorChangeFlag = 1;
            colorToShowR1 = colorR1;
            colorToShowG1 = colorG1;
            colorToShowB1 = colorB1;
            isChangeColor = 0;
        } else if(colorsChangeMode2 && BT2keyState == 0) {
            colorsChangeMode2 = 0;
            colorChangeFlag = 1;
            colorToShowR1 = 0;
            colorToShowG1 = 0;
            colorToShowB1 = 0;
            isChangeColor = 0;
        }
        break;
    }
    //熄灭
    case 3: {
        if(colorsChangeMode == 0 || colorsChangeMode2 == 0) {
            ResetLED();
            mDelaymS(1);
            colorsChangeMode = 1;
            colorsChangeMode2 = 1;
            colorToShowR0 = 0;
            colorToShowG0 = 0;
            colorToShowB0 = 0;
            colorToShowR1 = 0;
            colorToShowG1 = 0;
            colorToShowB1 = 0;
            colorChangeFlag = 1;
        }
        break;
    }
    //常亮
    case 2: {
        // 定期刷新
        // isChangeColor++;
        // if(isChangeColor == LED_CHANGE_DELAY) {
        //    colorChangeFlag = 1;
        //    isChangeColor = 0;
        // }
        if(colorsChangeMode == 0 || colorsChangeMode2 == 0) {
            ResetLED();
            mDelaymS(1);
            colorsChangeMode = 1;
            colorsChangeMode2 = 1;
            colorToShowR0 = colorR0;
            colorToShowG0 = colorG0;
            colorToShowB0 = colorB0;
            colorToShowR1 = colorR1;
            colorToShowG1 = colorG1;
            colorToShowB1 = colorB1;
            colorChangeFlag = 1;
        }
        break;
    }
    // 按键切换开关
    case 8: {
        // 定期刷新
        isChangeColor++;
        if(isChangeColor == LED_CHANGE_DELAY) {
            colorChangeFlag = 1;
            isChangeColor = 0;

            if(led0LightSwitchFlag) {
                colorToShowR0 = colorR0;
                colorToShowG0 = colorG0;
                colorToShowB0 = colorB0;
            } else {
                colorToShowR0 = 0;
                colorToShowG0 = 0;
                colorToShowB0 = 0;
            }

            if(led1LightSwitchFlag) {
                colorToShowR1 = colorR1;
                colorToShowG1 = colorG1;
                colorToShowB1 = colorB1;
            } else {
                colorToShowR1 = 0;
                colorToShowG1 = 0;
                colorToShowB1 = 0;
            }
        }
    }
    }
    if(colorChangeFlag) {
        colorChangeFlag = 0;
        // 刷新灯光
        switch(LED0_brightness) {
        default:
        case 4: {
            SetLED(colorToShowR0,colorToShowG0,colorToShowB0);
            break;
        }
        case 3: {
            SetLED(colorToShowR0 >> 1,colorToShowG0 >> 1,colorToShowB0 >> 1);
            break;
        }
        case 2: {
            SetLED(colorToShowR0 >> 2,colorToShowG0 >> 2,colorToShowB0 >> 2);
            break;
        }
        case 1: {
            SetLED(colorToShowR0 >> 3,colorToShowG0 >> 3,colorToShowB0 >> 3);
            break;
        }
        }
        // SetLED(colorToShowR1,colorToShowG1,colorToShowB1);
        // 刷新灯光
        switch(LED1_brightness) {
        default:
        case 4: {
            SetLED(colorToShowR1,colorToShowG1,colorToShowB1);
            break;
        }
        case 3: {
            SetLED(colorToShowR1 >> 1,colorToShowG1 >> 1,colorToShowB1 >> 1);
            break;
        }
        case 2: {
            SetLED(colorToShowR1 >> 2,colorToShowG1 >> 2,colorToShowB1 >> 2);
            break;
        }
        case 1: {
            SetLED(colorToShowR1 >> 3,colorToShowG1 >> 3,colorToShowB1 >> 3);
            break;
        }
        }
        ShowLED(); //显示
        // 反色
        /* if(lightMode == 0x01) {
            ShowLED(~colorToShowG0,~colorToShowR0,~colorToShowB0);
            ShowLED(~colorToShowG1,~colorToShowR1,~colorToShowB1);
        } */

        ResetLED();
    }
}


//扫描按键
void KeyScan() {
    if(BT1 == 0) {
        if(BT1keyState == 0) {
            Ep1Buffer[0] |= activeKeyCode1[0];
            Ep1Buffer[2] = activeKeyCode1[1];

            UEP1_T_LEN = 0x08;                                             //上传数据长度
            UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                //有数据时上传数据并应答ACK

            if(MouseOn) {
                Ep3Buffer[0] |= activeKeyCode1[2];
                UEP3_T_LEN = 0x04;
                UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                //有数据时上传数据并应答ACK
            }
            BT1keyState = KEY_DELAY;
            led0LightSwitchFlag = ~led0LightSwitchFlag;
        } else if(BT1keyState > KEY_DELAY_MIN) {
            // BT1keyState--;
        } else {
            BT1keyState = KEY_DELAY_MIN;
        }
    }
    if(BT2 == 0) {
        if(BT2keyState == 0) {
            Ep1Buffer[0] |= activeKeyCode2[0];
            Ep1Buffer[3] = activeKeyCode2[1];

            UEP1_T_LEN = 0x08;                                             //上传数据长度
            UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                //有数据时上传数据并应答ACK

            if(MouseOn) {
                Ep3Buffer[0] |= activeKeyCode2[2];
                UEP3_T_LEN = 0x04;
                UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                //有数据时上传数据并应答ACK
            }
            BT2keyState = KEY_DELAY;

            led1LightSwitchFlag = ~led1LightSwitchFlag;
        } else if(BT2keyState > KEY_DELAY_MIN) {
            // BT2keyState--;
        } else {
            BT2keyState = KEY_DELAY_MIN;
        }
    }
    if(BT3 == 0) {
        if(BT3keyState == 0) {
            // 键盘鼠标功能
            Ep1Buffer[0] |= keyCode3[0];
            Ep1Buffer[4] = keyCode3[1];

            UEP1_T_LEN = 0x08;                                             //上传数据长度
            UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                //有数据时上传数据并应答ACK

            if(MouseOn) {
                Ep3Buffer[0] |= keyCode3[2];
                UEP3_T_LEN = 0x04;
                UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                //有数据时上传数据并应答ACK
            }
            // 当按键3为按键1、2模式切换按钮时
            if(keyCode3[3]) {
                // 切换按键功能
                if(keyCode3[3] == 0x01) {
                    activeKeyCode1[0] = keyCode1G2[0];
                    activeKeyCode1[1] = keyCode1G2[1];
                    activeKeyCode1[2] = keyCode1G2[2];

                    activeKeyCode2[0] = keyCode2G2[0];
                    activeKeyCode2[1] = keyCode2G2[1];
                    activeKeyCode2[2] = keyCode2G2[2];

                    keyCode3[3] = 0x02;
                } else if(keyCode3[3] == 0x02) {
                    activeKeyCode1[0] = keyCode1[0];
                    activeKeyCode1[1] = keyCode1[1];
                    activeKeyCode1[2] = keyCode1[2];

                    activeKeyCode2[0] = keyCode2[0];
                    activeKeyCode2[1] = keyCode2[1];
                    activeKeyCode2[2] = keyCode2[2];

                    keyCode3[3] = 0x01;
                }

                CheckMouseOn();
            }
            BT3keyState = KEY_DELAY;
        } else if(BT3keyState > KEY_DELAY_MIN) {
            // BT3keyState--;
        } else {
            BT3keyState = KEY_DELAY_MIN;
        }
    }
		if(BT4 == 0) {
        if(BT4keyState == 0) {
            Ep1Buffer[0] |= keyCode4[0];
            Ep1Buffer[5] = keyCode4[1];

            UEP1_T_LEN = 0x08;                                             //上传数据长度
            UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                //有数据时上传数据并应答ACK

            if(MouseOn) {
                Ep3Buffer[0] |= keyCode4[2];
                UEP3_T_LEN = 0x04;
                UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                //有数据时上传数据并应答ACK
            }
						BT4keyState = KEY_DELAY;
        } else if(BT4keyState > KEY_DELAY_MIN) {
            // BT3keyState--;
        } else {
            BT4keyState = KEY_DELAY_MIN;
        }
    }
    if(BT5 == 0) {
        if(BT5keyState == 0) {
            Ep1Buffer[0] |= keyCode5[0];
            Ep1Buffer[6] = keyCode5[1];

            UEP1_T_LEN = 0x08;                                             //上传数据长度
            UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                //有数据时上传数据并应答ACK

            if(MouseOn) {
                Ep3Buffer[0] |= keyCode5[2];
                UEP3_T_LEN = 0x04;
                UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                //有数据时上传数据并应答ACK
            }
						BT5keyState = KEY_DELAY;
        } else if(BT5keyState > KEY_DELAY_MIN) {
            // BT3keyState--;
        } else {
            BT5keyState = KEY_DELAY_MIN;
        }
    }
    return;
}

//扫描按键
void KeyScanForLight() {
    if(BT1 == 0) {
        if(BT1keyState == 0) {
            led0LightSwitchFlag =~led0LightSwitchFlag;
        }
        BT1keyState = KEY_DELAY;
    }
    if(BT2 == 0) {
        if(BT2keyState == 0) {
            led1LightSwitchFlag =~led1LightSwitchFlag;
        }
        BT2keyState = KEY_DELAY;
    }
    return;
}

unsigned char data readBuff[4];

/*******************************************************************************
* Function Name  : mTimer_x_SetData(UINT8 x,UINT16 dat)
* Description    : CH554Timer0 TH0和TL0赋值
* Input          : UINT16 dat;定时器赋值
* Output         : None
* Return         : None
*******************************************************************************/
/*
void mTimer_2_SetData(UINT16 dat)
{
    UINT16 tmp;
    tmp = 0xFFFF - dat;
    RCAP2L = TL2 = tmp & 0xff;                                               //16位自动重载定时器
    RCAP2H = TH2 = (tmp>>8) & 0xff;
}
*/

/*
 * 0x04 按键1
 * 0x08 按键2
 * 0x0C 按键3 按键切换开关
 * 0x10 按键4 // Nano 不包含
 * 0x14 按键5 // Nano 不包含
 * 0x18 灯光组1
 * 0x1C 灯光组2
 * 0x20 灯光模式
 * 0x24 去抖设定
 * 0x28 急速模式 IAP入口 夜灯开关
 * 0x2C 灯光延迟设定
 * 0x30 按键1 切换模式2
 * 0x34 按键2 切换模式2
 * 0x38 按键1 切换模式3 // Nano 不包含
 * 0x3C 按键2 切换模式3 // Nano 不包含
 *
 * 0x70 芯片验证
 */

void refreshKeyCode() {
    //ReadDataFlash(0x00,14,DataToWrite);
    ReadDataFlash(0x04,3,keyCode1);
    ReadDataFlash(0x08,3,keyCode2);

    ReadDataFlash(0x04,3,activeKeyCode1);
    ReadDataFlash(0x08,3,activeKeyCode2);

    ReadDataFlash(0x0C,4,keyCode3); // 第4字节用于存储按键模式

    /* if(keyCode3[3]) {
        // 如果按键3处于按键切换模式，则清空按键配置
        keyCode3[0] = 0x00;
        keyCode3[1] = 0x00;
        keyCode3[2] = 0x00;
    }*/

    ReadDataFlash(0x10,3,keyCode4);
    ReadDataFlash(0x14,3,keyCode5);

    ReadDataFlash(0x30,3,keyCode1G2);
    ReadDataFlash(0x34,3,keyCode2G2);

    // ReadDataFlash(0x38,3,keyCode1G3);
    // ReadDataFlash(0x3C,3,keyCode2G3);

    CheckMouseOn(); // 检查是否需要开启鼠标兼容

    // 灯光组1
    ReadDataFlash(0x18,4,readBuff);
    colorR0 = readBuff[0];
    colorG0 = readBuff[1];
    colorB0 = readBuff[2];
    LED0_brightness = readBuff[3];
    if(LED0_brightness == 0x00 || LED0_brightness > 0x04) {
        LED0_brightness = 0x04;
    }
    // 灯光组2
    ReadDataFlash(0x1c,4,readBuff);
    colorR1 = readBuff[0];
    colorG1 = readBuff[1];
    colorB1 = readBuff[2];
    LED1_brightness = readBuff[3];
    if(LED1_brightness == 0x00 || LED1_brightness > 0x04) {
        LED1_brightness = 0x04;
    }
    // 灯光模式
    ReadDataFlash(0x20,1,readBuff);
    lightMode = readBuff[0];
    // lightMode = 0x06;
    // 灯光模式额外配置
    if(lightMode == 0x06 || lightMode == 0x07 || lightMode == 0x09) {
        // 彩虹模式
        // 这里当临时变量用而已
        // 找到颜色最浅的那个
        temp = colorR0;
        temp2 = 0;
        if(colorG0<temp) {
            temp = colorG0;
            temp2 = 1;
        }
        if(colorB0<temp) {
            temp = colorB0;
            temp2 = 2;
        }
        switch(temp2) {
        case 0:
            colorR0 = 0;
            if(colorG0 < colorB0) {
                colorB0 = 0xFF;
                if(colorG0 ==0) {
                    colorG0++;
                }
                colorsChangeMode = 0x03;

            } else {
                colorG0 = 0xFF;
                if(colorB0 == 0xFF) {
                    colorB0--;
                }
                colorsChangeMode = 0x02;
            }
            break;
        case 1:
            colorG0 = 0;
            if(colorB0 < colorR0) {
                colorR0 = 0xFF;
                if(colorB0 == 0) {
                    colorB0++;
                }
                colorsChangeMode = 0x05;
            } else {
                colorB0 = 0xFF;
                if(colorR0 == 0xFF) {
                    colorR0--;
                }
                colorsChangeMode = 0x04;
            }
            break;
        case 2:
            colorB0 = 0;
            if(colorR0 < colorG0) {
                colorG0 = 0xFF;
                if(colorR0 == 0) {
                    colorR0++;
                }
                colorsChangeMode = 0x01;
            } else {
                colorR0 = 0xFF;
                if(colorG0 == 0xFF) {
                    colorG0--;
                }
                colorsChangeMode = 0x00;
            }
            break;
        }
        temp = colorR1;
        temp2 = 0;
        if(colorG1<temp) {
            temp = colorG1;
            temp2 = 1;
        }
        if(colorB1<temp) {
            temp = colorB1;
            temp2 = 2;
        }
        switch(temp2) {
        case 0:
            colorR1 = 0;
            if(colorG1 < colorB1) {
                colorB1 = 0xFF;
                if(colorG1 ==0) {
                    colorG1++;
                }
                colorsChangeMode2 = 0x03;
            } else {
                colorG1 = 0xFF;
                if(colorB1 == 0xFF) {
                    colorB1--;
                }
                colorsChangeMode2 = 0x02;
            }
            break;
        case 1:
            colorG1 = 0;
            if(colorB1 < colorR1) {
                colorR1 = 0xFF;
                if(colorB1 == 0) {
                    colorB1++;
                }
                colorsChangeMode2 = 0x05;
            } else {
                colorB1 = 0xFF;
                if(colorR1 == 0xFF) {
                    colorR1--;
                }
                colorsChangeMode2 = 0x04;
            }
            break;
        case 2:
            colorB1 = 0;
            if(colorR1 < colorG1) {
                colorG1 = 0xFF;
                if(colorR1 == 0) {
                    colorR1++;
                }
                colorsChangeMode2 = 0x01;
            } else {
                colorR1 = 0xFF;
                if(colorG1 == 0xFF) {
                    colorG1--;
                }
                colorsChangeMode2 = 0x00;
            }
            break;
        }
        // 拷贝颜色设置
        colorToShowR0 = colorR0;
        colorToShowG0 = colorG0;
        colorToShowB0 = colorB0;
        colorToShowR1 = colorR1;
        colorToShowG1 = colorG1;
        colorToShowB1 = colorB1;
        // 用毕清零
        temp = 0;
        temp2 = 0;

        countEaseInOut();
    } else if(lightMode == 0x00 || lightMode == 0x01) {
        // 渐显 or 渐隐
        countEaseInOut();
        easeOutChangeMode = 0;
        easeOutChangeMode2 = 0;
    }
    else {
        // 其他模式下清零
        colorsChangeMode = 0;
        colorsChangeMode2 = 0;
    }
    // 清空颜色设置
    colorToShowR0 = 0;
    colorToShowG0 = 0;
    colorToShowB0 = 0;
    colorToShowR1 = 0;
    colorToShowG1 = 0;
    colorToShowB1 = 0;
    // 申请刷新灯光
    colorChangeFlag = 1;

    // 去抖设置
    ReadDataFlash(0x24,4,readBuff);
    KEY_DELAY =(int)readBuff[1]*0x010000+(int)readBuff[2]*0x0100+(int)readBuff[3];
    if(KEY_DELAY < 0x10) {
        KEY_DELAY = 0x10;
    }
    /* KEY_DELAY_MIN = KEY_DELAY >> 2;
    if(KEY_DELAY_MIN < 0x10) {
        KEY_DELAY_MIN = 0x10;
    }
		*/
		KEY_DELAY_MIN = KEY_DELAY;
    ReadDataFlash(0x28,4,readBuff);
    // 急速模式
    hiSpeedMode = readBuff[0];
    IAPflag = readBuff[1];
    if(IAPflag == 0xFF) {
        IAPflag = 0x00;
    }
    // 夜灯开关
    isDisableOfflineLight = readBuff[2];
    if(isDisableOfflineLight) {
        CfgDesc[7] = 0x80; // 关闭唤醒供电模式
    }
    // ReadDataFlash(0x2C,3,BT_RollerkeyState);

    // 灯光延迟
    ReadDataFlash(0x2C,4,readBuff);
    LED_EASE_DELAY = (UINT16)readBuff[0] << 8 | readBuff[1]; // 渐隐变色延迟
    LED_RAINBOW_DELAY =  (UINT16)readBuff[2] << 8 | readBuff[3]; // 彩虹速度延迟
    if(LED_RAINBOW_DELAY == 0) {
        LED_RAINBOW_DELAY = LED_CHANGE_DELAY;
    }
    if(LED_EASE_DELAY == 0) {
        LED_EASE_DELAY = LED_CHANGE_DELAY;
    }

    // 芯片验证
    ReadDataFlash(0x70,4,readBuff);

    chipIdCheck = ((chipId & 0x000000FF) << 24) + ((chipId & 0x0000FF00) << 8) + ((chipId & 0x00FF0000) >> 8) + ((chipId & 0xFF000000) >> 24);
    chipIdCheck = ~chipIdCheck;

    invalidROMChecker =(UINT32)readBuff[0]*0x01000000 + (UINT32)readBuff[1]*0x010000+(UINT32)readBuff[2]*0x0100+(UINT32)readBuff[3];

		// 魔法数字自动初始化
		if(invalidROMChecker == 0x07355608){
			DataToWrite[0] = (UINT8)((chipIdCheck & 0xFF000000) >> 24);
			DataToWrite[1] = (UINT8)((chipIdCheck & 0x00FF0000) >> 16);
			DataToWrite[2] = (UINT8)((chipIdCheck & 0x0000FF00) >> 8);
			DataToWrite[3] = (UINT8)(chipIdCheck & 0x000000FF);
			WriteDataFlash(0x70,DataToWrite,4);
			SetLED(0x40,0x40,0x40);
      SetLED(0x40,0x40,0x40);
      ShowLED(); // 显示
      ResetLED();
			mDelaymS(1000);
			
			// 重启设备
			SAFE_MOD = 0x55;
      SAFE_MOD = 0xAA;
      GLOBAL_CFG |=bSW_RESET;
			// chipIdCheck = 0x07355608;
		}

    // chipId = chipIdCheck; // DEBUG

    if(chipIdCheck == invalidROMChecker) {
        invalidROM = 0;
    } else {
			  // invalidROM = 0;
        invalidROM = 1;
    }
}

main() {
    //LED0 = 0;
    //LED1 = 0;
    //offLED0();
    //offLED1();

    CfgFsys( );                                                           //CH559时钟选择配置
    mDelaymS(10);                                                         //修改主频等待内部晶振稳定,必加
	
		// TypeC_UPF_PDInit();

    tasksArr[0] = (pTaskFn)(IAP_ProgrameStartAddr + 0x00);//设置IAP地址
    // 刷新灯控
    ResetLED();

    mDelaymS(1);

		// USB_C_CTRL &= ~bUCC1_PU1_EN & ~bUCC2_PU1_EN;
		// USB_C_CTRL |= bUCC1_PU0_EN | bUCC2_PU0_EN;
	
    FLAG = 0;
    Ready = 0;

    USBDeviceInit();                                                      //USB设备模式初始化
    EA = 1;                                                               //允许单片机中断

    IP_EX |= bIP_USB;																											//USB中断使用高优先级
    UEP1_T_LEN = 0;                                                       //预使用发送长度一定要清空
    UEP2_T_LEN = 0;                                                       //预使用发送长度一定要清空
    UEP3_T_LEN = 0;                                                       //预使用发送长度一定要清空

    chipId = GetChipID();//读取芯片ID

		SerialNumber[2] = CharToHex(chipId >> 28);
		SerialNumber[4] = CharToHex(chipId >> 24);
		SerialNumber[6] = CharToHex(chipId >> 20);
		SerialNumber[8] = CharToHex(chipId >> 16);
		SerialNumber[10] = CharToHex(chipId >> 12);
		SerialNumber[12] = CharToHex(chipId >> 8);
		SerialNumber[14] = CharToHex(chipId >> 4);
		SerialNumber[16] = CharToHex(chipId);
		
    refreshKeyCode();//读取参数，并验证单片机ID

    if(IAPflag) {
        SetLED(0xFF,0x00,0x00);
        SetLED(0xFF,0x00,0x00);
        ShowLED(); //显示
        DataToWrite[0]=hiSpeedMode;
        DataToWrite[1]=0x00;
        DataToWrite[2]=0x00;
        DataToWrite[3]=0x00;
        WriteDataFlash(0x28,DataToWrite,4);
        //mDelaymS(1000);
        (tasksArr[0])();      	//跳转至IAP程序区
    }

    mInitSTDIO( );                                                        //串口0初始化
#ifdef DE_PRINTF
    printf("start ...\n");
#endif

    ResetLED(); // 重置LED

    /*
        IP |= PX0;
        IP |= PX1; // 外部中断高优先级
    */
    // 定时器2 初始化
    T2MOD |= (bTMR_CLK | bT2_CLK);
    // T2MOD |= 0xC0;
		
		// 看门狗初始值
		WDOG_COUNT = 0x00;
		SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA;                                                           //进入安全模式
    GLOBAL_CFG |= bWDOG_EN;                                                    //启用看门狗
    SAFE_MOD = 0;                                                              //退出安全模式


    C_T2=0;
    RCLK = 0;
    TCLK = 0;
    CP_RL2 = 0;
    // mTimer_2_SetData(0xFFFF);

    // TR2 = 1; //T2定时器启动

    SetLED(0x00,0x00,0x00);
    SetLED(0x00,0x00,0x00);
    ShowLED(); // 显示
    ResetLED();
    mDelaymS(1);

    while(1)
    {
        // 只为亮灯
        if(!isDisableOfflineLight) {
            KeyScanForLight();
            countAndFlashLed();
        }
        mDelayuS( 2 );
        if(Ready)
        {

            /*
            ResetLED();
            mDelaymS(1);
              ShowLED(0x00,0x00,0x00);
            	ShowLED(0x00,0x00,0x00);
            	mDelaymS(1);
            	ResetLED();
            */

            // ET2 = 1; //T2定时器中断开启

            //offLED0();
            //offLED1();
            Ep1Buffer[0] = 0x00;
            Ep3Buffer[0] = 0x08;
            Ep1Buffer[1] = 0x00;
            Ep3Buffer[1] = 0x00;
            Ep1Buffer[2] = 0x00;
            Ep3Buffer[2] = 0x00;
            Ep1Buffer[3] = 0x00;
            Ep3Buffer[3] = 0x00;
            Ep1Buffer[4] = 0x00;
            // Ep3Buffer[4] = 0x00;
            Ep1Buffer[5] = 0x00;
            // Ep3Buffer[5] = 0x00;
            Ep1Buffer[6] = 0x00;
            // Ep3Buffer[6] = 0x00;
            Ep1Buffer[7] = 0x00;
            // Ep3Buffer[7] = 0x00;
            // Ep1Buffer[8] = 0x00;
            // Ep3Buffer[8] = 0x00;
            while(1) { //HIDValueHandle();
                KeyScan();
                countAndFlashLed();
							  WDOG_COUNT = 0x00;
                // KeyScan();
                //CheckCmdCode();
                while(invalidROM) {
								  WDOG_COUNT = 0x00;
								}
            }
        }
        while(invalidROM) {
				  WDOG_COUNT = 0x00;
				}
        // countAndFlashLed();
    }
}
