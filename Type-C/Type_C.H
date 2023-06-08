
#define     TYPE_C_DFP  
// #define     TYPE_C_UFP

#define  DufaultPowerMin  13
#define  DufaultPowerMax  31
#define  Power1_5AMin  36
#define  Power1_5AMax  59
#define  Power3_0AMin  67
#define  Power3_0AMax  104 

#ifdef TYPE_C_DFP
#define DFP_DisableRpUCC1()     {USB_C_CTRL = ~(bUCC1_PU1_EN | bUCC1_PU0_EN);}
#define DFP_DefaultPowerUCC1()  {USB_C_CTRL = USB_C_CTRL & ~bUCC1_PU1_EN | bUCC1_PU0_EN;}
#define DFP_1_5APowerUCC1()     {USB_C_CTRL = USB_C_CTRL | bUCC1_PU1_EN & ~bUCC1_PU0_EN; }
#define DFP_3_0APowerUCC1()     {USB_C_CTRL |= bUCC1_PU1_EN | bUCC1_PU0_EN;}

#define DFP_DisableRpUCC2()     {USB_C_CTRL = ~(bUCC2_PU1_EN | bUCC2_PU0_EN);}
#define DFP_DefaultPowerUCC2()  {USB_C_CTRL = USB_C_CTRL & ~bUCC2_PU1_EN | bUCC2_PU0_EN;}
#define DFP_1_5APowerUCC2()     {USB_C_CTRL = USB_C_CTRL | bUCC2_PU1_EN & ~bUCC2_PU0_EN;}
#define DFP_3_0APowerUCC2()     {USB_C_CTRL |= bUCC2_PU1_EN | bUCC2_PU0_EN;}

#define UCC_Connect_Vlaue  105 

#define	UCC_DISCONNECT	0x00			                                               //�豸δ����
#define UCC1_CONNECT		0x01			                                               //��������
#define	UCC2_CONNECT		0x02			                                               //��������
#define	UCC_CONNECT			0x03			                                               //C1��C2����������5.1K����,�޷��ж���������
/*******************************************************************************
* Function Name  : TypeC_DFP_Init(UINT8 Power)
* Description    : Type-C UPF����ʼ��
* Input          : UINT8 Power
                   0 ��ֹUCC1&2����
                   1 Ĭ�ϵ���
                   2 1.5A
                   3 3.0A		
* Output         : None
* Return         : 1   UCC1
                   2   UCC2
*******************************************************************************/
void TypeC_DFP_Init( UINT8 Power );

/*******************************************************************************
* Function Name  : TypeC_DFP_Channle(void)
* Description    : Type-C DPF���UFP���壬�����Լ�δ������Ѳ���
* Input          : NONE
* Output         : None
* Return         : 0   δ����
                   1   ��������
                   2   ��������
                   3   ���ӣ��޷��ж�����
*******************************************************************************/
UINT8 TypeC_DFP_Insert( void );

#endif

#ifdef TYPE_C_UFP
//CH554 UCC1&2 Rd ����(SS=1)/��ֹ(SS=0)
#define UPF_DisableRd(SS)  (USB_C_CTRL = SS ? (USB_C_CTRL|bUCC1_PD_EN|bUCC2_PD_EN) : 0)
#define UPF_PD_Normal			0x00			                                           //Ĭ�Ϲ�������500mA
#define	UPF_PD_1A5				0x01			                                           //��������1.5mA
#define	UPF_PD_3A					0x02			                                           //��������3mA
#define	UPD_PD_DISCONNECT 0xff			                                           //�豸δ����

/*******************************************************************************
* Function Name  : TypeC_UPF_PDInit()
* Description    : Type-C UPF��ʼ��
* Input          : None
* Output         : None
* Return         : None							 
*******************************************************************************/
void TypeC_UPF_PDInit( void );

/*******************************************************************************
* Function Name  : TypeC_UPF_PDCheck()
* Description    : Type-C UPF���DPF��������
* Input          : None
* Output         : None
* Return         : UINT8 RE    
                   0  defaultPower
                   1  1.5A
                   2  3.0A	
                   0xff ����									 
*******************************************************************************/
UINT8 TypeC_UPF_PDCheck();

#endif