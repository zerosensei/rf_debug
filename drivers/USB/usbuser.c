/********************************** (C) COPYRIGHT *******************************
* File Name          : usb_device.c
* Author             : tech12
* Version            : V1.0
* Date               : 2018/11/12
* Description        :
*******************************************************************************/




/******************************************************************************/

#include "CH58x_common.h"
#include "usbuser.h"
#include "RingBuffer/lwrb.h"
#include "CONFIG.h"


#ifdef CONFIG_USB_DEBUG
#define USB_DEBUG  PRINT
#else
#define USB_DEBUG
#endif

#ifndef USB_DRV_DEBUG
#define USB_DRV_DEBUG   PRINT
#endif

#define DevEP0SIZE              0x40
#define THIS_ENDP0_SIZE         64

__attribute__((aligned(4)))  UINT8 EP0_Databuf[64];
__attribute__((aligned(4)))  UINT8 EP1_Databuf[64];
__attribute__((aligned(4)))  UINT8 EP2_Databuf[64 + 64];    //ep2_out(64)+ep2_in(64)

uint8_t USBtaskID;
uint8_t USB_EP2IN_FLAG = 1;

const UINT8 TAB_USB_CDC_DEV_DES[18] =
{
  0x12,
  0x01,
  0x10,
  0x01,                 //BCD�� USB�淶�汾����

  0x02,                 //�����
  0x00,                 //�������
  0x00,                 //Э����

  THIS_ENDP0_SIZE,      //�˵�0�������Ϣ����С
  0x86, 0x1a,           //����ID
  0xD3, 0x55,           //��ƷID  e041

  //0x00, 0x30, //BCD �豸�汾��
  0x00, 0x40, //BCD �豸�汾��

  0x01,                 //�����ߵ��ַ���������������ֵ
  0x02,                 //��Ʒ���ַ���������������ֵ
  0x03,                 //��ŵ��ַ���������������ֵ

  0x01                  //�������õ���Ŀ
};

/* ���������� */
const UINT8 TAB_USB_CDC_CFG_DES[ ] =
{
    0x09,   /* ���������� */
    0x02,

    //0x8d, 0x00, //�����ô��ص��������ݴ�С
    //0x04,   //������֧�ֵĽӿ���Ŀ /* 4���ӿ� */
  //0x4B, 0x00, //�����ô��ص��������ݴ�С
  0x43, 0x00, //�����ô��ص��������ݴ�С
  0x02,   //������֧�ֵĽӿ���Ŀ /* 4���ӿ� */

  0x01,       //Set_Configuration��Set_Configuration����ı�ʶ
  0x00,       //�����õ��ַ���������
  //0xa0,       //�����Դ�Լ�Զ�̻�������
  0x80,   //E0
  0x30,       //��Ҫ���ߵ�ԴmA

  //09 04 00 00 01 02 02 01 00
  //����Ϊ�ӿ�0��CDC�ӿڣ�������
  0x09,       //CDC�ӿ�������(һ���˵�)
  0x04,
  0x00,       //ʶ��˽ӿڵ�����
  0x00,       //����ѡ��һ��������õ���ֵ
  0x01,       //���˶˵�0��֧�ֵĶ˵���
  0x02,       //����� -- Communication Interface Class 02h
  0x02,       //������� -- Abstract Control Model 02h
  0x01,       //Э���� -- V.25ter 01h (Common AT commands (also known as ��Hayes? compatible��))
  0x00,       //�˽ӿڵ��ַ�������������ֵ


//  //����Ϊ���������� Functional Descriptors
//    0x05,  //����//����������(ͷ)
//  0x24,  //CS_INTERFACE
//  0x00,  //Header Functional Descriptor
//  0x10,0x01, //bcdCDC
//
//
//  0x05,  //����//����������(������ӿ�1)
//  0x24,  //CS_INTERFACE
//  0x01,  //Call Management Functional Descriptor
//  //D7..D2: RESERVED (Reset to zero)
//  //D1:0 ��������ֻ����Communication Class interface
//  //D0:0 Device does not handle call management itself
//  0x00,
//  0x01, //Interface number of Data Class interfaceoptionally used for call management
//
//
//  0x04,  //����//֧��Set_Line_Coding��Set_Control_Line_State��Get_Line_Coding��Serial_State
//  0x24,  //CS_INTERFACE
//  0x02,  //Abstract Control Management Functional Descriptor
//  0x02,  //λ2��1 ��ʾ֧��Set_Line_Coding��Set_Control_Line_State��Get_Line_Coding��Serial_State
//
//
//  0x05,  //����//���Ϊ0��CDC�ӿ�;���1��������ӿ�
//  0x24,  //CS_INTERFACE
//  0x06,  //Union Functional descriptor
//  0x00,  //
//  0x01,  //Interface number of first slave or associated interface in the union

  0x05,0x24,0x00,0x10,0x01,
  0x05,0x24,0x01,0x00,0x01,
  0x04,0x24,0x02,0x02,
  0x05,0x24,0x06,0x00,0x01,
//  0x05,0x24,0x01,0x01,0x00,


//  0x05,0x24,0x00,0x10,0x01,
//  0x05,0x24,0x01,0x00,0x01,
//  0x04,0x24,0x02,0x02,
//  0x05,0x24,0x06,0x00,0x01,


  //0x07,0x05,0x84,0x03,0x08,0x00,0x01,                       //�ж��ϴ��˵�������
  //0x07,0x05,0x84,0x03,0x40,0x00,0x01,                       //�ж��ϴ��˵������� �˵�4
  0x07,0x05,0x81,0x03,0x40,0x00,0x01,                       //�ж��ϴ��˵������� �˵�1

  //09 04 01 00 02 0a 00 00 00
  //����Ϊ�ӿ�1�����ݽӿڣ�������
  0x09,   //���ݽӿ�������
  0x04,
  0x01,   //ʶ��˽ӿڵ�����
  0x00,   //����ѡ��һ��������õ���ֵ
  0x02,   //���˶˵�0��֧�ֵĶ˵���
  0x0a,   //����� -- Data Interface Class code  0ah
  0x00,   //������� -- Data Class SubClass code 00h
  0x00,   //Э���� -- USB specification  00h
  0x00,   //�˽ӿڵ��ַ�������������ֵ

//  0x07,0x05,0x01,0x02,0x40,0x00,0x00,                       //�˵������� �˵�1
//  0x07,0x05,0x81,0x02,0x40,0x00,0x00,                       //�˵������� �˵�1

  //0x07,0x05,0x02,0x02,0x40,0x00,0x00,                       //�˵������� �˵�2
  0x07,0x05,0x02,0x02,0x40,0x00,0x00,                       //�˵������� �˵�2
  0x07,0x05,0x82,0x02,0x40,0x00,0x00,                       //�˵������� �˵�2
};

/* �豸�޶������� */
const UINT8 My_QueDescr[ ] = { 0x0A, 0x06, 0x00, 0x02, 0xFF, 0x00, 0xFF, 0x40, 0x01, 0x00 };

UINT8 TAB_CDC_LINE_CODING[ ]  =
{
    0x85, /* baud rate*/
  0x20,
  0x00,
  0x00,
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* no. of bits 8*/
};


// ����������
const UINT8 TAB_USB_LID_STR_DES[ ] = { 0x04, 0x03, 0x09, 0x04 };

UINT8 DevConfig, Ready;
UINT8 SetupReqCode;
UINT16 SetupReqLen;
const UINT8 *pDescr;

//uint8_t usbout2_flag;
//uint8_t usbout2_len;

uint8_t Txlwrbfull_flag;
uint8_t Rxlwrbfull_flag = 0;
uint8_t Ep2InFlag = true;

INT32 set_bps;
UINT8  data_bit;
UINT8  stop_bit;
UINT8  ver_bit;
/*******************************************************************************
* Function Name  : USB_IRQHandler
* Description    : USB�жϷ������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
__attribute__((section(".highcode")))
void USB_IRQHandler(void)
{
  UINT8 len, chtype;
  UINT8 intflag, errflag = 0;

  intflag = R8_USB_INT_FG;
  if ( intflag & RB_UIF_TRANSFER )
  {
    if ( ( R8_USB_INT_ST & MASK_UIS_TOKEN ) != MASK_UIS_TOKEN )    // �ǿ���
    {
      switch ( R8_USB_INT_ST & ( MASK_UIS_TOKEN | MASK_UIS_ENDP ) )
      // �����������ƺͶ˵��
      {
        case UIS_TOKEN_IN :
        {
            USB_DEBUG("SetupReqCode:%#x \n",SetupReqCode);
          switch ( SetupReqCode )
          {
            case USB_GET_DESCRIPTOR :
              len = SetupReqLen >= DevEP0SIZE ?
                  DevEP0SIZE : SetupReqLen;    // ���δ��䳤��
              memcpy( pEP0_DataBuf, pDescr, len ); /* �����ϴ����� */
              SetupReqLen -= len;
              pDescr += len;
              R8_UEP0_T_LEN = len;
              R8_UEP0_CTRL ^= RB_UEP_T_TOG;                             // ��ת
              break;
            case USB_SET_ADDRESS :
              R8_USB_DEV_AD = ( R8_USB_DEV_AD & RB_UDA_GP_BIT ) | SetupReqLen;
              R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
              break;

            case DEF_SET_LINE_CODING:
            {
                USB_DEBUG("SET LINE\n");
              R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK|UEP_T_RES_ACK;
            }
            default :
              R8_UEP0_T_LEN = 0;                                      // ״̬�׶�����жϻ�����ǿ���ϴ�0�������ݰ��������ƴ���
              R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
              break;
          }
        }
          break;

        case UIS_TOKEN_OUT :
        {
          len = R8_USB_RX_LEN;
          if(len)
          {
            switch(SetupReqCode)
            {
              /* ���ô��� */
              case DEF_SET_LINE_CODING:
              {
                  USB_DEBUG("SET_LINE\n");


                //UINT8  set_stop_bit;

//                tmos_memcpy(&set_bps,EP0_Databuf,4);
//                stop_bit = EP0_Databuf[4];
//                ver_bit = EP0_Databuf[5];
//                data_bit = EP0_Databuf[6];
//
//                LOG_INFO("LINE_CODING %d %d %d %d\r\n"
//                                     ,(int)set_bps
//                                     ,data_bit
//                                     ,stop_bit
//                                     ,ver_bit);
                R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK|UEP_T_RES_ACK;
                break;
              }
              default:
              {
                R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
              }
                break;
            }
          }
          else
          {
            R8_UEP0_CTRL = RB_UEP_R_TOG|RB_UEP_T_TOG|UEP_R_RES_NAK | UEP_T_RES_NAK;
          }
        }
          break;

        case UIS_TOKEN_OUT | 1 :
        {
          if ( R8_USB_INT_ST & RB_UIS_TOG_OK )
          {                       // ��ͬ�������ݰ�������
            len = R8_USB_RX_LEN;
            DevEP1_OUT_Deal( len );

            R8_UEP1_CTRL = (R8_UEP1_CTRL & ~MASK_UEP_T_RES)| UEP_T_RES_ACK;
          }
        }
          break;

        case UIS_TOKEN_IN | 1 :
          R8_UEP1_CTRL = ( R8_UEP1_CTRL & ~MASK_UEP_T_RES ) | UEP_T_RES_NAK;
          break;

        case UIS_TOKEN_OUT | 2 :
        {
          if ( R8_USB_INT_ST & RB_UIS_TOG_OK )
          {                       // ��ͬ�������ݰ�������
              //TODO: out ������

//            usbout2_len = R8_USB_RX_LEN;
//            usbout2_flag = 1;
//            R8_UEP2_CTRL = (R8_UEP2_CTRL & 0xf3) | 0x08; //OUT_NAK
          }
        }
          break;

        case UIS_TOKEN_IN | 2 :
        {
            Ep2InFlag = 1;
            R8_UEP2_CTRL = ( R8_UEP2_CTRL & ~MASK_UEP_T_RES ) | UEP_T_RES_NAK;
        }break;

        default :
          break;
      }
      R8_USB_INT_FG = RB_UIF_TRANSFER;
    }
    if ( R8_USB_INT_ST & RB_UIS_SETUP_ACT )                  // Setup������
    {
      R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
      SetupReqLen = pSetupReqPak->wLength;
      SetupReqCode = pSetupReqPak->bRequest;
      chtype = pSetupReqPak->bRequestType;

      len = 0;
      errflag = 0;
      /* �������� */
     if( ( pSetupReqPak->bRequestType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_VENDOR )
     {
         USB_DEBUG("VENDOR:%#x\n",SetupReqCode);
//       switch(SetupReqCode)
//       {
//         case DEF_VEN_DEBUG_WRITE:  //д���� 0X9A
//         {
//           UINT32 bps = 0;
//           UINT8 write_reg_add1;
//           UINT8 write_reg_add2;
//           UINT8 write_reg_val1;
//           UINT8 write_reg_val2;
//
//           len = 0;
////           usb_work_mode = USB_VENDOR_MODE;//�л�������ģʽ
//
//           write_reg_add1 = EP0_Databuf[2];
//           write_reg_add2 = EP0_Databuf[3];
//           write_reg_val1 = EP0_Databuf[4];
//           write_reg_val2 = EP0_Databuf[5];
//
//           /* ���������ò����ʵļĴ��� */
//           if((write_reg_add1 == 0x12)&&(write_reg_add2 == 0x13))
//           {
//             /* �����ʴ�����ü���ֵ */
//             if((pSetupReqPak->wIndexL==0x87)&&(pSetupReqPak->wIndexH==0xf3))
//             {
//               bps = 921600;  //13 * 921600 = 11980800
//             }
//             else if((pSetupReqPak->wIndexL==0x87)&&(pSetupReqPak->wIndexH==0xd9))
//             {
//               bps = 307200;  //39 * 307200 = 11980800
//             }
//
//             //ϵͳ��Ƶ��  32000000
//             else if( pSetupReqPak->wIndexL == 0x8A )
//             {
//               UINT32 CalClock;
//               UINT8 CalDiv;
//
//               CalClock = 32000000 / 8;
//               CalDiv = 0 - UsbSetupBuf->wIndexH;
//               bps = CalClock / CalDiv;
//             }
//             else if( UsbSetupBuf->wIndexL == 0x8B )
//             {
//               UINT32 CalClock;
//               UINT8 CalDiv;
//
//               CalClock = 32000000 / 8 / 256;
//               CalDiv = 0 - UsbSetupBuf->wIndexH;
//               bps = CalClock / CalDiv;
//             }
//             else  //340
//             {
//               UINT32 CalClock;
//               UINT8 CalDiv;
//
//               //115384
//               if((UsbSetupBuf->wIndexL & 0x7f) == 3)
//               {
//                 CalClock = 6000000;
//                 CalDiv = 0 - UsbSetupBuf->wIndexH;
//                 bps = CalClock / CalDiv;
//               }
//               else if((UsbSetupBuf->wIndexL & 0x7f) == 2)
//               {
//                 CalClock = 750000;  //6000000 / 8
//                 CalDiv = 0 - UsbSetupBuf->wIndexH;
//                 bps = CalClock / CalDiv;
//               }
//               else if((UsbSetupBuf->wIndexL & 0x7f) == 1)
//               {
//                 CalClock = 93750; //64 ��Ƶ
//                 CalDiv = 0 - UsbSetupBuf->wIndexH;
//                 bps = CalClock / CalDiv;
//               }
//               else if((UsbSetupBuf->wIndexL & 0x7f) == 0)
//               {
//                 CalClock = 11719;  //Լ512
//                 CalDiv = 0 - UsbSetupBuf->wIndexH;
//                 bps = CalClock / CalDiv;
//               }
//               else
//               {
//                 bps = 115200;
//               }
//             }
//             Uart0Para.BaudRate = bps;
//             dg_log("set bps:%d\r\n",(int)bps);
//
//             //UART0BpsSet(bps);
//             VENSer0ParaChange = 1;
//
//             if(bps == USB_CFG_BPS)
//             {
//               dg_log("start cfg\r\n");
//               usb_u0_cfg_flag = 1;
//               UserDefineBpsCheck = 1; //���Ⲩ���ʵļ��ʹ��
//             }
//             else
//             {
//               usb_u0_cfg_flag = 0;
//               UserDefineBpsCheck = 0;
//             }
//           }
//           else
//           {
//             CH341RegWrite(write_reg_add1,write_reg_val1);
//             CH341RegWrite(write_reg_add2,write_reg_val2);
//           }
//
//           break;
//         }
//         case DEF_VEN_DEBUG_READ:   //��Ҫ�ش����� 0X95  /* ������Ĵ��� */
//         {
//           UINT8 read_reg_add1;
//           UINT8 read_reg_add2;
//           UINT8 read_reg_val1;
//           UINT8 read_reg_val2;
//
//           usb_work_mode = USB_VENDOR_MODE;//�л�������ģʽ
//
//           read_reg_add1 = UsbSetupBuf->wValueL;
//           read_reg_add2 = UsbSetupBuf->wValueH;
//
//           CH341RegRead(read_reg_add1,&read_reg_val1);
//           CH341RegRead(read_reg_add2,&read_reg_val2);
//
//           len = 2;
//           pDescr = buf;
//           buf[0] = read_reg_val1;
//           buf[1] = read_reg_val2;
//           SetupLen = len;
//           tmos_memcpy(Ep0Buffer, pDescr, len);
//
//           break;
//         }
//         //A1����Ҳ��Ҫ��ʼ�����ڵ�
//         case DEF_VEN_UART_INIT:  //��ʼ������ 0XA1
//         {
//           UINT8 reg_uart_ctrl;
//           UINT8  parity_val;
//           UINT8  data_bit_val;
//           UINT8  stop_bit_val;
//           UINT8  uart_reg1_val;
//           UINT8  uart_reg2_val;
//           UINT8  uart_set_m;
//
//           len = 0;
//
//           /* ����ģʽ�л� */
//           usb_work_mode = USB_VENDOR_MODE;//�л�������ģʽ
//
////                  UINT8 usb_cdc_mode_trans_en = 0;   //cdcģʽ�������ϴ�ʹ��
////                  UINT8 usb_ven_mode_trans_en = 0;   //����ģʽ�������ϴ�ʹ��
//
//           if(Ep0Buffer[2] & 0x80)
//           {
//             reg_uart_ctrl = Ep0Buffer[3];
//
//             data_bit_val = reg_uart_ctrl & 0x03;
//             if     (data_bit_val == 0x00) data_bit_val = HAL_UART_5_BITS_PER_CHAR;
//             else if(data_bit_val == 0x01) data_bit_val = HAL_UART_6_BITS_PER_CHAR;
//             else if(data_bit_val == 0x02) data_bit_val = HAL_UART_7_BITS_PER_CHAR;
//             else if(data_bit_val == 0x03) data_bit_val = HAL_UART_8_BITS_PER_CHAR;
//
//             stop_bit_val = reg_uart_ctrl & 0x04;
//             if(stop_bit_val) stop_bit_val = HAL_UART_TWO_STOP_BITS;
//             else             stop_bit_val = HAL_UART_ONE_STOP_BIT;
//
//             parity_val = reg_uart_ctrl & (0x38);
//             if     (parity_val == 0x00) parity_val = HAL_UART_NO_PARITY;
//             else if(parity_val == 0x08) parity_val = HAL_UART_ODD_PARITY;
//             else if(parity_val == 0x18) parity_val = HAL_UART_EVEN_PARITY;
//             else if(parity_val == 0x28) parity_val = HAL_UART_MARK_PARITY;
//             else if(parity_val == 0x38) parity_val = HAL_UART_SPACE_PARITY;
//
//             //Uart0Para.BaudRate;
//             Uart0Para.StopBits = stop_bit_val;
//             Uart0Para.ParityType = parity_val;
//             Uart0Para.DataBits = data_bit_val;
//
//             //ֱ�����üĴ���
//             //UART0ParaSet(data_bit_val, stop_bit_val,parity_val);
//             VENSer0ParaChange = 1;
//
//             uart_set_m = 0;
//             //[4],[5]
//             uart_reg1_val = UsbSetupBuf->wIndexL;
//             uart_reg2_val = UsbSetupBuf->wIndexH;
//
//             //uart_set_m = 1;
//             if(uart_reg1_val & (1<<6))  //�жϵ���λ
//             {
//               uart_set_m = 1;
//             }
//             else
//             {
//               uart_set_m = 1;
//               //���˸���λ�͵���λ��Ч�Ļ��ͷǷ�
//               //if(uart_reg1_val & 0x38)  uart_set_m = 0;
//               //������Ч�Ĳ���
//               uart_reg1_val = uart_reg1_val & 0xC7;
//             }
//
//             if(uart_set_m)
//             {
//               /* �����ʴ�����ü���ֵ */
//               if((uart_reg1_val == 0x87)&&(uart_reg2_val == 0xf3))
//               {
//                 bps = 921600;  //13 * 921600 = 11980800
//               }
//               else if((uart_reg1_val == 0x87)&&(uart_reg2_val == 0xd9))
//               {
//                 bps = 307200;  //39 * 307200 = 11980800
//               }
//
//               //ϵͳ��Ƶ��  32000000
//               else if( uart_reg1_val == 0xCA )
//               {
//                 UINT32 CalClock;
//                 UINT8 CalDiv;
//
//                 CalClock = 32000000 / 8;
//                 CalDiv = 0 - uart_reg2_val;
//                 bps = CalClock / CalDiv;
//               }
//               else if( uart_reg1_val == 0xCB )
//               {
//                 UINT32 CalClock;
//                 UINT8 CalDiv;
//
//                 CalClock = 32000000 / 8 / 256;
//                 CalDiv = 0 - uart_reg2_val;
//                 bps = CalClock / CalDiv;
//               }
//               else  //340
//               {
//                 UINT32 CalClock;
//                 UINT8 CalDiv;
//
//                 //115384
//                 if((uart_reg1_val & 0x7f) == 3)
//                 {
//                   CalClock = 6000000;
//                   CalDiv = 0 - uart_reg2_val;
//                   bps = CalClock / CalDiv;
//                 }
//                 else if((uart_reg1_val & 0x7f) == 2)
//                 {
//                   CalClock = 750000;  //6000000 / 8
//                   CalDiv = 0 - uart_reg2_val;
//                   bps = CalClock / CalDiv;
//                 }
//                 else if((uart_reg1_val & 0x7f) == 1)
//                 {
//                   CalClock = 93750; //64 ��Ƶ
//                   CalDiv = 0 - uart_reg2_val;
//                   bps = CalClock / CalDiv;
//                 }
//                 else if((uart_reg1_val & 0x7f) == 0)
//                 {
//                   CalClock = 11719;  //Լ512
//                   CalDiv = 0 - uart_reg2_val;
//                   bps = CalClock / CalDiv;
//                 }
//                 else
//                 {
//                   bps = 115200;
//                 }
//               }
//               Uart0Para.BaudRate = bps;
//               dg_log("CH341 set bps:%d\r\n",(int)bps);
//               //Uart0Para.BaudRate = bps;
//               //UART0BpsSet(bps);
//               VENSer0ParaChange = 1;
//
//               if(bps == USB_CFG_BPS)
//               {
//                 dg_log("start cfg\r\n");
//                 usb_u0_cfg_flag = 1;
//                 UserDefineBpsCheck = 1; //���Ⲩ���ʵļ��ʹ��
//               }
//               else
//               {
//                 usb_u0_cfg_flag = 0;
//                 UserDefineBpsCheck = 0;
//               }
//
//             }
//           }
//           else   //������ʽ��A1
//           {
//
//           }
//
//           break;
//         }
//         case DEF_VEN_UART_M_OUT:  //����MODEM�ź���� 0XA4
//         {
//           UINT8 reg_pb_out;
//
//           len = 0;
//
//           usb_work_mode = USB_VENDOR_MODE;//�л�������ģʽ
//
//           reg_pb_out = Ep0Buffer[2];
//
////                  dg_log("modem set:%02x\r\n",reg_val);
////                  SetUART0ModemVendorSta(reg_val);
//           dg_log("A4 modem set:%02x\r\n",reg_pb_out);
//
//           if(reg_pb_out & (1<<7)) SetUART0ModemVendorSta(0);
//           else                    SetUART0ModemVendorSta(1);
//
//           /* OUT���� */
//           if(reg_pb_out & (1<<4)) UART0_OUT_Val = 1;
//           else                    UART0_OUT_Val = 0;
//
//           /* DTR���� */
//           if(reg_pb_out & (1<<5))
//           {
//             UART0_DTR_Val = 1;
//             UART_DTR_SET;
//           }
//           else
//           {
//             UART0_DTR_Val = 0;
//             UART_DTR_RESET;
//           }
//
//           if(!uart0_modem_vendor_en) //Ӳ������û�п��������
//           {
//             /* RTS���� */
//             if(reg_pb_out & (1<<6))
//             {
//               UART0_RTS_Val = 1;
//               UART_RTS_SET;
//             }
//             else
//             {
//               UART0_RTS_Val = 0;
//               UART_RTS_RESET;
//             }
//           }
//           break;
//         }
//         case DEF_VEN_BUF_CLEAR: //0XB2  /* ���δ��ɵ����� */
//         {
//           len = 0;
//
//           //�������³�ʼ��
//           VENSer0ParaChange = 1; // ���³�ʼ������������е�����
//
//           break;
//         }
//         case DEF_VEN_I2C_CMD_X:  //0X54  ����I2C�ӿڵ�����,����ִ��
//         {
//           len = 0;
//
//           break;
//         }
//         case DEF_VEN_DELAY_MS:  //0X5E  ������Ϊ��λ��ʱָ��ʱ��
//         {
//           len = 0;
//
//           break;
//         }
//         case DEF_VEN_GET_VER:   //0X5E   ��ȡоƬ�汾 //��Ҫ�ش�����-->�汾��
//         {
//           len = 2;
//           pDescr = buf;
////                  CfgPara.dev_cdc_release_num[0] = USB_DEV_PARA_CDC_RELE_NUM & 0xff;
////                  CfgPara.dev_cdc_release_num[1] = (USB_DEV_PARA_CDC_RELE_NUM>>8) & 0xff;
//
//
//           buf[0] = 0x30;
//           buf[1] = 0x00;
//
//           buf[0] = CfgPara.dev_cdc_release_num[1];
//           buf[1] = CfgPara.dev_cdc_release_num[0];
//
//           SetupLen = len;
//           tmos_memcpy( Ep0Buffer, pDescr, len );
//
//           break;
//         }
//         case DEF_VEN_UART_GPIO:   /* ����GPIO��� */
//         {
//           len = 0;
//
//           //break��ѡ - 0x80
//           //breakȡ�� - 0x90
//           /* ����0��break���� */
//           if(Ep0Buffer[2] & (1<<7))
//           {
//             if(Ep0Buffer[2] & (1<<4)) SetUART0BreakENStatus(0); //�ر�break
//             else                      SetUART0BreakENStatus(1); //��break
//           }
//
//           break;
//         }
//         default:
//         {
//           //len = 0xFF;
//           len = 0;
//           break;
//         }
//       }
     }
      else if((pSetupReqPak->bRequestType & USB_REQ_TYP_MASK) == USB_REQ_TYP_STANDARD) /* ��׼���� */
      {
        switch ( SetupReqCode )
        {
          case USB_GET_DESCRIPTOR :
          {
              USB_DEBUG("GET DESCRIPTOR\n");
            switch ( ( ( pSetupReqPak->wValue ) >> 8 ) )
            {
              case USB_DESCR_TYP_DEVICE :
              {
                pDescr = TAB_USB_CDC_DEV_DES;
                len = sizeof( TAB_USB_CDC_DEV_DES );
              }
                break;

              case USB_DESCR_TYP_CONFIG :
              {
                pDescr = TAB_USB_CDC_CFG_DES;
                len = sizeof( TAB_USB_CDC_CFG_DES );
              }
                break;

              case USB_DESCR_TYP_STRING :
              {
                switch ( ( pSetupReqPak->wValue ) & 0xff )
                {
                  case 0:  //����������
                  {
                    pDescr = (PUINT8)( &TAB_USB_LID_STR_DES[0] );
                    len = sizeof( TAB_USB_LID_STR_DES );

                    break;
                  }
                  case 1:  //iManufacturer
                  case 2:   //iProduct
                  case 3:
                  default:
                  {
                    len = 0xFF;    // ��֧�ֵ�����������
                    break;
                  }

                }
              }
                break;

              default :
                errflag = 0xff;
                break;
            }
            if ( SetupReqLen > len )
              SetupReqLen = len;      //ʵ�����ϴ��ܳ���
            len = ( SetupReqLen >= DevEP0SIZE ) ?
                DevEP0SIZE : SetupReqLen;
            memcpy( pEP0_DataBuf, pDescr, len );
            pDescr += len;
          }
            break;

          case USB_SET_ADDRESS :
              USB_DEBUG("SET ADDRESS\n");
            SetupReqLen = ( pSetupReqPak->wValue ) & 0xff;
            break;

          case USB_GET_CONFIGURATION :
              USB_DEBUG("USB_GET_CONFIGURATION\n");
            pEP0_DataBuf[0] = DevConfig;
            if ( SetupReqLen > 1 )
              SetupReqLen = 1;
            break;

          case USB_SET_CONFIGURATION :
          {
              USB_DEBUG("USB_SET_CONFIGURATION\n");
            DevConfig = ( pSetupReqPak->wValue ) & 0xff;
            break;
          }
          case USB_CLEAR_FEATURE :
          {
              USB_DEBUG("USB_CLEAR_FEATURE\n");
            if ( ( pSetupReqPak->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )    // �˵�
            {
              switch ( ( pSetupReqPak->wIndex ) & 0xff )
              {
                case 0x82 :
                  R8_UEP2_CTRL = ( R8_UEP2_CTRL & ~( RB_UEP_T_TOG | MASK_UEP_T_RES ) ) | UEP_T_RES_NAK;
                  break;
                case 0x02 :
                  R8_UEP2_CTRL = ( R8_UEP2_CTRL & ~( RB_UEP_R_TOG | MASK_UEP_R_RES ) ) | UEP_R_RES_ACK;
                  break;
                case 0x81 :
                  R8_UEP1_CTRL = ( R8_UEP1_CTRL & ~( RB_UEP_T_TOG | MASK_UEP_T_RES ) ) | UEP_T_RES_NAK;
                  break;
                case 0x01 :
                  R8_UEP1_CTRL = ( R8_UEP1_CTRL & ~( RB_UEP_R_TOG | MASK_UEP_R_RES ) ) | UEP_R_RES_ACK;
                  break;
                default :
                  errflag = 0xFF;                                 // ��֧�ֵĶ˵�
                  break;
              }
            }
            else
              errflag = 0xFF;
          }
            break;

          case USB_GET_INTERFACE :
              USB_DEBUG("USB_GET_INTERFACE\n");
            pEP0_DataBuf[0] = 0x00;
            if ( SetupReqLen > 1 )
              SetupReqLen = 1;
            break;

          case USB_GET_STATUS :
              USB_DEBUG("USB_GET_STATUS\n");
            pEP0_DataBuf[0] = 0x00;
            pEP0_DataBuf[1] = 0x00;
            if ( SetupReqLen > 2 )
              SetupReqLen = 2;
            break;

          default :
            errflag = 0xff;
            break;
        }
      }
      else if( ( pSetupReqPak->bRequestType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_CLASS ) /* ������ */
      {
        /* �����´� */
        if(pSetupReqPak->bRequestType & USB_REQ_TYP_OUT)
        {
          switch( SetupReqCode )  // ������
          {
            case DEF_SET_LINE_CODING: /* SET_LINE_CODING */
            {
              uint8 tmp;

              USB_DEBUG("SET_LINE_CODING\r\n");
//              for(tmp=0; tmp<8; tmp++)
//              {
//                LOG_INFO("%02x ",EP0_Databuf[tmp]);
//              }

              if( EP0_Databuf[ 4 ] == 0x00 )
              {
                len = 0x00;
              }
              else len = 0xFF;

              break;
            }
            case DEF_SET_CONTROL_LINE_STATE:  /* SET_CONTROL_LINE_STATE */
            {
              // UINT8  carrier_sta;
              // UINT8  present_sta;

              // /* ��·״̬ */
              // LOG_INFO("ctl %02x %02x\r\n",EP0_Databuf[2],EP0_Databuf[3]);

              // carrier_sta = EP0_Databuf[2] & (1<<1);   //RTS״̬
              // present_sta = EP0_Databuf[2] & (1<<0);   //DTR״̬
              len = 0;

              break;
            }
            default:
            {
                USB_DEBUG("CDC ReqCode%x\r\n",SetupReqCode);
              len = 0xFF;                                       // ����ʧ��
              break;
            }
          }
        }
        /* �豸�ϴ� */
        else if(pSetupReqPak->bRequestType & USB_REQ_TYP_IN)
        {
          switch( SetupReqCode )  // ������
          {
            case DEF_GET_LINE_CODING: /* GET_LINE_CODING */
            {
                USB_DEBUG("GET_LINE_CODING:%d\r\n",EP0_Databuf[ 4 ]);
              pDescr = EP0_Databuf;
              len = sizeof( LINE_CODE );

              if( EP0_Databuf[ 4 ] == 0x00 )
              {

//                ( ( PLINE_CODE )EP0_Databuf )->BaudRate   = set_bps;
//                ( ( PLINE_CODE )EP0_Databuf )->StopBits   = stop_bit;
//                ( ( PLINE_CODE )EP0_Databuf )->ParityType = ver_bit;
//                ( ( PLINE_CODE )EP0_Databuf )->DataBits   = data_bit;

                ( ( PLINE_CODE )EP0_Databuf )->BaudRate   = 921600;
                ( ( PLINE_CODE )EP0_Databuf )->StopBits   = 1;
                ( ( PLINE_CODE )EP0_Databuf )->ParityType = 0;
                ( ( PLINE_CODE )EP0_Databuf )->DataBits   = 8;
              }
              else len = 0xFF;

              break;
            }
            case DEF_SERIAL_STATE:
            {
                USB_DEBUG("GET_SERIAL_STATE:%d\r\n",EP0_Databuf[ 4 ]);
              //SetupLen �ж��ܳ���
              len = 2;
              if( EP0_Databuf[ 4 ] == 0x00 )
              {
//                CDCSetSerIdx = 0;
                EP0_Databuf[0] = 0;
                EP0_Databuf[1] = 0;

//                if(UART_DCD_VAL == 0) EP0_Databuf[0] |= 0x01;
//                if(UART_DSR_VAL == 0) EP0_Databuf[0] |= 0x02;
//                if(UART_CTS_VAL == 0) EP0_Databuf[0] |= 0x04;
//                if(UART_RI_VAL == 0)  EP0_Databuf[0] |= 0x08;
              }
              else len = 0xFF;

              break;
            }
            default:
            {
                USB_DEBUG("CDC ReqCode%x\r\n",SetupReqCode);
              len = 0xFF;                                       // ����ʧ��
              break;
            }
          }
        }
        else
        {

        }
      }
      else
      {
        switch ( SetupReqCode )
        {
//          case 0x0a :
//            break;        //���һ��Ҫ��
//          case 0x09 :
//            break;
//          default :
//            errflag = 0xFF;
        }
      }


      if ( errflag == 0xff )        // �����֧��
      {
        // SetupReqCode = 0xFF;
        R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;    // STALL
      }
      else
      {
        if ( chtype & 0x80 )     // �ϴ�
        {
          len = ( SetupReqLen > DevEP0SIZE ) ?
              DevEP0SIZE : SetupReqLen;
          SetupReqLen -= len;
        }
        else
          len = 0;        // �´�
        R8_UEP0_T_LEN = len;
        R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;    // Ĭ�����ݰ���DATA1
      }

      R8_USB_INT_FG = RB_UIF_TRANSFER;
    }
  }
  else if ( intflag & RB_UIF_BUS_RST )
  {
    R8_USB_DEV_AD = 0;
    R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
    R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
    R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
    R8_UEP3_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
    R8_USB_INT_FG = RB_UIF_BUS_RST;
  }
  else if ( intflag & RB_UIF_SUSPEND )
  {
    if ( R8_USB_MIS_ST & RB_UMS_SUSPEND )
    {
      ;
    }    // ����
    else
    {
      ;
    }               // ����
    R8_USB_INT_FG = RB_UIF_SUSPEND;
  }
  else
  {
    R8_USB_INT_FG = intflag;
  }

}

/*******************************************************************************
* Function Name  : USBDevEPnINSetStatus
* Description    : �˵�״̬���ú���
* Input          : ep_num���˵��
                   type���˵㴫������
                   sta���л��Ķ˵�״̬
* Output         : None
* Return         : None
*******************************************************************************/
void USBDevEPnINSetStatus(u8 ep_num, u8 type, u8 sta)
{
  u8 *p_UEPn_CTRL;

  p_UEPn_CTRL = (u8 *)(USB_BASE_ADDR + 0x22 + ep_num * 4);
  if(type == ENDP_TYPE_IN) *((PUINT8V)p_UEPn_CTRL) = (*((PUINT8V)p_UEPn_CTRL) & (~(0x03))) | sta;
  else *((PUINT8V)p_UEPn_CTRL) = (*((PUINT8V)p_UEPn_CTRL) & (~(0x03<<2))) | (sta<<2);
}



void DevEP2_OUT_ACK( void )
{
  PFIC_DisableIRQ(USB_IRQn);
  Rxlwrbfull_flag = 0;
  R8_UEP2_CTRL = R8_UEP2_CTRL & 0xf3; //OUT_ACK
  PFIC_EnableIRQ(USB_IRQn);
}

void DevEP2_OUT_NAK( void )
{
  PFIC_DisableIRQ(USB_IRQn);

  R8_UEP2_CTRL = (R8_UEP2_CTRL & 0xf3) | 0x08; //OUT_NAK
  PFIC_EnableIRQ(USB_IRQn);
}

void DevEP1_OUT_Deal( UINT8 l )
{

}

//size_t DevEP2_OUT_Deal( UINT8 len )
//{
////    lwrb_write( &KEY_buff, EP2_Databuf, len);
////    return lwrb_get_free(&KEY_buff);
//    return 0;
//}

bool DevEPn_IN_Deal(void)
{
    size_t len = 0;
    bool ret = false;
    USB_DEBUG("Ep2InFlag=%#x\n", Ep2InFlag);
    if(Ep2InFlag == 1) {
      PFIC_DisableIRQ( USB_IRQn );
      len = lwrb_get_full(&RF_RCV);
      PFIC_EnableIRQ( USB_IRQn );
      if( len<=64 ) {
        lwrb_read(&RF_RCV, pEP2_RAM_Addr+64, len);
        ret = false;
      }
      else {
        len = 64;
        lwrb_read( &RF_RCV, pEP2_RAM_Addr+64, len);
        ret = true;
      }
      USB_DEBUG("USB EP2 IN=[");
      for(int i = 0; i < len; i++) {
          if(i) USB_DEBUG(" ");
          USB_DEBUG("%#x", pEP2_RAM_Addr[i+64]);
      }USB_DEBUG("]\n");

      Ep2InFlag = 0;
      PFIC_DisableIRQ( USB_IRQn );
      R8_UEP2_T_LEN = len;
      R8_UEP2_CTRL = (R8_UEP2_CTRL & ~MASK_UEP_T_RES)| UEP_T_RES_ACK;
      PFIC_EnableIRQ( USB_IRQn );
    }
    return ret;
}


void USB_Init(void)
{
  pEP0_RAM_Addr = EP0_Databuf;
  pEP1_RAM_Addr = EP1_Databuf;
  pEP2_RAM_Addr = EP2_Databuf;

  USB_DeviceInit();
  LOG_INFO("USB Device Init");
  PFIC_EnableIRQ( USB_IRQn );
  DelayMs(500);  //wait usb init
}

