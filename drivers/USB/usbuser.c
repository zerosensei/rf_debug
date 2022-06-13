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
  0x01,                 //BCD码 USB规范版本号码

  0x02,                 //类别码
  0x00,                 //子类别码
  0x00,                 //协议码

  THIS_ENDP0_SIZE,      //端点0的最大信息包大小
  0x86, 0x1a,           //厂商ID
  0xD3, 0x55,           //产品ID  e041

  //0x00, 0x30, //BCD 设备版本号
  0x00, 0x40, //BCD 设备版本号

  0x01,                 //制造者的字符串描述符的索引值
  0x02,                 //产品的字符串描述符的索引值
  0x03,                 //序号的字符串描述符的索引值

  0x01                  //可能配置的数目
};

/* 配置描述符 */
const UINT8 TAB_USB_CDC_CFG_DES[ ] =
{
    0x09,   /* 配置描述符 */
    0x02,

    //0x8d, 0x00, //此配置传回的所有数据大小
    //0x04,   //此配置支持的接口数目 /* 4个接口 */
  //0x4B, 0x00, //此配置传回的所有数据大小
  0x43, 0x00, //此配置传回的所有数据大小
  0x02,   //此配置支持的接口数目 /* 4个接口 */

  0x01,       //Set_Configuration与Set_Configuration请求的标识
  0x00,       //此配置的字符串描述符
  //0xa0,       //自身电源以及远程唤醒设置
  0x80,   //E0
  0x30,       //需要总线电源mA

  //09 04 00 00 01 02 02 01 00
  //以下为接口0（CDC接口）描述符
  0x09,       //CDC接口描述符(一个端点)
  0x04,
  0x00,       //识别此接口的数字
  0x00,       //用来选择一个替代设置的数值
  0x01,       //除了端点0外支持的端点数
  0x02,       //类别码 -- Communication Interface Class 02h
  0x02,       //子类别码 -- Abstract Control Model 02h
  0x01,       //协议码 -- V.25ter 01h (Common AT commands (also known as “Hayes? compatible”))
  0x00,       //此接口的字符串描述符索引值


//  //以下为功能描述符 Functional Descriptors
//    0x05,  //长度//功能描述符(头)
//  0x24,  //CS_INTERFACE
//  0x00,  //Header Functional Descriptor
//  0x10,0x01, //bcdCDC
//
//
//  0x05,  //长度//管理描述符(数据类接口1)
//  0x24,  //CS_INTERFACE
//  0x01,  //Call Management Functional Descriptor
//  //D7..D2: RESERVED (Reset to zero)
//  //D1:0 管理命令只能在Communication Class interface
//  //D0:0 Device does not handle call management itself
//  0x00,
//  0x01, //Interface number of Data Class interfaceoptionally used for call management
//
//
//  0x04,  //长度//支持Set_Line_Coding、Set_Control_Line_State、Get_Line_Coding、Serial_State
//  0x24,  //CS_INTERFACE
//  0x02,  //Abstract Control Management Functional Descriptor
//  0x02,  //位2置1 表示支持Set_Line_Coding、Set_Control_Line_State、Get_Line_Coding、Serial_State
//
//
//  0x05,  //长度//编号为0的CDC接口;编号1的数据类接口
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


  //0x07,0x05,0x84,0x03,0x08,0x00,0x01,                       //中断上传端点描述符
  //0x07,0x05,0x84,0x03,0x40,0x00,0x01,                       //中断上传端点描述符 端点4
  0x07,0x05,0x81,0x03,0x40,0x00,0x01,                       //中断上传端点描述符 端点1

  //09 04 01 00 02 0a 00 00 00
  //以下为接口1（数据接口）描述符
  0x09,   //数据接口描述符
  0x04,
  0x01,   //识别此接口的数字
  0x00,   //用来选择一个替代设置的数值
  0x02,   //除了端点0外支持的端点数
  0x0a,   //类别码 -- Data Interface Class code  0ah
  0x00,   //子类别码 -- Data Class SubClass code 00h
  0x00,   //协议码 -- USB specification  00h
  0x00,   //此接口的字符串描述符索引值

//  0x07,0x05,0x01,0x02,0x40,0x00,0x00,                       //端点描述符 端点1
//  0x07,0x05,0x81,0x02,0x40,0x00,0x00,                       //端点描述符 端点1

  //0x07,0x05,0x02,0x02,0x40,0x00,0x00,                       //端点描述符 端点2
  0x07,0x05,0x02,0x02,0x40,0x00,0x00,                       //端点描述符 端点2
  0x07,0x05,0x82,0x02,0x40,0x00,0x00,                       //端点描述符 端点2
};

/* 设备限定描述符 */
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


// 语言描述符
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
* Description    : USB中断服务程序
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
    if ( ( R8_USB_INT_ST & MASK_UIS_TOKEN ) != MASK_UIS_TOKEN )    // 非空闲
    {
      switch ( R8_USB_INT_ST & ( MASK_UIS_TOKEN | MASK_UIS_ENDP ) )
      // 分析操作令牌和端点号
      {
        case UIS_TOKEN_IN :
        {
            USB_DEBUG("SetupReqCode:%#x \n",SetupReqCode);
          switch ( SetupReqCode )
          {
            case USB_GET_DESCRIPTOR :
              len = SetupReqLen >= DevEP0SIZE ?
                  DevEP0SIZE : SetupReqLen;    // 本次传输长度
              memcpy( pEP0_DataBuf, pDescr, len ); /* 加载上传数据 */
              SetupReqLen -= len;
              pDescr += len;
              R8_UEP0_T_LEN = len;
              R8_UEP0_CTRL ^= RB_UEP_T_TOG;                             // 翻转
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
              R8_UEP0_T_LEN = 0;                                      // 状态阶段完成中断或者是强制上传0长度数据包结束控制传输
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
              /* 设置串口 */
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
          {                       // 不同步的数据包将丢弃
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
          {                       // 不同步的数据包将丢弃
              //TODO: out 包处理

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
    if ( R8_USB_INT_ST & RB_UIS_SETUP_ACT )                  // Setup包处理
    {
      R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
      SetupReqLen = pSetupReqPak->wLength;
      SetupReqCode = pSetupReqPak->bRequest;
      chtype = pSetupReqPak->bRequestType;

      len = 0;
      errflag = 0;
      /* 厂商请求 */
     if( ( pSetupReqPak->bRequestType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_VENDOR )
     {
         USB_DEBUG("VENDOR:%#x\n",SetupReqCode);
//       switch(SetupReqCode)
//       {
//         case DEF_VEN_DEBUG_WRITE:  //写两组 0X9A
//         {
//           UINT32 bps = 0;
//           UINT8 write_reg_add1;
//           UINT8 write_reg_add2;
//           UINT8 write_reg_val1;
//           UINT8 write_reg_val2;
//
//           len = 0;
////           usb_work_mode = USB_VENDOR_MODE;//切换至厂商模式
//
//           write_reg_add1 = EP0_Databuf[2];
//           write_reg_add2 = EP0_Databuf[3];
//           write_reg_val1 = EP0_Databuf[4];
//           write_reg_val2 = EP0_Databuf[5];
//
//           /* 该组是设置波特率的寄存器 */
//           if((write_reg_add1 == 0x12)&&(write_reg_add2 == 0x13))
//           {
//             /* 波特率处理采用计算值 */
//             if((pSetupReqPak->wIndexL==0x87)&&(pSetupReqPak->wIndexH==0xf3))
//             {
//               bps = 921600;  //13 * 921600 = 11980800
//             }
//             else if((pSetupReqPak->wIndexL==0x87)&&(pSetupReqPak->wIndexH==0xd9))
//             {
//               bps = 307200;  //39 * 307200 = 11980800
//             }
//
//             //系统主频：  32000000
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
//                 CalClock = 93750; //64 分频
//                 CalDiv = 0 - UsbSetupBuf->wIndexH;
//                 bps = CalClock / CalDiv;
//               }
//               else if((UsbSetupBuf->wIndexL & 0x7f) == 0)
//               {
//                 CalClock = 11719;  //约512
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
//               UserDefineBpsCheck = 1; //特殊波特率的检测使能
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
//         case DEF_VEN_DEBUG_READ:   //需要回传数据 0X95  /* 读两组寄存器 */
//         {
//           UINT8 read_reg_add1;
//           UINT8 read_reg_add2;
//           UINT8 read_reg_val1;
//           UINT8 read_reg_val2;
//
//           usb_work_mode = USB_VENDOR_MODE;//切换至厂商模式
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
//         //A1命令也是要初始化串口的
//         case DEF_VEN_UART_INIT:  //初始化串口 0XA1
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
//           /* 工作模式切换 */
//           usb_work_mode = USB_VENDOR_MODE;//切换至厂商模式
//
////                  UINT8 usb_cdc_mode_trans_en = 0;   //cdc模式下数据上传使能
////                  UINT8 usb_ven_mode_trans_en = 0;   //厂商模式下数据上传使能
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
//             //直接设置寄存器
//             //UART0ParaSet(data_bit_val, stop_bit_val,parity_val);
//             VENSer0ParaChange = 1;
//
//             uart_set_m = 0;
//             //[4],[5]
//             uart_reg1_val = UsbSetupBuf->wIndexL;
//             uart_reg2_val = UsbSetupBuf->wIndexH;
//
//             //uart_set_m = 1;
//             if(uart_reg1_val & (1<<6))  //判断第六位
//             {
//               uart_set_m = 1;
//             }
//             else
//             {
//               uart_set_m = 1;
//               //除了高两位和低三位有效的话就非法
//               //if(uart_reg1_val & 0x38)  uart_set_m = 0;
//               //保留有效的参数
//               uart_reg1_val = uart_reg1_val & 0xC7;
//             }
//
//             if(uart_set_m)
//             {
//               /* 波特率处理采用计算值 */
//               if((uart_reg1_val == 0x87)&&(uart_reg2_val == 0xf3))
//               {
//                 bps = 921600;  //13 * 921600 = 11980800
//               }
//               else if((uart_reg1_val == 0x87)&&(uart_reg2_val == 0xd9))
//               {
//                 bps = 307200;  //39 * 307200 = 11980800
//               }
//
//               //系统主频：  32000000
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
//                   CalClock = 93750; //64 分频
//                   CalDiv = 0 - uart_reg2_val;
//                   bps = CalClock / CalDiv;
//                 }
//                 else if((uart_reg1_val & 0x7f) == 0)
//                 {
//                   CalClock = 11719;  //约512
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
//                 UserDefineBpsCheck = 1; //特殊波特率的检测使能
//               }
//               else
//               {
//                 usb_u0_cfg_flag = 0;
//                 UserDefineBpsCheck = 0;
//               }
//
//             }
//           }
//           else   //其他形式的A1
//           {
//
//           }
//
//           break;
//         }
//         case DEF_VEN_UART_M_OUT:  //设置MODEM信号输出 0XA4
//         {
//           UINT8 reg_pb_out;
//
//           len = 0;
//
//           usb_work_mode = USB_VENDOR_MODE;//切换至厂商模式
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
//           /* OUT引脚 */
//           if(reg_pb_out & (1<<4)) UART0_OUT_Val = 1;
//           else                    UART0_OUT_Val = 0;
//
//           /* DTR引脚 */
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
//           if(!uart0_modem_vendor_en) //硬件流控没有开启的情况
//           {
//             /* RTS引脚 */
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
//         case DEF_VEN_BUF_CLEAR: //0XB2  /* 清除未完成的数据 */
//         {
//           len = 0;
//
//           //可以重新初始化
//           VENSer0ParaChange = 1; // 重新初始化即可清除所有的数据
//
//           break;
//         }
//         case DEF_VEN_I2C_CMD_X:  //0X54  发出I2C接口的命令,立即执行
//         {
//           len = 0;
//
//           break;
//         }
//         case DEF_VEN_DELAY_MS:  //0X5E  以亳秒为单位延时指定时间
//         {
//           len = 0;
//
//           break;
//         }
//         case DEF_VEN_GET_VER:   //0X5E   获取芯片版本 //需要回传数据-->版本号
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
//         case DEF_VEN_UART_GPIO:   /* 设置GPIO输出 */
//         {
//           len = 0;
//
//           //break勾选 - 0x80
//           //break取消 - 0x90
//           /* 串口0的break操作 */
//           if(Ep0Buffer[2] & (1<<7))
//           {
//             if(Ep0Buffer[2] & (1<<4)) SetUART0BreakENStatus(0); //关闭break
//             else                      SetUART0BreakENStatus(1); //打开break
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
      else if((pSetupReqPak->bRequestType & USB_REQ_TYP_MASK) == USB_REQ_TYP_STANDARD) /* 标准请求 */
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
                  case 0:  //语言描述符
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
                    len = 0xFF;    // 不支持的描述符类型
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
              SetupReqLen = len;      //实际需上传总长度
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
            if ( ( pSetupReqPak->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )    // 端点
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
                  errflag = 0xFF;                                 // 不支持的端点
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
      else if( ( pSetupReqPak->bRequestType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_CLASS ) /* 类请求 */
      {
        /* 主机下传 */
        if(pSetupReqPak->bRequestType & USB_REQ_TYP_OUT)
        {
          switch( SetupReqCode )  // 请求码
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

              // /* 线路状态 */
              // LOG_INFO("ctl %02x %02x\r\n",EP0_Databuf[2],EP0_Databuf[3]);

              // carrier_sta = EP0_Databuf[2] & (1<<1);   //RTS状态
              // present_sta = EP0_Databuf[2] & (1<<0);   //DTR状态
              len = 0;

              break;
            }
            default:
            {
                USB_DEBUG("CDC ReqCode%x\r\n",SetupReqCode);
              len = 0xFF;                                       // 操作失败
              break;
            }
          }
        }
        /* 设备上传 */
        else if(pSetupReqPak->bRequestType & USB_REQ_TYP_IN)
        {
          switch( SetupReqCode )  // 请求码
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
              //SetupLen 判断总长度
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
              len = 0xFF;                                       // 操作失败
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
//            break;        //这个一定要有
//          case 0x09 :
//            break;
//          default :
//            errflag = 0xFF;
        }
      }


      if ( errflag == 0xff )        // 错误或不支持
      {
        // SetupReqCode = 0xFF;
        R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;    // STALL
      }
      else
      {
        if ( chtype & 0x80 )     // 上传
        {
          len = ( SetupReqLen > DevEP0SIZE ) ?
              DevEP0SIZE : SetupReqLen;
          SetupReqLen -= len;
        }
        else
          len = 0;        // 下传
        R8_UEP0_T_LEN = len;
        R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;    // 默认数据包是DATA1
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
    }    // 挂起
    else
    {
      ;
    }               // 唤醒
    R8_USB_INT_FG = RB_UIF_SUSPEND;
  }
  else
  {
    R8_USB_INT_FG = intflag;
  }

}

/*******************************************************************************
* Function Name  : USBDevEPnINSetStatus
* Description    : 端点状态设置函数
* Input          : ep_num：端点号
                   type：端点传输类型
                   sta：切换的端点状态
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

