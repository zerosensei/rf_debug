#ifndef __USBUSER_H__
#define __USBUSER_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

  /*Define USB state*/
#define USB_NOData      0x01
#define USB_DataCome    0x02

#define USB_MSG_EVENT     0X01


#define USB_UPDATA_EVENT  0x0001
#define USB_TIMEOUT_EVENT 0x0002



/* 宸ヤ綔妯″紡 */
#define USB_VENDOR_MODE                0
#define USB_CDC_MODE                   1

/* 杩炴帴鐘舵�� */
#define USB_DISCONNECT                 0
#define USB_CONNECT                    1


    /* USB閫氳涓�浜涘畯瀹氫箟 */
/* Endpoints Max Packet Size Set */
#define USB_EP0_MAX_PKT                 64
#define USB_EP1_IN_MAX_PKT              16
#define USB_EP2_IN_MAX_PKT              64
#define USB_EP2_OUT_MAX_PKT             64

/* endpoints enumeration */
#define ENDP0                           0x00
#define ENDP1                           0x01
#define ENDP2                           0x02
#define ENDP3                           0x03
#define ENDP4                           0x04

/* ENDP x Type */
#define ENDP_TYPE_IN                    0x00                                    /* ENDP is IN Type */
#define ENDP_TYPE_OUT                   0x01                                    /* ENDP is OUT Type */

/* 绔偣搴旂瓟鐘舵�佸畾涔� */
/* OUT */
#define OUT_ACK                         0
#define OUT_TIMOUT                      1
#define OUT_NAK                         2
#define OUT_STALL                       3
/* IN */
#define IN_ACK                          0
#define IN_NORSP                        1
#define IN_NAK                          2
#define IN_STALL                        3

/* USB璁惧鍚勭鏍囧織浣嶅畾涔� */
#define DEF_BIT_USB_RESET               0x01                                    /* 鎬荤嚎澶嶄綅鏍囧織 */
#define DEF_BIT_USB_DEV_DESC            0x02                                    /* 鑾峰彇杩囪澶囨弿杩扮鏍囧織 */
#define DEF_BIT_USB_ADDRESS             0x04                                    /* 璁剧疆杩囧湴鍧�鏍囧織 */
#define DEF_BIT_USB_CFG_DESC            0x08                                    /* 鑾峰彇杩囬厤缃弿杩扮鏍囧織 */
#define DEF_BIT_USB_SET_CFG             0x10                                    /* 璁剧疆杩囬厤缃�兼爣蹇� */
#define DEF_BIT_USB_WAKE                0x20                                    /* USB鍞ら啋鏍囧織 */
#define DEF_BIT_USB_SUPD                0x40                                    /* USB鎬荤嚎鎸傝捣鏍囧織 */
#define DEF_BIT_USB_HS                  0x80                                    /* USB楂橀�熴�佸叏閫熸爣蹇� */


typedef struct _USB_SETUP_REQ_ {
    UINT8 bRequestType;
    UINT8 bRequest;
    UINT8 wValueL;
    UINT8 wValueH;
    UINT8 wIndexL;
    UINT8 wIndexH;
    UINT8 wLengthL;
    UINT8 wLengthH;
} USB_SETUP_REQ_t;


/* 搴曞眰浜嬩欢涓婃姤 */
#define  USB_LINK_CAHNNGE_EVT        0x0001    //link鐘舵��
#define  USB_REC_DATA_EVT            0x0002    //USB鎺ユ敹鍒版暟鎹�
#define  USB_SEND_DATA_COMP_EVT      0x0004    //鍙戦�佹暟鎹畬鎴�
#define  USB_SET_UART_PARA_EVT       0x0008    //USB璁剧疆涓插彛鍙傛暟

#define  USB_PERIODIC_EVT            0x0010    //USB瀹氭椂浠诲姟


/* CDC閫氳鐨勪竴浜涘抚缁撴瀯瀹氫箟 */

/* 绫昏姹� */
//  3.1 Requests---Abstract Control Model
#define DEF_SEND_ENCAPSULATED_COMMAND  0x00
#define DEF_GET_ENCAPSULATED_RESPONSE  0x01
#define DEF_SET_COMM_FEATURE           0x02
#define DEF_GET_COMM_FEATURE           0x03
#define DEF_CLEAR_COMM_FEATURE         0x04
#define DEF_SET_LINE_CODING            0x20     // Configures DTE rate, stop-bits, parity, and number-of-character
#define DEF_GET_LINE_CODING            0x21     // This request allows the host to find out the currently configured line coding.
//#define   DEF_SET_CTL_LINE_STE           0X22     // This request generates RS-232/V.24 style control signals.
#define DEF_SET_CONTROL_LINE_STATE     0x22
#define DEF_SEND_BREAK                 0x23

//  3.2 Notifications---Abstract Control Model
#define DEF_NETWORK_CONNECTION         0x00
#define DEF_RESPONSE_AVAILABLE         0x01
#define DEF_SERIAL_STATE               0x20

//USB璁惧瀹氫箟
#define BIT_USB_CFG     ( 1<<7 )        /* USB閰嶇疆鏍囧織 */

 typedef struct __PACKED _USB_DEV
{
    UINT8   StsFlag;        /* 璁惧鐘舵�佹爣蹇� */
    UINT16  UsbVid;     /* 鍘傚晢ID */
    UINT16  UsbPid;     /* 浜у搧ID */
    UINT16  UsbRid;     /* 鍙戝竷ID */
}USB_DEV, *PUSB_DEV;

#define DEF_PAR_TYPE_NONE   0x00
#define DEF_PAR_TYPE_ODD    0x01
#define DEF_PAR_TYPE_EVEN   0x02
#define DEF_PAR_TYPE_MARK   0x03
#define DEF_PAR_TYPE_SPACE  0x04

//Line Code缁撴瀯
 typedef struct __PACKED  _LINE_CODE
{
    UINT32  BaudRate;       /* 娉㈢壒鐜� */
    UINT8   StopBits;       /* 鍋滄浣嶈鏁帮紝0锛�1鍋滄浣嶏紝1锛�1.5鍋滄浣嶏紝2锛�2鍋滄浣� */
    UINT8   ParityType;     /* 鏍￠獙浣嶏紝0锛歂one锛�1锛歄dd锛�2锛欵ven锛�3锛歁ark锛�4锛歋pace */
    UINT8   DataBits;       /* 鏁版嵁浣嶈鏁帮細5锛�6锛�7锛�8锛�16 */
}LINE_CODE, *PLINE_CODE;

/* 鍙傛暟0 */
//#define   BIT_ARG_UART_TYPE   ( 1<<7 )        /* 绫诲瀷浣嶏紝0锛歎SB杞覆鍙ｏ紝1锛氱綉缁滆浆涓插彛 */
//#define   BIT_ARG_UART_MODE   ( 1<<6 )        /* 妯″紡浣嶏紝0锛�3绾夸覆鍙ｏ紝1锛�9绾夸覆鍙� */
//#define   BIT_ARG_UART_CNT5   ( 1<<5 )        /* 涓插彛璁℃暟浣�5 */
//#define   BIT_ARG_UART_CNT4   ( 1<<4 )        /* 涓插彛璁℃暟浣�4 */
//#define   BIT_ARG_UART_CNT3   ( 1<<3 )        /* 涓插彛璁℃暟浣�3 */
//#define   BIT_ARG_UART_CNT2   ( 1<<2 )        /* 涓插彛璁℃暟浣�2 */
//#define   BIT_ARG_UART_CNT1   ( 1<<1 )        /* 涓插彛璁℃暟浣�1 */
//#define   BIT_ARG_UART_CNT0   ( 1<<0 )        /* 涓插彛璁℃暟浣�0 */
//#define   DEF_ARG_UART_CNT_MASK   ( 0x3F )    /* 涓插彛璁℃暟浣嶆帺鐮� */
//#define   M_ARG_UART_CNT_SET( i, n )  ( ( ( i ) & ~DEF_ARG_UART_CNT_MASK ) | ( ( n ) & DEF_ARG_UART_CNT_MASK ) )



#define BIT_REQ_DIR     ( 1<<15 )       /* 璇锋眰鏂瑰悜锛�0锛氫笅浼狅紝1锛氫笂浼� */
#define BIT_REQ_TYPE1   ( 1<<14 )       /* 璇锋眰绫诲瀷浣�1锛屽畾涔夊悓USB_REQ_TYP_MASK */
#define BIT_REQ_TYPE0   ( 1<<13 )       /* 璇锋眰绫诲瀷浣�0 */
#define DEF_REQ_TYPE_MASK   ( BIT_REQ_TYPE1 | BIT_REQ_TYPE0 )       /* 璇锋眰绫诲瀷鎺╃爜 */
#define DEF_REQ_TYPE_SET( i )   ( ( ( i )<<13 ) & DEF_REQ_TYPE_MASK )       /* 璁剧疆璇锋眰绫诲瀷 */
#define DEF_REQ_TYPE_GET( i )   ( ( ( i ) & DEF_REQ_TYPE_MASK )>>13 )       /* 鑾峰彇璇锋眰绫诲瀷 */
#define DEF_REQ_STD_ACT( i )    ( DEF_REQ_TYPE_GET( i ) == 0x00 ? 1 : 0 )       /* 鏍囧噯璇锋眰 */
#define DEF_REQ_CLS_ACT( i )    ( DEF_REQ_TYPE_GET( i ) == 0x01 ? 1 : 0 )       /* 绫昏姹� */
#define DEF_REQ_VEN_ACT( i )    ( DEF_REQ_TYPE_GET( i ) == 0x02 ? 1 : 0 )       /* 鍘傚晢鍑嗚姹� */
#define BIT_REQ_CMD7    ( 1<<7 )        /* 璇锋眰鐮佷綅7 */
#define BIT_REQ_CMD6    ( 1<<6 )        /* 璇锋眰鐮佷綅6 */
#define BIT_REQ_CMD5    ( 1<<5 )        /* 璇锋眰鐮佷綅5 */
#define BIT_REQ_CMD4    ( 1<<4 )        /* 璇锋眰鐮佷綅4 */
#define BIT_REQ_CMD3    ( 1<<3 )        /* 璇锋眰鐮佷綅3 */
#define BIT_REQ_CMD2    ( 1<<2 )        /* 璇锋眰鐮佷綅2 */
#define BIT_REQ_CMD1    ( 1<<1 )        /* 璇锋眰鐮佷綅1 */
#define BIT_REQ_CMD0    ( 1<<0 )        /* 璇锋眰鐮佷綅0 */
#define DEF_REQ_CMD_MASK    ( 0xFF )        /* 璇锋眰鐮佹帺鐮� */
#define DEF_REQ_CMD_GET( i )    ( ( i ) & DEF_REQ_CMD_MASK )        /* 鑾峰彇璇锋眰绫诲瀷 */

/* 鍘傚晢鍛戒护瀹氫箟 */
#define USB_VEN_DEBUG_READ              0x95         /* 鍛戒护鐮�: 0x95----璇讳袱缁勫瘎瀛樺櫒 */
#define USB_VEN_DEBUG_WRITE             0x9A         /* 鍛戒护鐮�: 0x9A----鍐欎袱缁勫瘎瀛樺櫒 */
#define USB_VEN_READ_VER                0x96         /* 鍛戒护鐮�: 0X96----璇诲彇鑺墖鐗堟湰鍙� */
  /* 鍙傛暟0 */
  #define   BIT_ARG_UART_TYPE   ( 1<<7 )        /* 绫诲瀷浣嶏紝0锛歎SB杞覆鍙ｏ紝1锛氱綉缁滆浆涓插彛 */
  #define   BIT_ARG_UART_MODE   ( 1<<6 )        /* 妯″紡浣嶏紝0锛�3绾夸覆鍙ｏ紝1锛�9绾夸覆鍙� */
  #define   BIT_ARG_UART_CNT5   ( 1<<5 )        /* 涓插彛璁℃暟浣�5 */
  #define   BIT_ARG_UART_CNT4   ( 1<<4 )        /* 涓插彛璁℃暟浣�4 */
  #define   BIT_ARG_UART_CNT3   ( 1<<3 )        /* 涓插彛璁℃暟浣�3 */
  #define   BIT_ARG_UART_CNT2   ( 1<<2 )        /* 涓插彛璁℃暟浣�2 */
  #define   BIT_ARG_UART_CNT1   ( 1<<1 )        /* 涓插彛璁℃暟浣�1 */
  #define   BIT_ARG_UART_CNT0   ( 1<<0 )        /* 涓插彛璁℃暟浣�0 */
  #define   DEF_ARG_UART_CNT_MASK   ( BIT_ARG_UART_CNT5 | BIT_ARG_UART_CNT4 | BIT_ARG_UART_CNT3 | BIT_ARG_UART_CNT2 | BIT_ARG_UART_CNT1 | BIT_ARG_UART_CNT0 )       /* 涓插彛璁℃暟浣嶆帺鐮� */
  #define   M_ARG_UART_CNT_SET( i, n )  ( ( ( i ) & ~DEF_ARG_UART_CNT_MASK ) | ( ( n ) & DEF_ARG_UART_CNT_MASK ) )

#define USB_VEN_SET_TYPE                0x97         /* 鍛戒护鐮�: 0X97----璁剧疆鑺墖绫诲瀷 */
#define USB_VEN_CH438_READ              0x85         /* 鍛戒护鐮�: 0X85----瀵笴H438璇�1缁勫瘎瀛樺櫒 */
#define USB_VEN_CH438_WRITE             0x8A         /* 鍛戒护鐮�: 0X8A----瀵笴H438鍐�1缁勫瘎瀛樺櫒 */
#define USB_VEN_CH438_RESET             0x8B         /* 鍛戒护鐮�: 0X8B----瀵笴H438杩涜澶嶄綅 */
#define USB_VEN_READ_EEPROM             0x8C         /* 鍛戒护鐮�: 0X8C----瀵笶EPROM杩涜璇绘搷浣� */
#define USB_VEN_WRITE_EEPROM            0x8D         /* 鍛戒护鐮�: 0X8D----瀵笶EPROM杩涜鍐欐搷浣� */

/* CH341鐩稿叧鐨勫懡浠ゅ抚 */
#define DEF_VEN_DEBUG_READ              0X95         /* 璇讳袱缁勫瘎瀛樺櫒 */
#define DEF_VEN_DEBUG_WRITE             0X9A         /* 鍐欎袱缁勫瘎瀛樺櫒 */

#define DEF_VEN_UART_INIT               0XA1         /* 鍒濆鍖栦覆鍙� */
#define DEF_VEN_UART_M_OUT              0XA4         /* 璁剧疆MODEM淇″彿杈撳嚭 */

#define DEF_VEN_BUF_CLEAR               0XB2         /* 娓呴櫎鏈畬鎴愮殑鏁版嵁 */
#define DEF_VEN_I2C_CMD_X               0X54         /* 鍙戝嚭I2C鎺ュ彛鐨勫懡浠�,绔嬪嵆鎵ц */
#define DEF_VEN_DELAY_MS                0X5E         /* 浠ヤ撼绉掍负鍗曚綅寤舵椂鎸囧畾鏃堕棿 */
#define DEF_VEN_GET_VER                 0X5F         /* 鑾峰彇鑺墖鐗堟湰 */

/* CH343鏂板 */
#define DEF_VEN_UART_GPIO               0XA8         /* 璁剧疆GPIO杈撳嚭 */


//1銆�   璇诲瘎瀛樺櫒锛�
//0XC1锛�0X95锛屽湴鍧�1锛屽湴鍧�2锛�0X00锛�0X00锛�0X02锛�0X00
//2銆佸啓瀵勫瓨鍣細
//0X41锛�0X9A锛屽湴鍧�1锛屾暟鎹�1锛屽湴鍧�2锛屾暟鎹�2锛�0X00锛�0X00






/* 閰嶇疆鐩稿叧瀹氫箟 */

/* 涓�浜涢粯璁ゅ弬鏁板畾涔� */
//#define USB_DEV_PARA_FLASH_ADD          0x3E800     /* 纭欢鍦板潃瀹氫箟 */ //0x3EA00
#define USB_DEV_PARA_FLASH_ADD          0x3EA00
#define USB_DEV_PARA_FLAG               0x5A        /* 鍙傛暟瀹氫箟 */
#define USB_DEV_PARA_VER                0x0100
//0x00锛氱‖浠堕�夋嫨妯″紡 0x01锛氬己鍒禖DC妯″紡 0x02锛氬己鍒禫EN妯″紡
#define USB_DEV_PARA_FORCE_MODE         0X00
#define USB_DEV_PARA_RESET_CDC          0X01
#define USB_DEV_PARA_RESET_VEN          0X02
#define USB_DEV_PARA_RESET_ALL          0X03


//CH9342:
//VID PID
//CDC锛�
//0x86, 0x1a,           //鍘傚晢ID
//0x19, 0xE0,           //浜у搧ID
//VEN锛�
//0x86, 0x1A,           //鍘傚晢ID
//0x18, 0xE0,             //浜у搧ID

//CH343  VID= 0x1A86锛孭ID=0x55D3
//CH342  VID= 0x1A86锛孭ID=0x55D2
//CH9143 VID锛�0x1A86   PID锛�0x55D6

#define USB_DEV_PARA_CDC_VID               0X1A86
#define USB_DEV_PARA_CDC_PID               0x55D6 //0X55D3 //0X55D3 //0X55D2  //0X8040
#define USB_DEV_PARA_CDC_SERIAL_STR        "WCHxxxxxxTS1"//xxxxxx涓篒D鐨勫悗涓変釜瀛楄妭
#define USB_DEV_PARA_CDC_PRODUCT_STR       "USB2.0 To Serial Port"
#define USB_DEV_PARA_CDC_MANUFACTURE_STR   "wch.cn"
#define USB_DEV_PARA_CDC_POWER_DESP        0x32
#define USB_DEV_PARA_CDC_RELE_NUM          0x4000  //0x4000 //0x3000    //鐗堟湰鍙�
#define USB_DEV_PARA_CDC_ATTRIBUTE         0x80

//#define USB_DEV_PARA_VEN_VID            0X1A86
//#define USB_DEV_PARA_VEN_PID            0X7523
//#define USB_DEV_PARA_VEN_SERIAL_STR     "WCHxxxxxxTS2"//xxxxxx涓篒D鐨勫悗涓変釜瀛楄妭  //AL03TH04
//#define USB_DEV_PARA_VEN_PRODUCT_STR    "USB2.0 To Serial Port"
//#define USB_DEV_PARA_VEN_MANUFACTURE_STR "wch.cn"
//#define USB_DEV_PARA_VEN_POWER_DESP     0x32
//#define USB_DEV_PARA_VEN_RELE_NUM       0x3100
//#define USB_DEV_PARA_VEN_ATTRIBUTE      0x80


/* 閫氳鐨勫弬鏁拌缃� */
#define USB_CFG_BPS                     300   //杩涜閰嶇疆鐨勬尝鐗圭巼
#define FLAME_HEAD                      0xA1







/* 鍛戒护鐮� */
/* 鍔熻兘 */
#define CMD_GET_VER                     0X01
#define CMD_RESET_PARA                  0X02
#define CMD_RST_CHIP                    0X03
#define CMD_GET_FORCE_MODE              0X04
#define CMD_SET_FORCE_MODE              0X05
#define CMD_SAVE_PARA                   0X06

/* 鍙傛暟閰嶇疆 */
/* CDC */
#define CMD_CDC_GET_VID                 0X10
#define CMD_CDC_SET_VID                 0X11
#define CMD_CDC_GET_PID                 0X12
#define CMD_CDC_SET_PID                 0X13
#define CMD_CDC_GET_SERIAL_STR          0X14
#define CMD_CDC_SET_SERIAL_STR          0X15
#define CMD_CDC_GET_POWER               0X16
#define CMD_CDC_SET_POWER               0X17
#define CMD_CDC_GET_RELEASE             0X18
#define CMD_CDC_SET_RELEASE             0X19
#define CMD_CDC_GET_PRODUCT_STR         0X1A
#define CMD_CDC_SET_PRODUCT_STR         0X1B
#define CMD_CDC_GET_MANU_STR            0X1C
#define CMD_CDC_SET_MANU_STR            0X1D
#define CMD_CDC_GET_ATTRIBUTE           0X1E
#define CMD_CDC_SET_ATTRIBUTE           0X1F
/* VEN */
#define CMD_VEN_GET_VID                 0X30
#define CMD_VEN_SET_VID                 0X31
#define CMD_VEN_GET_PID                 0X32
#define CMD_VEN_SET_PID                 0X33
#define CMD_VEN_GET_SERIAL_STR          0X34
#define CMD_VEN_SET_SERIAL_STR          0X35
#define CMD_VEN_GET_POWER               0X36
#define CMD_VEN_SET_POWER               0X37
#define CMD_VEN_GET_RELEASE             0X38
#define CMD_VEN_SET_RELEASE             0X39
#define CMD_VEN_GET_PRODUCT_STR         0X3A
#define CMD_VEN_SET_PRODUCT_STR         0X3B
#define CMD_VEN_GET_MANU_STR            0X3C
#define CMD_VEN_SET_MANU_STR            0X3D
#define CMD_VEN_GET_ATTRIBUTE           0X3E
#define CMD_VEN_SET_ATTRIBUTE           0X3F


void USB_Init(void);
bool DevEPn_IN_Deal(void);

#ifdef __cplusplus
}
#endif

#endif /* __USBUSER_H__ */
