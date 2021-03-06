#include "uart.h"
#include "CH58x_common.h"
#include "RingBuffer/lwrb.h"
#include "sys/atomic.h"


ATOMIC_DEFINE(is_uart_timeout, 1);

__attribute__((section(".highcode")))
void UART0_IRQHandler(void)
{
    uint8_t rcv_buf[UART_FIFO_SIZE] = {0};
    uint8_t rcv_len  = 0;
    switch( UART0_GetITFlag() )
    {
        /* uart rx */
        //FIFO 4字节触发
        case UART_II_RECV_RDY:       //接收数据可用
            rcv_len = R8_UART0_RFC;
            rcv_len--; //进入超时
            for(int i = 0; i < rcv_len; i++) {
                rcv_buf[i] = R8_UART0_RBR;
            }

            lwrb_write(&RF_SEND, rcv_buf, rcv_len);
            atomic_set_bit_to(is_uart_timeout, 0, 0);

            break;
        case UART_II_RECV_TOUT:      //接收超时
            rcv_len = R8_UART0_RFC;
            for(int i = 0; i < rcv_len; i++) {
                rcv_buf[i] = R8_UART0_RBR;
            }
            lwrb_write(&RF_SEND, rcv_buf, rcv_len);
            atomic_set_bit_to(is_uart_timeout, 0, 1);

            break;

        case UART_II_LINE_STAT:      //err

            (void)R8_UART0_LSR;
            rcv_len = R8_UART0_RFC;
            for(int i = 0; i < rcv_len; i++) {
               rcv_buf[i] = R8_UART0_RBR;
            }
            lwrb_write(&RF_SEND, rcv_buf, rcv_len);

            break;


         /* uart tx */
        case RB_LSR_TX_FIFO_EMP:  //发送空



          break;

        default:
            break;
    }
}

bool is_uart0_got_data(void)
{
    return !!lwrb_get_full(&RF_SEND);
}

bool set_uart0_bps(uint32_t bps){

    UART0_BaudRateCfg( bps );
    //TODO: 计算误差
    return 0;
}



void uart0_init(void)
{

    GPIOB_ModeCfg(GPIO_Pin_4, GPIO_ModeIN_PU);
    GPIOB_SetBits(GPIO_Pin_7);
    GPIOB_ModeCfg(GPIO_Pin_7, GPIO_ModeOut_PP_5mA);

    UART0_DefInit();

    GPIOB_ModeCfg(GPIO_Pin_17 | GPIO_Pin_16, GPIO_ModeIN_PU);

    if(GPIOB_ReadPortPin(GPIO_Pin_16) == 0){
        DelayMs(100);
        if(GPIOB_ReadPortPin(GPIO_Pin_16) == 0){
            UART0_BaudRateCfg( 115200 );
        }
    } else if (GPIOB_ReadPortPin(GPIO_Pin_17) == 0) {
        DelayMs(100);
        if(GPIOB_ReadPortPin(GPIO_Pin_17) == 0){
            UART0_BaudRateCfg( 921600 );
        }
    } else{
        UART0_BaudRateCfg( 115200 );
    }

    atomic_set_bit_to(is_uart_timeout, 0, 0);

    UART0_ByteTrigCfg( UART_4BYTE_TRIG );
    UART0_INTCfg( ENABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);

    /* 开启中断 */
    PFIC_EnableIRQ(UART0_IRQn);
}
