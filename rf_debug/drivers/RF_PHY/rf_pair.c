#include <stdbool.h>

#include "rf_pair.h"
#include "CH58x_common.h"
#include "rf_process.h"
#include "config.h"
#include "rf_AC.h"

#ifdef CONFIG_RF_PAIR

#ifdef CONFIG_RF_PAIR_DEBUG
#define RF_PAIR_DBG  PRINT
#else
#define RF_PAIR_DBG
#endif
uint8_t rf_pair_task = 0;

static uint8_t pair_tx_step = 0;
static uint8_t pair_rx_step = 0;
static uint8_t Mac_addr[6] = {0};
static uint8_t rcv_addr[4] = {0};

void pair_set_resp(void)
{
    rf_pair_buf_t resp = {0};
    tmos_memcpy(resp.addr, rcv_addr, sizeof(uint32_t));
    rf_set_rxrespdata((uint8_t *)&resp, 5, sizeof(rf_pair_buf_t));
}

void pair_set_txpkt(uint8_t pkt)
{
    switch(pkt){

    case 1:
        tmos_start_task(rf_pair_task, RF_PAIR_TXPKT1_EVT, MS1_TO_SYSTEM_TIME(10));  //10ms后发送 PKT1

        break;

    case 2:
        tmos_start_task(rf_pair_task, RF_PAIR_TXPKT2_EVT, MS1_TO_SYSTEM_TIME(10));  //10ms后发送 PKT2

        break;

    default:
        break;
    }
}

uint8_t pair_success(uint32_t addr)  //TODO: 可以改变算法，让MAC地址更随机
{
    myrfConfig.accessAddress = addr;
    RF_Config(&myrfConfig);

    device_pair.Pair_flag = true;
    device_pair.Mac_addr = addr;
    RF_PAIR_DBG("pair success!");

    return SaveDeviceInfo("device_pair");
}

int pair_rx_process(rf_pair_buf_t *rxbuf)
{
    int ret = -1;
    if(device_pair.Pair_flag)
        return ret;

    switch(pair_rx_step){
    case 0:
        RF_PAIR_DBG("pair step 0");
        if(rxbuf->type == PARI_PACKET1){

            tmos_memcpy(rcv_addr, rxbuf->addr, sizeof(uint32_t));

            pair_set_resp();

            pair_rx_step++;
            tmos_start_task(rf_pair_task, RF_PAIR_RX_TIMEOUT_EVT, MS1_TO_SYSTEM_TIME(200)); //200ms内无法进入下一阶段超时
            ret = 1;
        }break ;

    case 1:
        RF_PAIR_DBG("pair step 1");
        if(rxbuf->type == PARI_PACKET2){
            uint32_t temp = 0;
            temp |= rcv_addr[0]<<0;
            temp |= rcv_addr[1]<<8;
            temp |= rcv_addr[2]<<16;
            temp |= rcv_addr[3]<<24;

            pair_rx_step = pair_success(temp) == SUCCESS?0:1;
            tmos_stop_task(rf_pair_task, RF_PAIR_RX_TIMEOUT_EVT);
            ret = 0;
            
        } else{
            pair_set_resp();
            ret = 2;
        }
        break;
    }
    return ret;
}


int pair_tx_process(rf_pair_buf_t *rxbuf)
{

    int ret = -1;

    if(device_pair.Pair_flag)
        return ret;

   switch(pair_tx_step){
   case 0:
       RF_PAIR_DBG("pair step 0");  //发送本机地址

       pair_tx_step++;
       tmos_start_task(rf_pair_task, RF_PAIR_TX_TIMEOUT_EVT, MS1_TO_SYSTEM_TIME(200)); //200ms内无法进入下一阶段超时
       pair_set_txpkt(1);
       ret = 1;
       break;

   case 1:
       RF_PAIR_DBG("pair step 1");  //收到地址应答，通知对方已收到
       //TODO: 这里注意下大小端问题
       if(true != tmos_memcmp(rxbuf->addr, Mac_addr, sizeof(uint32_t))){
            ret = 1;
            pair_tx_step = 1;

            pair_set_txpkt(1);

            RF_PAIR_DBG("mac error");
            break;
       }

       pair_tx_step++;
       tmos_start_task(rf_pair_task, RF_PAIR_TX_TIMEOUT_EVT, MS1_TO_SYSTEM_TIME(200)); //200ms内无法进入下一阶段超时
       pair_set_txpkt(2);
       ret = 2;
       break;

   case 2:
       RF_PAIR_DBG("pair step 2");   //对方接收到通知

       pair_tx_step = 0;
       tmos_stop_task(rf_pair_task, RF_PAIR_TX_TIMEOUT_EVT);

       uint32_t myaddr = 0;
       myaddr = Mac_addr[0] << 0;
       myaddr |= Mac_addr[1] << 8;
       myaddr |= Mac_addr[2] << 16;
       myaddr |= Mac_addr[3] << 24;
       ret = pair_success(myaddr);
       break;

   default:
       break;
   }

   return ret;
}

uint16_t RF_Pair_ProcessEvent(uint8_t task_id, uint16_t events)
{
    if (events & SYS_EVENT_MSG) {
        uint8 *pMsg;

        if ((pMsg = tmos_msg_receive(task_id)) != NULL) {

            // Release the TMOS message
            tmos_msg_deallocate(pMsg);
        }
        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }


    if(events & RF_PAIR_TX_TIMEOUT_EVT){
        pair_tx_step = 0;
        myrfConfig.accessAddress = device_pair.Mac_addr;;
        RF_Config(&myrfConfig);
        RF_PAIR_DBG("pair_tx timeout!");
        return (events ^ RF_PAIR_TX_TIMEOUT_EVT);
    }

    if(events & RF_PAIR_RX_TIMEOUT_EVT){
        pair_rx_step = 0;
        myrfConfig.accessAddress = device_pair.Mac_addr;
        RF_Config(&myrfConfig);
        RF_PAIR_DBG("pait_rx timeout!");
        return (events ^ RF_PAIR_RX_TIMEOUT_EVT);
    }

    if(events & RF_PAIR_TXPKT1_EVT){
        rf_pair_buf_t pair_tx_buf = {0};
        uint32_t sum = 0;

        pair_tx_buf.type = PARI_PACKET1;

        for(int i = 0; i < 4; i++){
            pair_tx_buf.addr[i] = Mac_addr[i];
            sum += pair_tx_buf.addr[i];
        }

        if(sum){
            rf_send((uint8_t *)&pair_tx_buf , sizeof(rf_pair_buf_t), true);
        }

        return (events ^ RF_PAIR_TXPKT1_EVT);
    }

    if(events & RF_PAIR_TXPKT2_EVT){
        rf_pair_buf_t pair_tx_buf = {0};

        pair_tx_buf.type = PARI_PACKET2;
        rf_send((uint8_t *)&pair_tx_buf , sizeof(rf_pair_buf_t), true);

        return (events ^ RF_PAIR_TXPKT2_EVT);
    }

    return 0;
}

int start_pair(void)
{
    rf_pair_buf_t temp = {0};
    RF_PAIR_DBG("start pair\n");
    myrfConfig.accessAddress = DEFAULT_MAC;
    if(RF_Config(&myrfConfig))
        return -1;
    return rf_send((uint8_t *)&temp, sizeof(temp), false);
}

void rf_pair_task_init(void)
{
    rf_pair_task = TMOS_ProcessEventRegister(RF_Pair_ProcessEvent);

#if ((defined (BLE_MAC)) && (BLE_MAC == TRUE))
       for ( int i = 0; i < 6; i++ )
           Mac_addr[i] = MacAddr[5 - i];
#else
       GetMACAddress( Mac_addr );
#endif
       RF_PAIR_DBG("got MAC=[");
       for(int i = 0; i < 6; i++) {
           if(i) RF_PAIR_DBG(" ");
           RF_PAIR_DBG("%#x", Mac_addr[i]);
       }RF_PAIR_DBG("]\n");
}

#endif /* CONFGI_RF_PAIR */
