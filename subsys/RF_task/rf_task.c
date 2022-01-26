#include "CONFIG.h"
#include "CH58x_common.h"
#include "HAL.h"
#include "rf_task.h"
#include "RingBuffer/lwrb.h"
#include "RF_PHY/rf_pair.h"
#include "RF_PHY/rf_process.h"
#include "uart_task/uart_task.h"
#include "USB_task/usb_task.h"
#include "sys/atomic.h"

tmosTaskID RFtaskID;
#if (CONFIG_RF_MODE == 1)
void tx_respond(uint8_t *buf, uint8_t len);
void tx_send_failed(void);
int rf_tx_deal(bool force);

static uint8_t send_buf[200] = { 0 };
static size_t send_len = 0;
#endif

#if(CONFIG_RF_MODE == 2)
void rf_read_deal(uint8_t *data, uint8_t len);
#endif


struct rf_process_cb rf_process_cbs = {
#if (CONFIG_RF_MODE == 1)
        .tx_respond_deal_cb = tx_respond,
        .tx_send_failed_cb = tx_send_failed,
#endif
#if (CONFIG_RF_MODE == 2)
        .rx_read_deal_cb = rf_read_deal,
#endif
};

#if (CONFIG_RF_MODE == 2)
void rf_read_deal(uint8_t *data, uint8_t len)
{

    PRINT("got %d bytes from rf=[", len);
    for(int i = 0; i < len; i++){
        if(i) PRINT(" ");
        PRINT("%#x", data[i]);
    }PRINT("]\n");

    if(lwrb_write(&RF_RCV, data, len) == len);
        OnBoard_SendMsg(USBTaskID, KEY_MESSAGE, 1, NULL);

}
#endif


#if (CONFIG_RF_MODE == 1)
void tx_send_failed(void)
{
    PRINT("send failed!\n");

    struct rf_status_t temp_sta = {
            .dongle_lost = 0,
            .idle = 1,
    };
    set_rf_status(&temp_sta);

//    tmos_start_task(RFtaskID, RF_SEND_EVENT, MS1_TO_SYSTEM_TIME(1));
}

__attribute__((section(".highcode")))
void tx_respond(uint8_t *buf, uint8_t len)
{
    lwrb_skip(&RF_SEND, send_len);
    LOG_INFO("tx success\n");
}


#define FULL_LEN        64   //虽然不知道为什么 但是改成64速度直接拉满而且不丢数据了

__HIGH_CODE
int rf_tx_deal(bool force)
{
    static bool is_set = false;

    send_len = lwrb_get_full(&RF_SEND);
    if(unlikely(send_len == 0))
        return -1;

    if(send_len < FULL_LEN && !force){
        if(is_set)
            return -1;

        is_set = true;
        tmos_start_task(RFtaskID, RF_SEND_LATER_EVENT, MS1_TO_SYSTEM_TIME(10));  //921600 bps 10ms 1000 bytes
        return -1;
    }
    is_set = false;

    send_len = min(send_len, FULL_LEN);

    lwrb_peek(&RF_SEND, 0, send_buf, send_len);

    rf_send((uint8_t *)&send_buf, send_len, false);

    return 1;
}
#endif



static void RF_ProcessTMOSMsg(tmos_event_hdr_t *pMsg)
{
    switch (pMsg->event) {
    case KEY_MESSAGE:{
        SendMSG_t *msg = (SendMSG_t *)pMsg;
        msg->hdr.status?tmos_set_event(RFtaskID, RF_SEND_EVENT):0;
    }  break;

    case RF_PAIR_MESSAGE:{
        SendMSG_t *msg = (SendMSG_t *)pMsg;
        msg->hdr.status?tmos_set_event(RFtaskID, PAIR_EVENT):0;
    }break;

    default:
        break;
    }
}

uint16 RF_task_Event(uint8 task_id, uint16 events)
{
    if (events & SYS_EVENT_MSG) {
        uint8 *pMsg;

        if ((pMsg = tmos_msg_receive(task_id)) != NULL) {

            RF_ProcessTMOSMsg((tmos_event_hdr_t *) pMsg);
            // Release the TMOS message
            tmos_msg_deallocate(pMsg);
        }
        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

#if (CONFIG_RF_MODE == 1)
    if (events & PAIR_EVENT) {
#ifdef CONFIG_RF_PAIR
        start_pair();
#endif
        return events ^ PAIR_EVENT;
    }

    if (events & RF_SEND_LATER_EVENT) {
        rf_tx_deal(true);

        return events ^ RF_SEND_LATER_EVENT;
    }

    if (events & RF_SEND_EVENT) {

        if(rf_tx_deal(false) > 0){
            tmos_start_task(RFtaskID, RF_SEND_EVENT, 0);
        } else {
            tmos_start_task(RFtaskID, RF_SEND_EVENT, 2);
        }

        return events ^ RF_SEND_EVENT;
    }


#endif /* CONFIG_RF_MODE */

    return 0;
}


void RFtaskInit(void)
{

    RFtaskID = TMOS_ProcessEventRegister(RF_task_Event);

    rf_process_init();
    rf_pro_RegisterCB(&rf_process_cbs);

    if(!device_pair.Pair_flag){
        tmos_start_task(RFtaskID, PAIR_EVENT, MS1_TO_SYSTEM_TIME(1000));
    }
//    tmos_start_task(RFtaskID, PAIR_EVENT, MS1_TO_SYSTEM_TIME(10));
//    tmos_start_task(RFtaskID, RF_TEST_EVENT, MS1_TO_SYSTEM_TIME(1000));

    tmos_start_task(RFtaskID, RF_SEND_EVENT, MS1_TO_SYSTEM_TIME(10));

}

