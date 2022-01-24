#include "uart_task.h"
#include "uart/uart.h"
#include "CH58xBLE_LIB.h"
#include "HAL/HAL.h"
#include "RF_task/rf_task.h"
#include "CH58x_common.h"

uint8_t uart_task_id = 0;

static void uart_ProcessTMOSMsg(tmos_event_hdr_t *pMsg)
{
    switch (pMsg->event) {

    case UART_START_MONIT_EVT:{
        SendMSG_t *msg = (SendMSG_t *)pMsg;
        msg->hdr.status?tmos_set_event(uart_task_id, UART_START_EVT):0;
    }
        break;

    default:
        break;
    }
}

uint16_t uart_ProcessEvent(uint8 task_id, uint16 events)
{

    if (events & SYS_EVENT_MSG) {
        uint8 *pMsg;

        if ((pMsg = tmos_msg_receive(uart_task_id)) != NULL) {
            uart_ProcessTMOSMsg((tmos_event_hdr_t *) pMsg);
            // Release the TMOS message
            tmos_msg_deallocate(pMsg);
        }
        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }


    if(events & UART_START_EVT){
        PRINT("start uart0 monitor\n");
        tmos_start_reload_task(uart_task_id, UART_TIMED_EVT, MS1_TO_SYSTEM_TIME(100));
        return (events ^ UART_START_EVT);
    }


    /* 空闲每隔100ms执行一次*/
    if(events & UART_TIMED_EVT){
        if(is_uart0_got_data()){
            OnBoard_SendMsg(RFtaskID, KEY_MESSAGE, 1, 0);
//            tmos_stop_task(uart_task_id, UART_TIMED_EVT);
        }
        return (events ^ UART_TIMED_EVT);
    }

    return 0;
}


void uart_task_init(void)
{
    uart0_init();
    uart_task_id = TMOS_ProcessEventRegister(uart_ProcessEvent);

//    tmos_start_reload_task(uart_task_id, UART_TIMED_EVT, MS1_TO_SYSTEM_TIME(100));

}
