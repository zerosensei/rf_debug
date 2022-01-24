/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2020/08/06
 * Description        : 外设从机应用主函数及任务系统初始化
 *******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "CH58x_common.h"
#include "HAL/config.h"
#include "HAL/HAL.h"
#include "HAL_FLASH/include/easyflash.h"
#include "device_config.h"
#include "RingBuffer/lwrb.h"
#include "config.h"
#include "RF_task/rf_task.h"
#include "USB_task/usb_task.h"
#include "uart_task/uart_task.h"
#include "RF_PHY/rf_AC.h"



#if (CONFIG_RF_MODE==1)
#pragma message "This is Tx"
#elif(CONFIG_RF_MODE==2)
#pragma message "This is Rx"
#endif

bool isUSBinsert = false;
/*********************************************************************
 * GLOBAL TYPEDEFS
 */
__attribute__((aligned(4)))   u32 MEM_BUF[BLE_MEMHEAP_SIZE / 4];

uint8_t MacAddr[6] = { 0x84, 0xC2, 0xE4, 0x13, 0x33, 0x44 };

/*******************************************************************************
 * Function Name  : Main_Circulation
 * Description    : 主循环
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
__attribute__((section(".highcode")))
void Main_Circulation() {
    while (1) {
        rf_tx_process();
        TMOS_SystemProcess();
    }
}

__HIGH_CODE
void testpirnt(void ){
    while(1){
        static uint32_t i = 0;
        PRINT("this is test count = %d\n", i);
        i++;
        DelayMs(100);
    }
}

/*******************************************************************************
 * Function Name  : main
 * Description    : 主函数
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
int main(void)
{
//    SysTick_Config(FREQ_SYS/1000);  //1ms
//    PowerMonitor(ENABLE, HALevel_2V5);
#if (defined (DCDC_ENABLE)) && (DCDC_ENABLE == TRUE)
    PWR_DCDCCfg(ENABLE);
#endif

#if (defined (HAL_SLEEP)) && (HAL_SLEEP == TRUE)
    GPIOA_ModeCfg( GPIO_Pin_All, GPIO_ModeIN_PD);
    GPIOB_ModeCfg( GPIO_Pin_All, GPIO_ModeIN_PD);
#endif

#ifdef DEBUG
    GPIOA_SetBits(bTXD1);
    GPIOA_ModeCfg(bTXD1, GPIO_ModeOut_PP_5mA);
    UART1_DefInit();
#endif
    DEBUG_Init();

    GPIOA_SetBits(GPIO_Pin_5);
    GPIOA_ModeCfg(GPIO_Pin_5, GPIO_ModeOut_PP_5mA);

    /*DataFlash初始化*/
    if (easyflash_init() != SUCCESS) {
        LOG_INFO("Date Flash init error!");
    }
    ef_print_env();
    ReadDeviceInfo("all");  //must process after easyflash_init()

    ring_buffer_init();

    CH57X_BLEInit( );
    HAL_Init( );

#if (CONFIG_RF_MODE == 1)
    uart_task_init();
#endif
#if (CONFIG_RF_MODE == 2)
    USB_Task_Init();
#endif

    RF_RoleInit( );
    RFtaskInit();

    Main_Circulation();
}

/******************************** endfile @ main ******************************/
