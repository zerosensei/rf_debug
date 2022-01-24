#include "HAL_FLASH/include/easyflash.h"
#include "device_config.h"
#include "HAL.h"
#include "CH58x_common.h"
#include "hidkbd.h"
#include "hiddev.h"
#include "RF_task/rf_task.h"
#include <string.h>
#include <USB_task/usb_task.h>

__attribute__((aligned(4))) uint8_t device_mode;
__attribute__((aligned(4))) Device_bond_t device_bond;
__attribute__((aligned(4))) Led_info_t device_led;
__attribute__((aligned(4))) RF_Pair_Info_t device_pair;


uint8_t SaveDeviceInfo( char * device_info )
{
    uint8_t ret = SUCCESS;
    bool is_all = false;

    if(strcmp(device_info, "all") == 0){
        is_all = true;
    }

    if(is_all || (strcmp(device_info, "device_pair") == 0))
        ret |= ef_set_env_blob("device_pair", &device_pair, sizeof(device_pair));

    return ret;
}


void ResetDeviceInfo(char * device_info)
{
    bool is_all = false;

    if(strcmp(device_info, "all") == 0){
        SaveDeviceInfo("all");
        return ;
    }


    if(strcmp(device_info, "device_pair") == 0){
        device_pair.Pair_flag = 0;
        device_pair.Mac_addr = DEFAULT_MAC;
        SaveDeviceInfo(device_info);
    }
}

void ReadDeviceInfo( char * device_info )
{
    bool is_all = false;

    if(strcmp(device_info, "all") == 0){
        is_all = true;
    }


    if(is_all || (strcmp(device_info, "device_pair") == 0))
    if(ef_get_env_blob("device_pair", &device_pair, sizeof(device_pair), NULL) == 0){
        device_pair.Pair_flag = 0;
        device_pair.Mac_addr = DEFAULT_MAC;
        SaveDeviceInfo("device_pair");
    }

    LOG_INFO("my mac addr=%#x", device_pair.Mac_addr);
}



