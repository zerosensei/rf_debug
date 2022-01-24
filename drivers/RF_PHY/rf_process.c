/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : RF_PHY.c
* Version            : V1.0
* Date               : 2020/08/06
* Description        : 
*******************************************************************************/

/******************************************************************************/
/* ͷ�ļ����� */
#include "CONFIG.h"
#include "CH58x_common.h"
#include "HAL.h"

#include "rf_process.h"
#include "rf_AC.h"
#include "rf_pair.h"
#include "device_config.h"

#ifdef CONFIG_RF_PRC_DEBUG
#define RF_PRC_DBG  PRINT
#else
#define RF_PRC_DBG
#endif
/*********************************************************************
 * GLOBAL TYPEDEFS
 */
uint8_t rf_process_taskID = INVALID_TASK_ID;
static uint8_t registeredTaskID = TASK_NO_TASK;
rfConfig_t myrfConfig;

struct rf_status_t rf_status = {
        .idle = 1,
};

static uint8_t send_num = 0;

#if(CONFIG_RF_MODE == 2)
static size_t rxBufferIndex = 0;
static size_t rxBufferLength = 0;
static uint8_t rxBuffer[BUFFER_LENGTH];
#endif

#if(CONFIG_RF_MODE == 1)
static uint8_t send_len = 0;
#endif

#ifdef CONFIG_RF_RESEND
static uint8_t resend_count = 0;
#endif

#ifdef CONFIG_RF_HOP
static bool is_hopping = false;
#endif

/*********************************************************************
 * CALLBACKS
 */
void read_rxcb(uint8_t *data, size_t len);
void tx_respondcb(uint8_t *data, size_t len);
void printRSSIcb(char rssi);

static struct rf_AC_cb ac_cbs = {
   .rf_AC_Rx = read_rxcb,
   .rf_AC_TxResp = tx_respondcb,
   .read_rssi = printRSSIcb,
};

struct rf_process_cb * rf_pro_cb;

void rf_pro_RegisterCB( struct rf_process_cb *cb )
{
    rf_pro_cb = cb;
}

/*******************************************************************************
* Function Name  : RF_2G4StatusCallBack
* Description    : RF ״̬�ص���ע�⣺�����ڴ˺�����ֱ�ӵ���RF���ջ��߷���API����Ҫʹ���¼��ķ�ʽ����
* Input          : sta - ״̬����
*                   crc - crcУ����
*                   rxBuf - ����bufָ��
* Output         : None
* Return         : None
*******************************************************************************/
void RF_2G4StatusCallBack( uint8 sta , uint8 crc, uint8 *rxBuf )
{
  switch( sta )
	{
    case TX_MODE_TX_FINISH:
    {
      rf_AC(NULL, crc, 0);
      break;
    }
    case TX_MODE_TX_FAIL:
    {
      break;
    }		
    case TX_MODE_RX_DATA:
    {
      rf_AC(rxBuf, crc, 1);
      break;
    }
    case TX_MODE_RX_TIMEOUT:		// Default timeout is 3ms
    {
      rf_AC(NULL, crc, 1);
      break;
    }		
    case RX_MODE_RX_DATA:  //�ɹ����շ���Ӧ�𣬷���Ӧ��
    {
      AC_timesync(rxBuf[1]);
      rf_AC(rxBuf, crc, 0);
      break;
    }
    case RX_MODE_TX_FINISH:
    {
      rf_AC(NULL, crc, 1);
      break;
    }
    case RX_MODE_TX_FAIL:
    {
      rf_AC(NULL, crc, 1);
      break;
    }

    case TX_MODE_HOP_SHUT:
    {
        PRINT("TX_MODE_HOP_SHUT...\n");
      tmos_set_event( rf_process_taskID , SBP_RF_CHANNEL_HOP_TX_EVT );
      break;
    }

    case RX_MODE_HOP_SHUT:
    {
        PRINT("RX_MODE_HOP_SHUT...\n");
      tmos_set_event( rf_process_taskID , SBP_RF_CHANNEL_HOP_RX_EVT );
      break;
    }
  }
}


// TODO: �������ݳɹ������ж���Ҫ��Ϊ��ȷ
__HIGH_CODE
void tx_respondcb(uint8_t *data, size_t len)
{
#if(CONFIG_RF_MODE == 1)
    uint16_t last_num = 0xffff;

  /* ���ݳ���Ϊ0�� ����ʧ��  */
   if(!len)
      return ;

#ifdef CONFIG_RF_RESEND
  resend_count = 0;
#endif /* CONFIG_RF_RESEND */

  rf_buf_t *rsp_buf = (rf_buf_t *)data;

  if(rsp_buf->num == last_num)
      return;

  if(rsp_buf->magicword != MAGIC_WORD)
    return;

  /* update status */
  rf_status.idle = 1;
  rf_status.sending = 0;
  last_num = rsp_buf->num;
  send_num++;

  tmos_stop_task(rf_process_taskID, SBP_RF_TX_TIMEOUT_EVT);


  /* get respond */
  RF_PRC_DBG("tx_respondcb get %d bytes=[", len);
  for(int i = 0; i < len; i++) {
    i && RF_PRC_DBG(" ");
    RF_PRC_DBG("%#x", data[i]);
  }RF_PRC_DBG("]\n\n");


#ifdef CONFIG_RF_PAIR
    int ret =  pair_tx_process((rf_pair_buf_t *)(rsp_buf->data));

    if(ret == 0)
        rf_status.paired = 1;
#endif

#ifdef CONFIG_RF_HOP
/* ���� CONFIG_RF_PAIR ��Գɹ���ʼ��Ƶ�ʣ�δ�����ɹ�ͨ�ź�ʼ��Ƶ */
#ifdef CONFIG_RF_PAIR
    if(likely(device_pair.Pair_flag))
#endif /* CONFIG_RF_PAIR */
    if(unlikely(!is_hopping)) {
        is_hopping = true;
        //���Ͷ˳ɹ��յ�Ӧ�����ʱ100ms��ʼ��Ƶ
        tmos_start_task(rf_process_taskID, SBP_RF_CHANNEL_HOP_TX_EVT, MS1_TO_SYSTEM_TIME(100));
    }
#endif /* CONFIG_RF_HOP */

    if(rf_pro_cb){
        if(rf_pro_cb->tx_respond_deal_cb)
            rf_pro_cb->tx_respond_deal_cb(rsp_buf->data, \
              len - (sizeof(rf_buf_t)-sizeof(rsp_buf->data)));
    }
#endif
  return;
}

__HIGH_CODE
void read_rxcb(uint8_t *data, size_t len)
{
#if(CONFIG_RF_MODE == 2)
    static uint16_t last_num = 0xffff;

    if(!len)
      return ;

    rf_buf_t *rcv_buf = (rf_buf_t *)data;

    if(MAGIC_WORD != rcv_buf->magicword)
      return;



    RF_PRC_DBG("read_rxcb get %d bytes=[", len);
    for(int i = 0; i < len; i++) {
      i && RF_PRC_DBG(" ");
      RF_PRC_DBG("%#x", data[i]);
    }RF_PRC_DBG("]\n");

    rf_buf_t respond_data = {0};

    if(data[0] == last_num){  //�ذ�������
        respond_data.num = send_num;
        respond_data.magicword = MAGIC_WORD;

        rf_set_rxrespdata((uint8_t *)&respond_data, 0, 5);
        return;
    }

    send_num++;
    last_num = data[0];

    respond_data.num = send_num;
    respond_data.magicword = MAGIC_WORD;

    rf_set_rxrespdata((uint8_t *)&respond_data, 0, 5);

  #ifdef CONFIG_RF_PAIR
      pair_rx_process((rf_pair_buf_t *)(rcv_buf->data));
  #endif /* CONFIG_RF_PAIR */


  #ifdef CONFIG_RF_HOP
        /* ���� CONFIG_RF_PAIR ��Գɹ���ʼ��Ƶ�ʣ�δ�����ɹ�ͨ�ź�ʼ��Ƶ */
  #ifdef CONFIG_RF_PAIR
        if(device_pair.Pair_flag)
  #endif /* CONFIG_RF_PAIR */
        if(unlikely(!is_hopping)) {
            is_hopping = true;
            tmos_set_event( rf_process_taskID , SBP_RF_CHANNEL_HOP_RX_EVT);//���ն˳ɹ��յ����ݺ�������ʼ��Ƶ
        }
  #endif /* CONFIG_RF_HOP */

    if(rf_pro_cb && device_pair.Pair_flag){
        if(rf_pro_cb->rx_read_deal_cb)
            rf_pro_cb->rx_read_deal_cb(rcv_buf->data, \
              (len - (sizeof(rf_buf_t) - sizeof(rcv_buf->data))));
    }

    RF_PRC_DBG("\n");
#endif
    return;
}

void printRSSIcb(char rssi)
{
  RF_PRC_DBG("RSSI: %d dBm\n", rssi);
}


/***************** �û�api ************************/
#if(CONFIG_RF_MODE == 2)
int rf_rx_available(void)
{
    return rxBufferLength - rxBufferIndex;
}

int rf_peek(uint8_t *data, size_t len)
{
  if(!rf_status.receiving)
    return -1;

  size_t getlen = rf_rx_available();
  if(getlen > 0){
    tmos_memcpy(data, rxBuffer, getlen);
  }
  return getlen;
}

int rf_read(uint8_t *data ,size_t len)
{
  if(!rf_status.receiving)
    return -1;

  size_t getlen = rf_rx_available();
  if(getlen > 0){
    tmos_memcpy(data, rxBuffer, getlen);
    rxBufferIndex += getlen;
  }
  return getlen;
}
#endif


/* FIXME�� ������շ��ɹ����յ����ݣ����Ƿ��ͷ���ʧӦ�����ݣ����·��ͷ������ظ�����
 * ��Ȼͨ������Ź����ظ����ݣ����ظ����ݷ���ʧ�ܻᵼ���ط�����ɴ������˷�
 */
__HIGH_CODE
bool rf_send(uint8_t *data, size_t len, bool force)
{
#if(CONFIG_RF_MODE == 1)
  /*
   * û��������޷��ٷ���
   * ������ɵĶ���Ϊ�� �ɹ��������ݣ�tx_respond���յ����ݣ������ط�����>5
   * */
  if(!rf_status.idle && !force){
    return false;
  }

  rf_status.idle = 0;
  rf_status.sending = 1;

  uint32_t magic_word = MAGIC_WORD;

  rf_set_txdata(&send_num, 0, sizeof(send_num));
  rf_set_txdata((uint8_t *)&magic_word, 1, sizeof(magic_word));
  rf_set_txdata(data, 5, len);
  send_len = len;  //�ط����ƻ���send_len�� ����send_len�����ݳ���

#ifdef DEBUG
  if(data){
    RF_PRC_DBG("rf send data=[");
    RF_PRC_DBG("%#x", send_num);
    uint8_t *p = (uint8_t *)&magic_word;
    for(int i = 0; i < 4; i++){
        RF_PRC_DBG(" ");
        RF_PRC_DBG("%#x", *p++);
    }
    for (int i = 0; i < len; i++) {
        RF_PRC_DBG(" ");
        RF_PRC_DBG("%#x", data[i]);
    }
    RF_PRC_DBG("]\n");
  }
#endif

  tmos_start_task(rf_process_taskID, SBP_RF_TX_TIMEOUT_EVT,\
                    MS1_TO_SYSTEM_TIME(RF_TX_TIMEOUT_MS));
#endif
  return true;
}

void rf_readtask_register(uint8_t taksid)
{
  registeredTaskID = taksid;
}

struct rf_status_t *get_rf_status(void)
{
    return &rf_status;
}


void set_rf_status(struct rf_status_t *sta)
{
    tmos_memcpy(&rf_status, sta, sizeof(struct rf_status_t));
    RF_PRC_DBG("set rf status: %#x\n", rf_status);
}
/*******************************************************************************
* Function Name  : RF_ProcessEvent
* Description    : RF �¼�����
* Input          : task_id - ����ID
*                   events - �¼���־
* Output         : None
* Return         : None
*******************************************************************************/
uint16 RF_ProcessEvent( uint8 task_id, uint16 events )
{
  static uint8_t rf_timeout_count = 0;
  static bool is_timeou_set = false;

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = tmos_msg_receive( task_id )) != NULL )
    {
      // Release the TMOS message
      tmos_msg_deallocate( pMsg );
    }
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
/* �������ط�5�������ж�Ϊ����ʧ���������˰�
  ��2s�������ʧ3���������ж�Ϊdongle loss
  dongle loss��ʼlow power sleep
  ���°��°��������dongle loss�ı�־ */
#if(CONFIG_RF_MODE == 1)
  if(events & SBP_RF_TX_TIMEOUT_EVT) {
#ifdef CONFIG_RF_RESEND
      if(resend_count >= RF_RESEND_NUM_MAX)
#endif /* CONFIG_RF_RESEND */
      {
          rf_timeout_count++;
#ifdef CONFIG_RF_RESEND
          resend_count = 0;
#endif /* CONFIG_RF_RESEND */
          rf_status.idle = 1;
          rf_status.sending = 0;

          if(!is_timeou_set){
              is_timeou_set = true;
              tmos_start_task(rf_process_taskID, SBP_RF_DGLOSS_TIMEOUT_EVT, MS1_TO_SYSTEM_TIME(2000));
          }

          if(rf_pro_cb){
              if(rf_pro_cb->tx_send_failed_cb)
                  rf_pro_cb->tx_send_failed_cb();
          }
      }
#ifdef CONFIG_RF_RESEND
      else {
          resend_count++;
          RF_PRC_DBG("resend %d\n", resend_count);
          GPIOA_InverseBits(GPIO_Pin_5);
          rf_send(NULL, send_len, true);
      }
#endif /* CONFIG_RF_RESEND */

    return (events ^ SBP_RF_TX_TIMEOUT_EVT);
  }
#endif /* CONFIG_RF_MODE == 1 */

#if(CONFIG_RF_MODE == 1)
  if( events & SBP_RF_DGLOSS_TIMEOUT_EVT ){

      RF_PRC_DBG("dongle lost timeout: %d\n", rf_timeout_count);
      is_timeou_set = false;

      if(rf_timeout_count >= 10){
          rf_status.dongle_lost = 1;
          rf_timeout_count = 0;
          //TODO: send dongle lost msg to other task
          tmos_set_event(halTaskID, HAL_SLEEP_EVENT);
      }


      return events^SBP_RF_DGLOSS_TIMEOUT_EVT;
  }
#endif /* CONFIG_RF_MODE == 1 */

  // ������Ƶ����
  if( events & SBP_RF_CHANNEL_HOP_TX_EVT ){

     uint8_t ret = RF_FrequencyHoppingTx( 16 ) ;
      PRINT("\n------------- hop tx... %d\n", ret);

    if( ret ){
      tmos_start_task( rf_process_taskID , SBP_RF_CHANNEL_HOP_TX_EVT ,100 );
    }

    return events^SBP_RF_CHANNEL_HOP_TX_EVT;
  }

  // ������Ƶ����
  if( events & SBP_RF_CHANNEL_HOP_RX_EVT ){

      uint8_t ret = RF_FrequencyHoppingRx( 200 );
      PRINT("hop rx... %d\n",ret);
    if(ret)
    {
      tmos_start_task( rf_process_taskID , SBP_RF_CHANNEL_HOP_RX_EVT ,100 );
    }
    else
    {
        extern uint8_t rcv_data[BLE_BUFF_MAX_LEN - 7];
        extern size_t rcv_len;
        RF_Rx( rcv_data, rcv_len, 0xFF, 0xFF );
    }
    return events^SBP_RF_CHANNEL_HOP_RX_EVT;
  }
  return 0;
}

#define CONFIG_RF_HOP_INTERVAL      2
/*******************************************************************************
* Function Name  : rf_process_init
* Description    : RF ��ʼ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void rf_process_init( void )
{
  uint8 state;

  rf_pro_cb = NULL;

  rf_process_taskID = TMOS_ProcessEventRegister( RF_ProcessEvent );
  tmos_memset( &myrfConfig, 0, sizeof(rfConfig_t) );
  myrfConfig.accessAddress = device_pair.Mac_addr;	// ��ֹʹ��0x55555555�Լ�0xAAAAAAAA ( ���鲻����24��λ��ת���Ҳ�����������6��0��1 )
  myrfConfig.CRCInit = 0x555555;
  myrfConfig.Channel = 24;
  myrfConfig.Frequency = 2480000;
  myrfConfig.LLEMode = LLE_MODE_AUTO | LLE_MODE_PHY_2M; // ʹ�� LLE_MODE_EX_CHANNEL ��ʾ ѡ�� rfConfig.Frequency ��Ϊͨ��Ƶ��
  myrfConfig.rfStatusCB = RF_2G4StatusCallBack;
#ifdef CONFIG_RF_HOP
  myrfConfig.HopPeriod = 16;
  myrfConfig.HeartPeriod = CONFIG_RF_HOP_INTERVAL;   //CONFIG_RF_HOP_INTERVAL * 100ms ����һ��������
#if ((CONFIG_RF_HOP_INTERVAL>2) && CLK_OSC32K)
#warning "Use internal 32KHz crystal, CONFIG_RF_HOP_INTERVAL cannot be too large!"
#endif
#endif /* CONFIG_RF_HOP */
  state = RF_Config( &myrfConfig );
  RF_PRC_DBG("rf 2.4g init: %x\n",state);

#if (!defined (CONFIG_RF_MODE) || !((CONFIG_RF_MODE == 0) || (CONFIG_RF_MODE== 1) || (CONFIG_RF_MODE == 2)))
#error "Invalid RF_MODE !"
#endif

#ifdef CONFIG_RF_PAIR
  rf_pair_task_init();
  if(device_pair.Pair_flag)
      rf_status.paired = 1;
#endif /* CONFIG_RF_PAIR */

#if (CONFIG_RF_MODE == 2)
  rf_buf_t temp ={
          .num = 0,
          .magicword = MAGIC_WORD,
          .data = {0},
  };

  rf_set_rxrespdata((uint8_t *)&temp, 0, sizeof(temp) - sizeof(temp.data));
  rf_status.receiving = 1;
#endif /* CONFIG_RF_MODE */

  rf_ACInit(&myrfConfig);
  rf_AC_RegisterCB(&ac_cbs);

}



/******************************** endfile @ main ******************************/