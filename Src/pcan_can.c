
#include <stm32f0xx_hal.h>
#include <string.h>
#include <assert.h>
#include "pcan_can.h"
#include "pcan_timestamp.h"

#define CAN_TX_FIFO_SIZE (100)
static CAN_HandleTypeDef g_hcan = { .Instance = CAN };
#define INTERNAL_CAN_IT_FLAGS          (  CAN_IT_TX_MAILBOX_EMPTY |\
                                          CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING |\
                                          CAN_IT_BUSOFF |\
                                          CAN_IT_ERROR_WARNING |\
                                          CAN_IT_ERROR_PASSIVE |\
                                          CAN_IT_BUSOFF |\
                                          CAN_IT_LAST_ERROR_CODE |\
                                          CAN_IT_ERROR )

static struct
{
  uint32_t tx_msgs;
  uint32_t tx_errs;
  uint32_t tx_ovfs;

  uint32_t rx_msgs;
  uint32_t rx_errs;
  uint32_t rx_ovfs;

  can_message_t tx_fifo[CAN_TX_FIFO_SIZE];
  uint32_t tx_head;
  uint32_t tx_tail;
  void (*rx_cb)(can_message_t *);
  void (*can_err_cb)( uint8_t err, uint8_t rx_err, uint8_t tx_err );
}
can_dev = { 0 };

void pcan_can_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  CAN_FilterTypeDef filter = { 0 };

  __HAL_RCC_CAN1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_CAN;
  HAL_GPIO_Init( GPIOB, &GPIO_InitStruct );

  HAL_CAN_DeInit( &g_hcan );

  g_hcan.Instance = CAN;
  g_hcan.Init.Prescaler = 16;
  g_hcan.Init.Mode = CAN_MODE_NORMAL;
  g_hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  g_hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  g_hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  g_hcan.Init.TimeTriggeredMode = DISABLE;
  g_hcan.Init.AutoBusOff = ENABLE;
  g_hcan.Init.AutoWakeUp = DISABLE;
  g_hcan.Init.AutoRetransmission = ENABLE;
  g_hcan.Init.ReceiveFifoLocked = DISABLE;
  g_hcan.Init.TransmitFifoPriority = ENABLE;

  if( HAL_CAN_Init( &g_hcan ) != HAL_OK )
  {
    assert( 0 );
  }

  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.FilterBank = 0;
  filter.SlaveStartFilterBank = 0;
  
  if( HAL_CAN_ConfigFilter( &g_hcan, &filter ) != HAL_OK )
  {
    assert( 0 );
  }

  if( HAL_CAN_ActivateNotification( &g_hcan, INTERNAL_CAN_IT_FLAGS ) != HAL_OK )
  {
    assert( 0 );
  }
}

void pcan_can_set_bitrate( uint16_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw )
{
  static const uint32_t sjw_table[] = 
  { 
    CAN_SJW_1TQ, CAN_SJW_2TQ, CAN_SJW_3TQ, CAN_SJW_4TQ 
  };
  static const uint32_t tseg1_table[] = 
  { 
    CAN_BS1_1TQ, CAN_BS1_2TQ, CAN_BS1_3TQ, CAN_BS1_4TQ, 
    CAN_BS1_5TQ, CAN_BS1_6TQ, CAN_BS1_7TQ, CAN_BS1_8TQ, 
    CAN_BS1_9TQ, CAN_BS1_10TQ, CAN_BS1_11TQ, CAN_BS1_12TQ,
    CAN_BS1_13TQ, CAN_BS1_14TQ, CAN_BS1_15TQ, CAN_BS1_16TQ
  };
  static const uint32_t tseg2_table[] =
  { 
    CAN_BS2_1TQ, CAN_BS2_2TQ, CAN_BS2_3TQ, CAN_BS2_4TQ,
    CAN_BS2_5TQ, CAN_BS2_6TQ, CAN_BS2_7TQ, CAN_BS2_8TQ
  };

  if( sjw > 4 )
    sjw = 4;
  if( tseg1 > 16 )
    tseg1 = 16;
  if( tseg2 > 8 )
    tseg2 = 8;

  /* CAN bus freq is 48 */
  g_hcan.Init.Prescaler = brp * 6;

  g_hcan.Init.SyncJumpWidth = sjw_table[sjw - 1];
  g_hcan.Init.TimeSeg1 = tseg1_table[tseg1 - 1];
  g_hcan.Init.TimeSeg2 = tseg2_table[tseg2 - 1];

  if( HAL_CAN_Init( &g_hcan ) != HAL_OK )
  {
    assert( 0 );
  }
}

void pcan_can_install_rx_callback( void (*cb)( can_message_t * ) )
{
  can_dev.rx_cb = cb;
}

void pcan_can_install_error_callback( void (*cb)( uint8_t, uint8_t, uint8_t ) )
{
  can_dev.can_err_cb = cb;
}

static int pcan_try_send_message( const can_message_t *p_msg )
{
  CAN_TxHeaderTypeDef msg = { .TransmitGlobalTime = DISABLE };
  uint32_t txMailbox = 0;

  if( p_msg->flags & CAN_FLAG_EXTID )
  {
    msg.ExtId = p_msg->id & 0x1FFFFFFF;
    msg.IDE = CAN_ID_EXT;
  }
  else
  {
    msg.StdId = p_msg->id & 0x7FF;
    msg.IDE = CAN_ID_STD;
  }
  
  msg.DLC = p_msg->dlc;
  msg.RTR = (p_msg->flags & CAN_FLAG_RTR)?CAN_RTR_REMOTE:CAN_RTR_DATA;
  
  if( HAL_CAN_AddTxMessage( &g_hcan, &msg, (void*)p_msg->data, &txMailbox ) != HAL_OK )
    return -1;

  return txMailbox;
}

static void pcan_can_flush_tx( void )
{
  can_message_t *p_msg;

  /* empty fifo */
  if( can_dev.tx_head == can_dev.tx_tail )
    return;
  
  p_msg = &can_dev.tx_fifo[can_dev.tx_tail];
  if( pcan_try_send_message( p_msg ) < 0 )
    return;
  /* update fifo index */
  uint32_t tail = can_dev.tx_tail+1;
  if( tail == CAN_TX_FIFO_SIZE )
    tail = 0;
  can_dev.tx_tail = tail;
}

int pcan_can_send_message( const can_message_t *p_msg )
{
  if( !p_msg )
    return 0;

  uint32_t head = can_dev.tx_head+1;
  if( head == CAN_TX_FIFO_SIZE )
    head = 0;
  /* overflow ? just skip it */
  if( head == can_dev.tx_tail )
  {
    ++can_dev.tx_ovfs;
    return -1;
  }

  can_dev.tx_fifo[can_dev.tx_head] = *p_msg;
  can_dev.tx_head = head;

  return 0;
}

void pcan_can_set_silent( uint8_t silent_mode )
{
  g_hcan.Init.Mode = silent_mode ? CAN_MODE_SILENT: CAN_MODE_NORMAL;
  if( HAL_CAN_Init( &g_hcan ) != HAL_OK )
  {
    assert( 0 );
  }
}

void pcan_can_set_loopback( uint8_t loopback )
{
  g_hcan.Init.Mode = loopback ? CAN_MODE_LOOPBACK: CAN_MODE_NORMAL;
  if( HAL_CAN_Init( &g_hcan ) != HAL_OK )
  {
    assert( 0 );
  }
}

void pcan_can_set_bus_active( uint16_t mode )
{
  if( mode )
  {
    HAL_CAN_Start( &g_hcan );
    HAL_CAN_AbortTxRequest( &g_hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2 );
  }
  else
  {
    HAL_CAN_AbortTxRequest( &g_hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2 );
    HAL_CAN_Stop( &g_hcan );
  }
}

static void pcan_can_rx_frame( CAN_HandleTypeDef *hcan, uint32_t fifo )
{
  CAN_RxHeaderTypeDef hdr = { 0 };
  can_message_t msg = { 0 };

  if( HAL_CAN_GetRxMessage( hcan, fifo, &hdr, msg.data ) != HAL_OK )
    return;

  if( hdr.IDE == CAN_ID_STD )
  {
    msg.id = hdr.StdId;
  }
  else
  {
    msg.id = hdr.ExtId;
    msg.flags |= CAN_FLAG_EXTID;
  }

  if( hdr.RTR == CAN_RTR_REMOTE )
  {
    msg.flags |= CAN_FLAG_RTR;
  }

  msg.dlc = hdr.DLC;
  msg.timestamp = pcan_timestamp_ticks();
  
  if( can_dev.rx_cb )
  {
    can_dev.rx_cb( &msg );
  }

  ++can_dev.rx_msgs;
}

void pcan_can_poll(void)
{
  HAL_CAN_IRQHandler( &g_hcan );
  pcan_can_flush_tx();
}

void HAL_CAN_RxFifo0MsgPendingCallback( CAN_HandleTypeDef *hcan )
{
  pcan_can_rx_frame( hcan, CAN_RX_FIFO0 );
}

void HAL_CAN_RxFifo1MsgPendingCallback( CAN_HandleTypeDef *hcan )
{
  pcan_can_rx_frame( hcan, CAN_RX_FIFO1 );
}

void HAL_CAN_TxMailbox0CompleteCallback( CAN_HandleTypeDef *hcan )
{
  UNUSED( hcan );
  ++can_dev.tx_msgs;
}

void HAL_CAN_TxMailbox1CompleteCallback( CAN_HandleTypeDef *hcan )
{
  UNUSED( hcan );
  ++can_dev.tx_msgs;
}

void HAL_CAN_TxMailbox2CompleteCallback( CAN_HandleTypeDef *hcan )
{
  UNUSED( hcan );
  ++can_dev.tx_msgs;
}

void HAL_CAN_RxFifo0FullCallback( CAN_HandleTypeDef *hcan )
{
  UNUSED( hcan );
  ++can_dev.rx_ovfs;
}

void HAL_CAN_RxFifo1FullCallback( CAN_HandleTypeDef *hcan )
{
  UNUSED( hcan );
  ++can_dev.rx_ovfs;
}

void HAL_CAN_SleepCallback( CAN_HandleTypeDef *hcan ){ UNUSED( hcan ); }
void HAL_CAN_WakeUpFromRxMsgCallback( CAN_HandleTypeDef *hcan ){ UNUSED( hcan ); }

void HAL_CAN_ErrorCallback( CAN_HandleTypeDef *hcan )
{
  /* handle errors */
  uint32_t err = HAL_CAN_GetError( hcan );
  uint8_t  can_err = 0;

  if ( err & ( HAL_CAN_ERROR_TX_TERR0 | HAL_CAN_ERROR_TX_TERR1 | HAL_CAN_ERROR_TX_TERR2 ) ) 
  {
    ++can_dev.tx_errs;
    can_err |= CAN_ERROR_FLAG_TX_ERR;
  }

  if( err & HAL_CAN_ERROR_BOF )
  {
    can_err |= CAN_ERROR_FLAG_BUSOFF;
  }

  if( err & ( HAL_CAN_ERROR_RX_FOV0 | HAL_CAN_ERROR_RX_FOV1 ) )
  {
    can_err |= CAN_ERROR_FLAG_RX_OVF;
  }

  if( can_dev.can_err_cb && can_err )
  {
    can_dev.can_err_cb( can_err, can_dev.tx_errs & 0xFF, can_dev.rx_errs );
  }
  
  HAL_CAN_ResetError( hcan );
}
