/**
  * @file    drv_can.c
  * @brief   FDCAN1 glue. Bit timing comes from CubeMX; this file adds TDC,
  *          filters, notifications and bus-off recovery.
  */
#include "drv_can.h"
#include "app_config.h"
#include "can_proto.h"
#include "dbg_log.h"
#include "fdcan.h"
#include "ring_buf.h"

static const uint32_t accept_ids[] = CAN_RX_ACCEPT_IDS;
#define ACCEPT_ID_COUNT (sizeof(accept_ids) / sizeof(accept_ids[0]))

static ring_buf_t    rx_ring;
static volatile bool busoff_pending;
static bool          recovering;
static uint32_t      recovery_ms;
static uint32_t      retries;
static uint32_t      busoff_count;

/* One dual-ID filter per pair of accepted IDs; an odd last ID is doubled. */
static void config_filters(void)
{
  FDCAN_FilterTypeDef f = {0};

  if (((ACCEPT_ID_COUNT + 1U) / 2U) > CAN_HANDLE.Init.StdFiltersNbr)
  {
    Error_Handler();
  }
  f.IdType = FDCAN_STANDARD_ID;
  f.FilterType = FDCAN_FILTER_DUAL;
  f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  for (uint32_t i = 0U; i < ACCEPT_ID_COUNT; i += 2U)
  {
    f.FilterIndex = i / 2U;
    f.FilterID1 = accept_ids[i];
    f.FilterID2 = accept_ids[((i + 1U) < ACCEPT_ID_COUNT) ? (i + 1U) : i];
    if (HAL_FDCAN_ConfigFilter(&CAN_HANDLE, &f) != HAL_OK)
    {
      Error_Handler();
    }
  }
  if (HAL_FDCAN_ConfigGlobalFilter(&CAN_HANDLE, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }
}

void drv_can_init(void)
{
  ring_buf_init(&rx_ring);
  if ((HAL_FDCAN_ConfigTxDelayCompensation(&CAN_HANDLE,
         CAN_TDC_OFFSET(CAN_HANDLE.Init.DataPrescaler, CAN_HANDLE.Init.DataTimeSeg1),
         CAN_TDC_FILTER) != HAL_OK)
      || (HAL_FDCAN_EnableTxDelayCompensation(&CAN_HANDLE) != HAL_OK))
  {
    Error_Handler();
  }
  config_filters();
  if ((HAL_FDCAN_ActivateNotification(&CAN_HANDLE,
         FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, 0U) != HAL_OK)
      || (HAL_FDCAN_Start(&CAN_HANDLE) != HAL_OK))
  {
    Error_Handler();
  }
}

bool drv_can_receive(can_frame_t *frame)
{
  if (!ring_buf_pop(&rx_ring, frame))
  {
    return false;
  }
  retries = 0U;
  return true;
}

bool drv_can_send(const can_frame_t *frame)
{
  FDCAN_TxHeaderTypeDef tx = {0};

  tx.Identifier = frame->id;
  tx.IdType = FDCAN_STANDARD_ID;
  tx.TxFrameType = FDCAN_DATA_FRAME;
  tx.DataLength = can_len_to_dlc(frame->len);  /* HAL G4: DataLength is the DLC code */
  tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx.BitRateSwitch = FDCAN_BRS_ON;
  tx.FDFormat = FDCAN_FD_CAN;
  tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx.MessageMarker = 0U;
  return HAL_FDCAN_AddMessageToTxFifoQ(&CAN_HANDLE, &tx, frame->data) == HAL_OK;
}

static bool restart(void)
{
  return (HAL_FDCAN_Stop(&CAN_HANDLE) == HAL_OK) && (HAL_FDCAN_Start(&CAN_HANDLE) == HAL_OK);
}

/* After a bus-off the FDCAN stays in INIT. Restarting it clears INIT and the
 * hardware then waits the mandatory 129 x 11 recessive bits before rejoining. */
bool drv_can_service(uint32_t now_ms)
{
  bool detected = __atomic_exchange_n(&busoff_pending, false, __ATOMIC_ACQ_REL);

  if (detected)
  {
    busoff_count++;
    recovering = true;
    recovery_ms = now_ms;
    dbg_log("CAN bus-off #%lu", (unsigned long)busoff_count);
  }
  if (recovering && (retries < CAN_BUSOFF_MAX_RETRIES)
      && ((now_ms - recovery_ms) >= CAN_BUSOFF_RETRY_DELAY_MS))
  {
    retries++;
    recovery_ms = now_ms;
    recovering = !restart();
    if (recovering)
    {
      dbg_log("CAN restart %lu failed", (unsigned long)retries);
    }
  }
  return detected;
}

bool drv_can_is_healthy(void)
{
  return !(recovering && (retries >= CAN_BUSOFF_MAX_RETRIES));
}

/* ISR: copy every pending frame into the ring and return. No decoding. */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef rx;
  can_frame_t frame;

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
  {
    return;
  }
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
  {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, frame.data) != HAL_OK)
    {
      return;
    }
    frame.id = rx.Identifier;
    frame.len = can_dlc_to_len((uint8_t)rx.DataLength);
    (void)ring_buf_push(&rx_ring, &frame);
  }
}

/* ISR: flag bus-off entry; the main loop stops the motors and recovers. */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  FDCAN_ProtocolStatusTypeDef status;

  if (((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U)
      && (HAL_FDCAN_GetProtocolStatus(hfdcan, &status) == HAL_OK) && (status.BusOff != 0U))
  {
    busoff_pending = true;
  }
}
