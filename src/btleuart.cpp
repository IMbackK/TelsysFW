#include "btleuart.h"

#include "ringbuffer.h"

//sdk includes
extern "C"
{
    #include "nordic_common.h"
    #include "nrf.h"
    #include "boards.h"
    #include "app_error.h"
    #include "ble.h"
    #include "ble_hci.h"
    #include "ble_srv_common.h"
    #include "ble_advdata.h"
    #include "ble_conn_params.h"
    #include "ble_conn_state.h"
    #include "nrf_sdh.h"
    #include "nrf_sdh_ble.h"
    #include "app_timer.h"
    #include "ble_lbs.h"
    #include "nrf_ble_gatt.h"
}

#define DEVICE_NAME                     "beaconName"                         /**< Name of device. Will be included in the advertising data. */

#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */
#define LINK_TOTAL                      NRF_SDH_BLE_PERIPHERAL_LINK_COUNT + NRF_SDH_BLE_CENTRAL_LINK_COUNT

#define APP_ADV_INTERVAL                MSEC_TO_UNITS(100, UNIT_0_625_MS)        /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2    /**< Reply when unsupported features are requested. */

/**@brief   Priority of the application BLE event handler.
 * @note    You shouldn't need to modify this value.
 */
#define APP_BLE_OBSERVER_PRIO    1
#define NRF_APP_PRIORITY_LOW 6

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};

BLE_LBS_DEF(m_lbs);                                                             //LED Button Service instance. (Why do we need this?)
NRF_BLE_GATT_DEF(m_gatt);                                                       //GATT module instance.

RingBuffer<128> ringBuffer; //Bluetooth ring buffer.

BtleUart::BtleUart()
{
    app_timer_init();
    _bleStackInit();
    _gapParamsInit();
    nrf_ble_gatt_init(&m_gatt, NULL);
    _servicesInit();
    _advertisingInit();
    _connectionParamatersInit();
}

//Enable Advertising
void BtleUart::start()
{
    ble_gap_adv_params_t adv_params;
    
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params, APP_BLE_CONN_CFG_TAG);
}

void BtleUart::_bleStackInit()
{
  nrf_sdh_enable_request();

  // Configure the BLE stack using the default settings.
  // Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);

  // Enable BLE stack.
  nrf_sdh_ble_enable(&ram_start);

  // Register a handler for BLE events.
  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, _btleEventHandler, NULL);
}


//Set Generic Access Profile info
void BtleUart::_gapParamsInit()
{
  ble_gap_conn_params_t   gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));

  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency     = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

  sd_ble_gap_ppcp_set(&gap_conn_params);
}

void BtleUart::_servicesInit()
{
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = _btleDataHandler;

    ble_nus_init(&m_nus, &nus_init);
}

//initalize btle advertising
void BtleUart::_advertisingInit()
{
    ble_advdata_t          advdata;
    ble_advdata_t          scanrsp;
    ble_adv_modes_config_t options;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    ble_advertising_init(&advdata, &scanrsp, &options, NULL, NULL);
}

static void BtleUart:_connParamsEvtHandler(ble_conn_params_evt_t * event)
{
    if (event->evt_type == BLE_CONN_PARAMS_EVT_FAILED) sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
}

//initalize CP module;
void BtleUart::_connectionParamatersInit()
{
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
}

void BtleUart::_connected(const ble_gap_evt_t * const p_gap_evt)
{
  uint32_t    periph_link_cnt = ble_conn_state_n_peripherals(); // Number of peripheral links.

  if (periph_link_cnt == NRF_SDH_BLE_PERIPHERAL_LINK_COUNT)
  {
  }
  else
  {
    // Continue advertising. More connections can be established because the maximum link count has not been reached.
    start();
  }
}

void BtleUart::_disconnected(ble_gap_evt_t const * const p_gap_evt)
{
  uint32_t    periph_link_cnt = ble_conn_state_n_peripherals(); // Number of peripheral links.
  if (periph_link_cnt == (NRF_SDH_BLE_PERIPHERAL_LINK_COUNT - 1))
  {
    // Advertising is not running when all connections are taken, and must therefore be started.
    start();
  }
}

void BtleUart::_onAuthorizeRequest(ble_evt_t const * p_ble_evt)
{
  ret_code_t                            err_code;
  ble_gatts_evt_rw_authorize_request_t  req;
  ble_gatts_rw_authorize_reply_params_t auth_reply;

  req = p_ble_evt->evt.gatts_evt.params.authorize_request;

  if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
  {
    if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
        (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
        (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
    {
      if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
      {
        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
      }
      else
      {
        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
      }
      auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
      err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle, &auth_reply);
      APP_ERROR_CHECK(err_code);
    }
  }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
void BtleUart::_btleEventHandler(ble_evt_t const * p_ble_evt, void * p_context)
{
  ret_code_t err_code;

  switch (p_ble_evt->header.evt_id)
  {
    case BLE_GAP_EVT_CONNECTED:
      _connected(&p_ble_evt->evt.gap_evt);
      break;

    case BLE_GAP_EVT_DISCONNECTED:
      _disconnected(&p_ble_evt->evt.gap_evt);
      break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
      // Pairing not supported
      err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle,
          BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
          NULL,
          NULL);
      APP_ERROR_CHECK(err_code);
      break;

#if defined(S132)
    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
      {
        NRF_LOG_DEBUG("PHY update request.");
        ble_gap_phys_t const phys = { BLE_GAP_PHY_AUTO, BLE_GAP_PHY_AUTO };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);
      } break;
#endif

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
      // No system attributes have been stored.
      err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gap_evt.conn_handle, NULL, 0, 0);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTC_EVT_TIMEOUT:
      // Disconnect on GATT Client timeout event.
      NRF_LOG_DEBUG("GATT Client Timeout.");
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
          BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTS_EVT_TIMEOUT:
      // Disconnect on GATT Server timeout event.
      NRF_LOG_DEBUG("GATT Server Timeout.");
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
          BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_EVT_USER_MEM_REQUEST:
      err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
      _onAuthorizeRequest(p_ble_evt);
      break;

    default:
      break;
  }
}

bool BtleUart::write(uint8_t* buffer, uint16_t length)
{
    if( length =< BLE_NUS_MAX_DATA_LEN )
    {
        ble_nus_string_send(&m_nus, buffer, length);
        return true;
    }
    else return false;
}

void BtleUart::_btleDataHandler(ble_nus_t* p_nus, uint8_t* data, uint16_t length)
{
    ringBuffer.write(data, length);
}
