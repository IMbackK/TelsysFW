/*UVOS*/

/* This file is part of TelemetrySystem.
 *
 * TelemetrySystem is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License (LGPL) version 3 as published by
 * the Free Software Foundation.
 *
 * TelemetrySystem is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with TelemetrySystem.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "btleserial.h"

#include "ringbuffer.h"

extern uint8_t __data_start__;    

//sdk includes
extern "C"
{
   /* #include "nordic_common.h"
    #include "nrf.h"
    #include "boards.h"
    #include "app_error.h"
    #include "ble.h"
    #include "ble_hci.h"
    #include "ble_srv_common.h"
    #include "ble_advdata.h"
    #include "ble_conn_params.h"
    #include "ble_conn_state.h"
    //#include "nrf_sdh.h"
    //#include "nrf_sdh_ble.h"
    #include "app_timer.h"
    #include "ble_lbs.h"
    #include "nrf_ble_gatt.h"*/
}


//these defines where lifted from the NORDIC SDK NRF UART BTLE example.

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */



#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */


/**@brief   Priority of the application BLE event handler.
 * @note    You shouldn't need to modify this value.
 */
#define APP_BLE_OBSERVER_PRIO    1
#define NRF_APP_PRIORITY_LOW 6

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};


static RingBuffer<128> ringBuffer; //Bluetooth ring buffer.

BtleSerial::BtleSerial(Serial* serial): _serial(serial)
{
    serial->write(" app timer init\n");
    
    serial->write(" stack init\n");
    _bleStackInit();
    serial->write(" gap init\n");
    _gapParamsInit();
   //nrf_ble_gatt_init(&m_gatt, NULL);
    serial->write(" services init\n");
    _servicesInit();
    serial->write(" avertising init\n");
    _advertisingInit();
    serial->write(" connection paramaters init\n");
    _connectionParamatersInit();
}

//Enable Advertising
void BtleSerial::start()
{
    ble_gap_adv_params_t adv_params;
    
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    sd_ble_gap_adv_start(&adv_params);
}

void BtleSerial::_bleStackInit()
{
     uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    _serial->write("  softdevice_handler_init");
    
    static uint32_t ble_evt_buffer[CEIL_DIV(BLE_STACK_EVT_MSG_BUF_SIZE, sizeof(uint32_t))];                                                                         
    err_code = softdevice_handler_init(&clock_lf_cfg,ble_evt_buffer, sizeof(ble_evt_buffer), NULL); 
    
    _serial->write(" error code: ");
    _serial->write(err_code);
    _serial->putChar('\n');
    APP_ERROR_CHECK(err_code);       

    ble_enable_params_t ble_enable_params;
    _serial->write("  softdevice_enable_get_default_config\n");
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    _serial->write("  Ram addr\n");
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    /*uint32_t app_ram_base = (uint32_t) &__data_start__;
    err_code = sd_ble_enable(&ble_enable_params, &app_ram_base);

    uint32_t correct_ram = app_ram_base;
    uint32_t current_ram = (uint32_t) &__data_start__;
    _serial->write("   ram schould be ");
    _serial->write(correct_ram);
    _serial->write(" is ");
    _serial->write(current_ram);
     _serial->putChar('\n');*/

    // Enable BLE stack.
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
    _serial->write("  softdevice_enable");
    err_code = softdevice_enable(&ble_enable_params);
    _serial->write(" error code: ");
    _serial->write(err_code);
    _serial->putChar('\n');
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    _serial->write("  softdevice_ble_evt_handler_set");
    err_code = softdevice_ble_evt_handler_set(_btleEventHandler);
    _serial->write(" error code: ");
    _serial->write(err_code);
    _serial->putChar('\n');
    APP_ERROR_CHECK(err_code);
}


//Set Generic Access Profile info
void BtleSerial::_gapParamsInit()
{
  uint32_t                err_code;
  ble_gap_conn_params_t   gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code =  sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)"UVOS Telemetry System", strlen( "UVOS Telemetry System"));
    APP_ERROR_CHECK(err_code);
  
  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency     = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

  
  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);
}

void BtleSerial::_servicesInit()
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = _btleDataHandler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

//initalize btle advertising
void BtleSerial::_advertisingInit()
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

    uint32_t err_code = ble_advertising_init(&advdata, &scanrsp, &options, NULL, NULL);
    APP_ERROR_CHECK(err_code);
}

void BtleSerial::_connParamsEvtHandler(ble_conn_params_evt_t * event)
{
    if (event->evt_type == BLE_CONN_PARAMS_EVT_FAILED) sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
}

//initalize CP module;
void BtleSerial::_connectionParamatersInit()
{
    
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = _connParamsEvtHandler;
    cp_init.error_handler                  = _connParamsErrorHandler;

    uint32_t err_code  = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

void BtleSerial::_connected(const ble_gap_evt_t * const p_gap_evt)
{
  uint32_t    periph_link_cnt = ble_conn_state_n_peripherals(); // Number of peripheral links.

  if (periph_link_cnt == PERIPHERAL_LINK_COUNT)
  {
  }
  else
  {
    // Continue advertising. More connections can be established because the maximum link count has not been reached.
    BtleSerial::start();
  }
}

void BtleSerial::_disconnected(ble_gap_evt_t const * const p_gap_evt)
{
  uint32_t    periph_link_cnt = ble_conn_state_n_peripherals(); // Number of peripheral links.
  if (periph_link_cnt == (PERIPHERAL_LINK_COUNT - 1))
  {
    // Advertising is not running when all connections are taken, and must therefore be started.
    BtleSerial::start();
  }
}

void BtleSerial::_onAuthorizeRequest(ble_evt_t const * p_ble_evt)
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


void BtleSerial::_btleDataHandler(ble_nus_t* p_nus, uint8_t* data, uint16_t length)
{
    ringBuffer.write(data, length);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * 
 */
void BtleSerial::_btleEventHandler(ble_evt_t * p_ble_evt)
{
  ret_code_t err_code;
  
  ble_conn_params_on_ble_evt(p_ble_evt);
  ble_nus_on_ble_evt(&m_nus, p_ble_evt);
  ble_advertising_on_ble_evt(p_ble_evt);
  ble_conn_state_on_ble_evt(p_ble_evt);

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
      
    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
      // No system attributes have been stored.
      err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gap_evt.conn_handle, NULL, 0, 0);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTC_EVT_TIMEOUT:
      // Disconnect on GATT Client timeout event.
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
          BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTS_EVT_TIMEOUT:
      // Disconnect on GATT Server timeout event.
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
      
    case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle, NRF_BLE_MAX_MTU_SIZE);
    break;
    default:
      break;
  }
}

bool BtleSerial::write(uint8_t* buffer, uint32_t length)
{
    if( length <= BLE_NUS_MAX_DATA_LEN && ble_nus_string_send(&m_nus, buffer, length) == NRF_SUCCESS ) return true;
    else return false;
}

bool BtleSerial::write(const char* in, const unsigned int length)
{
    if( length <= BLE_NUS_MAX_DATA_LEN && ble_nus_string_send(&m_nus, (uint8_t*)in, length) == NRF_SUCCESS ) return true;
    else return false;
}

bool BtleSerial::write(const char in[])
{
    return write(in, strlen(in));
}

bool  BtleSerial::write(int32_t in)
{
    if(in == 0)
    {
        return putChar('0');
    }
    else
    {
        bool sign = false;
        char str[64] = { 0 }; 
        int16_t i = 62;
        if (in < 0) 
        {
            sign = true;
            in = abs(in);
        }

        while (in != 0 && i > 0) 
        { 
            str[i--] = (in % 10) + '0';
            in /= 10;
        }

        if (sign) str[i--] = '-';
        return write(str + i + 1, 64-(i+1));
    }
}
bool BtleSerial::putChar(const char c)
{
    return write(&c,1);
}

bool BtleSerial::isConnected()
{
    return ble_conn_state_n_peripherals() > 0;
}
