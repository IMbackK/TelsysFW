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

#include "btle.h"

#include <nrf52.h>

extern uint8_t __data_start__;  //needed only by memcheck

#define DEVICE_BLE_NAME "UVOS Telemetry System"

#define TELSYS_BASE_UUID                  {{0x9E, 0xAF, 0x12, 0x58, 0x77, 0x54, 0xA8, 0xD0, 0x84, 0x42, 0x65, 0xB5, 0x00, 0x30, 0x41, 0x66}} //vendor specific uuid

#define UUID_TELSYS_SERVICE 0x0001
#define UUID_RX_CHARACTERISTIC 0x0002                    
#define UUID_ADC_CHARACTERISTIC 0x0003
#define UUID_AUX_CHARACTERISTIC 0x0004

#define MAXIMUM_DATA_LENGTH (GATT_MTU_SIZE_DEFAULT - 3)

//these defines where lifted from the NORDIC SDK NRF UART BTLE example.

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_BLE_OBSERVER_PRIO 1
#define NRF_APP_PRIORITY_LOW  6

static Btle* notification_instance = nullptr;

static uint16_t _connHandle = BLE_CONN_HANDLE_INVALID;             //Handle of the current connection. BLE_CONN_HANDLE_INVALID if not in a connection.

Btle::Btle()
{
    _bleStackInit();
    _gapParamsInit();
    _servicesInit();
    _advertisingInit();
    _connectionParamatersInit();
    notification_instance = this;
}

//Enable Advertising
void Btle::start()
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

void Btle::_bleStackInit()
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    static uint32_t ble_evt_buffer[CEIL_DIV(BLE_STACK_EVT_MSG_BUF_SIZE, sizeof(uint32_t))];                                                                         
    err_code = softdevice_handler_init(&clock_lf_cfg,ble_evt_buffer, sizeof(ble_evt_buffer), NULL); 
    
    APP_ERROR_CHECK(err_code);       

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

#ifdef CALC_SD_RAM //this code block calculates softdevice needed ram.
    uint32_t app_ram_base = (uint32_t) &__data_start__;
    err_code = sd_ble_enable(&ble_enable_params, &app_ram_base);

    uint32_t correct_ram = app_ram_base;
    uint32_t current_ram = (uint32_t) &__data_start__;
    _serial->write("   ram schould be ");
    _serial->write(correct_ram);
    _serial->write(" is ");
    _serial->write(current_ram);
     _serial->putChar('\n');
#endif

    // Enable BLE stack.
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    err_code = softdevice_ble_evt_handler_set(_btleEventHandler);
    APP_ERROR_CHECK(err_code);
}


//set Generic Access Profile info
void Btle::_gapParamsInit()
{
  uint32_t                err_code;
  ble_gap_conn_params_t   gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code =  sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_BLE_NAME, strlen(DEVICE_BLE_NAME));
    APP_ERROR_CHECK(err_code);
  
  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency     = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

  
  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);
}

//enable base service and caracteristcs
void Btle::_servicesInit()
{
    uint32_t      err_code;
    
    //Add a custom base UUID.
    ble_uuid128_t baseUuid = TELSYS_BASE_UUID;
    uint8_t bleUuidType = BLE_UUID_TYPE_VENDOR_BEGIN;
    err_code = sd_ble_uuid_vs_add(&baseUuid, &bleUuidType);
    APP_ERROR_CHECK(err_code);

    //Add the service.
    ble_uuid_t    bleUuid;
    bleUuid.type = bleUuidType;
    bleUuid.uuid = UUID_TELSYS_SERVICE;
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &bleUuid, &_serviceHandle);
    APP_ERROR_CHECK(err_code);

    //Add the RX Characteristic.
    err_code = _rxCharacteristicInit();
    APP_ERROR_CHECK(err_code);

    //Add the ADC Characteristic.
    err_code = _adcCharacteristicInit();
    APP_ERROR_CHECK(err_code);
    
    //Add the AUX Characteristic.
    err_code = _auxCharacteristicInit();
    APP_ERROR_CHECK(err_code);
}

//setup rx caracteristc for reciving config and cal data.
uint32_t Btle::_rxCharacteristicInit()
{    
    ble_gatts_char_md_t charMd;
    memset(&charMd, 0, sizeof(charMd));
    charMd.char_props.write         = 1;
    charMd.char_props.write_wo_resp = 1;
    charMd.p_char_user_desc         = NULL;
    charMd.p_char_pf                = NULL;
    charMd.p_user_desc_md           = NULL;
    charMd.p_cccd_md                = NULL;
    charMd.p_sccd_md                = NULL;

    ble_gatts_attr_md_t attrMd;
    memset(&attrMd, 0, sizeof(attrMd));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attrMd.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attrMd.write_perm);
    attrMd.vloc    = BLE_GATTS_VLOC_STACK;
    attrMd.rd_auth = 0;
    attrMd.wr_auth = 0;
    attrMd.vlen    = 1;
    
    ble_uuid_t bleUuid;
    bleUuid.type = BLE_UUID_TYPE_VENDOR_BEGIN;
    bleUuid.uuid = UUID_RX_CHARACTERISTIC;
    
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid    = &bleUuid;
    attr_char_value.p_attr_md = &attrMd;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = MAXIMUM_DATA_LENGTH;

    return sd_ble_gatts_characteristic_add(_serviceHandle, &charMd, &attr_char_value, &_rxHandles);
}

//setup and enable adc characteristic
uint32_t Btle::_adcCharacteristicInit()
{
    ble_gatts_attr_md_t cccdMd;
    memset(&cccdMd, 0, sizeof(cccdMd));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccdMd.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccdMd.write_perm);
    cccdMd.vloc = BLE_GATTS_VLOC_STACK;

    ble_gatts_char_md_t charMd;
    memset(&charMd, 0, sizeof(charMd));
    charMd.char_props.notify = 1;
    charMd.p_char_user_desc  = NULL;
    charMd.p_char_pf         = NULL;
    charMd.p_user_desc_md    = NULL;
    charMd.p_cccd_md         = &cccdMd;
    charMd.p_sccd_md         = NULL;

    ble_gatts_attr_md_t attrMd;
    memset(&attrMd, 0, sizeof(attrMd));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attrMd.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attrMd.write_perm);
    attrMd.vloc    = BLE_GATTS_VLOC_STACK;
    attrMd.rd_auth = 0;
    attrMd.wr_auth = 0;
    attrMd.vlen    = 1;
    
    ble_uuid_t bleUuid;
    bleUuid.type = BLE_UUID_TYPE_VENDOR_BEGIN;
    bleUuid.uuid = UUID_ADC_CHARACTERISTIC;

    ble_gatts_attr_t    adc_attr_char_value;
    memset(&adc_attr_char_value, 0, sizeof(adc_attr_char_value));
    adc_attr_char_value.p_uuid    = &bleUuid;
    adc_attr_char_value.p_attr_md = &attrMd;
    adc_attr_char_value.init_len  = 1;
    adc_attr_char_value.init_offs = 0;
    adc_attr_char_value.max_len   = MAXIMUM_DATA_LENGTH;

    return sd_ble_gatts_characteristic_add(_serviceHandle, &charMd, &adc_attr_char_value, &_adcHandles);
}

//setup and enable aux characteristic for MPU data
uint32_t Btle::_auxCharacteristicInit()
{
    ble_gatts_attr_md_t cccdMd;
    memset(&cccdMd, 0, sizeof(cccdMd));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccdMd.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccdMd.write_perm);
    cccdMd.vloc = BLE_GATTS_VLOC_STACK;

    ble_gatts_char_md_t charMd;
    memset(&charMd, 0, sizeof(charMd));
    charMd.char_props.notify = 1;
    charMd.p_char_user_desc  = NULL;
    charMd.p_char_pf         = NULL;
    charMd.p_user_desc_md    = NULL;
    charMd.p_cccd_md         = &cccdMd;
    charMd.p_sccd_md         = NULL;

    ble_gatts_attr_md_t attrMd;
    memset(&attrMd, 0, sizeof(attrMd));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attrMd.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attrMd.write_perm);
    attrMd.vloc    = BLE_GATTS_VLOC_STACK;
    attrMd.rd_auth = 0;
    attrMd.wr_auth = 0;
    attrMd.vlen    = 1;
    
    ble_uuid_t bleUuid;
    bleUuid.type = BLE_UUID_TYPE_VENDOR_BEGIN;
    bleUuid.uuid = UUID_AUX_CHARACTERISTIC;

    ble_gatts_attr_t    aux_attr_char_value;
    memset(&aux_attr_char_value, 0, sizeof(aux_attr_char_value));
    aux_attr_char_value.p_uuid    = &bleUuid;
    aux_attr_char_value.p_attr_md = &attrMd;
    aux_attr_char_value.init_len  = 1;
    aux_attr_char_value.init_offs = 0;
    aux_attr_char_value.max_len   = MAXIMUM_DATA_LENGTH;

    return sd_ble_gatts_characteristic_add(_serviceHandle, &charMd, &aux_attr_char_value, &_auxHandles);
}


//initalize btle advertising
void Btle::_advertisingInit()
{
    ble_advdata_t          advdata;
    ble_advdata_t          scanrsp;
    ble_adv_modes_config_t options;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    ble_uuid_t m_adv_uuids[] = {{UUID_TELSYS_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN}};
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

void Btle::_connParamsEvtHandler(ble_conn_params_evt_t * event)
{
    if (event->evt_type == BLE_CONN_PARAMS_EVT_FAILED) sd_ble_gap_disconnect(_connHandle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
}

//initalize CP module;
void Btle::_connectionParamatersInit()
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

void Btle::_connected(ble_evt_t* p_ble_evt)
{
  uint32_t    periph_link_cnt = ble_conn_state_n_peripherals(); //number of peripheral links.
  
  _connHandle = p_ble_evt->evt.gap_evt.conn_handle;

  if (periph_link_cnt == PERIPHERAL_LINK_COUNT)
  {
  }
  else
  {
    //continue advertising, max connections not reatched.
    Btle::start();
  }
  if(notification_instance->_connectCb) notification_instance->_connectCb();
}

void Btle::_disconnected(ble_evt_t* p_ble_evt)
{
  uint32_t    periph_link_cnt = ble_conn_state_n_peripherals(); //number of peripheral links.
  
  _connHandle = BLE_CONN_HANDLE_INVALID;
  
  if (periph_link_cnt == (PERIPHERAL_LINK_COUNT - 1))
  {
    //advertising is not running when all connections are taken, and must therefore be started.
    Btle::start();
  }
  
  if(notification_instance->_disconnectCb) notification_instance->_disconnectCb();
}

void Btle::_onAuthorizeRequest(ble_evt_t const * p_ble_evt)
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

//deal with data writen by connected central
void Btle::_onWrite(ble_evt_t* p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == _adcHandles.cccd_handle && p_evt_write->len == 2)
    {
        if(ble_srv_is_notification_enabled(p_evt_write->data)) _adcNotificationEnabled = true;
        else                                                    _adcNotificationEnabled = false;
    }
    else if (p_evt_write->handle == _auxHandles.cccd_handle && p_evt_write->len == 2)
    {
        if(ble_srv_is_notification_enabled(p_evt_write->data)) _auxNotificationEnabled = true;
        else                                                    _auxNotificationEnabled = false;
    }
     else if (p_evt_write->handle == _rxHandles.value_handle && _rxCb) _rxCb(p_evt_write->data, p_evt_write->len);
}

void Btle::setDisconnectCallback(std::function<void(void)> disconnectCb)
{
    _disconnectCb = disconnectCb;
}

void Btle::setConnectCallback(std::function<void(void)> connectCb)
{
    _connectCb = connectCb;
}

void Btle::setRxCallback(std::function<void(uint8_t*, uint32_t)> rxCb)
{
    _rxCb = rxCb;
}

///Handle BLE events emmited by softdevice.
void Btle::_btleEventHandler(ble_evt_t* p_ble_evt)
{
  ret_code_t err_code;
  
  ble_conn_params_on_ble_evt(p_ble_evt);
  ble_advertising_on_ble_evt(p_ble_evt);
  ble_conn_state_on_ble_evt(p_ble_evt);

  switch (p_ble_evt->header.evt_id)
  {
    case BLE_GAP_EVT_CONNECTED:
      _connected(p_ble_evt);
      break;

    case BLE_GAP_EVT_DISCONNECTED:
      _disconnected(p_ble_evt);
      break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
      //pairing not supported
      err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle,
          BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
          NULL,
          NULL);
      APP_ERROR_CHECK(err_code);
      break;
      
    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
      //no system attributes have been stored.
      err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gap_evt.conn_handle, NULL, 0, 0);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTC_EVT_TIMEOUT:
      //disconnect on gatt client timeout event.
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
          BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTS_EVT_TIMEOUT:
      //disconnect on gatt server timeout event.
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
    
    case BLE_GATTS_EVT_WRITE:
            notification_instance->_onWrite(p_ble_evt);
    break;
            
    default:
      break;
  }
}

//write to characteristic
uint32_t Btle::write(uint8_t* data, uint16_t length, ble_gatts_char_handles_t& handle)
{
    if (_connHandle == BLE_CONN_HANDLE_INVALID || !_adcNotificationEnabled) return NRF_ERROR_INVALID_STATE;
    else if (length > MAXIMUM_DATA_LENGTH) return NRF_ERROR_INVALID_PARAM;

    ble_gatts_hvx_params_t hvxParams;
    memset(&hvxParams, 0, sizeof(hvxParams));

    hvxParams.handle = handle.value_handle;
    hvxParams.p_data = data;
    hvxParams.p_len  = &length;
    hvxParams.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(_connHandle, &hvxParams);
}

//package and send adc samples.
void Btle::sendAdcSamples(uint16_t* samples, const uint8_t amount, const uint16_t usPacketDelta)
{
    if(isConnected() && amount*2+3 <= MAXIMUM_DATA_LENGTH ) 
    {
        uint8_t packet[MAXIMUM_DATA_LENGTH];

        packet[0] = amount;
        packet[1] = usPacketDelta >> 8; 
        packet[2] = usPacketDelta & 0x00FF;
        for(uint8_t i = 0; i < amount; i++)
        {
            packet[i*2+3] = samples[i] >> 8; 
            packet[i*2+4] = samples[i] & 0x00FF;
        }
        write(packet, amount*2+3, _adcHandles);
    }
}

void Btle::sendAuxPacket(const Point3D<int16_t>& accel, const Point3D<int16_t>& magn, const uint8_t temperature, const uint16_t usPacketDelta)
{
    uint8_t packet[15];
    packet[0] = usPacketDelta >> 8; 
    packet[1] = usPacketDelta & 0x00FF;
    int accelsize = accel.serialize(packet+2, 13);
    magn.serialize(packet+2+accelsize, 13);
    packet[2+accelsize*2] = temperature;
    write(packet, 15, _auxHandles);
}

bool Btle::isConnected()
{
    return ble_conn_state_n_peripherals() > 0;
}
