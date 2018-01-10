//UVOS
#pragma once

#include <stdint.h>


//SDK Headers
extern "C"
{
    #include "nordic_common.h"
    #include "nrf.h"
    #include "ble_hci.h"
    #include "ble_gap.h"
    #include "ble_advdata.h"
    #include "ble_advertising.h"
    #include "ble_conn_params.h"
    #include "softdevice_handler.h"
    #include "app_timer.h"
    #include "ble_nus.h"
    #include "app_util_platform.h"
    #include "ble_conn_state.h"
//#include "bsp.h"
  //  #include "bsp_btn_ble.h"
}

class BtleUart
{
    
private:
    
    void _bleStackInit();
    void _advertisingInit();
    void _servicesInit();
    void _gapParamsInit();
    void _connectionParamatersInit();
    static void _connected(const ble_gap_evt_t * const p_gap_evt);
    static void _disconnected(ble_gap_evt_t const * const p_gap_evt);
    static void _onAuthorizeRequest(ble_evt_t const * p_ble_evt);
    static void _btleEventHandler(ble_evt_t * p_ble_evts);
    static void _connParamsErrorHandler(uint32_t nrf_error){} //yes I'm serious
    static void _connParamsEvtHandler(ble_conn_params_evt_t * p_evt);
    static void _btleDataHandler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length);
    
public:
    BtleUart();
    static void start();
    bool dataIsWaiting();
    bool write( uint8_t* buffer, uint32_t length);
    bool isConnected();
};
