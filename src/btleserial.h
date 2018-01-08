//UVOS
#pragma once

#include <stdint.h>


//SDK Headers
extern "C"
{
    #include "ble_gap.h"
    #include "ble.h"
}

class BtleUart
{
    
private:
    
    void _bleStackInit();
    void _advertisingInit();
    void _servicesInit();
    void _gapParamsInit();
    void _connected();
    void _disconnected(ble_gap_evt_t const * const p_gap_evt);
    void _onAuthorizeRequest(ble_evt_t const * p_ble_evt);
    static void _btleEventHandler(ble_evt_t const * p_ble_evt, void * p_context);
    static void _connParamsErrorHandler(uint32_t nrf_error){} //yes I'm serious
    static void _connParamsEvtHandler(ble_conn_params_evt_t * p_evt);
    static void _btleDataHandler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length);
    
public:
    BleWrapper();
    void start();
    bool dataIsWaiting();
    bool write( uint8_t* buffer, uint32_t length);
};
