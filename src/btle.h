//UVOS
#pragma once


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

#include <stdint.h>
#include <functional>

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
    #include "app_util_platform.h"
    #include "ble_conn_state.h"
    #include "nrf_ble_gatt.h"
}

#include "point3D.h"
#include "board.h"
#include "ringbuffer.h"

class Btle
{
    
private:
    
    uint16_t                 _serviceHandle;          
    ble_gatts_char_handles_t _rxHandles;              
    ble_gatts_char_handles_t _adcHandles;             
    ble_gatts_char_handles_t _auxHandles;
    
    bool                     _adcNotificationEnabled = false; //Variable to indicate if the peer has enabled notification of the ADC characteristic.
    bool                     _auxNotificationEnabled = false; //Variable to indicate if the peer has enabled notification of the AUX characteristic.
    
    char _terminator = '\n';
        
    void _bleStackInit();
    void _advertisingInit();
    void _servicesInit();
    uint32_t _rxCharacteristicInit();
    uint32_t _adcCharacteristicInit();
    uint32_t _auxCharacteristicInit();
    void _gapParamsInit();
    void _connectionParamatersInit();
    
    std::function<void(void)> _disconnectCb;
    std::function<void(void)> _connectCb;
    std::function<void(uint8_t*, uint32_t)> _rxCb;
    
    static void _connected(ble_evt_t* p_ble_evt);
    static void _disconnected(ble_evt_t* p_ble_evt);
    static void _onAuthorizeRequest(ble_evt_t const * p_ble_evt);
    void _onWrite(ble_evt_t* p_ble_evt);
    static void _btleEventHandler(ble_evt_t * p_ble_evts);
    static void _connParamsErrorHandler(uint32_t nrf_error){APP_ERROR_HANDLER(nrf_error);}
    static void _connParamsEvtHandler(ble_conn_params_evt_t * p_evt);
    
    uint32_t write(uint8_t* data, uint16_t length, ble_gatts_char_handles_t& handle);
    
public:
    
    Btle();
    
    static void start();
    bool isConnected();
    
    void sendAdcSamples(uint16_t* samples, const uint8_t amount, const uint16_t usPacketDelta);
    void sendAuxPacket(const Point3D<int16_t>& accel, const Point3D<int16_t>& magn, const uint8_t temperature, const uint16_t usPacketDelta);
    
    void setDisconnectCallback(std::function<void(void)> disconnectCb);
    void setConnectCallback(std::function<void(void)> connectCb);
    
    void setRxCallback(std::function<void(uint8_t*, uint32_t)> rxCb);
    
    bool dataIsWaiting();
    char getChar();
    unsigned int getString(char* buffer, const unsigned int length);
    unsigned int read(uint8_t* buffer, const unsigned int length);
    void setTerminator(const char terminator);
    void flush();
    
};
