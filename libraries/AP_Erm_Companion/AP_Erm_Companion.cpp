#include "AP_Erm_Companion.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

AP_Erm_Companion::AP_Erm_Companion(void) {

    if (_singleton != nullptr) {
        AP_HAL::panic("Erm Companion must be singleton");
    }
    _singleton = this;
}

void AP_Erm_Companion::init(void) {
    
    const AP_SerialManager& serial_manager = AP::serialmanager();

    // check for ecu protocol
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_AR_Erm_Companion, 0))) {
        _initialised = true;

        gcs().send_text(MAV_SEVERITY_INFO, "Erm Companion initialized");
    }
}

void AP_Erm_Companion::update(void) {

    if (!_initialised) {
        return;
    }

    read_incoming();
    send_byte(0x04);
    send_byte(0x04);
}

void AP_Erm_Companion::read_incoming(void) {

    uint8_t data;
    int16_t numc;

    numc = _port->available();

    if (numc < 0 ){
        _status = 0;
        return;
    }

    _payload_counter = 0;

    for (int16_t i = 0; i < numc; i++) { // Process bytes received
        data = _port->read();
        switch (_step) {
            case 0:
                if ( 0x05 == data) {
                    _step = 1;
                    _checksum = data;
                }
                break;

            case 1: // parsing body
                _checksum += data;
                if (_payload_counter < sizeof(tracking_coord_feedback)) {
                    _receiveBuffer[_payload_counter] = data;
                }
                if (++_payload_counter == sizeof(tracking_coord_feedback))
                    _step++;
                break;

            case 2:// body checksum
                _step = 0;
                if (_checksum  != data) {
                    _status = 0;
                    gcs().send_text(MAV_SEVERITY_INFO, "wrong checksum!");
                    break;
                }
                parse_body();
        }
    }

}

void AP_Erm_Companion::parse_body(void) {

    // need to put the real status here when supported
    _status = 1;

    _x0 = _receiveBuffer.tracker_feedback.x0;
    _x1 = _receiveBuffer.tracker_feedback.x1;
    _y0 = _receiveBuffer.tracker_feedback.y0;
    _y1 = _receiveBuffer.tracker_feedback.y1;
}

void AP_Erm_Companion::send_tracker_status(mavlink_channel_t chan) {

    mavlink_msg_erm_tracker_send(chan, _status, _x0, _x1, _y0, _y1);
}


void AP_Erm_Companion::send_byte(uint8_t cmd) {
    
    if (_port->txspace() < 1) {
        return;
    }

    _port->write(cmd);
}

void AP_Erm_Companion::send_command(uint8_t cmd, uint8_t* data, uint8_t size)
{
    if (_port->txspace() < (size + 5U)) {
        return;
    }
    uint8_t checksum = cmd;
    _port->write( cmd );  // write command id

    for (uint8_t i = 0;  i != size ; i++) {
        checksum += data[i];
        _port->write( data[i] );
    }
    _port->write(checksum);
}

void AP_Erm_Companion::startTracking(uint16_t x0, uint16_t x1, uint16_t y0, uint16_t y1) {

    gcs().send_text(MAV_SEVERITY_INFO, "x0: %x x1: %x y0: %x y1: %x", x0, x1, y0, y1);

    _sendBuffer.start_tracking.x0 = x0;
    _sendBuffer.start_tracking.y0 = y0;
    _sendBuffer.start_tracking.x1 = x1;
    _sendBuffer.start_tracking.y1 = y1;
    send_command(1, (uint8_t*)&_sendBuffer, sizeof(start_tracking_request));
}

void AP_Erm_Companion::stopTracking() {
    
    send_byte(0x02);
    send_byte(0x02);
}

// singleton instance. Should only ever be set in the constructor.
AP_Erm_Companion *AP_Erm_Companion::_singleton;
namespace AP {
AP_Erm_Companion *erm_companion() {
        return AP_Erm_Companion::get_singleton();
    }
}
