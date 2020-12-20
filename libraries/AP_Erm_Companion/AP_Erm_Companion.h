// This class implements the serial protocol for HFE international ECU system as
// described in document HFEDCN0191_REV_F.pdf

#pragma once

#include <AP_HAL/AP_HAL.h>

// just for debugging
#include <GCS_MAVLink/GCS.h>

//definition of commands
#define CMD_READ_DATA 'a'

class AP_Erm_Companion {
public:

    AP_Erm_Companion();

    void update(void);

    void init();

    static AP_Erm_Companion *get_singleton() { return _singleton; }

    void startTracking(uint16_t x0, uint16_t x1, uint16_t y0, uint16_t y1);

    void stopTracking();

    void send_tracker_status(mavlink_channel_t chan);

private:

    struct PACKED start_tracking_request {
        uint16_t x0;
        uint16_t y0;
        uint16_t x1;
        uint16_t y1;
    };

    struct PACKED tracking_coord_feedback {
        uint16_t x0;
        uint16_t y0;
        uint16_t x1;
        uint16_t y1;         
    };

    union PACKED bytes_group {
        DEFINE_BYTE_ARRAY_METHODS
        start_tracking_request start_tracking;
        tracking_coord_feedback tracker_feedback;
    } _sendBuffer, _receiveBuffer;

    static AP_Erm_Companion *_singleton;

    void parse_body();

    void read_incoming();

    void send_byte(uint8_t cmd);

    void send_command(uint8_t cmd, uint8_t* data, uint8_t size);

    AP_HAL::UARTDriver *_port;
    bool _initialised = false;

    uint8_t _payload_counter;
    uint8_t _payload_length;
    uint8_t _step;
    uint8_t _checksum;

    bool _tracking_active = false;

    uint8_t _status;
    uint16_t _x0;    
    uint16_t _x1;    
    uint16_t _y0;    
    uint16_t _y1;
};

namespace AP {
    AP_Erm_Companion *erm_companion();
};