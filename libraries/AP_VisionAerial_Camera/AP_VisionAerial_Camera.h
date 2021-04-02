//Driver for the conexion and communication with the WIRIS
//camera lineup.

//Acording to the WIRIS documentation the parameters can be set 
//either via index or name (string) in some cases. For the sake 
//of simplicity we will only use index. This way we can send the 
//information for any given camera or parameter with the same 
//message just stating the camera and the parameter we changed

#pragma once

#include <ctype.h>
#include <vector>
#include <stdlib.h>

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>//For debbuging pourpuses

//Definitions for ASCII correlations
#define ASCII_SPACE 32
#define ASCII_NUMBER_0 48
//Definitions for WIRIS Protocol correlations
#define WIRIS_MSG_HEADER 2  //Hex 0x02
#define WIRIS_MSG_FOOTER 3  //Hex 0x02
#define WIRIS_PAYLOAD_END 10 //HEX 0x0A in ASCII is LF (end of line)
//Definitions for first character of possible received answer
#define OK_FIRST_CHAR 'O'
#define NOT_READY_FIRST_CHAR 'N'
#define TRUE_FIRST_CHAR 'T'
#define FALSE_FIRST_CHAR 'F'
//Definition for parameter defaults
#define JOYSTICK_SPEED_MULT 1.0

class AP_VisionAerial_Camera {

public:

    AP_VisionAerial_Camera();

    void    init(void);

    void    update(void);

    //Send current value of zoom via mavlink
    void    send_status(mavlink_channel_t chan);

    //---------------Zoom functions---------------
    void    getZoomVisible(void);
    
    void    setZoomVisible(uint8_t zoom);//Set desire value of zoom in the visible camera by index

    void    getZoomThermal(void);

    void    setZoomThermal(uint8_t zoom);//Set desire value of zoom in the thermal camera by index
    //-----------------------------------------------

    //---------------Trigger functions---------------
    void    capturePhoto(void);

    void    startRecording(void);

    void    stopRecording(void);
    //-----------------------------------------------

    //-------------Appearenace functions-------------
    void    setLayout(uint8_t layout); //0->INSPECTION, 1->SECURITY, 2->FULLSCREEN, 3->PIP

    void    setMainCamera(uint8_t camera);//0->VISIBLE, 1->THERMAL
    //-----------------------------------------------

    void sendCameraStatus(mavlink_channel_t chan);

    static AP_VisionAerial_Camera *get_singleton() { return _singleton; }

    // parameter var table
    static const struct AP_Param::GroupInfo        var_info[];

protected:

    //AP parameter
    AP_Float     _joystick_speed_multiplier;

private:

    struct PACKED possible_Answers {
        const char okAnswer[4] =        "OK\n";
        const char notReadyAnswer[11] = "NOT_READY\n";
        const char trueAnswer[6] =      "TRUE\n";
        const char falseAnswer[7] =     "FALSE\n";
    } possibleAnswers;

    struct PACKED trigger_Commands {
        const char capturePhotoCmd[5] =     "CPTR";//Triggers a photo capture. Possible answers: OK or NOT_READY
        const char isCapturingPhotoCmd[5] = "ICPT";//Ask if camera is taking photo at the moment. Possible answers: TRUE or FALSE
        const char startRecordingCmd[5] =   "RCRS";//Starts recording video. Possible answers: OK or NOT_READY
        const char stopRecordingCmd[5] =    "RCRF";//Stops recording video. Possible answers: OK or NOT_READY
        const char isRecordingCmd[5] =      "IRCR";//Ask if camera is recording video at the moment. Possible answers: TRUE or FALSE
    } triggerCmds;

    struct PACKED zoom_Commands {
        const char getZoomVisibleCmd[5] =   "GZVV";//Gets the actual value of zoom for the visible camera
        const char setZoomVisibleCmd[5] =   "SZVN";//Sets especific zoom value for visible.This will need an index for the zoom. ie: "SZVN 1"
        const char getZoomThermalCmd[5] =   "GZTV";//Gets the actual value of zoom for the thermal camera
        const char setZoomThermalCmd[5] =   "SZTN";//Sets especific zoom value for thermal.This will need an index for the zoom. ie: "SZTN 1"
    } zoomCmds;

    struct PACKED basic_Commands {
        const char checkConnection[5] = "HIWS";//Check if the camera is connected
    } basicCmds;

    struct PACKED appearance_Commands {
        const char setLayoutInspection[16] =    "SLAY INSPECTION";
        const char setLayoutSecurity[14] =      "SLAY SECURITY";
        const char setLayoutFullscreen[16] =    "SLAY FULLSCREEN";
        const char setLayoutPip[9] =            "SLAY PIP";
        const char setMainCameraVisible[13] =   "SMCA VISIBLE";
        const char setMainCameraThermal[13] =   "SMCA THERMAL";
    } appearanceCmds;

    enum RequestMade {
        NO_REQUEST = 0,
        CAPTURE_PHOTO = 1,
        START_RECORDING = 2,
        STOP_RECORDING = 3,
        GET_ZOOM_VISIBLE = 4,
        SET_ZOOM_VISIBLE = 5,
        GET_ZOOM_THERMAL = 6,
        SET_ZOOM_THERMAL = 7,
        CHECK_CONNECTION = 8,
        SET_LAYOUT_INSPECTION = 9,
        SET_LAYOUT_SECURITY = 10,
        SET_LAYOUT_FULLSCREEN = 11,
        SET_LAYOUT_PIP = 12,
        SET_MAIN_CAMERA_VISIBLE = 13,
        SET_MAIN_CAMERA_THERMAL = 14
    }; 

    enum LayoutAppeareance {
        LAYOUT_UNKNOWN = 0,
        LAYOUT_INSPECTION = 1,
        LAYOUT_SECURITY = 2,
        LAYOUT_FULLSCREEN = 3,
        LAYOUT_PIP = 4
    };

    enum MainCamera {
        MAIN_CAMERA_UNKOWN = 0,
        MAIN_CAMERA_VISIBLE = 1,
        MAIN_CAMERA_THERMAL = 2
    };
    struct PACKED requested_Index {
        uint8_t zoomVisible;
        uint8_t zoomThermal;
    } requestedIndex;

    void read_incoming(void);

    void parse_body(void);

    void manageOkAnswer(void);

    void manageNotReadyAnswer(void);

    void manageTrueAnswer(void);

    void manageFalseAnswer(void);

    void parseNumericalAnswer(void);

    void updateCorrespondingIndex(uint8_t index);
    
    void checkConnectionStatus(void);
    
    void sendCommand(const char* cmd, uint8_t index, uint8_t size);//If no index is needed set index as UINT8_MAX

    void updateJoystickSpeedMultiplier(void);

    uint16_t setNthBit(uint16_t bitmask, uint8_t n);

    static AP_VisionAerial_Camera *_singleton;

    AP_HAL::UARTDriver *_port;
    bool _initialized_port = false;
    bool _initialized_camera = false;
    bool _is_first_call = true;

    //Buffers needed
    char    _receiveBuffer[128];//The size of the payload should not exced this

    //Current state of the camera
    bool    _isCapturing;
    bool    _isRecording;
    
    //Current state of the camera zoom by index
    uint8_t     _visibleZoomLevel = UINT8_MAX; //UINT8_MAX means we dont know the value
    uint8_t     _thermalZoomLevel = UINT8_MAX; //UINT8_MAX means we dont know the value

    //Indicator for wich request has been made to manage answer
    RequestMade lastRequestMade = NO_REQUEST;

    //Current state of camera appearance layout and main camera
    LayoutAppeareance   _currentLayout = LAYOUT_UNKNOWN; //UINT8_MAX means we dont know the value
    MainCamera          _currentMainCamera = MAIN_CAMERA_UNKOWN; //UINT8_MAX means we dont know the value
};

namespace AP {
    AP_VisionAerial_Camera *visionAerial_camera();
}