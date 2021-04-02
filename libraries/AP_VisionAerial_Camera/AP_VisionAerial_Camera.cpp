#include "AP_VisionAerial_Camera.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Mount/AP_Mount.h>

const AP_Param::GroupInfo AP_VisionAerial_Camera::var_info[] = {

    // @Param: SPD_MULT
    // @DisplayName: Joystick speed zoom multiplier
    // @Description: Factor to multiply zoom index in order to get a smoother camera control
    // @Range:  1 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("SPD_MULT", 1,  AP_VisionAerial_Camera, _joystick_speed_multiplier, JOYSTICK_SPEED_MULT),

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

AP_VisionAerial_Camera::AP_VisionAerial_Camera(void) 
{

    if (_singleton != nullptr) {
        AP_HAL::panic("VisionAerial Camera must be singleton");
    }
    _singleton = this;
}

void AP_VisionAerial_Camera::init(void) 
{

    const AP_SerialManager& serial_manager = AP::serialmanager();

    //Check for WIRIS protocol
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_VisionAerial_Camera,0))) {
        _initialized_port = true;
    }
}

void AP_VisionAerial_Camera::update(void) 
{

    if (!_initialized_port) {
        return;
    }

    //Check if we already have connectiong with the camera. If we don't, keep checking until we do.
    if(!_initialized_camera) {
        checkConnectionStatus();
    } else {
        if (_is_first_call) { //If its the first time after initialization
            getZoomVisible();
            setLayout(2); //Index 2 corresponds to fullscreen
            setMainCamera(0);//Index 0 corresponds to VISIBLE
            if ((_visibleZoomLevel != UINT8_MAX) && (_currentLayout != LAYOUT_UNKNOWN) && (_currentMainCamera != MAIN_CAMERA_UNKOWN)) { //If we receive a correct index for the zoom
                _is_first_call = false;
            }
        } else {
            read_incoming();
        }
    }
}

void AP_VisionAerial_Camera::read_incoming(void) 
{

    uint8_t     data;
    int32_t     numc;
    uint8_t     step = 0;
    uint8_t     sizeLeast = 0;//Least significant byte of the size bytes (little-endian)
    uint8_t     sizeMost = 0;//Most significant byte of the size bytes (little-endian)
    uint16_t    sizeTotal = 0;//Combined size wich is the sum of the most significant and the least significant bytes
    uint16_t    checksum = 0;
    uint16_t    payload_length = 0;

    numc = _port->available();

    if (numc < 0 ) {
        return;
    } 

    //Process every byte separettly
    for (int16_t i = 0; i < numc; i++) {
        data = _port->read();
        switch (step) {
            case 0://Searching for header byte
                if ( data == WIRIS_MSG_HEADER) {
                    step = 1;
                    checksum = 0;
                }
                break;
            case 1://First size byte (least significant)
                sizeLeast = data;
                step++;
                break;
            case 2://Second size byte (most significant)
                sizeMost = data;
                sizeTotal = sizeMost;
                sizeTotal = sizeTotal<<8;//We sift the most significant byte to the be in the correct position
                sizeTotal += sizeLeast;
                step++;
                break;
            case 3://Parse body
                checksum ++;
                if (payload_length < sizeTotal) {
                    _receiveBuffer[payload_length] = data;
                }
                if (++payload_length == sizeTotal) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Received buffer is: %s",_receiveBuffer);
                    if (checksum != payload_length) {
                        //This means we have gotten a wrong checksum
                        gcs().send_text(MAV_SEVERITY_INFO, "Wrong checksum when parsing msg payload");
                        step = 0;
                        checksum =0;
                        break;
                    }
                    step++;
                }
                break;
            case 4://Check end of message is correct
                if (data == WIRIS_MSG_FOOTER) {
                    parse_body();
                } else {
                    gcs().send_text(MAV_SEVERITY_INFO, "End of message not detected correctly. EXIT read_incoming");
                }
                step = 0;
                checksum =0;
                break;
            default:
                step = 0;
                checksum =0;
        }
    }
    //After reading it, we need to empty the receive buffer 
    memset(_receiveBuffer,0,sizeof(_receiveBuffer));
    //After reading the answer, we set the request made to NONE
    lastRequestMade = NO_REQUEST;
}

//This function checks the first character of the receivedbuffer in order to 
//just compare it with the possible answer and not have to compare every possible option
//for every byte
void AP_VisionAerial_Camera::parse_body() 
{
    switch (_receiveBuffer[0]) {
        case OK_FIRST_CHAR: //The possible answer is "OK"
            if (!strcmp(_receiveBuffer,possibleAnswers.okAnswer)) {
                gcs().send_text(MAV_SEVERITY_INFO, "Command received correctly: %s", _receiveBuffer);
                manageOkAnswer();
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "Error: expected command is OK. Instead we received %s: ", _receiveBuffer);
            }
            break;
        case NOT_READY_FIRST_CHAR: //Means the possible answer is NOT_READY
            if (!strcmp(_receiveBuffer,possibleAnswers.notReadyAnswer)) {
                gcs().send_text(MAV_SEVERITY_INFO, "Command received correctly: %s", _receiveBuffer);
                manageNotReadyAnswer();
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "Error: expected command is NOT_READY. Instead we received %s: ", _receiveBuffer);
            }
            break;
        case TRUE_FIRST_CHAR: //Means the possible answer is NOT_READY
            if (!strcmp(_receiveBuffer,possibleAnswers.trueAnswer)) {
                gcs().send_text(MAV_SEVERITY_INFO, "Command received correctly: %s", _receiveBuffer);
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "Error: expected command is TRUE. Instead we received %s: ", _receiveBuffer);
            }
            break;
        case FALSE_FIRST_CHAR: //Means the possible answer is FALSE
            if (!strcmp(_receiveBuffer,possibleAnswers.falseAnswer)) {
                gcs().send_text(MAV_SEVERITY_INFO, "Command received correctly: %s", _receiveBuffer);
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "Error: expected command is FALSE. Instead we received %s: ", _receiveBuffer);
            }
            break;
        default: //For any other alphabetic or numerical answer
            parseNumericalAnswer();
    }
}

//Manage OK answer depending on what was the command we had sent
void AP_VisionAerial_Camera::manageOkAnswer(void) 
{
    switch (lastRequestMade) {
        case CAPTURE_PHOTO:
            gcs().send_text(MAV_SEVERITY_INFO, "Photo taken");
            _isCapturing = false;
            break;
        case START_RECORDING:
            gcs().send_text(MAV_SEVERITY_INFO, "Recording started");
            _isRecording = true;
            break;
        case STOP_RECORDING:
            gcs().send_text(MAV_SEVERITY_INFO, "Recording stoped");
            _isRecording = false;
            break;
        case SET_ZOOM_VISIBLE:
            _visibleZoomLevel = requestedIndex.zoomVisible;
            gcs().send_text(MAV_SEVERITY_INFO, "Zoom for visible camera set correctly. Index set to: %u", _visibleZoomLevel);
            break;
        case SET_ZOOM_THERMAL:
            _thermalZoomLevel = requestedIndex.zoomThermal;
            gcs().send_text(MAV_SEVERITY_INFO, "Zoom for thermal camera set correctly. Index set to: %u", _thermalZoomLevel);
            break;
        case CHECK_CONNECTION:
            _initialized_camera = true;
            gcs().send_text(MAV_SEVERITY_INFO, "Camera is connected");
            break;
        case SET_LAYOUT_INSPECTION:
            _currentLayout = LAYOUT_INSPECTION;
            gcs().send_text(MAV_SEVERITY_INFO, "Camera layout is now in INSPECTON mode");
            break;
        case SET_LAYOUT_SECURITY:
            _currentLayout = LAYOUT_SECURITY;
            gcs().send_text(MAV_SEVERITY_INFO, "Camera layout is now in SECURITY mode");
            break;
        case SET_LAYOUT_FULLSCREEN:
            _currentLayout = LAYOUT_FULLSCREEN;
            gcs().send_text(MAV_SEVERITY_INFO, "Camera layout is now in FULLSCREEN mode");
            break;
        case SET_LAYOUT_PIP:
            _currentLayout = LAYOUT_PIP;
            gcs().send_text(MAV_SEVERITY_INFO, "Camera layout is now in PIP mode");
            break;
        case SET_MAIN_CAMERA_VISIBLE:
            _currentMainCamera = MAIN_CAMERA_VISIBLE;
            gcs().send_text(MAV_SEVERITY_INFO, "Main camera is now VISIBLE camera");
            break;
        case SET_MAIN_CAMERA_THERMAL:
            _currentMainCamera = MAIN_CAMERA_THERMAL;
            gcs().send_text(MAV_SEVERITY_INFO, "Main camera is now THERMAL camera");
            break;
        default:
            gcs().send_text(MAV_SEVERITY_INFO, "No request was made. Unknown reference to OK received answer.");
    }
}

//Manage NOT_READY answer depending on what was the command we had sent
void AP_VisionAerial_Camera::manageNotReadyAnswer(void) 
{
    switch (lastRequestMade) {
        case CAPTURE_PHOTO:
            gcs().send_text(MAV_SEVERITY_INFO, "Photo could not be taken");
            break;
        case START_RECORDING:
            gcs().send_text(MAV_SEVERITY_INFO, "Recording could not start");
            break;
        case STOP_RECORDING:
            gcs().send_text(MAV_SEVERITY_INFO, "Recording could not be stoped");
            break;
        default:
            gcs().send_text(MAV_SEVERITY_INFO, "No request was made. Unknown reference to NOT_READY received answer.");
    }
}

//Right now the only other possible answers are a value of zoom and a 
void AP_VisionAerial_Camera::parseNumericalAnswer()
{
    uint8_t _newIndex;
    float   _newRatio;

    if (isdigit(_receiveBuffer[0])) {//First value is an integer between 0-9 
        _newIndex = _receiveBuffer[0] - ASCII_NUMBER_0;//In order to transform ASCII to uint we need to subtract
        updateCorrespondingIndex(_newIndex);//This will set a zoom value

        if (_receiveBuffer[1] == ASCII_SPACE) {

            _receiveBuffer[0] = ASCII_SPACE;//We do this in order to be able then to use the atof() function and just
                                            //extract the float corresponding to the ratio

            if (isdigit(_receiveBuffer[2])) {
                _newRatio = atof(_receiveBuffer);
                gcs().send_text(MAV_SEVERITY_INFO, "New zoom index is %u and new ratio is: %f", _newIndex, _newRatio);
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "Answer is not a zoom value");
            }
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "Error parsing numerical answer. Ratio is not a number");
        }
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Error parsing numerical answer. First digit is not a number");
    }
}

void AP_VisionAerial_Camera::updateCorrespondingIndex(uint8_t index) 
{
    switch (lastRequestMade) {
        case GET_ZOOM_VISIBLE:
            _visibleZoomLevel = index;
            updateJoystickSpeedMultiplier();
            break;
        case GET_ZOOM_THERMAL:
            _thermalZoomLevel = index;
            break;
        default:
            gcs().send_text(MAV_SEVERITY_INFO, "Receive index doesn't correspond to any request made");
    }
}

//---------------Trigger functions---------------
void AP_VisionAerial_Camera::capturePhoto(void) 
{
    if (!lastRequestMade) {
        lastRequestMade = CAPTURE_PHOTO;
        _isCapturing = true;
        sendCommand(triggerCmds.capturePhotoCmd, UINT8_MAX, sizeof(triggerCmds.capturePhotoCmd));
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "CapturePhoto denied. Waiting for another command answer.");
    }
}

void AP_VisionAerial_Camera::startRecording(void) 
{
    if (!lastRequestMade) {
        if (!_isRecording) {
            lastRequestMade = START_RECORDING;
            sendCommand(triggerCmds.startRecordingCmd, UINT8_MAX, sizeof(triggerCmds.startRecordingCmd));
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "Camera already recording");
        }
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "StartRecording denied. Waiting for another command answer.");
    } 
}

void AP_VisionAerial_Camera::stopRecording(void) 
{
    if (!lastRequestMade) {
        if (_isRecording) {
            lastRequestMade = STOP_RECORDING;
            sendCommand(triggerCmds.stopRecordingCmd, UINT8_MAX, sizeof(triggerCmds.stopRecordingCmd));
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "Camera isn't recording");
        }
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "StopRecording denied. Waiting for another command answer.");
    }
}
//-----------------------------------------------

//---------------Zoom functions---------------
void AP_VisionAerial_Camera::getZoomVisible(void) 
{
    if (!lastRequestMade) {
        lastRequestMade = GET_ZOOM_VISIBLE;
        sendCommand(zoomCmds.getZoomVisibleCmd, UINT8_MAX, sizeof(zoomCmds.getZoomVisibleCmd));
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "GetZoomVisible denied. Waiting for another command answer.");
    } 
}

void AP_VisionAerial_Camera::setZoomVisible(uint8_t zoom) 
{
    if (!lastRequestMade) {
        lastRequestMade = SET_ZOOM_VISIBLE;
        requestedIndex.zoomVisible = zoom;
        sendCommand(zoomCmds.setZoomVisibleCmd, zoom, sizeof(zoomCmds.setZoomVisibleCmd));
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "SetZoomVisible denied. Waiting for another command answer.");
    }
}

void AP_VisionAerial_Camera::getZoomThermal(void) 
{
    if (!lastRequestMade) {
        lastRequestMade = GET_ZOOM_THERMAL;
        sendCommand(zoomCmds.getZoomThermalCmd, UINT8_MAX, sizeof(zoomCmds.getZoomThermalCmd));
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "GetZoomThermal denied. Waiting for another command answer.");
    }
}

void AP_VisionAerial_Camera::setZoomThermal(uint8_t zoom) 
{
    if (!lastRequestMade) {
        lastRequestMade = SET_ZOOM_THERMAL;
        requestedIndex.zoomThermal = zoom;
        sendCommand(zoomCmds.setZoomThermalCmd, zoom, sizeof(zoomCmds.setZoomThermalCmd));
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "SetZoomThermal denied. Waiting for another command answer.");
    }
}
//-----------------------------------------------

//-------------Appearenace functions-------------
void    AP_VisionAerial_Camera::setLayout(uint8_t layout)
{
    if (!lastRequestMade) {
        switch (layout) { //0->INSPECTION, 1->SECURITY, 2->FULLSCREEN, 3->PIP
            case 0:
                lastRequestMade = SET_LAYOUT_INSPECTION;
                sendCommand(appearanceCmds.setLayoutInspection, UINT8_MAX, sizeof(appearanceCmds.setLayoutInspection));
                break;
            case 1:
                lastRequestMade = SET_LAYOUT_SECURITY;
                sendCommand(appearanceCmds.setLayoutSecurity, UINT8_MAX, sizeof(appearanceCmds.setLayoutSecurity));
                break;
            case 2:
                lastRequestMade = SET_LAYOUT_FULLSCREEN;
                sendCommand(appearanceCmds.setLayoutFullscreen, UINT8_MAX, sizeof(appearanceCmds.setLayoutFullscreen));
                break;
            case 3:
                lastRequestMade = SET_LAYOUT_PIP;
                sendCommand(appearanceCmds.setLayoutPip, UINT8_MAX, sizeof(appearanceCmds.setLayoutPip));
                break;
            default:
                gcs().send_text(MAV_SEVERITY_INFO, "Unknow layout requested.");
        }
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "setLayout denied. Waiting for another command answer.");
    }
}

void    AP_VisionAerial_Camera::setMainCamera(uint8_t camera)
{
    if (!lastRequestMade) {
        
        switch (camera) { //0->VISIBLE, 1->THERMAL
            case 0:
                lastRequestMade = SET_MAIN_CAMERA_VISIBLE;
                sendCommand(appearanceCmds.setMainCameraVisible, UINT8_MAX, sizeof(appearanceCmds.setMainCameraVisible));
                break;
            case 1:
                lastRequestMade = SET_MAIN_CAMERA_THERMAL;
                sendCommand(appearanceCmds.setMainCameraThermal, UINT8_MAX, sizeof(appearanceCmds.setMainCameraThermal));
                break;
            default:
                gcs().send_text(MAV_SEVERITY_INFO, "Unknow main camera requested.");
        }
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "setMainCamera denied. Waiting for another command answer.");
    }
}
//-----------------------------------------------

void AP_VisionAerial_Camera::checkConnectionStatus(void) 
{
    if (!lastRequestMade) {
        lastRequestMade = CHECK_CONNECTION;
        sendCommand(basicCmds.checkConnection, UINT8_MAX, sizeof(basicCmds.checkConnection));
    }
}

void AP_VisionAerial_Camera::sendCommand(const char* cmd, uint8_t index, uint8_t size) 
{
    uint8_t actualSize;//Size of the payload taking into account the addition of an index

    //Check if we need to add space for an index to the payload
    if (index < UINT8_MAX) {
        actualSize = size + 2U;
    } else {
        actualSize = size;
    }
    
    if (_port->txspace() < (actualSize + 4U)) {
        return;
    }

    char _sendBuffer[actualSize + 4U];

    //---------------Fill send buffer---------------
    //Fill the send buffer with its corresponding message
    _sendBuffer[0] = WIRIS_MSG_HEADER;//Set message header on the first byte
    _sendBuffer[1] = actualSize;//Set meesagge size least significant byte (little endian)
    _sendBuffer[2] = 0;//Set meesagge size most significant byte (little endian)

    //Fill the payload of the message adding an index when necessary
    if (index < UINT8_MAX) {
        for (uint8_t i = 3; i < actualSize + 1; i++) {
            _sendBuffer[i] = cmd[i-3];
        }
        _sendBuffer[actualSize] = ASCII_SPACE;//Add a "space" between the command and the index
        _sendBuffer[actualSize + 1U] = index;//Add a "space" between the command and the index
    } else {
        for (uint8_t i = 3; i < actualSize + 3; i++) {
            _sendBuffer[i] = cmd[i-3];
        }
    }

    _sendBuffer[actualSize + 2U] = WIRIS_PAYLOAD_END;//Set end of payload on the penultimum byte
    _sendBuffer[actualSize + 3U] = WIRIS_MSG_FOOTER;//Set message footer on the last byte
    //-----------------------------------------------

    //Write send buffer to serial port
    for (uint8_t i = 0; i < sizeof(_sendBuffer); i++) {
        _port->write(_sendBuffer[i]);
    }

    //Call read_incomming to check the answer as soon as possible
    read_incoming();
}

void AP_VisionAerial_Camera::updateJoystickSpeedMultiplier(void)
{
    float newFactor = 1;

    if (_visibleZoomLevel < UINT8_MAX) {//Check if we have a correct zoom value
        newFactor = _joystick_speed_multiplier / (_visibleZoomLevel + 1);
    }
    
    AP_Mount *mount = AP::mount();
    if (mount != nullptr) {
        mount->set_joystick_speed_multiplier(newFactor);
    }
}

//Function that sends the status of the camera via Mavlink
//We will use the first field as a bitmask to send camera status:
//Using a n 16bit number we will use the bits as follows:
//When ths bits are set to 1 it means we are in the given mode
//  0th bit (least-significant): camera is initialized
//  1th bit: photo mode
//  2th bit: video mode
//  3th bit: Main camera is VISIBLE 
//  4th bit: Main camera is THERMAL
//  5th bit: Layout INSPECTION
//  6th bit: Layout SECURITY
//  7th bit: Layout FULLSCREEN
//  8th bit: Layout PIP
void AP_VisionAerial_Camera::sendCameraStatus(mavlink_channel_t chan)
{
    uint16_t statusBitmask = 0;

    //Camera is initialized
    if (_initialized_camera) {
        statusBitmask = setNthBit(statusBitmask,0);
    }

    //Set capture/video bits
    if (_isCapturing) {
        statusBitmask = setNthBit(statusBitmask,1);
    }
    if (_isRecording) {
        statusBitmask = setNthBit(statusBitmask,2);
    }

    //Set main camera bits
    switch (_currentMainCamera) {
        case MAIN_CAMERA_VISIBLE:
            statusBitmask = setNthBit(statusBitmask,3);
            break;
        case MAIN_CAMERA_THERMAL:
            statusBitmask = setNthBit(statusBitmask,4);
            break;
        default:
            //Enable below message for debugging otherwise do nothing
            //gcs().send_text(MAV_SEVERITY_INFO, "Unknow main camera status.");
            break;
    }

    //Set layout bits
    switch (_currentLayout) {
        case LAYOUT_INSPECTION:
            statusBitmask = setNthBit(statusBitmask,5);
            break;
        case LAYOUT_SECURITY:
            statusBitmask = setNthBit(statusBitmask,6);
            break;
        case LAYOUT_FULLSCREEN:
            statusBitmask = setNthBit(statusBitmask,7);
            break;
        case LAYOUT_PIP:
            statusBitmask = setNthBit(statusBitmask,8);
            break;
        default:
            //Enable below message for debugging otherwise do nothing
            //gcs().send_text(MAV_SEVERITY_INFO, "Unknow layout camera status.");
            break;
    }

    mavlink_msg_va_camera_status_send(chan, statusBitmask, _visibleZoomLevel, _thermalZoomLevel);
}

//Function that sets a given bit to 1
uint16_t AP_VisionAerial_Camera::setNthBit(uint16_t bitmask, uint8_t n)
{
    // nth bit of bitmask is being set by this operation
   return ((1 << n) | bitmask);
}

AP_VisionAerial_Camera *AP_VisionAerial_Camera::_singleton;
namespace AP 
{
    AP_VisionAerial_Camera *visionAerial_camera() {
        return AP_VisionAerial_Camera::get_singleton();
    }
}
