#pragma once
#include "USBHID.h"
#if CONFIG_TINYUSB_HID_ENABLED

#define BUTTON_A        0
#define BUTTON_B        1
#define BUTTON_C        2
#define BUTTON_X        3
#define BUTTON_Y        4
#define BUTTON_Z        5
#define BUTTON_TL       6
#define BUTTON_TR       7
#define BUTTON_TL2      8
#define BUTTON_TR2      9
#define BUTTON_SELECT   10
#define BUTTON_START    11
#define BUTTON_MODE     12
#define BUTTON_THUMBL   13
#define BUTTON_THUMBR   14

#define BUTTON_SOUTH    BUTTON_A
#define BUTTON_EAST     BUTTON_B
#define BUTTON_NORTH    BUTTON_X
#define BUTTON_WEST     BUTTON_Y

#define HAT_CENTER      0
#define HAT_UP          1
#define HAT_UP_RIGHT    2
#define HAT_RIGHT       3
#define HAT_DOWN_RIGHT  4
#define HAT_DOWN        5
#define HAT_DOWN_LEFT   6
#define HAT_LEFT        7
#define HAT_UP_LEFT     8

typedef struct __attribute__((packed)) {
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t rz;
    int16_t rx;
    int16_t ry;
    uint8_t hat;
    uint32_t buttons;
} hid_gamepad16_report_t;

class USBHIDGamepad16: public USBHIDDevice {

    private:
        USBHID hid;
        void (*_outputCallback)(uint8_t report_id, const uint8_t* buffer, uint16_t len);
        uint16_t (*_getFeatureCallback)(uint8_t report_id, uint8_t* buffer, uint16_t len);
        void (*_setFeatureCallback)(uint8_t report_id, const uint8_t* buffer, uint16_t len);
        int16_t _x;
        int16_t _y;
        int16_t _z;
        int16_t _rz;
        int16_t _rx;
        int16_t _ry;
        uint8_t _hat;
        uint32_t _buttons;
        bool write();

    public:
        USBHIDGamepad16(void);
        void begin(void);
        void end(void);
        bool leftStick(int16_t x, int16_t y);
        bool rightStick(int16_t z, int16_t rz);
        bool leftTrigger(int16_t rx);
        bool rightTrigger(int16_t ry);
        bool hat(uint8_t hat);
        bool pressButton(uint8_t button);
        bool releaseButton(uint8_t button);
        bool send(int16_t x, int16_t y, int16_t z, int16_t rz, int16_t rx, int16_t ry, uint8_t hat, uint32_t buttons);
        bool sendRawReport(uint8_t report_id, const void* data, uint8_t len);
        void setOutputCallback(void (*callback)(uint8_t report_id, const uint8_t* buffer, uint16_t len));
        void setGetFeatureCallback(uint16_t (*callback)(uint8_t report_id, uint8_t* buffer, uint16_t len));
        void setSetFeatureCallback(void (*callback)(uint8_t report_id, const uint8_t* buffer, uint16_t len));
        uint16_t _onGetDescriptor(uint8_t* buffer);
        uint16_t _onGetFeature(uint8_t report_id, uint8_t* buffer, uint16_t len);
        void _onSetFeature(uint8_t report_id, const uint8_t* buffer, uint16_t len);
        void _onOutput(uint8_t report_id, const uint8_t* buffer, uint16_t len);
};

#endif
