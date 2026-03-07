#include "USBHID.h"

#if CONFIG_TINYUSB_HID_ENABLED

#include "USBHIDGamepad16.h"

// Clean stable descriptor:
// - 16-bit axes + hat + 32 buttons
// - Optional vendor output report (RID 0x20) for host bridge torque commands
static const uint8_t report_descriptor_gamepad16[] = {
    0x05, 0x01,
    0x09, 0x05,
    0xA1, 0x01,
    0x85, HID_REPORT_ID_GAMEPAD,

    0x16, 0x00, 0x80,
    0x26, 0xFF, 0x7F,
    0x75, 0x10,
    0x95, 0x06,
    0x09, 0x30,
    0x09, 0x31,
    0x09, 0x32,
    0x09, 0x35,
    0x09, 0x33,
    0x09, 0x34,
    0x81, 0x02,

    0x05, 0x01,
    0x09, 0x39,
    0x15, 0x00,
    0x25, 0x08,
    0x35, 0x00,
    0x46, 0x3B, 0x01,
    0x65, 0x14,
    0x75, 0x04,
    0x95, 0x01,
    0x81, 0x42,

    0x65, 0x00,
    0x75, 0x04,
    0x95, 0x01,
    0x81, 0x01,

    0x05, 0x09,
    0x19, 0x01,
    0x29, 0x20,
    0x15, 0x00,
    0x25, 0x01,
    0x75, 0x01,
    0x95, 0x20,
    0x81, 0x02,

    // Host-bridge output report channel.
    0x06, 0x00, 0xFF,
    0x09, 0x01,
    0x85, 0x20,
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x08,
    0x91, 0x02,

    0xC0
};

USBHIDGamepad16::USBHIDGamepad16()
    : hid(),
      _outputCallback(nullptr),
      _getFeatureCallback(nullptr),
      _setFeatureCallback(nullptr),
      _x(0),
      _y(0),
      _z(0),
      _rz(0),
      _rx(0),
      _ry(0),
      _hat(0),
      _buttons(0) {
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        hid.addDevice(this, sizeof(report_descriptor_gamepad16));
    }
}

uint16_t USBHIDGamepad16::_onGetDescriptor(uint8_t* dst) {
    memcpy(dst, report_descriptor_gamepad16, sizeof(report_descriptor_gamepad16));
    return sizeof(report_descriptor_gamepad16);
}

void USBHIDGamepad16::begin() {
    hid.begin();
}

void USBHIDGamepad16::end() {
}

bool USBHIDGamepad16::write() {
    hid_gamepad16_report_t report = {
        .x = _x,
        .y = _y,
        .z = _z,
        .rz = _rz,
        .rx = _rx,
        .ry = _ry,
        .hat = _hat,
        .buttons = _buttons
    };
    return hid.SendReport(HID_REPORT_ID_GAMEPAD, &report, sizeof(report));
}

bool USBHIDGamepad16::leftStick(int16_t x, int16_t y) {
    _x = x;
    _y = y;
    return write();
}

bool USBHIDGamepad16::rightStick(int16_t z, int16_t rz) {
    _z = z;
    _rz = rz;
    return write();
}

bool USBHIDGamepad16::leftTrigger(int16_t rx) {
    _rx = rx;
    return write();
}

bool USBHIDGamepad16::rightTrigger(int16_t ry) {
    _ry = ry;
    return write();
}

bool USBHIDGamepad16::hat(uint8_t hat) {
    if (hat > 9) {
        return false;
    }
    _hat = hat;
    return write();
}

bool USBHIDGamepad16::pressButton(uint8_t button) {
    if (button > 31) {
        return false;
    }
    _buttons |= (1u << button);
    return write();
}

bool USBHIDGamepad16::releaseButton(uint8_t button) {
    if (button > 31) {
        return false;
    }
    _buttons &= ~(1u << button);
    return write();
}

bool USBHIDGamepad16::send(int16_t x, int16_t y, int16_t z, int16_t rz, int16_t rx, int16_t ry, uint8_t hat, uint32_t buttons) {
    if (hat > 9) {
        return false;
    }
    _x = x;
    _y = y;
    _z = z;
    _rz = rz;
    _rx = rx;
    _ry = ry;
    _hat = hat;
    _buttons = buttons;
    return write();
}

bool USBHIDGamepad16::sendRawReport(uint8_t report_id, const void* data, uint8_t len) {
    return hid.SendReport(report_id, data, len);
}

void USBHIDGamepad16::setOutputCallback(void (*callback)(uint8_t report_id, const uint8_t* buffer, uint16_t len)) {
    _outputCallback = callback;
}

void USBHIDGamepad16::setGetFeatureCallback(uint16_t (*callback)(uint8_t report_id, uint8_t* buffer, uint16_t len)) {
    _getFeatureCallback = callback;
}

void USBHIDGamepad16::setSetFeatureCallback(void (*callback)(uint8_t report_id, const uint8_t* buffer, uint16_t len)) {
    _setFeatureCallback = callback;
}

uint16_t USBHIDGamepad16::_onGetFeature(uint8_t report_id, uint8_t* buffer, uint16_t len) {
    if (_getFeatureCallback) {
        return _getFeatureCallback(report_id, buffer, len);
    }
    return 0;
}

void USBHIDGamepad16::_onSetFeature(uint8_t report_id, const uint8_t* buffer, uint16_t len) {
    if (_setFeatureCallback) {
        _setFeatureCallback(report_id, buffer, len);
    }
}

void USBHIDGamepad16::_onOutput(uint8_t report_id, const uint8_t* buffer, uint16_t len) {
    if (_outputCallback) {
        _outputCallback(report_id, buffer, len);
    }
}

#endif
