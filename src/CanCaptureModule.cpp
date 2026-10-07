#include "CanCaptureModule.h"

CanCaptureModule::CanCaptureModule(ICanRx& rx) : _rx(rx), _core(rx, _sink) {}

size_t CanCaptureModule::SerialSink::availableForWrite() {
    const int room = Serial.availableForWrite();
    return room > 0 ? (size_t)room : 0;
}

void CanCaptureModule::SerialSink::write(const char* data, size_t len) {
    Serial.write(reinterpret_cast<const uint8_t*>(data), len);
}

bool CanCaptureModule::begin() {
    _started = _rx.begin();
    if (_started) {
        Serial.println("# moto-connectivity-node CAN capture: TWAI listen-only, 500 kbps, accept-all.");
        Serial.println("# F,<t_us>,<id hex>,<S|X>,<dlc>,<data hex> | S,... summary every 10 s | FINAL,... at 300 s");
        _lastMicros32 = (uint32_t)micros();
    }
    return _started;
}

void CanCaptureModule::update(SystemState& state) {
    (void)state; // a producer by interface; the capture writes nothing into SystemState
    const uint32_t now32 = (uint32_t)micros();
    _timeUs += (uint32_t)(now32 - _lastMicros32); // wrap-safe while update() runs at least every 71 min
    _lastMicros32 = now32;
    _core.step(_timeUs);
}
