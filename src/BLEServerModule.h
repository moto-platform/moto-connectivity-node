#ifndef BLE_SERVER_MODULE_H
#define BLE_SERVER_MODULE_H

#include "IModule.h"
#include "BLETelemetryPacket.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// G1.2 -- Producer/consumer handoff for BLE telematics writes.
// onConnect/onDisconnect/onWrite run in the NimBLE/Bluedroid stack's own FreeRTOS
// task, not the Arduino main loop task. They used to write straight into the shared
// SystemState while the main loop (Nextion/Logger/WiFi) read it concurrently with no
// synchronization -- a real data race on state.telematics. Instead, callbacks now only
// build a local TelematicsEvent and push it onto _telematicsQueue; only update()
// (running on the main loop task) ever reads xQueueReceive() and writes SystemState.
enum class TelematicsEventType : uint8_t { CONNECTION, DATA };

struct TelematicsEvent {
    TelematicsEventType type = TelematicsEventType::DATA;

    // Valid when type == CONNECTION.
    bool connected = false;

    // Valid when type == DATA. Each field is applied in update() only if its
    // "has" flag is set, preserving the original "partial message" semantics
    // (e.g. a DIST-only write must not clear songTitle/artistName).
    bool hasSong = false;
    char songTitle[32] = {0};
    bool hasArtist = false;
    char artistName[32] = {0};
    bool hasDistance = false;
    uint16_t navDistance = 0;
    bool hasIcon = false;
    uint8_t navIconID = 0;
};

/**
 * @brief Bluetooth Low Energy (BLE) Server module.
 * Streams real-time telemetry packets to connected mobile devices (iOS / Android)
 * and receives smartphone telematics (music track info, turn-by-turn navigation data).
 */
class BLEServerModule : public IProducerModule, public BLEServerCallbacks, public BLECharacteristicCallbacks {
private:
    BLEServer* _pServer = nullptr;
    BLECharacteristic* _pTxCharacteristic = nullptr;
    BLECharacteristic* _pRxCharacteristic = nullptr;

    bool _initialized = false;
    bool _deviceConnected = false;
    bool _oldDeviceConnected = false;
    unsigned long _lastNotify = 0;
    uint8_t _txSeq = 0; // G3.3 -- rolling packet sequence counter, see BLETelemetryPacket.h

    QueueHandle_t _telematicsQueue = nullptr;

public:
    BLEServerModule();
    virtual ~BLEServerModule() {}

    bool begin() override;
    void update(SystemState& state) override;
    bool isHealthy() const override { return _initialized; }

    // BLEServerCallbacks
    void onConnect(BLEServer* pServer) override;
    void onDisconnect(BLEServer* pServer) override;

    // BLECharacteristicCallbacks
    void onWrite(BLECharacteristic* pCharacteristic) override;
};

#endif // BLE_SERVER_MODULE_H
