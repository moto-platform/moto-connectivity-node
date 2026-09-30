#include "BLEServerModule.h"
#include <cstring>

// G4.2 -- Real GATT-level security API (Bluedroid, esp32-arduino framework), not NimBLE.
// Verified against the framework headers actually installed for this project:
//   ~/.platformio/packages/framework-arduinoespressif32/libraries/BLE/src/BLESecurity.h
//   .../esp_gap_ble_api.h  (ESP_LE_AUTH_REQ_SC_BOND, ESP_IO_CAP_NONE)
//   .../esp_gatt_defs.h    (ESP_GATT_PERM_WRITE_ENCRYPTED)
// This BLECharacteristic implementation has NO PROPERTY_WRITE_ENC flag -- encryption is
// enforced purely via BLECharacteristic::setAccessPermissions(esp_gatt_perm_t), which is
// GATT-server-side and independent of the ATT PROPERTY_* bits.
#include <BLESecurity.h>

// Custom UUIDs for Honda Telemetry BLE Service & Characteristics
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_TX "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_RX "828919fe-e41c-40ee-b4c6-2c974c2d3345"
// IMU block notifications (docs/ble_telemetry_packet_schema.json gatt.characteristics.imu).
#define CHARACTERISTIC_UUID_IMU "f62bc083-e25d-46b3-aa0b-2e9e6bc8f1e5"

// Standard Device Information Service UUID (0x180A)
#define DEVICE_INFO_SERVICE_UUID "0000180a-0000-1000-8000-00805f9b34fb"

std::atomic<uint16_t> BLEServerModule::s_peerMtu{BLE_DEFAULT_MTU};

BLEServerModule::BLEServerModule(ImuRing* imuRing) : _imuRing(imuRing) {}

// BLE stack task context. The central starts the MTU exchange (moto-mobile requests
// 185); until then, and for every new connection, the ATT default of 23 applies.
void BLEServerModule::gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gattsIf,
                                        esp_ble_gatts_cb_param_t* param) {
    (void)gattsIf;
    switch (event) {
        case ESP_GATTS_CONNECT_EVT:
        case ESP_GATTS_DISCONNECT_EVT:
            s_peerMtu.store(BLE_DEFAULT_MTU, std::memory_order_relaxed);
            break;
        case ESP_GATTS_MTU_EVT:
            s_peerMtu.store(param->mtu.mtu, std::memory_order_relaxed);
            break;
        default:
            break;
    }
}

bool BLEServerModule::begin() {
    // Initialize BLE Device with maximum MTU (512 bytes)
    BLEDevice::init("Honda-CL250-Telemetry");
    BLEDevice::setMTU(512);
    BLEDevice::setCustomGattsHandler(&BLEServerModule::gattsEventHandler);

    // G4.2 -- BLE access control: the RX (write) characteristic accepts phone-controlled
    // strings (song title/artist, nav data) that end up on the rider's Nextion dashboard.
    // Without this, ANY BLE device in range can write to it without ever pairing. This
    // forces bonding + link encryption before a write is accepted; onWrite() will simply
    // never fire for an unbonded/unencrypted peer (the stack rejects the ATT write itself).
    //
    // IO capability is ESP_IO_CAP_NONE ("Just Works" pairing) because this board has no
    // display or keypad to show/enter a 6-digit passkey -- it's the only realistic option
    // for this hardware. Just Works still requires the OS-level pairing/bonding dialog and
    // encrypts the link, but it does NOT protect against a man-in-the-middle during the
    // initial pairing (no ESP_LE_AUTH_REQ_MITM). That tradeoff is acceptable here: the goal
    // is to stop opportunistic/unpaired writes from strangers in BLE range, not to defend
    // against an active attacker during the one-time pairing handshake.
    //
    // UX note: because bonding is now mandatory, the phone app (moto-mobile) will see an
    // OS-level "Pair with Honda-CL250-Telemetry?" prompt on first connection. No mobile
    // code change is required -- the OS Bluetooth stack triggers this pairing dialog
    // automatically in response to the peripheral's security requirements.
    // Heap use (PLATFORM-RULES 5 "avoided elsewhere"): the Arduino-ESP32 BLE API only
    // accepts heap-allocated BLESecurity/BLE2902 objects. They are created once in
    // begin(), never freed or re-allocated, and this is not a safety path.
    BLESecurity* pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
    pSecurity->setCapability(ESP_IO_CAP_NONE);
    pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    pSecurity->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    pSecurity->setKeySize(16);

    // Create BLE Server instance
    _pServer = BLEDevice::createServer();
    _pServer->setCallbacks(this);

    // 1. Create Standard Device Information Service (0x180A)
    BLEService* pInfoService = _pServer->createService(BLEUUID(DEVICE_INFO_SERVICE_UUID));
    
    // Manufacturer Name String (0x2A29)
    BLECharacteristic* pMfgChar = pInfoService->createCharacteristic(
        BLEUUID((uint16_t)0x2A29),
        BLECharacteristic::PROPERTY_READ
    );
    pMfgChar->setValue("Honda Telemetry Labs");

    // Model Number String (0x2A24)
    BLECharacteristic* pModelChar = pInfoService->createCharacteristic(
        BLEUUID((uint16_t)0x2A24),
        BLECharacteristic::PROPERTY_READ
    );
    pModelChar->setValue("CL250-v1.0");

    pInfoService->start();

    // 2. Create Custom Telemetry Service
    BLEService* pService = _pServer->createService(SERVICE_UUID);

    // Create Telemetry TX (Notify) Characteristic
    _pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    _pTxCharacteristic->addDescriptor(new BLE2902()); // one-time, see begin() note on heap use

    // IMU block (Notify) Characteristic, D-032. Present even without a sensor so the phone
    // sees a stable GATT table; it just never notifies then.
    _pImuCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_IMU,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    _pImuCharacteristic->addDescriptor(new BLE2902()); // one-time, see begin() note on heap use

    // Create Telematics RX (Write) Characteristic
    _pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    // G4.2 -- require an encrypted (bonded) link to write here. This framework's
    // BLECharacteristic has no PROPERTY_WRITE_ENC bit; encryption is enforced solely via
    // the GATT access permission below. Deliberately NOT OR'd with ESP_GATT_PERM_WRITE --
    // that would still permit plaintext writes and defeat the purpose.
    _pRxCharacteristic->setAccessPermissions(ESP_GATT_PERM_WRITE_ENCRYPTED);
    _pRxCharacteristic->setCallbacks(this);

    pService->start();

    // 3. Configure Fast Auto-Advertising for iOS & Android
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // Preferred connection interval for fast iPhone pairing
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    // G1.2: depth 8 is comfortably more than one event per BLE stack callback burst
    // (connect/disconnect + a handful of telematics writes) between two main-loop passes.
    _telematicsQueue = xQueueCreate(8, sizeof(TelematicsEvent));
    if (_telematicsQueue == nullptr) {
        Serial.println("[BLE ERROR] Failed to create telematics event queue!");
    }

    _initialized = true;
    return true;
}

void BLEServerModule::onConnect(BLEServer* pServer) {
    // Runs on the BLE stack task -- do not touch SystemState here, just enqueue.
    TelematicsEvent evt;
    evt.type = TelematicsEventType::CONNECTION;
    evt.connected = true;
    if (_telematicsQueue) {
        xQueueSend(_telematicsQueue, &evt, 0);
    }
}

void BLEServerModule::onDisconnect(BLEServer* pServer) {
    TelematicsEvent evt;
    evt.type = TelematicsEventType::CONNECTION;
    evt.connected = false;
    if (_telematicsQueue) {
        xQueueSend(_telematicsQueue, &evt, 0);
    }
}

void BLEServerModule::onWrite(BLECharacteristic* pCharacteristic) {
    // Runs on the BLE stack task -- parse into a local, stack-only event and enqueue it.
    // SystemState itself is only ever written from update() on the main loop task.
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() == 0 || !_telematicsQueue) {
        return;
    }

    const char* data = rxValue.c_str();
    TelematicsEvent evt;
    evt.type = TelematicsEventType::DATA;

    const char* songPtr = strstr(data, "SONG:");
    if (songPtr) {
        sscanf(songPtr, "SONG:%31[^|]", evt.songTitle);
        evt.hasSong = true;
    }

    const char* artistPtr = strstr(data, "ARTIST:");
    if (artistPtr) {
        sscanf(artistPtr, "ARTIST:%31[^|]", evt.artistName);
        evt.hasArtist = true;
    }

    const char* distPtr = strstr(data, "DIST:");
    if (distPtr) {
        int distVal = 0;
        if (sscanf(distPtr, "DIST:%d", &distVal) == 1) {
            evt.navDistance = (uint16_t)distVal;
            evt.hasDistance = true;
        }
    }

    const char* iconPtr = strstr(data, "ICON:");
    if (iconPtr) {
        int iconVal = 0;
        if (sscanf(iconPtr, "ICON:%d", &iconVal) == 1) {
            evt.navIconID = (uint8_t)iconVal;
            evt.hasIcon = true;
        }
    }

    if (evt.hasSong || evt.hasArtist || evt.hasDistance || evt.hasIcon) {
        xQueueSend(_telematicsQueue, &evt, 0);
    }
}

void BLEServerModule::update(SystemState& state) {
    unsigned long now = millis();

    // Drain every queued BLE-stack-task event here, on the main loop task -- the only
    // place SystemState.telematics and _deviceConnected are written (G1.2).
    if (_telematicsQueue) {
        TelematicsEvent evt;
        while (xQueueReceive(_telematicsQueue, &evt, 0) == pdTRUE) {
            if (evt.type == TelematicsEventType::CONNECTION) {
                _deviceConnected = evt.connected;
                state.telematics.phoneConnected = evt.connected;
            } else {
                if (evt.hasSong)     strncpy(state.telematics.songTitle, evt.songTitle, sizeof(state.telematics.songTitle) - 1);
                if (evt.hasArtist)   strncpy(state.telematics.artistName, evt.artistName, sizeof(state.telematics.artistName) - 1);
                if (evt.hasDistance) state.telematics.navDistance = evt.navDistance;
                if (evt.hasIcon)     state.telematics.navIconID = evt.navIconID;
            }
        }
    }

    // Stream telemetry and IMU blocks at 10 Hz. Notifications are queued to the BLE stack
    // task and never wait for the phone, so this cannot hold up the CAN poller.
    if (_deviceConnected && (now - _lastNotify >= 100)) {
        _lastNotify = now;
        uint16_t mtu = s_peerMtu.load(std::memory_order_relaxed);
        if (mtu != _loggedMtu) {
            _loggedMtu = mtu;
            Serial.printf("[BLE] Peer MTU %u: telemetry v%u, %u IMU samples per block.\n",
                mtu, telemetryVersionForMtu(mtu), imuSamplesPerBlock(mtu));
        }
        sendTelemetry(state, now, mtu);
        _imuDue = true;
    } else if (_deviceConnected && _imuDue) {
        // IMU blocks go out on the pass after the telemetry notify, so one loop pass never
        // carries more than IMU_MAX_BLOCKS_PER_NOTIFY notifications (CAN poller timing).
        _imuDue = false;
        sendImuBlocks(s_peerMtu.load(std::memory_order_relaxed));
    } else if (!_deviceConnected && _imuRing) {
        // Nobody to send to: a new connection starts with fresh samples and no old events.
        _imuRing->clear();
        _imuRing->takeEvents();
    }

    // Auto-restart BLE advertising upon disconnection so phone can reconnect. Waits 500 ms
    // without blocking the loop (the legacy delay(500) stalled the CAN poller).
    if (!_deviceConnected && _oldDeviceConnected) {
        if (_disconnectedAtMs == 0) {
            _disconnectedAtMs = now == 0 ? 1 : now;
        } else if (now - _disconnectedAtMs >= 500) {
            _pServer->startAdvertising();
            _oldDeviceConnected = _deviceConnected;
            _disconnectedAtMs = 0;
        }
    }

    if (_deviceConnected && !_oldDeviceConnected) {
        _oldDeviceConnected = _deviceConnected;
        _disconnectedAtMs = 0;
    }
}

void BLEServerModule::sendTelemetry(const SystemState& state, unsigned long now, uint16_t mtu) {
    // Never larger than MTU - 3 (the stack would silently truncate it): below the v3 size
    // the v2 fallback goes out instead. Both share the sequence counter.
    if (telemetryVersionForMtu(mtu) == BLE_PACKET_VERSION) {
        BLETelemetryPacketV3 packet = buildTelemetryPacketV3(state, _txSeq++, (uint32_t)now);
        _pTxCharacteristic->setValue(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
    } else {
        BLETelemetryPacketV2 packet = buildTelemetryPacketV2(state, _txSeq++);
        _pTxCharacteristic->setValue(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
    }
    _pTxCharacteristic->notify();
}

void BLEServerModule::sendImuBlocks(uint16_t mtu) {
    if (_imuRing == nullptr) {
        return;
    }
    uint8_t samplesPerBlock = imuSamplesPerBlock(mtu);
    if (samplesPerBlock == 0) {
        // MTU too small: suspended; the receiver sees the lost samples as an index gap.
        _imuRing->clear();
        _imuRing->takeEvents();
        return;
    }
    // Events are taken only when a block will carry them, so none is lost on an empty pass.
    uint8_t events = _imuRing->size() > 0 ? _imuRing->takeEvents() : 0;
    for (uint8_t i = 0; i < IMU_MAX_BLOCKS_PER_NOTIFY; i++) {
        size_t len = packImuBlock(*_imuRing, samplesPerBlock, _imuSeq, events,
                                  _imuBlock, sizeof(_imuBlock));
        if (len == 0) {
            break;
        }
        _imuSeq++;
        events = 0; // reported once, with the first block after the event
        _pImuCharacteristic->setValue(_imuBlock, len);
        _pImuCharacteristic->notify();
    }
}
