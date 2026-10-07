#ifndef CAN_FRAME_H
#define CAN_FRAME_H

#include <stdint.h>

/**
 * @brief Hardware-independent CAN frame, decoupled from ESP-IDF's twai_message_t so
 * protocol code (HondaCANModule) never needs to know it's talking to a TWAI peripheral.
 */
struct CanFrame {
    uint32_t id = 0;
    bool extended = false;
    uint8_t dlc = 0;
    uint8_t data[8] = {0};
};

#endif // CAN_FRAME_H
