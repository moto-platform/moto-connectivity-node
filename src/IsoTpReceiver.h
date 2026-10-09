#ifndef ISOTP_RECEIVER_H
#define ISOTP_RECEIVER_H

#include <stdint.h>
#include <string.h>

#include "hal/CanFrame.h"
#include "vehicle_cl250.h" // generated: external/moto-vehicle-defs/gen/c/conn/

/**
 * @brief D-059 item 3: reassembly of ONE segmented ISO-TP answer (ISO 15765-2 First Frame
 * + Consecutive Frames) into a static buffer of VEHICLE_CL250_MAX_FF_DL bytes. Only the
 * discovery probe env (CONN_DISCOVERY_PROBE) uses it. TEMPORARY, like conn's tester
 * (D-023): moto-rt-core owns the long-term ISO-TP client.
 *
 * Pure logic, no CAN access and no transmission: HondaCANModule decides whether a First
 * Frame gets the one FC.CTS, sends it through the D-020 frame gate and only then calls
 * start(). Every limit comes from the generated defs: FF_DL <= VEHICLE_CL250_MAX_FF_DL, and
 * N_Cr = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS (uds/vehicle_cl250.yaml transport comment).
 * A reception is aborted on N_Cr, a sequence-number gap, a lost frame (the driver's lost
 * count moved since the First Frame), an unexpected frame, or when the whole reception
 * takes longer than VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS.
 *
 * Times are the drain time of each frame (the caller's millis() of that pass), and
 * expired() is evaluated only after the RX queue is drained, so a slow loop pass with the
 * CFs already queued is not an N_Cr abort (architecture-guard).
 */
class IsoTpReceiver {
public:
    enum class Result : uint8_t { IN_PROGRESS, COMPLETE, ABORTED };
    enum class AbortReason : uint8_t {
        NONE,
        N_CR_TIMEOUT,     // the next CF did not arrive within N_Cr
        TOTAL_TIMEOUT,    // the reception took longer than the generated maximum wait
        SEQUENCE_GAP,     // a CF with an unexpected sequence number
        LOST_FRAME,       // the receiver lost a frame during the reception
        UNEXPECTED_FRAME, // a First or Single Frame on the reception's ID, or a short CF
    };

    // ISO 15765-2 PCI frame types (high nibble of byte 0) and the CF sequence number mask.
    static const uint8_t kPciSingleFrame = 0x0;
    static const uint8_t kPciFirstFrame = 0x1;
    static const uint8_t kPciConsecutiveFrame = 0x2;
    static const uint8_t kSequenceMask = 0x0F;
    static const uint8_t kFirstFramePayload = 6; // data bytes after the 2-byte FF PCI
    static const uint8_t kConsecutivePayload = 7; // data bytes after the 1-byte CF PCI
    // With 8-byte CAN frames a message of up to 7 bytes is a Single Frame; an FF below 8 is
    // invalid (ISO 15765-2), and FF_DL 0 announces the 32-bit escape length (refused).
    static const uint16_t kMinFirstFrameLength = 8;

    static uint8_t frameType(const CanFrame& f) { return (uint8_t)((f.data[0] >> 4) & 0x0F); }

    /**
     * @brief FF_DL of an acceptable First Frame, or 0: the frame must be an 8-byte FF with
     * kMinFirstFrameLength <= FF_DL <= VEHICLE_CL250_MAX_FF_DL. Anything else gets no FC.
     */
    static uint16_t acceptableLength(const CanFrame& f) {
        uint16_t len = firstFrameLength(f);
        if (f.dlc != VEHICLE_CL250_FRAME_DLC || frameType(f) != kPciFirstFrame ||
            len < kMinFirstFrameLength || len > VEHICLE_CL250_MAX_FF_DL) {
            return 0;
        }
        return len;
    }

    // The 12-bit FF_DL as announced (also for refused frames, for the report).
    static uint16_t firstFrameLength(const CanFrame& f) {
        return (uint16_t)(((uint16_t)(f.data[0] & 0x0F) << 8) | f.data[1]);
    }

    /**
     * @brief Starts a reception with an acceptable First Frame (acceptableLength() != 0),
     * after its FC.CTS went out. `lostBaseline` is the driver's lost-frame count now.
     * @return false (and nothing started) for a frame acceptableLength() refuses.
     */
    bool start(const CanFrame& ff, unsigned long nowMs, uint32_t lostBaseline) {
        reset();
        uint16_t len = acceptableLength(ff);
        if (len == 0) {
            return false;
        }
        _length = len;
        memcpy(_buffer, &ff.data[2], kFirstFramePayload);
        _received = kFirstFramePayload;
        _nextSequence = 1;
        _id = ff.id;
        _extended = ff.extended;
        _startMs = nowMs;
        _lastFrameMs = nowMs;
        _lostBaseline = lostBaseline;
        _active = true;
        return true;
    }

    // True for a frame on the ID and format the running reception came on.
    bool fromReceptionId(const CanFrame& f) const {
        return _active && f.id == _id && f.extended == _extended;
    }

    /**
     * @brief One Consecutive Frame from the reception's ID, drained at `nowMs`.
     */
    Result onConsecutive(const CanFrame& cf, unsigned long nowMs) {
        if (!_active) {
            return Result::ABORTED;
        }
        if (frameType(cf) != kPciConsecutiveFrame || (cf.data[0] & kSequenceMask) != _nextSequence) {
            abort(AbortReason::SEQUENCE_GAP);
            return Result::ABORTED;
        }
        uint16_t left = (uint16_t)(_length - _received);
        uint8_t take = (left < kConsecutivePayload) ? (uint8_t)left : (uint8_t)kConsecutivePayload;
        if (cf.dlc < (uint8_t)(1u + take)) {
            abort(AbortReason::UNEXPECTED_FRAME);
            return Result::ABORTED;
        }
        memcpy(&_buffer[_received], &cf.data[1], take);
        _received = (uint16_t)(_received + take);
        _nextSequence = (uint8_t)((_nextSequence + 1u) & kSequenceMask);
        _lastFrameMs = nowMs;
        if (_received >= _length) {
            _active = false;
            _complete = true;
            return Result::COMPLETE;
        }
        return Result::IN_PROGRESS;
    }

    /**
     * @brief Checked after the RX queue is drained: aborts on a lost frame or a timeout.
     * @return true if the reception was aborted by this call.
     */
    bool checkAfterDrain(unsigned long nowMs, uint32_t lostCount) {
        if (!_active) {
            return false;
        }
        if (lostCount != _lostBaseline) {
            abort(AbortReason::LOST_FRAME);
        } else if (nowMs - _lastFrameMs > (unsigned long)VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS) {
            abort(AbortReason::N_CR_TIMEOUT);
        } else if (nowMs - _startMs > (unsigned long)VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS) {
            abort(AbortReason::TOTAL_TIMEOUT);
        }
        return !_active;
    }

    // Also before start() (the FC still pending): the first reason sticks.
    void abort(AbortReason reason) {
        if (!_complete && _abortReason == AbortReason::NONE) {
            _active = false;
            _abortReason = reason;
        }
    }

    // Also clears the buffer, so no answer (e.g. the VIN) stays in RAM after it was reported.
    void reset() {
        memset(_buffer, 0, sizeof(_buffer));
        _active = false;
        _complete = false;
        _abortReason = AbortReason::NONE;
        _length = 0;
        _received = 0;
    }

    bool active() const { return _active; }
    bool complete() const { return _complete; }
    AbortReason abortReason() const { return _abortReason; }
    const uint8_t* payload() const { return _buffer; }
    uint16_t length() const { return _length; }
    uint16_t received() const { return _received; }

    static const char* reasonText(AbortReason r) {
        switch (r) {
            case AbortReason::N_CR_TIMEOUT: return "N_Cr timeout";
            case AbortReason::TOTAL_TIMEOUT: return "reception too long";
            case AbortReason::SEQUENCE_GAP: return "sequence gap";
            case AbortReason::LOST_FRAME: return "lost frame";
            case AbortReason::UNEXPECTED_FRAME: return "unexpected frame";
            default: return "none";
        }
    }

private:
    uint8_t _buffer[VEHICLE_CL250_MAX_FF_DL] = {0};
    uint16_t _length = 0;
    uint16_t _received = 0;
    uint8_t _nextSequence = 1;
    uint32_t _id = 0;
    bool _extended = false;
    bool _active = false;
    bool _complete = false;
    AbortReason _abortReason = AbortReason::NONE;
    unsigned long _startMs = 0;
    unsigned long _lastFrameMs = 0;
    uint32_t _lostBaseline = 0;
};

#endif // ISOTP_RECEIVER_H
