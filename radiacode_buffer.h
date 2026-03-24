#pragma once
#include <vector>
#include <cstdint>
#include <mutex>
#include <thread>
#include <chrono>
#include "radiacode_lib.h"

// ----------------------------------------------------------------------------
// RC-102 ACTUAL WIRE FORMAT (observed on device, differs from spec PDF)
//
// Both REQUESTS and RESPONSES use the same framing:
//   [Length(4LE)] [Type(2LE)] [Spare(1)] [Seq(1)] [Payload...]
//
// Length = number of bytes AFTER the 4-byte length field itself, i.e.
//   Length = 4 (type+spare+seq) + sizeof(payload)
//
// Example SET_EXCHANGE response captured from RC-102:
//   10 00 00 00  <- length = 16
//   07 00        <- type = 0x0007
//   00           <- spare
//   80           <- seq
//   05 01 da da 20 10 00 10 01 02 00 00  <- 12 bytes payload
//   total = 4 (length field) + 16 (length value) = 20 bytes  ✓
// ----------------------------------------------------------------------------

// FRAME_* offset constants are defined in radiacode_lib.h (included above).

class RadiacodeBuffer {
private:
    std::vector<uint8_t> buf_;
    mutable std::mutex   mtx_;

public:
    // Called from the BLE notify callback (may be on a different thread).
    void addData(const uint8_t* data, size_t length) {
        std::lock_guard<std::mutex> lock(mtx_);
        buf_.insert(buf_.end(), data, data + length);
    }

    // Non-destructive snapshot (useful for debug printing on timeout).
    std::vector<uint8_t> getData() {
        std::lock_guard<std::mutex> lock(mtx_);
        return buf_;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mtx_);
        buf_.clear();
    }

    // ------------------------------------------------------------------------
    // waitForResponse
    //
    // Blocks up to timeout_ms scanning for a complete framed response whose
    // Type field matches cmd_id.
    //
    // Frame layout (RC-102 actual):
    //   [Length(4LE)] [Type(2LE)] [Spare(1)] [Seq(1)] [Payload(Length-4 bytes)]
    //
    // On success:
    //   - out.raw holds the full frame bytes (including the length prefix)
    //   - The frame (and any preceding unrelated frames) is consumed from buf_
    //   - Returns true
    //
    // Use ParsedResponse accessors to read fields — they account for the
    // 4-byte length prefix at the front of out.raw.
    // ------------------------------------------------------------------------
    bool waitForResponse(uint16_t        cmd_id,
                         ParsedResponse& out,
                         int             timeout_ms = 3000)
    {
        using clock = std::chrono::steady_clock;
        auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);

        while (clock::now() < deadline) {
            {
                std::lock_guard<std::mutex> lock(mtx_);
                if (tryParse(buf_, cmd_id, out))
                    return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return false;
    }

private:
    static bool tryParse(std::vector<uint8_t>& buf,
                         uint16_t              cmd_id,
                         ParsedResponse&       out)
    {
        while (buf.size() >= FRAME_HEADER_SIZE) {

            uint32_t length_val = read_u32_le(buf.data() + FRAME_LENGTH_OFFSET);
            size_t   total      = FRAME_LENGTH_FIELD + length_val;

            // Sanity-check the length to avoid spinning on corrupt data
            if (length_val < 4 || length_val > 4096) {
                buf.erase(buf.begin());  // skip one byte and retry
                continue;
            }

            // Full packet not arrived yet — wait for more BLE fragments
            if (buf.size() < total) return false;

            uint16_t found_type = read_u16_le(buf.data() + FRAME_TYPE_OFFSET);

            if (found_type == cmd_id) {
                out.raw.assign(buf.begin(), buf.begin() + total);
                buf.erase(buf.begin(), buf.begin() + total);
                return true;
            }

            // Different command type (e.g. unsolicited notification) — discard
            // this frame and keep scanning.
            buf.erase(buf.begin(), buf.begin() + total);
        }
        return false;
    }
};