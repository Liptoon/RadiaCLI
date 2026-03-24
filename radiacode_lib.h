#pragma once
#include <cstdint>
#include <cstring>
#include <vector>
#include <string>
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include <cmath>

// ---------------------------------------------------------------------------
// Wire layout  (RC-102 observed behaviour — differs from spec PDF)
//
// Both REQUESTS and RESPONSES share the same frame structure:
//   [Length(4LE)] [Type(2LE)] [Spare(1=0x00)] [Seq(1)] [Payload...]
//
//   Length = total bytes AFTER the 4-byte length field itself
//          = 4 (type+spare+seq) + sizeof(payload)
//          = buffer.size() - 4
//
// Example SET_EXCHANGE response captured from RC-102:
//   10 00 00 00  <- length = 16
//   07 00        <- type = 0x0007
//   00 80        <- spare, seq
//   05 01 da da 20 10 00 10 01 02 00 00  <- payload (12 B)
//   total on wire = 4 + 16 = 20 bytes
// ---------------------------------------------------------------------------

// Frame field offsets (same for both requests and responses)
static constexpr size_t FRAME_LENGTH_OFFSET  = 0;  // uint32_t LE
static constexpr size_t FRAME_TYPE_OFFSET    = 4;  // uint16_t LE
static constexpr size_t FRAME_SPARE_OFFSET   = 6;  // uint8_t
static constexpr size_t FRAME_SEQ_OFFSET     = 7;  // uint8_t
static constexpr size_t FRAME_PAYLOAD_OFFSET = 8;  // payload starts here
static constexpr size_t FRAME_HEADER_SIZE    = 8;  // length(4)+type(2)+spare(1)+seq(1)
static constexpr size_t FRAME_LENGTH_FIELD   = 4;  // size of the length field itself

#pragma pack(push, 1)
struct RequestHeader {
    uint32_t length;    // bytes after this field (type+spare+seq+payload)
    uint16_t type;
    uint8_t  spare;     // always 0x00
    uint8_t  sequence;
};
#pragma pack(pop)

static_assert(sizeof(RequestHeader) == 8, "RequestHeader size mismatch");

// ---------------------------------------------------------------------------
// Command IDs
// ---------------------------------------------------------------------------
enum class Command : uint16_t {
    GET_STATUS        = 0x0005,
    SET_EXCHANGE      = 0x0007,
    GET_VERSION       = 0x000A,
    GET_SERIAL        = 0x000B,  // hardware serial (binary groups)
    FW_IMAGE_GET_INFO = 0x0012,
    FW_SIGNATURE      = 0x0101,
    RD_FLASH          = 0x081C,
    RD_VIRT_SFR       = 0x0824,  // single-register read — verify on RC-102
    WR_VIRT_SFR       = 0x0825,
    RD_VIRT_STRING    = 0x0826,
    WR_VIRT_STRING    = 0x0827,
    RD_VIRT_SFR_BATCH = 0x082A,
    WR_VIRT_SFR_BATCH = 0x082B,
    SET_TIME          = 0x0A04
};

// ---------------------------------------------------------------------------
// Virtual Special Function Register IDs
// ---------------------------------------------------------------------------
enum class VSFR : uint32_t {
    DEVICE_CTRL       = 0x0500,
    DEVICE_LANG       = 0x0502,
    DEVICE_ON         = 0x0503,
    DEVICE_TIME       = 0x0504,
    DISP_CTRL         = 0x0510,
    DISP_BRT          = 0x0511,
    DISP_OFF_TIME     = 0x0513,
    DISP_DIR          = 0x0515,
    SOUND_CTRL        = 0x0520,
    SOUND_ON          = 0x0522,
    VIBRO_CTRL        = 0x0530,
    VIBRO_ON          = 0x0531,
    ALARM_MODE        = 0x05E0,
    DR_LEV1_uR_h      = 0x8000,
    DR_LEV2_uR_h      = 0x8001,
    DS_UNITS          = 0x8004,
    DOSE_RESET        = 0x8007,
    CR_LEV1_cp10s     = 0x8008,
    CR_LEV2_cp10s     = 0x8009,
    CHN_TO_keV_A0     = 0x8010,
    CHN_TO_keV_A1     = 0x8011,
    CHN_TO_keV_A2     = 0x8012,
    CR_UNITS          = 0x8013,
    DS_LEV1_uR        = 0x8014,
    DS_LEV2_uR        = 0x8015,
    TEMP_UNITS        = 0x8016,
    TEMP_degC         = 0x8024,
    RAW_TEMP_degC     = 0x8033,
    TEMP_UP_degC      = 0x8034,
    TEMP_DN_degC      = 0x8035
};

// ---------------------------------------------------------------------------
// Virtual String IDs
// ---------------------------------------------------------------------------
enum class VS : uint32_t {
    CONFIGURATION     = 0x02,
    SERIAL_NUMBER     = 0x08,
    TEXT_MESSAGE      = 0x0F,
    DATA_BUF          = 0x100,
    SFR_FILE          = 0x101,
    SPECTRUM          = 0x200,
    ENERGY_CALIB      = 0x202,
    SPEC_ACCUM        = 0x205
};

// ---------------------------------------------------------------------------
// Safe little-endian read helpers — avoids unaligned reinterpret_cast UB
// ---------------------------------------------------------------------------
inline uint16_t read_u16_le(const uint8_t* p) {
    uint16_t v; std::memcpy(&v, p, 2); return v;
}
inline uint32_t read_u32_le(const uint8_t* p) {
    uint32_t v; std::memcpy(&v, p, 4); return v;
}
inline float read_f32_le(const uint8_t* p) {
    float v; std::memcpy(&v, p, 4); return v;
}

// ---------------------------------------------------------------------------
// ParsedResponse
//
// Wraps a received frame (including the 4-byte length prefix).
//
// Frame layout in raw[]:
//   [0..3]  length (uint32 LE)
//   [4..5]  type   (uint16 LE)
//   [6]     spare
//   [7]     seq
//   [8...]  payload
// ---------------------------------------------------------------------------
struct ParsedResponse {
    std::vector<uint8_t> raw;

    bool valid() const { return raw.size() >= FRAME_HEADER_SIZE; }

    uint32_t length()   const { return read_u32_le(raw.data() + FRAME_LENGTH_OFFSET); }
    uint16_t type()     const { return read_u16_le(raw.data() + FRAME_TYPE_OFFSET); }
    uint8_t  spare()    const { return raw[FRAME_SPARE_OFFSET]; }
    uint8_t  sequence() const { return raw[FRAME_SEQ_OFFSET]; }

    const uint8_t* payload()     const { return raw.data() + FRAME_PAYLOAD_OFFSET; }
    size_t         payloadSize() const {
        return raw.size() > FRAME_PAYLOAD_OFFSET
               ? raw.size() - FRAME_PAYLOAD_OFFSET : 0;
    }

    uint32_t payloadU32(size_t offset) const {
        if (offset + 4 > payloadSize())
            throw std::runtime_error("payload too small for u32");
        return read_u32_le(payload() + offset);
    }
    float payloadF32(size_t offset) const {
        if (offset + 4 > payloadSize())
            throw std::runtime_error("payload too small for f32");
        return read_f32_le(payload() + offset);
    }

    // RD_VIRT_STRING helpers: payload = [RetCode(4)][StrLen(4)][Data...]
    uint32_t returnCode() const { return payloadU32(0); }

    // Returns the raw VS data bytes (after RetCode + StrLen)
    std::vector<uint8_t> vsData() const {
        if (payloadSize() < 8)
            throw std::runtime_error("response too small for VS header");
        uint32_t data_len = payloadU32(4);
        if (payloadSize() < 8 + data_len)
            throw std::runtime_error("VS data truncated");
        const uint8_t* start = payload() + 8;
        return std::vector<uint8_t>(start, start + data_len);
    }

    // Convenience for text VS (SERIAL_NUMBER, TEXT_MESSAGE, etc.)
    std::string virtualString() const {
        auto d = vsData();
        return std::string(reinterpret_cast<const char*>(d.data()), d.size());
    }
};

// ---------------------------------------------------------------------------
// GET_VERSION parsed result
//
// Response payload layout (no length envelope — uses embedded length bytes):
//   [BootMinor(2LE)][BootMajor(2LE)]
//   [BootDateLen(1)][BootDateStr(BootDateLen, NOT null-terminated)]
//   [TargetMinor(2LE)][TargetMajor(2LE)]
//   [TargetDateLen(1)][TargetDateStr(TargetDateLen, null-terminated)]
// ---------------------------------------------------------------------------
struct FirmwareVersion {
    uint16_t    boot_major   = 0;
    uint16_t    boot_minor   = 0;
    std::string boot_date;
    uint16_t    target_major = 0;
    uint16_t    target_minor = 0;
    std::string target_date;

    std::string toString() const {
        std::ostringstream os;
        os << "Boot:   " << boot_major   << '.' << std::setw(2) << std::setfill('0') << boot_minor
           << "  (" << boot_date   << ")\n"
           << "Target: " << target_major << '.' << std::setw(2) << std::setfill('0') << target_minor
           << "  (" << target_date << ')';
        return os.str();
    }
};

inline FirmwareVersion parseGetVersion(const ParsedResponse& resp) {
    const uint8_t* p   = resp.payload();
    size_t         rem = resp.payloadSize();

    auto need = [&](size_t n) {
        if (rem < n) throw std::runtime_error("GET_VERSION response truncated");
    };

    FirmwareVersion v;

    need(4);
    v.boot_minor = read_u16_le(p);     p += 2; rem -= 2;
    v.boot_major = read_u16_le(p);     p += 2; rem -= 2;

    need(1);
    uint8_t boot_date_len = *p++;      rem--;
    need(boot_date_len);
    v.boot_date = std::string(reinterpret_cast<const char*>(p), boot_date_len);
    p += boot_date_len; rem -= boot_date_len;

    need(4);
    v.target_minor = read_u16_le(p);   p += 2; rem -= 2;
    v.target_major = read_u16_le(p);   p += 2; rem -= 2;

    need(1);
    uint8_t target_date_len = *p++;    rem--;
    need(target_date_len);
    // Target date string may be null-terminated — strip the null if present
    size_t tlen = target_date_len;
    if (tlen > 0 && p[tlen - 1] == '\0') --tlen;
    v.target_date = std::string(reinterpret_cast<const char*>(p), tlen);

    return v;
}

// ---------------------------------------------------------------------------
// DATA_BUF (VS 0x100) parsed records
//
// The VS data is a stream of variable-length records:
//   [RecordLength(4LE)] [SeqNum(1)] [EID(1)] [GID(1)]
//   [TimestampOffset(4LE)] [RecordData...]
//
// RecordLength = bytes AFTER the 4-byte length field itself.
//
// Record types by GID (EID is always 0x00 for measurement data):
//   GID 0x00 — real-time:  [count_rate(f32)][dose_rate(f32)]
//   GID 0x01 — raw:        [count_rate(f32)][dose_rate(f32)]
//   GID 0x02 — dose rate:  (format TBD by firmware)
//   GID 0x03 — rare:       [temperature(f32)][charge_level(f32)]
//
// dose_rate (raw) × 10000 = µSv/h
// ---------------------------------------------------------------------------
enum class DataRecordGID : uint8_t {
    REAL_TIME  = 0x00,
    RAW        = 0x01,
    DOSE_RATE  = 0x02,
    RARE       = 0x03
};

struct DataRecord {
    uint8_t  seq              = 0;
    uint8_t  eid              = 0;
    uint8_t  gid              = 0;
    uint32_t timestamp_offset = 0;

    // GID 0x00 / 0x01
    float count_rate          = 0.0f;  // CPS or CPM depending on CR_UNITS VSFR
    float dose_rate_raw       = 0.0f;  // multiply by 10000 for µSv/h
    float count_rate_err      = 0.0f;  // Poisson uncertainty % (from history buffer)

    // GID 0x03
    float temperature         = 0.0f;  // °C
    float charge_level        = 0.0f;  // %

    float doseRateUSvH() const { return dose_rate_raw * 10000.0f; }
};

// Aggregate of the most-recent values across all record types in one DATA_BUF read
struct DeviceMeasurements {
    bool  valid            = false;
    float count_rate       = 0.0f;  // CPS or CPM
    float dose_rate_uSv_h  = 0.0f;  // µSv/h
    float temperature      = 0.0f;  // °C
    float charge_level     = 0.0f;  // %
    float err_pct          = 0.0f;  // statistical uncertainty % (Poisson from history)
};

// ---------------------------------------------------------------------------
// parseDataBuf — empirically determined layout for RC-102 firmware 4.14
//
// vsdata layout:
//   [0]       u8:  write_counter (increments each cycle)
//   [1..2]    u16: always 0x0000
//   [3..6]    u32: last_timestamp (firmware tick units)
//   [7..10]   f32: count_rate (CPS; CPM if CR_UNITS VSFR = 1)
//   [11..14]  f32: dose_rate_raw (×10000 = µSv/h)
//   [15..31]  mixed: padding / metadata (partially decoded)
//   [31]      u8:  charge_level (battery %, 0-100)
//   [32..34]  padding (0x00)
//   [35..]    history: N × 8-byte records [cps_1s(f32)][dose_raw_1s(f32)]
//             each record = one ~1-second measurement sample
//
// Error rate is Poisson-derived from the history buffer:
//   err_pct = 100 / sqrt(total_counts)   where total_counts = sum of all cps_1s values
//
// Temperature is not in DATA_BUF for this firmware — use VSFR::TEMP_degC instead.
// ---------------------------------------------------------------------------
static constexpr size_t VSDATA_HISTORY_OFFSET = 35;
static constexpr size_t VSDATA_HISTORY_RECORD  = 8;   // [cps(4)][dose_raw(4)]

inline std::vector<DataRecord> parseDataBuf(const ParsedResponse& resp) {
    std::vector<DataRecord> records;

    auto data = resp.vsData();
    if (data.size() < 15) return records;

    DataRecord r;
    r.gid           = static_cast<uint8_t>(DataRecordGID::REAL_TIME);
    r.count_rate    = read_f32_le(data.data() + 7);
    r.dose_rate_raw = read_f32_le(data.data() + 11);

    if (data.size() > 31)
        r.charge_level = static_cast<float>(data[31]);

    // Compute Poisson error from 1-second history samples at offset 35
    if (data.size() >= VSDATA_HISTORY_OFFSET + VSDATA_HISTORY_RECORD) {
        double total_counts = 0.0;
        size_t n_hist = 0;
        for (size_t off = VSDATA_HISTORY_OFFSET;
             off + VSDATA_HISTORY_RECORD <= data.size();
             off += VSDATA_HISTORY_RECORD)
        {
            float cps_sample = read_f32_le(data.data() + off);
            if (!std::isnan(cps_sample) && !std::isinf(cps_sample) && cps_sample >= 0.0f) {
                total_counts += cps_sample;
                ++n_hist;
            }
        }
        if (total_counts > 0.0)
            r.count_rate_err = static_cast<float>(100.0 / std::sqrt(total_counts));
    }

    if (!std::isnan(r.count_rate) && !std::isinf(r.count_rate) &&
        !std::isnan(r.dose_rate_raw) && !std::isinf(r.dose_rate_raw) &&
        r.count_rate >= 0.0f && r.dose_rate_raw >= 0.0f)
    {
        records.push_back(r);
    }

    return records;
}

// Collapse the parsed record(s) into a DeviceMeasurements snapshot.
inline DeviceMeasurements summariseDataBuf(const std::vector<DataRecord>& records) {
    DeviceMeasurements m;
    for (const auto& r : records) {
        m.count_rate      = r.count_rate;
        m.dose_rate_uSv_h = r.doseRateUSvH();
        m.charge_level    = r.charge_level;
        m.err_pct         = r.count_rate_err;
        m.valid           = true;
    }
    return m;
}

// ---------------------------------------------------------------------------
// Protocol builder
// ---------------------------------------------------------------------------
class RadiaCodeProtocol {
private:
    uint8_t sequence_counter = 0;

    uint8_t nextSeq() {
        uint8_t seq = 0x80 + (sequence_counter & 0x1F);
        sequence_counter = (sequence_counter + 1) & 0x1F;
        return seq;
    }

    std::vector<uint8_t> makeRequest(Command cmd, size_t payloadBytes, uint8_t& seq_out) {
        size_t total = sizeof(RequestHeader) + payloadBytes;
        std::vector<uint8_t> buf(total, 0x00);
        auto* hdr     = reinterpret_cast<RequestHeader*>(buf.data());
        hdr->length   = static_cast<uint32_t>(total - 4);
        hdr->type     = static_cast<uint16_t>(cmd);
        hdr->spare    = 0x00;
        hdr->sequence = nextSeq();
        seq_out       = hdr->sequence;
        return buf;
    }

public:
    // SET_EXCHANGE (0x0007)
    std::vector<uint8_t> buildSetExchange(uint8_t& seq_out) {
        auto buf = makeRequest(Command::SET_EXCHANGE, 4, seq_out);
        uint8_t* p = buf.data() + sizeof(RequestHeader);
        p[0] = 0x01; p[1] = 0xFF; p[2] = 0x12; p[3] = 0xFF;
        return buf;
    }

    // GET_STATUS (0x0005) — response payload: [StatusFlags(4)]
    std::vector<uint8_t> buildGetStatus(uint8_t& seq_out) {
        return makeRequest(Command::GET_STATUS, 0, seq_out);
    }

    // GET_VERSION (0x000A) — parse with parseGetVersion()
    std::vector<uint8_t> buildGetVersion(uint8_t& seq_out) {
        return makeRequest(Command::GET_VERSION, 0, seq_out);
    }

    // GET_SERIAL (0x000B) — hardware serial as hyphenated hex groups
    std::vector<uint8_t> buildGetHwSerial(uint8_t& seq_out) {
        return makeRequest(Command::GET_SERIAL, 0, seq_out);
    }

    // RD_VIRT_STRING (0x0826) — generic
    std::vector<uint8_t> buildReadVirtualString(VS vs_id, uint8_t& seq_out) {
        auto buf = makeRequest(Command::RD_VIRT_STRING, 4, seq_out);
        uint32_t id = static_cast<uint32_t>(vs_id);
        std::memcpy(buf.data() + sizeof(RequestHeader), &id, 4);
        return buf;
    }

    // VS::SERIAL_NUMBER (0x08) — user-friendly serial e.g. "RC-102-012345"
    std::vector<uint8_t> buildReadSerialNumber(uint8_t& seq_out) {
        return buildReadVirtualString(VS::SERIAL_NUMBER, seq_out);
    }

    // VS::DATA_BUF (0x100) — real-time measurements; parse with parseDataBuf()
    std::vector<uint8_t> buildReadDataBuf(uint8_t& seq_out) {
        return buildReadVirtualString(VS::DATA_BUF, seq_out);
    }

    // RD_VIRT_SFR_BATCH (0x082A) — response: [ValidFlags(4)][Value0(4)]...
    std::vector<uint8_t> buildReadVSFRBatch(const std::vector<VSFR>& ids,
                                             uint8_t& seq_out) {
        size_t n = ids.size();
        auto buf = makeRequest(Command::RD_VIRT_SFR_BATCH, 4 + n * 4, seq_out);
        uint8_t* p = buf.data() + sizeof(RequestHeader);
        uint32_t count = static_cast<uint32_t>(n);
        std::memcpy(p, &count, 4);
        for (size_t i = 0; i < n; ++i) {
            uint32_t id = static_cast<uint32_t>(ids[i]);
            std::memcpy(p + 4 + i * 4, &id, 4);
        }
        return buf;
    }

    // WR_VIRT_SFR (0x0825) — response: [RetCode(4)]
    std::vector<uint8_t> buildWriteVSFR(VSFR vsfr_id, uint32_t value,
                                         uint8_t& seq_out) {
        auto buf = makeRequest(Command::WR_VIRT_SFR, 8, seq_out);
        uint8_t* p = buf.data() + sizeof(RequestHeader);
        uint32_t id = static_cast<uint32_t>(vsfr_id);
        std::memcpy(p,     &id,    4);
        std::memcpy(p + 4, &value, 4);
        return buf;
    }

    std::vector<uint8_t> buildWriteVSFR(VSFR vsfr_id, float value,
                                         uint8_t& seq_out) {
        uint32_t raw; std::memcpy(&raw, &value, 4);
        return buildWriteVSFR(vsfr_id, raw, seq_out);
    }
};