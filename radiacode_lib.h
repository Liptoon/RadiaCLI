#pragma once
#include <cstdint>
#include <cstring>
#include <cmath>
#include <ctime>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <stdexcept>

// ---------------------------------------------------------------------------
// Frame format (RC-102, confirmed by capture — both directions identical):
//   [Length(4LE)] [Type(2LE)] [Spare(1=0x00)] [Seq(1)] [Payload...]
//   Length = bytes after the length field itself (= 4 + sizeof(Payload))
// ---------------------------------------------------------------------------

static constexpr size_t FRAME_LENGTH_OFFSET  = 0;
static constexpr size_t FRAME_TYPE_OFFSET    = 4;
static constexpr size_t FRAME_SPARE_OFFSET   = 6;
static constexpr size_t FRAME_SEQ_OFFSET     = 7;
static constexpr size_t FRAME_PAYLOAD_OFFSET = 8;
static constexpr size_t FRAME_HEADER_SIZE    = 8;
static constexpr size_t FRAME_LENGTH_FIELD   = 4;

#pragma pack(push, 1)
struct RequestHeader {
    uint32_t length;
    uint16_t type;
    uint8_t  spare;
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
    GET_SERIAL        = 0x000B,
    FW_IMAGE_GET_INFO = 0x0012,
    FW_SIGNATURE      = 0x0101,
    RD_FLASH          = 0x081C,
    RD_VIRT_SFR       = 0x0824,
    WR_VIRT_SFR       = 0x0825,
    RD_VIRT_STRING    = 0x0826,
    WR_VIRT_STRING    = 0x0827,
    RD_VIRT_SFR_BATCH = 0x082A,
    WR_VIRT_SFR_BATCH = 0x082B,
    SET_TIME          = 0x0A04
};

// ---------------------------------------------------------------------------
// Virtual SFR IDs — confirmed on RC-102 fw4.14
// ---------------------------------------------------------------------------
enum class VSFR : uint32_t {
    DEVICE_CTRL       = 0x0500,
    DEVICE_LANG       = 0x0502,
    DEVICE_ON         = 0x0503,
    DEVICE_TIME       = 0x0504,  // ms since power-on (NOT Unix time)
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
    TEMP_degC         = 0x8024,  // current temperature, f32 °C
    RAW_TEMP_degC     = 0x8033,
    TEMP_UP_degC      = 0x8034,
    TEMP_DN_degC      = 0x8035
};

// ---------------------------------------------------------------------------
// Virtual String IDs
// ---------------------------------------------------------------------------
enum class VS : uint32_t {
    CONFIGURATION = 0x02,
    SERIAL_NUMBER = 0x08,
    TEXT_MESSAGE  = 0x0F,
    DATA_BUF      = 0x100,
    SFR_FILE      = 0x101,
    SPECTRUM      = 0x200,
    ENERGY_CALIB  = 0x202,
    SPEC_ACCUM    = 0x205
};

// ---------------------------------------------------------------------------
// LE read helpers — memcpy-based to avoid UB on unaligned pointers
// ---------------------------------------------------------------------------
inline uint16_t read_u16_le(const uint8_t* p) { uint16_t v; std::memcpy(&v, p, 2); return v; }
inline uint32_t read_u32_le(const uint8_t* p) { uint32_t v; std::memcpy(&v, p, 4); return v; }
inline float    read_f32_le(const uint8_t* p) { float    v; std::memcpy(&v, p, 4); return v; }

// ---------------------------------------------------------------------------
// ParsedResponse — wraps one complete received frame
// ---------------------------------------------------------------------------
struct ParsedResponse {
    std::vector<uint8_t> raw;  // full frame bytes including length prefix

    bool     valid()    const { return raw.size() >= FRAME_HEADER_SIZE; }
    uint32_t length()   const { return read_u32_le(raw.data() + FRAME_LENGTH_OFFSET); }
    uint16_t type()     const { return read_u16_le(raw.data() + FRAME_TYPE_OFFSET); }
    uint8_t  spare()    const { return raw[FRAME_SPARE_OFFSET]; }
    uint8_t  sequence() const { return raw[FRAME_SEQ_OFFSET]; }

    const uint8_t* payload()     const { return raw.data() + FRAME_PAYLOAD_OFFSET; }
    size_t         payloadSize() const {
        return raw.size() > FRAME_PAYLOAD_OFFSET ? raw.size() - FRAME_PAYLOAD_OFFSET : 0;
    }

    uint32_t payloadU32(size_t off) const {
        if (off + 4 > payloadSize()) throw std::runtime_error("payload too small for u32");
        return read_u32_le(payload() + off);
    }
    float payloadF32(size_t off) const {
        if (off + 4 > payloadSize()) throw std::runtime_error("payload too small for f32");
        return read_f32_le(payload() + off);
    }

    // RD_VIRT_STRING payload layout: [RetCode(4)][DataLen(4)][Data...]
    uint32_t returnCode() const { return payloadU32(0); }

    std::vector<uint8_t> vsData() const {
        if (payloadSize() < 8) throw std::runtime_error("response too small for VS header");
        uint32_t data_len = payloadU32(4);
        if (payloadSize() < 8 + data_len) throw std::runtime_error("VS data truncated");
        const uint8_t* start = payload() + 8;
        return std::vector<uint8_t>(start, start + data_len);
    }

    std::string virtualString() const {
        auto d = vsData();
        return std::string(reinterpret_cast<const char*>(d.data()), d.size());
    }
};

// ---------------------------------------------------------------------------
// GET_VERSION response parser
// ---------------------------------------------------------------------------
struct FirmwareVersion {
    uint16_t    boot_major = 0, boot_minor = 0;
    std::string boot_date;
    uint16_t    target_major = 0, target_minor = 0;
    std::string target_date;

    std::string toString() const {
        std::ostringstream os;
        os << "Boot:   " << boot_major << '.' << std::setw(2) << std::setfill('0') << boot_minor
           << "  (" << boot_date << ")\n"
           << "Target: " << target_major << '.' << std::setw(2) << std::setfill('0') << target_minor
           << "  (" << target_date << ')';
        return os.str();
    }
};

inline FirmwareVersion parseGetVersion(const ParsedResponse& resp) {
    const uint8_t* p = resp.payload();
    size_t rem = resp.payloadSize();
    auto need = [&](size_t n) {
        if (rem < n) throw std::runtime_error("GET_VERSION response truncated");
    };
    FirmwareVersion v;
    need(4);
    v.boot_minor = read_u16_le(p); p+=2; rem-=2;
    v.boot_major = read_u16_le(p); p+=2; rem-=2;
    need(1); uint8_t bdl = *p++; rem--;
    need(bdl); v.boot_date = std::string(reinterpret_cast<const char*>(p), bdl); p+=bdl; rem-=bdl;
    need(4);
    v.target_minor = read_u16_le(p); p+=2; rem-=2;
    v.target_major = read_u16_le(p); p+=2; rem-=2;
    need(1); uint8_t tdl = *p++; rem--;
    need(tdl);
    size_t tl = tdl;
    if (tl > 0 && p[tl-1] == '\0') --tl;
    v.target_date = std::string(reinterpret_cast<const char*>(p), tl);
    return v;
}

// ---------------------------------------------------------------------------
// DATA_BUF (VS 0x100) — tagged variable-length record stream
//
// Confirmed format (RC-102 fw4.14, 2026-03-26):
//   vsData byte 0 is the start of the first record — no buffer-level header.
//
// Each record: [seq(u8)][eid(u8)][gid(u8)][ts_offset(i32LE)] + body
//   total header = 7 bytes; body size depends on gid.
//
// GID=0  RealTimeData  body=15B  total=22B
//   count_rate(f32)        CPS; always CPS regardless of CR_UNITS VSFR
//   dose_rate(f32)         multiply by 10000 for µSv/h
//   count_rate_err(u16)    divide by 10 for %; displayed as ±% on device screen
//   dose_rate_err(u16)     divide by 10 for %
//   flags(u16), rt_flags(u8)
//
// GID=1  RawData        body=8B   total=15B
//   count_rate(f32), dose_rate(f32)
//
// GID=2  DoseRateDB     body=16B  total=23B
//   count(u32), count_rate(f32), dose_rate(f32), dose_rate_err(u16), flags(u16)
//
// GID=3  RareData       body=14B  total=21B  — ~every 30 seconds
//   duration(u32), dose(f32)
//   temperature(u16)   decode: (raw - 2000) / 100 = °C
//   charge_level(u16)  decode: raw / 100 = %
//   flags(u16)
//
// GID=7  Event          body=4B   total=11B
//   event(u8), event_param1(u8), flags(u16)
//
// EID=1, GID=1/2/3: variable-length bulk sample records; skipped via sample count.
//
// Wall-clock time is NOT in DATA_BUF and not available via any VSFR.
// ---------------------------------------------------------------------------

struct DeviceMeasurements {
    bool  valid           = false;
    float count_rate      = 0.0f;  // CPS
    float dose_rate_uSv_h = 0.0f;  // µSv/h
    float err_pct         = 0.0f;  // count_rate_err % (from GID=0, count_rate_err÷10)
    float dose_err_pct    = 0.0f;  // dose_rate_err % (from GID=0, dose_rate_err÷10)
    float temperature     = 0.0f;  // °C (from GID=3, ~every 30s)
    float charge_level    = 0.0f;  // battery % (from GID=3, ~every 30s)
};

inline DeviceMeasurements parseDataBuf(const ParsedResponse& resp) {
    DeviceMeasurements m;
    auto data = resp.vsData();
    const uint8_t* p = data.data();
    size_t rem = data.size();
    uint8_t next_seq = 0xFF;  // 0xFF = accept first record unconditionally

    while (rem >= 7) {
        uint8_t seq = p[0], eid = p[1], gid = p[2];

        if (next_seq != 0xFF && seq != next_seq) break;  // sequence gap → stop
        next_seq = (seq + 1) & 0xFF;

        const uint8_t* body = p + 7;
        size_t body_rem = rem - 7;

        if (eid == 0 && gid == 0) {                 // GID=0: RealTimeData (22B total)
            if (body_rem < 15) break;
            float cr = read_f32_le(body);
            float dr = read_f32_le(body + 4);
            uint16_t cr_err = read_u16_le(body + 8);  // count_rate_err ÷10 = %
            uint16_t dr_err = read_u16_le(body + 10); // dose_rate_err ÷10 = %
            if (!std::isnan(cr) && !std::isinf(cr) && cr >= 0.0f &&
                !std::isnan(dr) && !std::isinf(dr) && dr >= 0.0f) {
                m.count_rate      = cr;
                m.dose_rate_uSv_h = dr * 10000.0f;
                m.err_pct         = static_cast<float>(cr_err) / 10.0f;
                m.dose_err_pct    = static_cast<float>(dr_err) / 10.0f;
                m.valid           = true;
            }
            p += 22; rem -= 22;

        } else if (eid == 0 && gid == 1) {          // GID=1: RawData (15B total)
            if (body_rem < 8) break;
            p += 15; rem -= 15;

        } else if (eid == 0 && gid == 2) {          // GID=2: DoseRateDB (23B total)
            if (body_rem < 16) break;
            p += 23; rem -= 23;

        } else if (eid == 0 && gid == 3) {          // GID=3: RareData (21B total) — temp + battery
            if (body_rem < 14) break;
            uint16_t raw_temp   = read_u16_le(body + 8);
            uint16_t raw_charge = read_u16_le(body + 10);
            float temp   = (static_cast<float>(raw_temp)   - 2000.0f) / 100.0f;
            float charge =  static_cast<float>(raw_charge) / 100.0f;
            if (temp   >= -20.0f && temp   <= 85.0f)  m.temperature  = temp;
            if (charge >=   0.0f && charge <= 100.0f) m.charge_level = charge;
            p += 21; rem -= 21;

        } else if (eid == 0 && gid == 4) {          // GID=4: UserData (23B total, skipped)
            if (body_rem < 16) break; p += 23; rem -= 23;
        } else if (eid == 0 && gid == 5) {          // GID=5: ScheduleData (23B total, skipped)
            if (body_rem < 16) break; p += 23; rem -= 23;
        } else if (eid == 0 && gid == 6) {          // GID=6: AccelData (13B total, skipped)
            if (body_rem < 6)  break; p += 13; rem -= 13;
        } else if (eid == 0 && gid == 7) {          // GID=7: Event (11B total, skipped)
            if (body_rem < 4)  break; p += 11; rem -= 11;
        } else if (eid == 0 && gid == 8) {          // GID=8: RawCountRate (13B total, skipped)
            if (body_rem < 6)  break; p += 13; rem -= 13;
        } else if (eid == 0 && gid == 9) {          // GID=9: RawDoseRate (13B total, skipped)
            if (body_rem < 6)  break; p += 13; rem -= 13;
        } else if (eid == 1 && (gid == 1 || gid == 2 || gid == 3)) { // EID=1: bulk samples
            if (body_rem < 6) break;
            uint16_t n = read_u16_le(body);
            size_t bytes_per = (gid == 1) ? 8 : (gid == 2) ? 16 : 14;
            size_t skip = 7 + 6 + n * bytes_per;
            if (rem < skip) break;
            p += skip; rem -= skip;
        } else {
            break;  // unknown record type
        }
    }
    return m;
}

// ---------------------------------------------------------------------------
// Protocol command builder
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
    std::vector<uint8_t> buildSimpleCommand(Command cmd, uint8_t& seq_out) {
        return makeRequest(cmd, 0, seq_out);
    }

    // Mandatory session handshake — must be the first command sent.
    std::vector<uint8_t> buildSetExchange(uint8_t& seq_out) {
        auto buf = makeRequest(Command::SET_EXCHANGE, 4, seq_out);
        uint8_t* p = buf.data() + sizeof(RequestHeader);
        p[0]=0x01; p[1]=0xFF; p[2]=0x12; p[3]=0xFF;
        return buf;
    }

    std::vector<uint8_t> buildGetStatus(uint8_t& seq_out) {
        return makeRequest(Command::GET_STATUS, 0, seq_out);
    }

    std::vector<uint8_t> buildGetVersion(uint8_t& seq_out) {
        return makeRequest(Command::GET_VERSION, 0, seq_out);
    }

    std::vector<uint8_t> buildGetHwSerial(uint8_t& seq_out) {
        return makeRequest(Command::GET_SERIAL, 0, seq_out);
    }

    std::vector<uint8_t> buildReadVirtualString(VS vs_id, uint8_t& seq_out) {
        auto buf = makeRequest(Command::RD_VIRT_STRING, 4, seq_out);
        uint32_t id = static_cast<uint32_t>(vs_id);
        std::memcpy(buf.data() + sizeof(RequestHeader), &id, 4);
        return buf;
    }

    std::vector<uint8_t> buildReadSerialNumber(uint8_t& seq_out) {
        return buildReadVirtualString(VS::SERIAL_NUMBER, seq_out);
    }

    std::vector<uint8_t> buildReadDataBuf(uint8_t& seq_out) {
        return buildReadVirtualString(VS::DATA_BUF, seq_out);
    }

    // Sync device RTC to current PC Unix time (4-byte LE).
    std::vector<uint8_t> buildSetTime(uint8_t& seq_out) {
        auto buf = makeRequest(Command::SET_TIME, 4, seq_out);
        uint32_t now = static_cast<uint32_t>(std::time(nullptr));
        std::memcpy(buf.data() + sizeof(RequestHeader), &now, 4);
        return buf;
    }

    // BLE MTU=20B → max payload=12B → max n=2 VSFRs. Use n=1 for safety.
    std::vector<uint8_t> buildReadVSFRBatch(const std::vector<VSFR>& ids, uint8_t& seq_out) {
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

    std::vector<uint8_t> buildWriteVSFR(VSFR vsfr_id, uint32_t value, uint8_t& seq_out) {
        auto buf = makeRequest(Command::WR_VIRT_SFR, 8, seq_out);
        uint8_t* p = buf.data() + sizeof(RequestHeader);
        uint32_t id = static_cast<uint32_t>(vsfr_id);
        std::memcpy(p,     &id,    4);
        std::memcpy(p + 4, &value, 4);
        return buf;
    }

    std::vector<uint8_t> buildWriteVSFR(VSFR vsfr_id, float value, uint8_t& seq_out) {
        uint32_t raw; std::memcpy(&raw, &value, 4);
        return buildWriteVSFR(vsfr_id, raw, seq_out);
    }
};