#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <algorithm>
#include <cctype>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <csignal>
#include <atomic>
#include <stdexcept>
#include <cstring>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <simpleble/SimpleBLE.h>
#include "radiacode_lib.h"
#include "radiacode_buffer.h"

using namespace SimpleBLE;
using Clock     = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

// ── BLE UUIDs (RC-102 specific, sniffed — differ from RC-103 spec) ────────────
static const std::string SERVICE_UUID = "e63215e5-7003-49d8-96b0-b024798fb901";
static const std::string WRITE_UUID   = "e63215e6-7003-49d8-96b0-b024798fb901";
static const std::string NOTIFY_UUID  = "e63215e7-7003-49d8-96b0-b024798fb901";

// ── Globals ───────────────────────────────────────────────────────────────────
static std::atomic<bool> g_quit{false};

static void sig_handler(int) { g_quit = true; }

// Non-blocking single keypress check (returns 0 if nothing pending)
static int kbhit_char() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    int oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    int ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    return (ch == EOF) ? 0 : ch;
}

// ── Helpers ───────────────────────────────────────────────────────────────────
static std::string to_upper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c){ return std::toupper(c); });
    return s;
}

static void print_hex(const std::vector<uint8_t>& data) {
    for (auto b : data)
        std::cerr << std::hex << std::setw(2) << std::setfill('0') << (int)b << ' ';
    std::cerr << std::dec << '\n';
}

static std::string iso_now() {
    auto now   = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    char buf[20];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", std::localtime(&t));
    return buf;
}

// ── BLE helpers ───────────────────────────────────────────────────────────────
static bool sendAndWait(Peripheral&                 device,
                        RadiacodeBuffer&             rx,
                        const std::vector<uint8_t>& cmd,
                        uint16_t                    expected_type,
                        ParsedResponse&             out,
                        int                         timeout_ms = 3000)
{
    rx.clear();
    device.write_request(SERVICE_UUID, WRITE_UUID,
                         ByteArray(reinterpret_cast<const char*>(cmd.data()),
                                   cmd.size()));
    return rx.waitForResponse(expected_type, out, timeout_ms);
}

// When true, dumps full vsdata hex to stderr once to help locate unknown fields.
static bool g_vsdata_debug = false;

// Read one DATA_BUF snapshot; returns false on comms error.
static bool readDataBuf(Peripheral& device, RadiacodeBuffer& rx,
                        RadiaCodeProtocol& proto, DeviceMeasurements& out)
{
    uint8_t seq; ParsedResponse resp;
    if (!sendAndWait(device, rx, proto.buildReadDataBuf(seq),
                     static_cast<uint16_t>(Command::RD_VIRT_STRING), resp, 6000))
    {
        std::cerr << "\n[!] Timeout DATA_BUF. Buffer: ";
        print_hex(rx.getData());
        return false;
    }
    if (resp.returnCode() != 1) {
        std::cerr << "\n[!] DATA_BUF error: " << resp.returnCode() << '\n';
        return false;
    }

    if (g_vsdata_debug) {
        g_vsdata_debug = false;  // dump only once
        try {
            auto vsdata = resp.vsData();
            size_t dump = std::min(vsdata.size(), size_t(128));
            std::cerr << "\n[DBG] vsdata " << vsdata.size() << " bytes (first "
                      << dump << "):\n";
            for (size_t i = 0; i < dump; ++i) {
                std::cerr << std::hex << std::setw(2) << std::setfill('0')
                          << (int)vsdata[i] << ' ';
                if ((i+1) % 16 == 0) std::cerr << '\n';
            }
            std::cerr << std::dec << '\n';
        } catch (...) {}
    }

    auto records = parseDataBuf(resp);
    if (!records.empty())
        out = summariseDataBuf(records);
    return true;
}

// Read temperature once via VSFR batch.
static float readTemperature(Peripheral& device, RadiacodeBuffer& rx,
                             RadiaCodeProtocol& proto)
{
    uint8_t seq; ParsedResponse resp;
    if (!sendAndWait(device, rx,
                     proto.buildReadVSFRBatch({VSFR::TEMP_degC}, seq),
                     static_cast<uint16_t>(Command::RD_VIRT_SFR_BATCH), resp))
        return -999.0f;
    if (!(resp.payloadU32(0) & 0x01)) return -999.0f;
    return resp.payloadF32(4);
}

// ── Measurement accumulator ───────────────────────────────────────────────────
struct Accumulator {
    double sum_cps  = 0.0;
    double sum_dose = 0.0;  // µSv/h
    double sum_err  = 0.0;  // err_pct from device
    int    count    = 0;
    int    err_count= 0;

    void add(const DeviceMeasurements& m) {
        if (!m.valid) return;
        sum_cps  += m.count_rate;
        sum_dose += m.dose_rate_uSv_h;
        if (m.err_pct > 0.0f) { sum_err += m.err_pct; ++err_count; }
        ++count;
    }

    bool   empty()   const { return count == 0; }
    double avgCPS()  const { return count     ? sum_cps  / count      : 0.0; }
    double avgDose() const { return count     ? sum_dose / count      : 0.0; }
    double avgErr()  const { return err_count ? sum_err  / err_count  : 0.0; }
};

// ── Formatted measurement line ────────────────────────────────────────────────
// t=<ISO> HT=<µSv/h µR/h> CPS=<> CPM=<> Err=<±%> Temp=<°C> Bat=<%>
static std::string format_line(const Accumulator& acc, float temp_c,
                                float charge_pct, int /*interval_s*/)
{
    double cps       = acc.avgCPS();
    double dose_uSv  = acc.avgDose();
    double dose_uR   = dose_uSv * 115.0;
    double cpm       = cps * 60.0;
    // Use device-provided Poisson error from history buffer (averaged over interval)
    double err_pct   = acc.avgErr();

    std::ostringstream os;
    os << std::fixed;
    os << "t=" << iso_now() << " ";
    os << "HT=" << std::setprecision(4) << dose_uSv << "µSv/h "
                << std::setprecision(2) << dose_uR   << "µR/h ";
    os << "CPS=" << std::setprecision(3) << cps << " ";
    os << "CPM=" << std::setprecision(1) << cpm << " ";
    if (err_pct > 0.0)
        os << "Err=±" << std::setprecision(1) << err_pct << "% ";
    else
        os << "Err=N/A ";
    if (temp_c > -900.0f)
        os << "Temp=" << std::setprecision(1) << temp_c << "°C ";
    else
        os << "Temp=N/A ";
    if (charge_pct > 0.0f)
        os << "Bat=" << std::setprecision(0) << charge_pct << "%";
    else
        os << "Bat=N/A";
    return os.str();
}

// ── Collect samples over an interval window ───────────────────────────────────
// Polls DATA_BUF roughly every poll_ms milliseconds until interval_s seconds
// have elapsed or g_quit is set.
static Accumulator collect_interval(Peripheral& device, RadiacodeBuffer& rx,
                                     RadiaCodeProtocol& proto,
                                     int interval_s, int poll_ms = 500)
{
    Accumulator acc;
    auto deadline = Clock::now() + std::chrono::seconds(interval_s);

    while (Clock::now() < deadline && !g_quit) {
        DeviceMeasurements m;
        if (readDataBuf(device, rx, proto, m))
            acc.add(m);
        // Sleep in small chunks so we can react to g_quit promptly
        auto remaining = deadline - Clock::now();
        auto sleep_dur = std::chrono::milliseconds(poll_ms);
        if (remaining < sleep_dur) sleep_dur = std::chrono::duration_cast<std::chrono::milliseconds>(remaining);
        if (sleep_dur.count() > 0)
            std::this_thread::sleep_for(sleep_dur);
    }
    return acc;
}

// ── Argument parsing ──────────────────────────────────────────────────────────
struct AppConfig {
    std::string mac;
    bool        live       = false;
    int         runonce    = 1;    // number of measurements in runonce mode
    int         interval   = 3;   // averaging interval in seconds
    bool        mode_live  = false;
    bool        mode_once  = false;
    bool        debug_vsdata = false;
};

static void usage(const char* prog) {
    std::cerr
        << "Usage: " << prog << " [OPTIONS]\n"
        << "\n"
        << "Options:\n"
        << "  -m, --mac <MAC>       Device MAC address (prompt if omitted)\n"
        << "  -l, --live            Live mode: continuous display until q or Ctrl+C\n"
        << "  -r, --runonce <N>     Run-once mode: print N measurements and exit (default 1)\n"
        << "  -i, --interval <Y>    Averaging interval in seconds (default 3)\n"
        << "\n"
        << "Output format:\n"
        << "  t=<ISO8601> HT=<µSv/h µR/h> CPS=<> CPM=<> Err=±<%%> Temp=<°C> Bat=<%%>\n"
        << "\n"
        << "If no mode flag is given, behaves as -r 1.\n";
}

static AppConfig parse_args(int argc, char* argv[]) {
    AppConfig cfg;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "-m" || a == "--mac") {
            if (++i >= argc) throw std::runtime_error("--mac requires an argument");
            cfg.mac = argv[i];
        } else if (a == "-l" || a == "--live") {
            cfg.mode_live = true;
        } else if (a == "-r" || a == "--runonce") {
            cfg.mode_once = true;
            if (i + 1 < argc && argv[i+1][0] != '-') {
                cfg.runonce = std::stoi(argv[++i]);
                if (cfg.runonce < 1) throw std::runtime_error("--runonce must be >= 1");
            }
        } else if (a == "-i" || a == "--interval") {
            if (++i >= argc) throw std::runtime_error("--interval requires an argument");
            cfg.interval = std::stoi(argv[i]);
            if (cfg.interval < 1) throw std::runtime_error("--interval must be >= 1");
        } else if (a == "--debug-vsdata") {
            cfg.debug_vsdata = true;
        } else if (a == "-h" || a == "--help") {
            usage(argv[0]);
            exit(0);
        } else {
            throw std::runtime_error("Unknown option: " + a);
        }
    }
    // Default: behave like -r 1
    if (!cfg.mode_live && !cfg.mode_once) cfg.mode_once = true;
    // -l and -r are mutually exclusive
    if (cfg.mode_live && cfg.mode_once)
        throw std::runtime_error("-l and -r are mutually exclusive");
    return cfg;
}

// ── Device connect + init ─────────────────────────────────────────────────────
static bool connect_device(const std::string& mac, Peripheral& out_device,
                            RadiacodeBuffer& rx, RadiaCodeProtocol& proto)
{
    std::vector<Adapter> adapters = Adapter::get_adapters();
    if (adapters.empty()) {
        std::cerr << "[!] No Bluetooth adapters found.\n";
        return false;
    }

    std::cerr << "Scanning for " << mac << "...\n";
    auto& adapter = adapters[0];
    adapter.scan_for(3000);
    bool found = false;
    for (auto& p : adapter.scan_get_results()) {
        if (to_upper(p.address()) == to_upper(mac)) {
            out_device = p;
            found = true;
            break;
        }
    }
    if (!found) {
        std::cerr << "[!] Device " << mac << " not found.\n";
        return false;
    }

    out_device.connect();
    std::cerr << "[OK] Connected. Registering notifications...\n";

    out_device.notify(SERVICE_UUID, NOTIFY_UUID, [&](ByteArray data) {
        rx.addData(reinterpret_cast<const uint8_t*>(data.data()), data.size());
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Handshake
    uint8_t seq; ParsedResponse resp;
    if (!sendAndWait(out_device, rx, proto.buildSetExchange(seq),
                     static_cast<uint16_t>(Command::SET_EXCHANGE), resp)) {
        std::cerr << "[!] Handshake timeout.\n";
        return false;
    }

    // Reset measurement timer
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    sendAndWait(out_device, rx,
                proto.buildWriteVSFR(VSFR::DEVICE_TIME, uint32_t(0), seq),
                static_cast<uint16_t>(Command::WR_VIRT_SFR), resp);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Print device info to stderr so stdout stays clean for data
    if (sendAndWait(out_device, rx, proto.buildGetVersion(seq),
                    static_cast<uint16_t>(Command::GET_VERSION), resp)) {
        try { std::cerr << parseGetVersion(resp).toString() << '\n'; }
        catch (...) {}
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if (sendAndWait(out_device, rx, proto.buildReadSerialNumber(seq),
                    static_cast<uint16_t>(Command::RD_VIRT_STRING), resp, 4000)) {
        if (resp.returnCode() == 1)
            std::cerr << "S/N: " << resp.virtualString() << '\n';
    }

    std::cerr << "[OK] Ready. Waiting for first measurement cycle...\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));
    return true;
}

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    AppConfig cfg;
    try {
        cfg = parse_args(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "[!] " << e.what() << '\n';
        usage(argv[0]);
        return 1;
    }

    // MAC: from flag or prompt
    if (cfg.mac.empty()) {
        std::cerr << "Enter device MAC address: ";
        std::cin >> cfg.mac;
    }

    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    Peripheral        device;
    RadiacodeBuffer   rx;
    RadiaCodeProtocol proto;

    try {
        if (!connect_device(cfg.mac, device, rx, proto))
            return 1;
        g_vsdata_debug = cfg.debug_vsdata;

        // ── RUN-ONCE MODE ─────────────────────────────────────────────────
        if (cfg.mode_once) {
            int remaining = cfg.runonce;
            while (remaining > 0 && !g_quit) {
                // Read temperature once per measurement line
                float temp = readTemperature(device, rx, proto);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                Accumulator acc = collect_interval(device, rx, proto, cfg.interval);
                if (g_quit) break;

                float charge = 0.0f;
                {   // grab latest charge from one more DATA_BUF read
                    DeviceMeasurements m;
                    readDataBuf(device, rx, proto, m);
                    charge = m.charge_level;
                }

                if (!acc.empty())
                    std::cout << format_line(acc, temp, charge, cfg.interval) << '\n';
                else
                    std::cerr << "[!] No valid measurements in this interval.\n";

                --remaining;
            }
        }

        // ── LIVE MODE ─────────────────────────────────────────────────────
        else {
            std::cerr << "Live mode — press 'q' or Ctrl+C to quit.\n";

            // Temperature is slow to change — refresh every ~10 intervals
            float  temp       = readTemperature(device, rx, proto);
            float  charge     = 0.0f;
            int    temp_tick  = 0;
            bool   first_line = true;

            while (!g_quit) {
                // Check keyboard
                int ch = kbhit_char();
                if (ch == 'q' || ch == 'Q') break;

                // Refresh temperature periodically
                if (temp_tick == 0) {
                    float t = readTemperature(device, rx, proto);
                    if (t > -900.0f) temp = t;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                temp_tick = (temp_tick + 1) % 10;

                Accumulator acc = collect_interval(device, rx, proto, cfg.interval);
                if (g_quit) break;

                // Grab latest charge
                {
                    DeviceMeasurements m;
                    readDataBuf(device, rx, proto, m);
                    if (m.charge_level > 0.0f) charge = m.charge_level;
                }

                if (!acc.empty()) {
                    std::string line = format_line(acc, temp, charge, cfg.interval);
                    // Overwrite the current line in the terminal
                    std::cout << '\r' << line << std::flush;
                    first_line = false;
                }

                // Check quit key again immediately after output
                ch = kbhit_char();
                if (ch == 'q' || ch == 'Q') break;
            }
            if (!first_line) std::cout << '\n';  // leave cursor on new line
        }

    } catch (const std::exception& e) {
        std::cerr << "\n[!] Exception: " << e.what() << '\n';
    }

    if (device.is_connected()) device.disconnect();
    return 0;
}