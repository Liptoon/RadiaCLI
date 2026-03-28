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
#include <ctime>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <simpleble/SimpleBLE.h>
#include "radiacode_lib.h"
#include "radiacode_buffer.h"

using namespace SimpleBLE;
using Clock = std::chrono::steady_clock;

// RC-102 BLE service and characteristic UUIDs (sniffed; differ from RC-103 spec)
static const std::string SERVICE_UUID = "e63215e5-7003-49d8-96b0-b024798fb901";
static const std::string WRITE_UUID   = "e63215e6-7003-49d8-96b0-b024798fb901";
static const std::string NOTIFY_UUID  = "e63215e7-7003-49d8-96b0-b024798fb901";

static std::atomic<bool> g_quit{false};
static bool g_quiet_mode = false;
static void sig_handler(int) { g_quit = true; }

// Non-blocking keypress — returns 0 if nothing pending.
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

static std::string to_upper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::toupper(c); });
    return s;
}

static void print_hex(const std::vector<uint8_t>& data) {
    for (auto b : data)
        std::cerr << std::hex << std::setw(2) << std::setfill('0') << (int)b << ' ';
    std::cerr << std::dec << '\n';
}

// PC system clock as HH:MM:SS.
// Wall-clock time is not accessible via any RC-102 BLE interface;
// device RTC matches PC time in practice.
static std::string pc_time_str() {
    std::time_t t = std::time(nullptr);
    char buf[10];
    std::strftime(buf, sizeof(buf), "%H:%M:%S", std::localtime(&t));
    return buf;
}

// ── BLE helpers ───────────────────────────────────────────────────────────────

static bool sendAndWait(Peripheral& device, RadiacodeBuffer& rx,
                        const std::vector<uint8_t>& cmd, uint16_t expected,
                        ParsedResponse& out, int timeout_ms = 3000) {
    rx.clear();
    device.write_request(SERVICE_UUID, WRITE_UUID,
                         ByteArray(reinterpret_cast<const char*>(cmd.data()), cmd.size()));
    return rx.waitForResponse(expected, out, timeout_ms);
}

// When true, dumps the first 128 vsdata bytes to stderr once then resets.
static bool g_vsdata_debug = false;

static bool readDataBuf(Peripheral& device, RadiacodeBuffer& rx,
                        RadiaCodeProtocol& proto, DeviceMeasurements& out) {
    uint8_t seq; ParsedResponse resp;
    if (!sendAndWait(device, rx, proto.buildReadDataBuf(seq),
                     static_cast<uint16_t>(Command::RD_VIRT_STRING), resp, 6000)) {
        std::cerr << "\n[!] Timeout DATA_BUF. Buffer: "; print_hex(rx.getData());
        return false;
    }
    if (resp.returnCode() != 1) {
        std::cerr << "\n[!] DATA_BUF error: " << resp.returnCode() << '\n';
        return false;
    }
    if (g_vsdata_debug) {
        g_vsdata_debug = false;
        try {
            auto vsdata = resp.vsData();
            size_t dump = std::min(vsdata.size(), size_t(128));
            std::cerr << "\n[DBG] vsdata " << vsdata.size() << " bytes (first " << dump << "):\n";
            for (size_t i = 0; i < dump; ++i) {
                std::cerr << std::hex << std::setw(2) << std::setfill('0') << (int)vsdata[i] << ' ';
                if ((i+1) % 16 == 0) std::cerr << '\n';
            }
            std::cerr << std::dec << '\n';
        } catch (...) {}
    }
    out = parseDataBuf(resp);
    return true;
}

// Read temperature via VSFR::TEMP_degC. Returns -999 on failure.
// Used at startup; GID=3 RareData records in DATA_BUF also carry temperature
// but appear only ~every 30 s.
static float readTemperature(Peripheral& device, RadiacodeBuffer& rx,
                             RadiaCodeProtocol& proto) {
    uint8_t seq; ParsedResponse resp;
    if (!sendAndWait(device, rx, proto.buildReadVSFRBatch({VSFR::TEMP_degC}, seq),
                     static_cast<uint16_t>(Command::RD_VIRT_SFR_BATCH), resp, 3000))
        return -999.0f;
    if (!(resp.payloadU32(0) & 0x01)) return -999.0f;
    return resp.payloadF32(4);
}

// ── Measurement accumulator ───────────────────────────────────────────────────

struct Accumulator {
    double sum_cps = 0.0, sum_dose = 0.0, sum_err = 0.0, sum_dose_err = 0.0;
    int    count = 0, err_count = 0, dose_err_count = 0;

    void add(const DeviceMeasurements& m) {
        if (!m.valid) return;
        sum_cps  += m.count_rate;
        sum_dose += m.dose_rate_uSv_h;
        if (m.err_pct > 0.0f) { sum_err += m.err_pct; ++err_count; }
        if (m.dose_err_pct > 0.0f) { sum_dose_err += m.dose_err_pct; ++dose_err_count; }
        ++count;
    }

    bool   empty()       const { return count == 0; }
    double avgCPS()      const { return count     ? sum_cps  / count     : 0.0; }
    double avgDose()     const { return count     ? sum_dose / count     : 0.0; }
    double avgErr()      const { return err_count ? sum_err  / err_count : 0.0; }
    double avgDoseErr()  const { return dose_err_count ? sum_dose_err / dose_err_count : 0.0; }
};

// Format one measurement line.
// Temperature and battery are printed once at startup (from VSFR / GID=3 RareData),
// not repeated here to keep the output clean.
// Output: [HH:MM:SS] HT=<µSv/h µR/h> HTErr=±<> CPS=<> CPM=<> CPSErr=±<>
static std::string format_line(const Accumulator& acc) {
    double cps     = acc.avgCPS();
    double dose    = acc.avgDose();
    double dose_uR = dose * 115.0;   // 1 µSv/h = 115 µR/h
    double cpm     = cps  * 60.0;
    double err     = acc.avgErr();
    double dose_err = acc.avgDoseErr();

    std::ostringstream os;
    os << std::fixed;
    os << "[" << pc_time_str() << "] ";
    os << "HT=" << std::setprecision(4) << dose  << "\xc2\xb5Sv/h "
               << std::setprecision(2) << dose_uR << "\xc2\xb5R/h ";
    os << "HTErr=" << (dose_err > 0.0 
           ? "\xc2\xb1" + ([&]{ std::ostringstream t; t << std::fixed
                                << std::setprecision(1) << dose_err; return t.str(); }()) + "%"
           : "N/A") << " ";
    os << "CPS=" << std::setprecision(3) << cps << " ";
    os << "CPM=" << std::setprecision(1) << cpm << " ";
    os << "CPSErr=" << (err > 0.0
           ? "\xc2\xb1" + ([&]{ std::ostringstream t; t << std::fixed
                                << std::setprecision(1) << err; return t.str(); }()) + "%"
           : "N/A");
    return os.str();
}

// Poll DATA_BUF at poll_ms intervals for interval_s seconds, averaging all samples.
static Accumulator collect_interval(Peripheral& device, RadiacodeBuffer& rx,
                                    RadiaCodeProtocol& proto, int interval_s,
                                    int poll_ms = 500) {
    Accumulator acc;
    auto deadline = Clock::now() + std::chrono::seconds(interval_s);
    while (Clock::now() < deadline && !g_quit) {
        DeviceMeasurements m;
        if (readDataBuf(device, rx, proto, m)) acc.add(m);
        auto remaining = deadline - Clock::now();
        auto sleep_dur = std::chrono::milliseconds(poll_ms);
        if (remaining < sleep_dur)
            sleep_dur = std::chrono::duration_cast<std::chrono::milliseconds>(remaining);
        if (sleep_dur.count() > 0) std::this_thread::sleep_for(sleep_dur);
    }
    return acc;
}

// ── CLI argument parsing ──────────────────────────────────────────────────────

struct AppConfig {
    std::string mac;
    bool mode_live = false, mode_once = false, mode_quiet = false;
    int  runonce = 1, interval = 3;
    bool debug_vsdata = false;
};

static void usage(const char* prog) {
    std::cerr
        << "Usage: " << prog << " [OPTIONS]\n\n"
        << "  -m, --mac <MAC>       Device MAC address (skips auto-discovery)\n"
        << "  -l, --live            Continuous single-line display; q or Ctrl+C to quit\n"
        << "  -r, --runonce [N]     Print N averaged measurements and exit (default 1)\n"
        << "  -q, --quiet           Quiet mode: compact single-line output only\n"
        << "  -i, --interval <Y>    Averaging window in seconds (default 3)\n"
        << "  --debug-vsdata        Dump first 128 vsdata bytes to stderr once\n"
        << "\nDefault mode (no -l/-r/-q): behaves as -r 1.\n"
        << "Device Discovery: If -m is not provided, scans for RadiaCode devices.\n"
        << "  - 1 RadiaCode device found: connects automatically\n"
        << "  - Multiple RadiaCode devices or none found: prompts for MAC address\n"
        << "Output: [HH:MM:SS] HT=<uSv/h uR/h> CPS=<> CPM=<> Err=+-<%%>\n";
}

static AppConfig parse_args(int argc, char* argv[]) {
    AppConfig cfg;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "-m" || a == "--mac") {
            if (++i >= argc) throw std::runtime_error("--mac needs argument");
            cfg.mac = argv[i];
        } else if (a == "-l" || a == "--live") {
            cfg.mode_live = true;
        } else if (a == "-q" || a == "--quiet") {
            cfg.mode_quiet = true;
        } else if (a == "-r" || a == "--runonce") {
            cfg.mode_once = true;
            if (i+1 < argc && argv[i+1][0] != '-') cfg.runonce = std::stoi(argv[++i]);
            if (cfg.runonce < 1) throw std::runtime_error("--runonce >= 1");
        } else if (a == "-i" || a == "--interval") {
            if (++i >= argc) throw std::runtime_error("--interval needs argument");
            cfg.interval = std::stoi(argv[i]);
            if (cfg.interval < 1) throw std::runtime_error("--interval >= 1");
        } else if (a == "--debug-vsdata") {
            cfg.debug_vsdata = true;
        } else if (a == "-h" || a == "--help") {
            usage(argv[0]); exit(0);
        } else {
            throw std::runtime_error("Unknown option: " + a);
        }
    }
    if (!cfg.mode_live && !cfg.mode_once && !cfg.mode_quiet) cfg.mode_once = true;
    if ((cfg.mode_live + cfg.mode_once + cfg.mode_quiet) > 1) 
        throw std::runtime_error("-l, -r, and -q are mutually exclusive");
    return cfg;
}

// ── Device discovery by RadiaCode name ────────────────────────────────────────

struct DiscoveredDevice {
    std::string name;
    std::string mac;
};

static std::vector<DiscoveredDevice> discover_radiacode_devices() {
    std::vector<DiscoveredDevice> devices;
    std::vector<Adapter> adapters = Adapter::get_adapters();
    if (adapters.empty()) return devices;

    if (!g_quiet_mode) std::cerr << "[*] Scanning for RadiaCode devices...\n";
    adapters[0].scan_for(2000);

    for (auto& p : adapters[0].scan_get_results()) {
        std::string name = p.identifier();
        // Filter for RadiaCode devices only
        if (name.find("RadiaCode") != std::string::npos) {
            devices.push_back({name, p.address()});
            if (!g_quiet_mode) std::cerr << "[*] Found RadiaCode: " << name << " (" << p.address() << ")\n";
        }
    }
    return devices;
}

// Interactive MAC input with user prompt
static std::string interactive_mac_input() {
    std::string mac;
    const char* allowed_chars = "0123456789ABCDEFabcdef";
    std::cerr << "Enter RadiaCode MAC (format: XX:XX:XX:XX:XX:XX):\n";
    while (mac.length() < 17) {
        std::cerr << "\rMAC: " << mac << std::string(17 - mac.length(), '_');
        std::cerr.flush();
        int ch = getchar();
        if (ch == '\n' || ch == '\r' || ch == 'q' || ch == 'Q') {
            if (mac.length() == 17) break;
            std::cerr << "\n[!] Invalid MAC (need 17 chars).\n";
            mac.clear();
            continue;
        }
        if (ch == 127 || ch == 8) {  // backspace
            if (mac.length() > 0) {
                mac.pop_back();
                if (mac.length() > 0 && mac.back() == ':') mac.pop_back();
            }
            continue;
        }
        ch = std::toupper(ch);
        if (std::strchr(allowed_chars, ch)) {
            mac += ch;
            if (mac.length() % 3 == 2) mac += ':';
        }
    }
    std::cerr << "\rMAC: " << mac << " [OK]\n";
    return mac;
}

// ── Device connection and initialisation ─────────────────────────────────────

static bool connect_device(const std::string& mac, Peripheral& out_device,
                            RadiacodeBuffer& rx, RadiaCodeProtocol& proto) {
    std::vector<Adapter> adapters = Adapter::get_adapters();
    if (adapters.empty()) { if (!g_quiet_mode) std::cerr << "[!] No BT adapters.\n"; return false; }

    if (!g_quiet_mode) std::cerr << "Connecting to " << mac << "...\n";
    adapters[0].scan_for(3000);
    bool found = false;
    for (auto& p : adapters[0].scan_get_results())
        if (to_upper(p.address()) == to_upper(mac)) { out_device = p; found = true; break; }
    if (!found) { if (!g_quiet_mode) std::cerr << "[!] Device not found.\n"; return false; }

    out_device.connect();
    if (!g_quiet_mode) std::cerr << "[OK] Connected. Registering notifications...\n";
    out_device.notify(SERVICE_UUID, NOTIFY_UUID, [&](ByteArray data) {
        rx.addData(reinterpret_cast<const uint8_t*>(data.data()), data.size());
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    uint8_t seq; ParsedResponse resp;

    // Mandatory handshake — unlocks data commands.
    if (!sendAndWait(out_device, rx, proto.buildSetExchange(seq),
                     static_cast<uint16_t>(Command::SET_EXCHANGE), resp)) {
        if (!g_quiet_mode) std::cerr << "[!] Handshake timeout.\n"; return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Sync device RTC so DATA_BUF timestamps are meaningful.
    if (sendAndWait(out_device, rx, proto.buildSetTime(seq),
                    static_cast<uint16_t>(Command::SET_TIME), resp, 2000)) {
        if (!g_quiet_mode) std::cerr << "[OK] Device clock synced.\n";
    } else {
        if (!g_quiet_mode) std::cerr << "[!] SET_TIME failed (non-fatal).\n";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Reset measurement timer so DATA_BUF starts filling with fresh samples.
    if (!sendAndWait(out_device, rx,
                     proto.buildWriteVSFR(VSFR::DEVICE_TIME, uint32_t(0), seq),
                     static_cast<uint16_t>(Command::WR_VIRT_SFR), resp))
        if (!g_quiet_mode) std::cerr << "[!] DEVICE_TIME reset failed (non-fatal).\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Firmware version and serial number go to stderr.
    if (sendAndWait(out_device, rx, proto.buildGetVersion(seq),
                    static_cast<uint16_t>(Command::GET_VERSION), resp))
        if (!g_quiet_mode) try { std::cerr << parseGetVersion(resp).toString() << '\n'; } catch (...) {}
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if (sendAndWait(out_device, rx, proto.buildReadSerialNumber(seq),
                    static_cast<uint16_t>(Command::RD_VIRT_STRING), resp, 4000))
        if (resp.returnCode() == 1 && !g_quiet_mode)
            std::cerr << "S/N: " << resp.virtualString() << '\n';

    // Temperature via VSFR — always available, printed once at startup.
    float temp = readTemperature(out_device, rx, proto);
    if (!g_quiet_mode) {
        std::cerr << "Temperature: ";
        if (temp > -900.0f) std::cerr << std::fixed << std::setprecision(1) << temp << "\xc2\xb0""C\n";
        else                std::cerr << "N/A\n";
    }

    // Battery from first DATA_BUF read. GID=3 RareData appears ~every 30 s;
    // may not be present yet — shown as N/A if missing.
    DeviceMeasurements m;
    if (!g_quiet_mode) std::cerr << "Battery: ";
    if (readDataBuf(out_device, rx, proto, m) && m.charge_level >= 0.1f && m.charge_level <= 100.0f) {
        if (!g_quiet_mode) std::cerr << std::fixed << std::setprecision(0) << m.charge_level << "%\n";
    } else {
        if (!g_quiet_mode) std::cerr << "N/A\n";
    }

    if (!g_quiet_mode) std::cerr << "[OK] Ready. Waiting for stable buffer...\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));
    return true;
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    AppConfig cfg;
    try { cfg = parse_args(argc, argv); }
    catch (const std::exception& e) {
        std::cerr << "[!] " << e.what() << '\n'; usage(argv[0]); return 1;
    }
    
    g_quiet_mode = cfg.mode_quiet;
    std::signal(SIGINT, sig_handler);
    std::signal(SIGTERM, sig_handler);

    // Device discovery by RadiaCode name if no MAC provided via -m flag
    if (cfg.mac.empty()) {
        auto discovered = discover_radiacode_devices();
        
        if (discovered.size() == 1) {
            if (!g_quiet_mode) std::cerr << "[OK] Found single RadiaCode device: " << discovered[0].name << " (" << discovered[0].mac << ")\n";
            cfg.mac = discovered[0].mac;
        } else if (discovered.size() > 1) {
            if (!g_quiet_mode) {
                std::cerr << "[!] Found " << discovered.size() << " RadiaCode devices:\n";
                for (const auto& dev : discovered) {
                    std::cerr << "    " << dev.name << " (" << dev.mac << ")\n";
                }
                std::cerr << "[!] Please specify MAC with -m flag.\n";
            }
            cfg.mac = interactive_mac_input();
        } else {
            if (!g_quiet_mode) std::cerr << "[!] No RadiaCode devices found.\n";
            cfg.mac = interactive_mac_input();
        }
    }

    Peripheral device; RadiacodeBuffer rx; RadiaCodeProtocol proto;
    try {
        if (!connect_device(cfg.mac, device, rx, proto)) return 1;
        g_vsdata_debug = cfg.debug_vsdata;

        // ── Run-once mode (includes quiet mode) ────────────────────────
        if (cfg.mode_once || cfg.mode_quiet) {
            int remaining = cfg.runonce;
            while (remaining > 0 && !g_quit) {
                Accumulator acc = collect_interval(device, rx, proto, cfg.interval);
                if (g_quit) break;
                if (!acc.empty()) std::cout << format_line(acc) << '\n';
                else              std::cerr << "[!] No valid measurements.\n";
                --remaining;
            }
        }

        // ── Live mode — single line refreshed in place ───────────────────
        else {
            std::cerr << "Live mode — press 'q' or Ctrl+C to quit.\n";
            // Redirect stderr to /dev/null during polling so BLE diagnostic
            // messages don't break the single-line overwrite.
            int saved_stderr = dup(STDERR_FILENO);
            int devnull      = open("/dev/null", O_WRONLY);

            while (!g_quit) {
                if (kbhit_char() == 'q') break;

                dup2(devnull, STDERR_FILENO);
                Accumulator acc = collect_interval(device, rx, proto, cfg.interval);
                dup2(saved_stderr, STDERR_FILENO);

                if (g_quit) break;
                if (!acc.empty())
                    std::cout << "\033[2K\r" << format_line(acc) << std::flush;

                if (kbhit_char() == 'q') break;
            }
            dup2(saved_stderr, STDERR_FILENO);
            close(devnull); close(saved_stderr);
            std::cout << '\n';
        }

    } catch (const std::exception& e) {
        std::cerr << "\n[!] Exception: " << e.what() << '\n';
    }

    if (device.is_connected()) device.disconnect();
    return 0;
}