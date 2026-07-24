#include <WiFi.h>
#include <WebServer.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <atomic>
#include <string_view>
#if __has_include(<soc/soc_caps.h>)
#include <soc/soc_caps.h>
#endif
#include <esp_heap_caps.h>  // PSRAM-aware heap allocation (heap_caps_malloc)

// --- WI-FI HOTSPOT CAPABILITY SELECTION ---
// On boards where SOC_WIFI_SUPPORTED is 0 but an on-board Wi-Fi module is
// present (e.g. Waveshare ESP32-P4-NANO with companion ESP32-C6), uncomment
// the line below to force-enable the hotspot regardless of the SOC flag.
// Set to 0 to force-disable Wi-Fi at compile time.
#define WIFI_HOTSPOT_ENABLED 1

#if defined(WIFI_HOTSPOT_ENABLED)
  // Explicit user override takes priority over SOC detection.
  #define _WIFI_ACTIVE WIFI_HOTSPOT_ENABLED
#elif defined(SOC_WIFI_SUPPORTED)
  // Trust the SOC capability flag when the header is present.
  #define _WIFI_ACTIVE SOC_WIFI_SUPPORTED
#else
  // soc_caps.h was not found; WiFi.h is already included so assume Wi-Fi
  // is available on this toolchain target.
  #define _WIFI_ACTIVE 1
#endif

#include "VehicleInterpreters.h"
#include "VehicleSimulator.h"

// --- HARDWARE CONFIGURATION MAPPINGS ---
// 40-Pin Expansion Header Layout Assignments
#define CH0_TX 4
#define CH0_RX 5
#define CH1_TX 6
#define CH1_RX 7
#define CH2_TX 8
#define CH2_RX 9

// Integrated Audio Amplifier Header mapping for the Waveshare P4
#define AUDIO_PWM_PIN 45  

// --- SAFETY CRITICAL THRESHOLDS ---
#define MAX_SAFE_OIL_TEMP 115     // Alarm activates over 115°C
#define MAX_SAFE_COOLANT_TEMP 105 // Alarm activates over 105°C

// Uncomment to enable per-frame CAN hex logging (high UART bandwidth – bench use only)
// #define DEBUG_CAN_FRAMES

static constexpr uint32_t SERIAL_BAUD_RATE = 921600; // USB CDC virtual port; 921600 is the conventional high-throughput setting

// --- WI-FI ACCESS POINT CREDENTIALS ---
// SECURITY: AP_PASSWORD MUST be changed before deploying to a vehicle.
//   A guessable password allows unauthenticated access to live CAN telemetry.
//   Minimum recommended: 12 characters, mixed case + digits + symbols.
#define AP_SSID     "Audi_S3_Telemetry"
#define AP_PASSWORD_DEFAULT "ChangeMe_S3AP!"   // <-- REPLACE BEFORE DEPLOYMENT
// SECURITY: Change AP_PASSWORD_DEFAULT above before deploying to a real vehicle.

// --- THREAD-SAFE FIXED CHAR BUFFER ARRAY ---
static char global_ws_buffer[512]; // Extended to accommodate new telemetry fields
// std::atomic<bool> with acquire/release semantics provides the memory-ordering
// fence needed on RISC-V (ESP32-P4) so the buffer writes are visible to Core 0
// before it observes the flag as true.  Plain 'volatile' does NOT provide this.
static std::atomic<bool> ws_payload_ready{false};
static std::atomic<bool> ui_profile_refresh_pending{false};

// g_twai0_valid: set false before port-0 bus-off recovery uninstall, true after reinstall.
// Prevents Core 0 runBenchTelemetrySimulation from using the handle while it is invalid.
// Starts false; set true in setup() after startTwaiChannel(0) succeeds.
std::atomic<bool> g_twai0_valid{false};

// --- MULTICORE SYNCHRONISATION PRIMITIVES ---
// g_metrics_mux : protects sys_ctx->metrics fields (Core-0 bench-sim writes vs
//                 Core-1 interpreter writes and UI reads).
// g_interpreter_mutex : FreeRTOS mutex protecting sys_ctx->interpreter pointer
//                       and active_vehicle_profile (rarely written from Core 0
//                       via serial VIN injection).
portMUX_TYPE      g_metrics_mux      = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t g_interpreter_mutex = NULL;

// --- DYNAMIC GRAPHICAL UI OBJECT POINTERS ---
// (Removed 'static' so our class files can access them without conflicts)
lv_obj_t *tv; 
lv_obj_t *rpm_meter;
lv_obj_t *boost_meter;
lv_obj_t *oil_arc;
lv_obj_t *coolant_arc;

lv_obj_t *lbl_rpm_val;
lv_obj_t *lbl_boost_val;
lv_obj_t *lbl_temps_val;
lv_obj_t *label_comfort;
lv_obj_t *label_infotainment;

// --- EXTENDED UI LABEL POINTERS (new telemetry panels) ---
lv_obj_t *lbl_speed_val;
lv_obj_t *lbl_throttle_val;
lv_obj_t *lbl_comfort_climate;
lv_obj_t *lbl_infomt_detail;
lv_obj_t *lbl_diag;

// --- PLACE THIS DIRECTLY INSIDE Audi_S3_8V.ino (Lines 45-55) ---
LiveTelemetryMetrics s3_live_metrics;
DecodedVehicleMetrics active_vehicle_profile;
BaseVehicleInterpreter* currentCarInterpreter = NULL;

// --- COLOR AND TIMER VARIABLES ---
lv_color_t color_cool_blue;
lv_color_t color_normal_green; // Removed static to match header definition
lv_color_t color_alert_red;

static uint32_t last_beep_time = 0;
static bool alarm_sounding = false;

// Network Web Interface Handles
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
static uint32_t last_web_broadcast = 0;
static bool g_web_dashboard_ready = false;
static bool g_wifi_routes_registered = false;  // Ensure server routes are only added once.
static char g_ap_password[64] = AP_PASSWORD_DEFAULT;
static bool g_serial_waiting_for_password = false;
portMUX_TYPE g_ap_password_queue_mux = portMUX_INITIALIZER_UNLOCKED;
static char g_pending_ap_password[64] = {0};
static bool g_pending_ap_password_ready = false;

static lv_obj_t *g_pwd_modal = nullptr;
static lv_obj_t *g_pwd_textarea = nullptr;
static lv_obj_t *g_pwd_keyboard = nullptr;

// --- BENCH FULLTEST VIN SWEEP CONFIGURATION ---
struct BenchVinSignature {
    const char* wmi;
    const char* chassis;
};

struct BenchChassisYearRange {
    const char* chassis;
    int start_year;
    int end_year;
};

static constexpr char kVinYearTokens[] = "123456789ABCDEFGHJKLMNPRSTVWXY";
static constexpr uint32_t FULLTEST_STEP_INTERVAL_MS = 3000;
static constexpr int FULLTEST_BASE_YEAR = 2001;
// Temporary fixed year cap requested for bench test runs (RTC integration later).
static constexpr int FULLTEST_CURRENT_YEAR = 2026;

static const BenchVinSignature kBenchVinSignatures[] = {
    // Audi
    {"WAU", "8P"}, {"WAU", "8V"}, {"WAU", "GY"}, {"WAU", "8Y"}, {"WAU", "8K"}, {"WAU", "8W"}, {"WAU", "F4"},
    {"WAU", "4F"}, {"WAU", "4G"}, {"WAU", "4K"}, {"WAU", "8T"}, {"WAU", "8F"}, {"WAU", "4H"}, {"WAU", "4N"},
    {"WAU", "8U"}, {"WAU", "F3"}, {"WAU", "8R"}, {"WAU", "FY"}, {"WAU", "4L"}, {"WAU", "4M"}, {"WAU", "8J"},
    {"WAU", "8S"}, {"WAU", "GA"}, {"WAU", "8X"}, {"WAU", "GB"},
    // Volkswagen
    {"WVW", "1K"}, {"WVW", "5K"}, {"WVW", "AJ"}, {"WVW", "5G"}, {"WVW", "BA"}, {"WVW", "AM"}, {"WVW", "AU"},
    {"WVW", "CD"}, {"WVW", "3C"}, {"WVW", "AN"}, {"WVW", "3G"}, {"WVW", "CB"}, {"WVW", "A3"}, {"WVW", "13"},
    {"WVW", "5N"}, {"WVW", "AD"}, {"WVW", "AX"}, {"WVW", "CT"}, {"WVW", "6R"}, {"WVW", "6C"}, {"WVW", "AW"},
    {"WVW", "3H"},
    // Seat / Cupra
    {"VSS", "1P"}, {"VSS", "5F"}, {"VSS", "KL"}, {"VSS", "KJ"},
    // Skoda
    {"TMB", "1Z"}, {"TMB", "5E"}, {"TMB", "NX"}, {"TMB", "3T"}, {"TMB", "3V"},
    // Porsche
    {"WP0", "92"}, {"WP0", "9B"}
};

static const BenchChassisYearRange kBenchChassisYearRanges[] = {
    // Audi
    {"8P", 2003, 2012}, {"8V", 2013, 2020}, {"GY", 2020, FULLTEST_CURRENT_YEAR}, {"8Y", 2020, FULLTEST_CURRENT_YEAR},
    {"8K", 2007, 2016}, {"8W", 2016, FULLTEST_CURRENT_YEAR}, {"F4", 2016, FULLTEST_CURRENT_YEAR},
    {"4F", 2004, 2011}, {"4G", 2011, 2018}, {"4K", 2018, FULLTEST_CURRENT_YEAR},
    {"8T", 2007, 2017}, {"8F", 2009, 2017}, {"4H", 2009, 2017}, {"4N", 2017, FULLTEST_CURRENT_YEAR},
    {"8U", 2011, 2018}, {"F3", 2018, FULLTEST_CURRENT_YEAR}, {"8R", 2008, 2017}, {"FY", 2017, FULLTEST_CURRENT_YEAR},
    {"4L", 2005, 2015}, {"4M", 2015, FULLTEST_CURRENT_YEAR}, {"8J", 2006, 2014}, {"8S", 2014, 2023},
    {"GA", 2016, FULLTEST_CURRENT_YEAR}, {"8X", 2010, 2018}, {"GB", 2018, FULLTEST_CURRENT_YEAR},
    // Volkswagen
    {"1K", 2003, 2009}, {"5K", 2008, 2013}, {"AJ", 2005, 2015}, {"5G", 2012, 2021},
    {"BA", 2012, 2021}, {"AM", 2012, 2021}, {"AU", 2012, 2021}, {"CD", 2019, FULLTEST_CURRENT_YEAR},
    {"3C", 2005, 2010}, {"AN", 2010, 2015}, {"3G", 2014, FULLTEST_CURRENT_YEAR}, {"CB", 2014, FULLTEST_CURRENT_YEAR},
    {"A3", 2023, FULLTEST_CURRENT_YEAR}, {"13", 2008, 2017}, {"5N", 2007, 2017},
    {"AD", 2016, 2023}, {"AX", 2016, 2023}, {"CT", 2023, FULLTEST_CURRENT_YEAR},
    {"6R", 2009, 2018}, {"6C", 2014, 2018}, {"AW", 2017, FULLTEST_CURRENT_YEAR}, {"3H", 2017, FULLTEST_CURRENT_YEAR},
    // Seat / Cupra
    {"1P", 2005, 2012}, {"5F", 2012, 2020}, {"KL", 2020, FULLTEST_CURRENT_YEAR}, {"KJ", 2017, FULLTEST_CURRENT_YEAR},
    // Skoda
    {"1Z", 2004, 2013}, {"5E", 2012, 2020}, {"NX", 2020, FULLTEST_CURRENT_YEAR}, {"3T", 2008, 2015}, {"3V", 2015, FULLTEST_CURRENT_YEAR},
    // Porsche
    {"92", 2010, 2018}, {"9B", 2014, 2023}
};

static bool g_fulltest_active = false;
static size_t g_fulltest_sig_index = 0;
static size_t g_fulltest_year_index = 0;
static uint32_t g_fulltest_last_step_ms = 0;
static size_t g_fulltest_total_steps = 0;
static size_t g_fulltest_completed_steps = 0;

void applyUiProfileForCurrentInterpreter() {
    // LVGL is not thread-safe. Queue the UI profile refresh so the cockpit task
    // applies it on the same core/thread that already runs lv_timer_handler().
    ui_profile_refresh_pending.store(true, std::memory_order_release);
}

void refreshUiProfileIfPending() {
    if (!ui_profile_refresh_pending.exchange(false, std::memory_order_acq_rel)) return;

    if (g_interpreter_mutex != NULL) xSemaphoreTake(g_interpreter_mutex, portMAX_DELAY);
    if (sys_ctx != nullptr && sys_ctx->interpreter != nullptr) {
        sys_ctx->interpreter->configureUiLimits();
    }
    if (g_interpreter_mutex != NULL) xSemaphoreGive(g_interpreter_mutex);
}

void revertToGenericVehicleProfile() {
    if (g_interpreter_mutex != NULL) xSemaphoreTake(g_interpreter_mutex, portMAX_DELAY);
    active_vehicle_profile = DecodedVehicleMetrics{};
    if (sys_ctx != nullptr) {
        if (sys_ctx->interpreter != nullptr) {
            delete sys_ctx->interpreter;
            sys_ctx->interpreter = nullptr;
        }
        sys_ctx->interpreter = new GenericVehicleInterpreter();
    }
    if (g_interpreter_mutex != NULL) xSemaphoreGive(g_interpreter_mutex);
}

void restoreLiveVehicleIdentity() {
    Serial.println("[SYSTEM] Re-checking Powertrain Bus for a live VIN...");

    char live_vin[18] = { 0 };
    if (requestVehicleVIN(live_vin, sizeof(live_vin))) {
        Serial.print("[SYSTEM] SUCCESS! Detected Car VIN: ");
        Serial.println(live_vin);
        decodeAndPrintVehicleIdentity(live_vin);
    } else {
        Serial.println("[SYSTEM] WARNING: VIN query timed out. Defaulting to generic layout profiles.");
        revertToGenericVehicleProfile();
    }

    applyUiProfileForCurrentInterpreter();
}

void buildBenchVin(const BenchVinSignature& sig, char year_token, char* out_vin_18) {
    snprintf(out_vin_18, 18, "%sZZZ%s0%cA000000", sig.wmi, sig.chassis, year_token);
}

int benchVinYearFromToken(char token) {
    const char* match = strchr(kVinYearTokens, token);
    if (match == nullptr) return 0;
    return FULLTEST_BASE_YEAR + (int)(match - kVinYearTokens);
}

bool isBenchYearValidForChassis(const char* chassis, int year) {
    // Enforce fixed current year cap for bench test sweeps.
    if (year > FULLTEST_CURRENT_YEAR) return false;

    for (size_t i = 0; i < sizeof(kBenchChassisYearRanges) / sizeof(kBenchChassisYearRanges[0]); i++) {
        const BenchChassisYearRange& range = kBenchChassisYearRanges[i];
        if (strcmp(chassis, range.chassis) == 0) {
            return year >= range.start_year && year <= range.end_year;
        }
    }

    // Unknown future signatures still get the broad sweep within the global cap.
    return year >= FULLTEST_BASE_YEAR;
}

size_t countValidFulltestSteps() {
    const size_t sig_count = sizeof(kBenchVinSignatures) / sizeof(kBenchVinSignatures[0]);
    const size_t year_count = sizeof(kVinYearTokens) - 1; // exclude null terminator
    size_t total = 0;

    for (size_t s = 0; s < sig_count; s++) {
        for (size_t y = 0; y < year_count; y++) {
            const int year = benchVinYearFromToken(kVinYearTokens[y]);
            if (isBenchYearValidForChassis(kBenchVinSignatures[s].chassis, year)) {
                total++;
            }
        }
    }
    return total;
}

void advanceFulltestCursor(size_t year_count) {
    g_fulltest_year_index++;
    if (g_fulltest_year_index >= year_count) {
        g_fulltest_year_index = 0;
        g_fulltest_sig_index++;
    }
}

void printSystemStatus() {
    Serial.println();
    Serial.println("=== SYSTEM STATUS SUMMARY ===");

    // CAN channel health check (non-destructive: queries running drivers in place)
    const char* kChannelNames[] = { "Drive Train", "Comfort", "Infotainment" };
    for (int ch = 0; ch < 3; ch++) {
        twai_status_info_t info;
        if (twai_get_status_info_v2(twai_ports[ch], &info) == ESP_OK) {
            const char* state_str = "UNKNOWN";
            switch (info.state) {
                case TWAI_STATE_STOPPED:   state_str = "STOPPED";   break;
                case TWAI_STATE_RUNNING:   state_str = "RUNNING";   break;
                case TWAI_STATE_BUS_OFF:   state_str = "BUS-OFF";   break;
                case TWAI_STATE_RECOVERING: state_str = "RECOVERING"; break;
            }
            Serial.printf("[CAN CH%d] %-15s | State: %-10s | TX Err: %u  RX Err: %u\n",
                          ch, kChannelNames[ch], state_str,
                          (unsigned)info.tx_error_counter, (unsigned)info.rx_error_counter);
        } else {
            Serial.printf("[CAN CH%d] %-15s | Status query failed (driver not installed?)\n",
                          ch, kChannelNames[ch]);
        }
    }

#if _WIFI_ACTIVE
    if (g_web_dashboard_ready) {
        Serial.print("[WIFI] Dashboard URL: http://");
        Serial.println(WiFi.softAPIP());
        Serial.print("[WIFI] SSID: ");
        Serial.println(AP_SSID);
    } else {
        Serial.println("[WIFI] Hotspot not active.");
    }
#else
    Serial.println("[WIFI] Hotspot disabled at compile time.");
#endif

    Serial.println("=============================");
    Serial.printf("[HEAP]  Internal free: %u bytes | PSRAM free: %u / %u bytes\n",
                  (unsigned)ESP.getFreeHeap(),
                  (unsigned)ESP.getFreePsram(),
                  (unsigned)ESP.getPsramSize());
    Serial.println("=============================");
    Serial.println();
}

const char* validateApPassword(const char* password) {
    if (password == nullptr) return "Password cannot be empty.";
    const size_t len = strlen(password);
    if (len < 8 || len > 63) return "Password must be 8 to 63 characters.";
    for (size_t i = 0; i < len; i++) {
        const unsigned char c = static_cast<unsigned char>(password[i]);
        if (c < 32 || c > 126) return "Password must use printable ASCII characters only.";
    }
    return nullptr;
}

bool saveApPasswordToNvs(const char* password) {
    Preferences prefs;
    if (!prefs.begin("can-decoder", false)) {
        Serial.println("[WIFI] ERROR: Failed to open NVS namespace for password save.");
        return false;
    }
    const size_t bytes = prefs.putString("ap_pass", password);
    prefs.end();
    return bytes > 0;
}

void loadApPasswordFromNvs() {
    Preferences prefs;
    if (!prefs.begin("can-decoder", true)) {
        Serial.println("[WIFI] WARNING: Could not open NVS namespace. Using default AP password.");
        return;
    }
    String saved = prefs.getString("ap_pass", "");
    prefs.end();

    if (saved.length() == 0) return;

    const char* validation_error = validateApPassword(saved.c_str());
    if (validation_error != nullptr) {
        Serial.printf("[WIFI] WARNING: Saved AP password rejected: %s Using default password.\n", validation_error);
        return;
    }

    snprintf(g_ap_password, sizeof(g_ap_password), "%s", saved.c_str());
    Serial.println("[WIFI] Loaded AP password from NVS.");
}

bool queueApPasswordUpdate(const char* password) {
    if (password == nullptr) return false;
    const char* validation_error = validateApPassword(password);
    if (validation_error != nullptr) return false;

    portENTER_CRITICAL(&g_ap_password_queue_mux);
    snprintf(g_pending_ap_password, sizeof(g_pending_ap_password), "%s", password);
    g_pending_ap_password_ready = true;
    portEXIT_CRITICAL(&g_ap_password_queue_mux);
    return true;
}

bool applyAndPersistApPassword(const char* password, const char* source) {
    const char* validation_error = validateApPassword(password);
    if (validation_error != nullptr) {
        Serial.printf("[WIFI] %s password update rejected: %s\n", source, validation_error);
        return false;
    }

    bool save_ok = saveApPasswordToNvs(password);
    if (!save_ok) {
        Serial.println("[WIFI] WARNING: Password applied but could not be saved to NVS.");
    }

    snprintf(g_ap_password, sizeof(g_ap_password), "%s", password);

#if _WIFI_ACTIVE
    if (g_web_dashboard_ready) {
        Serial.println("[WIFI] Applying new AP password now. Restarting hotspot...");
        stopWifiHotspot();
        startWifiHotspot();
    }
#endif

    Serial.printf("[WIFI] AP password updated from %s.\n", source);
    return true;
}

void processQueuedPasswordChange() {
    if (!g_pending_ap_password_ready) return;

    char pending_password[64];
    portENTER_CRITICAL(&g_ap_password_queue_mux);
    if (!g_pending_ap_password_ready) {
        portEXIT_CRITICAL(&g_ap_password_queue_mux);
        return;
    }
    snprintf(pending_password, sizeof(pending_password), "%s", g_pending_ap_password);
    g_pending_ap_password_ready = false;
    g_pending_ap_password[0] = '\0';
    portEXIT_CRITICAL(&g_ap_password_queue_mux);

    applyAndPersistApPassword(pending_password, "UI/Web");
}

// --- WI-FI HOTSPOT RUNTIME CONTROL ---
// NOTE: startWifiHotspot() and stopWifiHotspot() are only ever called from
// Core 0 (setup() then loop()), which are sequential, so g_wifi_routes_registered
// requires no additional synchronisation.
#if _WIFI_ACTIVE
void startWifiHotspot() {
    if (g_web_dashboard_ready) {
        Serial.println("[WIFI] Hotspot is already running.");
        return;
    }
    // Runtime password safety reminder (replaces the removed compile-time guard).
    if (strcmp(g_ap_password, AP_PASSWORD_DEFAULT) == 0) {
        Serial.println("[WIFI] WARNING: AP password is still the default. Change it before deploying to a vehicle!");
    }
    if (WiFi.softAP(AP_SSID, g_ap_password)) {
        if (!g_wifi_routes_registered) {
            ws.onEvent(onWsEvent);
            server.addHandler(&ws);
            server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
                request->send_P(200, "text/html", index_html);
            });
            server.begin();
            g_wifi_routes_registered = true;
        }
        g_web_dashboard_ready = true;
        Serial.print("[WIFI] Hotspot started. SSID: "); Serial.println(AP_SSID);
        Serial.print("[WIFI] Dashboard URL: http://"); Serial.println(WiFi.softAPIP());
    } else {
        Serial.println("[WIFI] ERROR: Failed to start hotspot.");
    }
}

void stopWifiHotspot() {
    if (!g_web_dashboard_ready) {
        Serial.println("[WIFI] Hotspot is not running.");
        return;
    }
    WiFi.softAPdisconnect(true);
    g_web_dashboard_ready = false;
    Serial.println("[WIFI] Hotspot stopped.");
}
#endif

// --- COMFORT TEST: toggle all comfort fields for bench verification ---
void runComfortTest() {
    if (sys_ctx == nullptr) {
        Serial.println("[COMFORTTEST] sys_ctx not ready.");
        return;
    }
    portENTER_CRITICAL(&g_metrics_mux);
    sys_ctx->metrics.driver_door_open    = !sys_ctx->metrics.driver_door_open;
    sys_ctx->metrics.passenger_door_open = !sys_ctx->metrics.passenger_door_open;
    sys_ctx->metrics.rear_left_door_open = !sys_ctx->metrics.rear_left_door_open;
    sys_ctx->metrics.rear_right_door_open= !sys_ctx->metrics.rear_right_door_open;
    sys_ctx->metrics.handbrake_active    = !sys_ctx->metrics.handbrake_active;
    // Cycle target temp between two bench values on each call.
    sys_ctx->metrics.target_temp = (sys_ctx->metrics.target_temp == 0.0f) ? 22.0f : 0.0f;
    portEXIT_CRITICAL(&g_metrics_mux);
    Serial.println("[COMFORTTEST] Comfort fields toggled:");
    Serial.printf("  driver_door=%s  passenger_door=%s  rear_left=%s  rear_right=%s\n",
        sys_ctx->metrics.driver_door_open    ? "OPEN" : "CLOSED",
        sys_ctx->metrics.passenger_door_open ? "OPEN" : "CLOSED",
        sys_ctx->metrics.rear_left_door_open ? "OPEN" : "CLOSED",
        sys_ctx->metrics.rear_right_door_open? "OPEN" : "CLOSED");
    Serial.printf("  handbrake=%s  target_temp=%.1f C\n",
        sys_ctx->metrics.handbrake_active ? "ON" : "OFF",
        sys_ctx->metrics.target_temp);
}

void beginFullBenchVinTest() {
    g_fulltest_sig_index = 0;
    g_fulltest_year_index = 0;
    g_fulltest_completed_steps = 0;
    g_fulltest_total_steps = countValidFulltestSteps();

    if (g_fulltest_total_steps == 0) {
        g_fulltest_active = false;
        Serial.println("\n[FULLTEST] No valid VIN combinations for current fulltest year cap.");
        return;
    }

    g_fulltest_active = true;
    g_fulltest_last_step_ms = millis() - FULLTEST_STEP_INTERVAL_MS; // Trigger first VIN immediately.
    Serial.printf("\n[FULLTEST] Starting VIN sweep (%u signatures, current-year cap %u => %u valid test VINs)\n",
                  (unsigned)(sizeof(kBenchVinSignatures) / sizeof(kBenchVinSignatures[0])),
                  (unsigned)FULLTEST_CURRENT_YEAR,
                  (unsigned)g_fulltest_total_steps);
    Serial.println("[FULLTEST] A new VIN will be injected every 3 seconds.");
}

void runFullBenchVinTestStep() {
    if (!g_fulltest_active) return;
    if (millis() - g_fulltest_last_step_ms < FULLTEST_STEP_INTERVAL_MS) return;
    g_fulltest_last_step_ms = millis();

    const size_t sig_count = sizeof(kBenchVinSignatures) / sizeof(kBenchVinSignatures[0]);
    const size_t year_count = sizeof(kVinYearTokens) - 1; // exclude null terminator

    while (g_fulltest_sig_index < sig_count) {
        const BenchVinSignature& sig = kBenchVinSignatures[g_fulltest_sig_index];
        const char year_token = kVinYearTokens[g_fulltest_year_index];
        const int year = benchVinYearFromToken(year_token);
        const bool valid = isBenchYearValidForChassis(sig.chassis, year);

        if (valid) {
            char vin[18];
            buildBenchVin(sig, year_token, vin);
            g_fulltest_completed_steps++;
            Serial.printf("\n[FULLTEST] (%u/%u) Testing VIN: %s\n",
                          (unsigned)g_fulltest_completed_steps,
                          (unsigned)g_fulltest_total_steps,
                          vin);
            decodeAndPrintVehicleIdentity(vin);
            applyUiProfileForCurrentInterpreter();
            advanceFulltestCursor(year_count);
            return;
        }

        advanceFulltestCursor(year_count);
    }

    g_fulltest_active = false;
    Serial.println("\n[FULLTEST] VIN sweep completed.");
    restoreLiveVehicleIdentity();
}


// --- EMBEDDED DASHBOARD HTML PAGE ---
const char index_html[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>VAG CAN Decoder</title>
    <style>
        *{box-sizing:border-box;margin:0;padding:0}
        body{background:#0d0d0d;color:#eee;font-family:sans-serif;min-height:100vh}
        header{background:#1a1a1a;padding:12px 20px;display:flex;align-items:center;justify-content:space-between;border-bottom:2px solid #333}
        header h1{font-size:1.1em;letter-spacing:2px;color:#32c832}
        #ws_status{font-size:0.75em;padding:4px 10px;border-radius:20px;background:#333;color:#888}
        #ws_status.connected{background:#1a3a1a;color:#32c832}
        nav{display:flex;background:#1a1a1a;border-bottom:1px solid #333}
        nav button{flex:1;padding:14px;background:none;border:none;color:#888;cursor:pointer;font-size:0.9em;letter-spacing:1px;border-bottom:3px solid transparent;transition:all .2s}
        nav button.active{color:#fff;border-bottom-color:#32c832}
        nav button:hover{color:#ccc}
        .tab{display:none;padding:20px;max-width:1200px;margin:0 auto}
        .tab.active{display:block}
        .grid{display:flex;flex-wrap:wrap;gap:16px;justify-content:center}
        .card{background:#1c1c1c;border-radius:12px;padding:18px 22px;min-width:160px;text-align:center;box-shadow:0 4px 12px rgba(0,0,0,.6)}
        .card .lbl{font-size:0.72em;color:#666;text-transform:uppercase;letter-spacing:1px;margin-bottom:6px}
        .card .val{font-size:2.4em;font-weight:700;transition:color .2s}
        .card .unit{font-size:0.75em;color:#555;margin-top:4px}
        .green{color:#32c832}.blue{color:#0096ff}.red{color:#ff3e3e}.white{color:#fff}.amber{color:#f0a000}
        @keyframes blink{0%,100%{opacity:1}50%{opacity:.45}}
        .blink{animation:blink .35s infinite}
        button.action{background:#2a2a2a;color:#ccc;border:1px solid #444;padding:7px 14px;border-radius:6px;cursor:pointer;margin-top:10px;font-size:0.8em}
        button.action:hover{background:#333}
        /* Door grid */
        .door-grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;max-width:480px;margin:0 auto}
        .door-cell{background:#222;border-radius:10px;padding:16px 12px;text-align:center;border:2px solid #333;transition:background .3s,border-color .3s}
        .door-cell.open{background:#3a1a00;border-color:#f0a000}
        .door-cell .door-icon{font-size:2em}
        .door-cell .door-lbl{font-size:0.7em;color:#888;margin-top:4px;letter-spacing:1px}
        .door-cell .door-state{font-size:1em;font-weight:700;margin-top:4px}
        .door-cell.open .door-state{color:#f0a000}
        .door-cell:not(.open) .door-state{color:#32c832}
        /* Diagnostic table */
        table{width:100%;border-collapse:collapse;font-size:0.9em}
        td{padding:10px 14px;border-bottom:1px solid #2a2a2a}
        td:first-child{color:#666;width:40%}
        td:last-child{font-weight:600;color:#eee}
    </style>
</head>
<body>
<header>
    <h1 id="car_banner">VAG CAN DECODER</h1>
    <span id="ws_status">CONNECTING...</span>
</header>
<nav>
    <button class="active" onclick="showTab('perf',this)">PERFORMANCE</button>
    <button onclick="showTab('comfort',this)">COMFORT</button>
    <button onclick="showTab('info',this)">INFOTAINMENT</button>
    <button onclick="showTab('diag',this)">DIAGNOSTIC</button>
</nav>

<!-- TAB 1: PERFORMANCE -->
<div id="perf" class="tab active">
<div class="grid">
    <div class="card">
        <div class="lbl">Engine Speed</div>
        <div id="rpm" class="val green">0</div>
        <div class="unit">RPM</div>
    </div>
    <div class="card">
        <div class="lbl">Vehicle Speed</div>
        <div id="spd" class="val white">0</div>
        <div class="unit">km/h</div>
    </div>
    <div class="card">
        <div class="lbl">Turbo Boost</div>
        <div id="boost" class="val green">0.00</div>
        <div class="unit">Bar</div>
        <button class="action" onclick="resetPeak()">Reset Peak (<span id="peak">0.00</span>)</button>
    </div>
    <div class="card">
        <div class="lbl">Throttle</div>
        <div id="thr" class="val white">0</div>
        <div class="unit">%</div>
    </div>
    <div class="card">
        <div class="lbl">Engine Oil</div>
        <div id="oil" class="val blue">0</div>
        <div class="unit">&deg;C</div>
    </div>
    <div class="card">
        <div class="lbl">Coolant Temp</div>
        <div id="coolant" class="val blue">0</div>
        <div class="unit">&deg;C</div>
    </div>
</div>
</div>

<!-- TAB 2: COMFORT -->
<div id="comfort" class="tab">
<div class="grid" style="margin-bottom:24px">
    <div class="card">
        <div class="lbl">Exterior Temp</div>
        <div id="ext" class="val blue">0</div>
        <div class="unit">&deg;C</div>
    </div>
    <div class="card">
        <div class="lbl">Target Climate</div>
        <div id="tgt" class="val white">0</div>
        <div class="unit">&deg;C</div>
    </div>
    <div class="card">
        <div class="lbl">Handbrake</div>
        <div id="hb" class="val green">OFF</div>
    </div>
</div>
<div class="door-grid">
    <div class="door-cell" id="dc_dd">
        <div class="door-icon">&#x1F6AA;</div>
        <div class="door-lbl">DRIVER</div>
        <div class="door-state" id="ds_dd">CLOSED</div>
    </div>
    <div class="door-cell" id="dc_pd">
        <div class="door-icon">&#x1F6AA;</div>
        <div class="door-lbl">PASSENGER</div>
        <div class="door-state" id="ds_pd">CLOSED</div>
    </div>
    <div class="door-cell" id="dc_rld">
        <div class="door-icon">&#x1F6AA;</div>
        <div class="door-lbl">REAR LEFT</div>
        <div class="door-state" id="ds_rld">CLOSED</div>
    </div>
    <div class="door-cell" id="dc_rrd">
        <div class="door-icon">&#x1F6AA;</div>
        <div class="door-lbl">REAR RIGHT</div>
        <div class="door-state" id="ds_rrd">CLOSED</div>
    </div>
</div>
</div>

<!-- TAB 3: INFOTAINMENT -->
<div id="info" class="tab">
<div class="grid">
    <div class="card" style="min-width:300px">
        <div class="lbl">MMI Key Input</div>
        <div id="mmi_hex" class="val white">0x00</div>
        <div id="mmi_name" class="unit" style="font-size:1.1em;color:#f0a000;margin-top:8px">IDLE</div>
    </div>
    <div class="card" style="min-width:260px">
        <div class="lbl">Electrical Bus</div>
        <div id="bus" class="val white" style="font-size:1.4em">---</div>
    </div>
</div>
</div>

<!-- TAB 4: DIAGNOSTIC -->
<div id="diag" class="tab">
<div class="card" style="max-width:600px;margin:0 auto;text-align:left">
<table>
    <tr><td>Brand</td><td id="dg_brand">---</td></tr>
    <tr><td>Model</td><td id="dg_car">---</td></tr>
    <tr><td>Bus Platform</td><td id="dg_bus">---</td></tr>
    <tr><td>Production Year</td><td id="dg_year">---</td></tr>
    <tr><td>WebSocket</td><td id="dg_ws">CONNECTING</td></tr>
</table>
<div style="margin-top:14px">
    <button onclick="promptPasswordChange()">Change AP Password</button>
</div>
<div class="unit" style="margin-top:8px">WiFi will restart after save. Reconnect using the new password.</div>
</div>
</div>

<script>
var gw = `ws://${window.location.hostname}/ws`;
var ws;
window.addEventListener('load', connect);

function connect() {
    ws = new WebSocket(gw);
    ws.onopen  = function() { setStatus(true); };
    ws.onclose = function() { setStatus(false); setTimeout(connect, 2000); };
    ws.onmessage = function(e) {
        try { update(JSON.parse(e.data)); } catch(x) { console.error(x); }
    };
}

function setStatus(ok) {
    var el = document.getElementById('ws_status');
    el.textContent = ok ? 'LIVE' : 'RECONNECTING...';
    el.className = ok ? 'connected' : '';
    document.getElementById('dg_ws').textContent = ok ? 'CONNECTED' : 'DISCONNECTED';
}

function colorTemp(v, cold, hot) {
    if (v < cold) return 'blue';
    if (v > hot)  return 'red blink';
    return 'green';
}

function doorCell(cellId, stateId, open) {
    document.getElementById(cellId).className = 'door-cell' + (open ? ' open' : '');
    document.getElementById(stateId).textContent = open ? 'OPEN' : 'CLOSED';
}

function decodeMmi(code) {
    var map = {0x00:'IDLE',0x01:'VOL+',0x81:'VOL-',0x02:'TRACK+',0x82:'TRACK-',
               0x04:'MUTE',0x08:'MEDIA',0x10:'NAV',0x20:'PHONE',0x40:'VOICE'};
    return map[code] !== undefined ? map[code] : 'UNKNOWN';
}

function update(d) {
    if (d.car)   document.getElementById('car_banner').textContent = d.car;
    if (d.brand) document.getElementById('dg_brand').textContent   = d.brand;
    if (d.bus)   { document.getElementById('dg_bus').textContent   = d.bus;
                   document.getElementById('bus').textContent       = d.bus; }
    if (d.year)  document.getElementById('dg_year').textContent    = d.year;
    if (d.car)   document.getElementById('dg_car').textContent     = d.car;

    // Performance
    var rpm = d.rpm||0;
    var rpmEl = document.getElementById('rpm');
    rpmEl.textContent = rpm.toFixed(0);
    rpmEl.className = 'val ' + (rpm >= 6500 ? 'red blink' : 'green');

    document.getElementById('spd').textContent   = (d.spd||0).toFixed(1);
    document.getElementById('thr').textContent   = (d.thr||0).toFixed(0);

    var bEl = document.getElementById('boost');
    bEl.textContent = (d.boost||0).toFixed(2);
    bEl.className = 'val ' + ((d.boost||0) > 1.8 ? 'red' : 'green');
    document.getElementById('peak').textContent  = (d.peak||0).toFixed(2);

    var oEl = document.getElementById('oil');
    oEl.textContent = d.oil||0;
    oEl.className = 'val ' + colorTemp(d.oil||0, 75, 115);

    var cEl = document.getElementById('coolant');
    cEl.textContent = d.h2o||0;
    cEl.className = 'val ' + colorTemp(d.h2o||0, 70, 105);

    // Comfort
    document.getElementById('ext').textContent = (d.ext||0).toFixed(1);
    document.getElementById('ext').className   = 'val ' + ((d.ext||0) < 3 ? 'blue' : 'white');
    document.getElementById('tgt').textContent = (d.tgt||0).toFixed(1);
    var hbEl = document.getElementById('hb');
    hbEl.textContent = d.hb ? 'ON' : 'OFF';
    hbEl.className = 'val ' + (d.hb ? 'amber' : 'green');
    doorCell('dc_dd',  'ds_dd',  d.dd);
    doorCell('dc_pd',  'ds_pd',  d.pd);
    doorCell('dc_rld', 'ds_rld', d.rld);
    doorCell('dc_rrd', 'ds_rrd', d.rrd);

    // Infotainment
    var mmi = d.mmi||0;
    document.getElementById('mmi_hex').textContent  = '0x'+mmi.toString(16).toUpperCase().padStart(2,'0');
    document.getElementById('mmi_name').textContent = decodeMmi(mmi);
}

function showTab(id, btn) {
    document.querySelectorAll('.tab').forEach(function(t){ t.classList.remove('active'); });
    document.querySelectorAll('nav button').forEach(function(b){ b.classList.remove('active'); });
    document.getElementById(id).classList.add('active');
    btn.classList.add('active');
}

function resetPeak() {
    if (ws && ws.readyState === WebSocket.OPEN) ws.send('RESET_PEAK');
}

function promptPasswordChange() {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        alert('WebSocket not connected.');
        return;
    }
    var next = prompt('Enter new AP password (8-63 printable ASCII chars):');
    if (next === null) return;
    next = next.trim();
    if (next.length < 8 || next.length > 63) {
        alert('Password must be 8-63 characters.');
        return;
    }
    ws.send('SET_AP_PASSWORD ' + next);
    alert('Password change requested. The hotspot will restart.');
}
</script>
</body>
</html>
)rawhtml";


// Core Handles for the V2 Multi-Port API
twai_handle_t twai_ports[3];



// =========================================================================
//  ABSTRACT MULTI-VEHICLE PARSER INTERFACE & IMPLEMENTATIONS
// =========================================================================



// Forward Declarations for LVGL Touch Callbacks
static void handleBoostResetTouch(lv_event_t * e);
static void handlePasswordButtonTouch(lv_event_t * e);
static void handlePasswordSaveTouch(lv_event_t * e);
static void handlePasswordCancelTouch(lv_event_t * e);

const char* validateApPassword(const char* password);
bool saveApPasswordToNvs(const char* password);
void loadApPasswordFromNvs();
bool queueApPasswordUpdate(const char* password);
bool applyAndPersistApPassword(const char* password, const char* source);
void processQueuedPasswordChange();
void showPasswordEditorOverlay();
void closePasswordEditorOverlay();
#if _WIFI_ACTIVE
void startWifiHotspot();
void stopWifiHotspot();
#endif

// --- DISPLAY BUFFER BLOCK ALLOCATION FOR LVGL ---
// #define DISP_HOR_RES 800  // Set this to your specific Waveshare screen width
// #define DISP_VER_RES 480  // Set this to your specific Waveshare screen height

// --- DISPLAY RESOLUTION ADJUSTED FOR LANDSCAPE ROTATION ---
#define DISP_HOR_RES 1280 // Becomes your horizontal width when turned sideways
#define DISP_VER_RES 720  // Becomes your vertical height when turned sideways

static lv_disp_draw_buf_t draw_buf;
// Full-frame draw buffers – allocated from OPI PSRAM in setup().
// The ESP32-P4NRW32 has 32 MB of stacked OPI PSRAM (~200 MB/s bandwidth), so
// two full 1280×720 RGB565 frames (~1.75 MB each, ~3.5 MB total) fit easily and
// eliminate the partial-frame tearing that a small 10-line scratch buffer causes.
static lv_color_t *buf1 = nullptr;
static lv_color_t *buf2 = nullptr;
static lv_disp_drv_t disp_drv;

// Mandatory LVGL display driver callback function
void dummy_display_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {
    lv_disp_flush_ready(disp_drv);
}

// --- GT911 CAPACITIVE TOUCH LAYER VARIABLES ---
static lv_indev_drv_t indev_drv;

// Non-blocking callback: Reads the GT911 hardware over I2C and swaps axes for landscape
void landscape_touch_read_cb(lv_indev_drv_t * indev_driver, lv_indev_data_t * data) {
    // 1. Replace this section with your specific GT911 hardware check logic
    // Example pseudocode signature: if (gt911.isTouched()) { ... }
    bool touch_hardware_active = false; 

    if (touch_hardware_active) {
        // Read raw portrait hardware registers directly from the chip
        int raw_portrait_x = 0; // Read raw hardware X
        int raw_portrait_y = 0; // Read raw hardware Y

        // 2. MATHEMATICAL TRANSFORMATION MATRIX FOR 90-DEGREE ROTATION
        // Swap the axes and mirror the horizontal scale to convert Portrait to Landscape
        data->point.x = raw_portrait_y;
        data->point.y = 720 - 1 - raw_portrait_x; 

        data->state = LV_INDEV_STATE_PR; // Set state to "Pressed"
    } else {
        data->state = LV_INDEV_STATE_REL; // Set state to "Released"
    }
}

// Isolated High-Memory Task Pointer Handle
TaskHandle_t CockpitTaskHandle = NULL;

// CAN TX/RX pin table for bus-off recovery (mirrors the setup() calls)
static const int kCanTxPins[] = {CH0_TX, CH1_TX, CH2_TX};
static const int kCanRxPins[] = {CH0_RX, CH1_RX, CH2_RX};

// Forward declaration of the custom thread runner
void CockpitCoreProcessor(void *pvParameters) {
  Serial.println("[SYSTEM] High-Memory Telemetry Task Thread bound to Core 1 successfully.");
  
  for(;;) {
    lv_timer_handler(); 
    refreshUiProfileIfPending();
    
    processInboundFrames(0, "DRIVE TRAIN");
    processInboundFrames(1, "COMFORT");
    processInboundFrames(2, "INFOTAINMENT");
    
    updateUIElements();
    runAcousticAlertEngine();

    // H-2: Periodic CAN bus-off recovery (checked every 5 seconds).
    // If any controller enters the bus-off state (TEC > 255) it stops
    // RX/TX silently.  Detect and fully reinstall the driver to recover.
    static uint32_t last_busoff_check = 0;
    if (millis() - last_busoff_check > 5000) {
      last_busoff_check = millis();
      for (int ch = 0; ch < 3; ch++) {
        twai_status_info_t info;
        if (twai_get_status_info_v2(twai_ports[ch], &info) == ESP_OK) {
          if (info.state == TWAI_STATE_BUS_OFF) {
            Serial.printf("[RECOVERY] CAN Channel %d bus-off detected. Reinitialising...\n", ch);
            // C-5: Port 0 is also written by Core 0 (runBenchTelemetrySimulation).
            //      Signal Core 0 to stop using the handle, then wait long enough for
            //      any in-flight twai_transmit_v2(timeout=0) call to return before
            //      we uninstall the driver.  5 ms >> the sub-microsecond non-blocking
            //      transmit call, so the handle is guaranteed to be unused by the
            //      time twai_driver_uninstall_v2 runs.
            if (ch == 0) {
              g_twai0_valid.store(false, std::memory_order_release);
              vTaskDelay(pdMS_TO_TICKS(5));
            }
            twai_stop_v2(twai_ports[ch]);
            twai_driver_uninstall_v2(twai_ports[ch]);
            startTwaiChannel(ch, kCanTxPins[ch], kCanRxPins[ch]);
            if (ch == 0) g_twai0_valid.store(true, std::memory_order_release);
          }
        }
      }
    }

    // ASYNCHRONOUS TELEMETRY WEB STREAM OVERLAY (100ms / 10Hz)
    static uint32_t last_timer_tick = 0;
    if (millis() - last_timer_tick > 100) { 
      last_timer_tick = millis();
      
      // Only write to the buffer if Core 0 has dispatched the previous packet.
      // Use relaxed load here – we only need the acquire on the read side in loop().
      if (!ws_payload_ready.load(std::memory_order_relaxed)) { 
        static JsonDocument doc; 
        doc.clear(); 

        // C-4: Snapshot metrics under the spinlock to prevent torn reads from
        //      Core 0's runBenchTelemetrySimulation writing concurrently.
        LiveTelemetryMetrics m_snap;
        portENTER_CRITICAL(&g_metrics_mux);
        m_snap = sys_ctx->metrics;
        portEXIT_CRITICAL(&g_metrics_mux);

        // H-3: Read vehicle profile fields under the interpreter mutex.
        const char* car_name  = "GENERIC";
        const char* car_brand = "GENERIC";
        const char* car_bus   = "---";
        uint16_t    car_year  = 0;
        if (g_interpreter_mutex != NULL) {
          xSemaphoreTake(g_interpreter_mutex, portMAX_DELAY);
          car_name  = active_vehicle_profile.model_name;
          car_brand = active_vehicle_profile.brand;
          car_bus   = active_vehicle_profile.electrical_bus;
          car_year  = active_vehicle_profile.production_year;
          xSemaphoreGive(g_interpreter_mutex);
        }

        doc["rpm"]   = m_snap.engine_rpm;
        doc["boost"] = m_snap.boost_bar;
        doc["peak"]  = m_snap.peak_boost_bar;
        doc["oil"]   = m_snap.oil_temp;
        doc["h2o"]   = m_snap.coolant_temp;
        doc["car"]   = car_name;
        doc["brand"] = car_brand;
        doc["bus"]   = car_bus;
        doc["year"]  = car_year;
        doc["spd"]   = m_snap.vehicle_speed;
        doc["thr"]   = m_snap.throttle_pct;
        doc["ext"]   = m_snap.exterior_temp;
        doc["dd"]    = m_snap.driver_door_open;
        doc["pd"]    = m_snap.passenger_door_open;
        doc["rld"]   = m_snap.rear_left_door_open;
        doc["rrd"]   = m_snap.rear_right_door_open;
        doc["hb"]    = m_snap.handbrake_active;
        doc["mmi"]   = m_snap.mmi_key_code;
        doc["tgt"]   = m_snap.target_temp;
        
        serializeJson(doc, global_ws_buffer, sizeof(global_ws_buffer));
        
        // C-2: Release store ensures all preceding writes to global_ws_buffer
        //      are visible to Core 0 before it observes ws_payload_ready = true.
        ws_payload_ready.store(true, std::memory_order_release);
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(1)); 
  }
}


void setup() {
    // --- INJECT THIS LINE AT THE ABSOLUTE START OF SETUP() ---

  Serial.begin(SERIAL_BAUD_RATE);

    // Strict blocking loop: Forces the ESP32 to wait until the PC monitor opens!
    delay(500); 

  // 2. STABILISATION GATE: Gives your PC's USB port time to fully connect
  // (Flushes any phantom data out of the line so your text can print)
  for (int i = 5; i > 0; i--) {
      delay(400); 
      Serial.printf("[BOOT] Initializing terminal interface... Ready in %d seconds.\n", i);
  }

  Serial.println();
  Serial.printf("[BOOT] Serial console online at %lu baud.\n", (unsigned long)SERIAL_BAUD_RATE);
  Serial.printf("[BOOT] Free heap: %u bytes | PSRAM total: %u bytes | PSRAM free: %u bytes\n",
                (unsigned)ESP.getFreeHeap(),
                (unsigned)ESP.getPsramSize(),
                (unsigned)ESP.getFreePsram());

  sys_ctx = new GlobalFrameworkContext();

  // Create the interpreter mutex before any task can access sys_ctx->interpreter
  // or active_vehicle_profile.
  g_interpreter_mutex = xSemaphoreCreateMutex();

   delay(200); 

  sys_ctx->interpreter = new GenericVehicleInterpreter();
   delay(200);
  Serial.println("\n=== PILOT COCKPIT SAFETY PLATFORM INITIALISING ===");

  pinMode(AUDIO_PWM_PIN, OUTPUT);
  digitalWrite(AUDIO_PWM_PIN, LOW);

  // 1. RUN HARDWARE TRANSCIEVER ELECTRICAL DIAGNOSTICS ON BOOT
  bool system_safe = true;
  system_safe &= runBootDiagnostic(0, CH0_TX, CH0_RX, "Drive Train Transceiver");
   delay(100);
  system_safe &= runBootDiagnostic(1, CH1_TX, CH1_RX, "Comfort Transceiver");
   delay(100);
  system_safe &= runBootDiagnostic(2, CH2_TX, CH2_RX, "Infotainment Transceiver");
 delay(100);

  if (!system_safe) {
    Serial.println("\n[CRITICAL ERROR] Transceiver hardware diagnostic check failed!");
    while(1) delay(1000); 
  }

  // 2. PRODUCTION TWAI INTERFACE STARTUP
  startTwaiChannel(0, CH0_TX, CH0_RX);  delay(100);
  g_twai0_valid.store(true, std::memory_order_release);  // Port 0 handle is now valid
  startTwaiChannel(1, CH1_TX, CH1_RX);  delay(100);
  startTwaiChannel(2, CH2_TX, CH2_RX);  delay(100);

  // --- NEW ACTIVE VEHICLE IDENTIFICATION STAGE ---
    Serial.println("[SYSTEM] Interrogating Powertrain Bus for Vehicle Identification...");
  char global_vin[18] = { 0 };
  if (requestVehicleVIN(global_vin, sizeof(global_vin))) {
      Serial.print("[SYSTEM] SUCCESS! Detected Car VIN: ");
      Serial.println(global_vin);
      
      // Call the dynamic parsing matrix safely
      decodeAndPrintVehicleIdentity(global_vin);
  } else {
      Serial.println("[SYSTEM] WARNING: VIN query timed out. Defaulting to generic layout profiles.");
  }


  // 2b. ACTIVATE ASYNCHRONOUS COCKPIT HOTSPOT AP NETWORK
#if _WIFI_ACTIVE
  loadApPasswordFromNvs();
  startWifiHotspot();
#else
  Serial.println("[SYSTEM] Wi-Fi hotspot disabled: WIFI_HOTSPOT_ENABLED=0 set at compile time.");
#endif


  // 3. GRAPHICS DISPLAY & TOUCH ENVIRONMENT ENVIRONMENT BINDING
  // Allocate full-frame double buffers from OPI PSRAM before lv_init().
  // Two full 1280×720 RGB565 frames consume ~3.5 MB of the 32 MB OPI PSRAM.
  buf1 = (lv_color_t*)heap_caps_malloc(DISP_HOR_RES * DISP_VER_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  buf2 = (lv_color_t*)heap_caps_malloc(DISP_HOR_RES * DISP_VER_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  if (buf1 == nullptr || buf2 == nullptr) {
      Serial.println("[CRITICAL] OPI PSRAM display buffer allocation failed! Enable PSRAM (OPI PSRAM) in Arduino board settings.");
      while (1) delay(1000);
  }
  Serial.printf("[SYSTEM] LVGL double-frame buffers allocated in OPI PSRAM (%u KB each, %u KB total).\n",
                (unsigned)(DISP_HOR_RES * DISP_VER_RES * sizeof(lv_color_t) / 1024),
                (unsigned)(2 * DISP_HOR_RES * DISP_VER_RES * sizeof(lv_color_t) / 1024));

  lv_init();

  // Initialize the drawing buffer with full-frame double buffering
  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, DISP_HOR_RES * DISP_VER_RES);

  // Initialize the display driver structural tracker
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = DISP_HOR_RES;
  disp_drv.ver_res = DISP_VER_RES;
  disp_drv.draw_buf = &draw_buf;
  disp_drv.flush_cb = dummy_display_flush; 
  
  // FORCE LANDSCAPE ORIENTATION: Rotates the visual matrix 90 degrees clockwise
  disp_drv.rotated = LV_DISP_ROT_90; 
  
  // Register the driver inside the master LVGL engine
  lv_disp_drv_register(&disp_drv);

// 4. GT911 CAPACITIVE TOUCH INTERFACE INITIALIZATION
  lv_indev_drv_init(&indev_drv);
  
  // Define this input device type as a Touchpad panel
  indev_drv.type = LV_INDEV_TYPE_POINTER; 
  
  // Bind your landscape transformation math callback routine
  indev_drv.read_cb = landscape_touch_read_cb; 
  
  // FIX: Changed from lv_indev_register to lv_indev_drv_register
  lv_indev_drv_register(&indev_drv); 

  // Now it is completely safe to construct horizontal visual elements
  buildCockpitUI();
    // 5. SPAWN INDEPENDENT HIGH-MEMORY THREAD
   xTaskCreatePinnedToCore(
    CockpitCoreProcessor,     // Target function to execute
    "CockpitTask",            // Descriptive tag
    32768,                    // Allocates massive 32KB stack layout
    NULL,                     // Task parameters input
    1,                        // ◄ Priority 1: raised from idle (0) so the cockpit task is not starved by any brief Core 1 work
    &CockpitTaskHandle,       // Thread handle tracking variable
    1                         // Pin to Core 1
  );
  
  Serial.println("=== SYSTEM PRE-FLIGHT INITIALIZATION COMPLETED CLEANLY ===");
}

void loop() {
  static uint32_t last_cleanup = 0;
  if (g_web_dashboard_ready && millis() - last_cleanup > 1000) {
    last_cleanup = millis();
    ws.cleanupClients();
  }

  // --- BENCH DESK SHORTCUT COMMAND INJECTOR ---
  if (Serial.available() > 0) {
    String testVin = Serial.readStringUntil('\n');
    testVin.trim(); // Clean trailing whitespace feeds safely

    if (g_serial_waiting_for_password) {
        g_serial_waiting_for_password = false;
        if (!applyAndPersistApPassword(testVin.c_str(), "Serial console")) {
            Serial.println("[WIFI] Password unchanged. Try 'setpass' again.");
        }
    } else if (testVin.equalsIgnoreCase("setpass")) {
        g_serial_waiting_for_password = true;
        Serial.println("[WIFI] Enter new AP password (8-63 printable ASCII chars):");
    } else if (testVin.startsWith("setpass ")) {
        String newPass = testVin.substring(8);
        newPass.trim();
        if (!applyAndPersistApPassword(newPass.c_str(), "Serial console")) {
            Serial.println("[WIFI] Password unchanged. Use: setpass <new_password>");
        }
    } else if (testVin.equalsIgnoreCase("stoptest")) {
        if (g_fulltest_active) {
            g_fulltest_active = false;
            Serial.println("\n[FULLTEST] Test stopped by user command.");
        } else {
            Serial.println("\n[STOPTEST] No test was running.");
        }
        restoreLiveVehicleIdentity();
        printSystemStatus();
    } else if (testVin.equalsIgnoreCase("fulltest")) {
        beginFullBenchVinTest();
    } else if (testVin.equalsIgnoreCase("comforttest")) {
        runComfortTest();
    } else if (testVin.equalsIgnoreCase("wifi on")) {
#if _WIFI_ACTIVE
        startWifiHotspot();
#else
        Serial.println("[WIFI] Wi-Fi disabled at compile time.");
#endif
    } else if (testVin.equalsIgnoreCase("wifi off")) {
#if _WIFI_ACTIVE
        stopWifiHotspot();
#else
        Serial.println("[WIFI] Wi-Fi disabled at compile time.");
#endif
    } else if (testVin.length() == 17) {
        g_fulltest_active = false; // Manual single-VIN injection cancels any running full test sweep.
        Serial.println("\n[DEBUG] Injecting Bench Test VIN into Profile Matrix...");
        decodeAndPrintVehicleIdentity(testVin.c_str());
        applyUiProfileForCurrentInterpreter();
        Serial.println("[DEBUG] UI morphing execution complete.");
    } else {
        Serial.println("[DEBUG] Invalid input. Enter a 17-char VIN, 'fulltest', 'stoptest', 'comforttest', 'wifi on', 'wifi off', or 'setpass'.");
    }
  }

  processQueuedPasswordChange();
  runFullBenchVinTestStep();

  // --- FIXED TELEMETRY SWEEP INTERFACE GATING ---
  // (Removed the network family restrictions so it continues sweeping across ALL bench testing profiles!)
  {
      static float mock_rpm = 800.0;
      static bool ascending = true;

      if (ascending) {
          mock_rpm += 25.0;
          if (mock_rpm >= 5500.0) ascending = false;
      } else {
          mock_rpm -= 25.0;
          if (mock_rpm <= 800.0) ascending = true;
      }

      // Continuously inject Engine RPM, Turbo Boost, Oil Temp, and Coolant Temp values
      runBenchTelemetrySimulation(mock_rpm, 1.25, 95.0, 90.0);
  }

  // C-2: Acquire load ensures we see all Core-1 writes to global_ws_buffer
  //      that happened before the release store of ws_payload_ready = true.
  if (g_web_dashboard_ready && ws_payload_ready.load(std::memory_order_acquire)) {
    if (ws.count() > 0 && ws.availableForWriteAll()) {
      ws.textAll(global_ws_buffer);
    }
    ws_payload_ready.store(false, std::memory_order_relaxed);
  }
  delay(1);
}




// -------------------------------------------------------------
// RADIAL DASHBOARD CONTEXT MATRIX (LVGL ENGINE BUILD)
// -------------------------------------------------------------
void buildCockpitUI() {
  // Tab bar visible at 50px so users can tap to switch tabs
  sys_ctx->tv = lv_tabview_create(lv_scr_act(), LV_DIR_TOP, 50);
  lv_obj_t *t1 = lv_tabview_add_tab(sys_ctx->tv, "PERFORMANCE");
  lv_obj_t *t2 = lv_tabview_add_tab(sys_ctx->tv, "COMFORT");
  lv_obj_t *t3 = lv_tabview_add_tab(sys_ctx->tv, "INFOTAINMENT");
  lv_obj_t *t4 = lv_tabview_add_tab(sys_ctx->tv, "DIAGNOSTIC");

  // Style overall dashboard black background matrix
  lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
  lv_obj_set_style_bg_color(t1, lv_color_black(), 0);
  lv_obj_set_style_bg_color(t2, lv_color_black(), 0);
  lv_obj_set_style_bg_color(t3, lv_color_black(), 0);
  lv_obj_set_style_bg_color(t4, lv_color_black(), 0);

  // =========================================================================
  // TAB 1: RADIAL INSTRUMENTS & DIALS + SPEED / THROTTLE
  // =========================================================================

  // Allocate Engine Tachometer to Context Pointer
  sys_ctx->rpm_meter = lv_arc_create(t1);
  lv_obj_set_size(sys_ctx->rpm_meter, 220, 220);
  lv_obj_align(sys_ctx->rpm_meter, LV_ALIGN_LEFT_MID, 10, -20);
  lv_arc_set_bg_angles(sys_ctx->rpm_meter, 135, 45);
  lv_obj_remove_style(sys_ctx->rpm_meter, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(sys_ctx->rpm_meter, LV_OBJ_FLAG_CLICKABLE);

  // Allocate Boost Pressure Component to Context Pointer
  sys_ctx->boost_meter = lv_bar_create(t1);
  lv_obj_set_size(sys_ctx->boost_meter, 30, 160);
  lv_obj_align(sys_ctx->boost_meter, LV_ALIGN_RIGHT_MID, -140, -20);

  // Allocate Thermal Arcs to Context Pointers
  sys_ctx->oil_arc = lv_arc_create(t1);
  lv_obj_set_size(sys_ctx->oil_arc, 90, 90);
  lv_obj_align(sys_ctx->oil_arc, LV_ALIGN_RIGHT_MID, -20, -60);
  lv_arc_set_bg_angles(sys_ctx->oil_arc, 135, 45);
  lv_arc_set_range(sys_ctx->oil_arc, 50, 150);
  lv_obj_remove_style(sys_ctx->oil_arc, NULL, LV_PART_KNOB);

  sys_ctx->coolant_arc = lv_arc_create(t1);
  lv_obj_set_size(sys_ctx->coolant_arc, 90, 90);
  lv_obj_align(sys_ctx->coolant_arc, LV_ALIGN_RIGHT_MID, -20, 50);
  lv_arc_set_bg_angles(sys_ctx->coolant_arc, 135, 45);
  lv_arc_set_range(sys_ctx->coolant_arc, 50, 130);
  lv_obj_remove_style(sys_ctx->coolant_arc, NULL, LV_PART_KNOB);

  // Text Data Readout Value Overlays
  lbl_rpm_val = lv_label_create(sys_ctx->rpm_meter);
  lv_obj_align(lbl_rpm_val, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_text_color(lbl_rpm_val, lv_color_white(), 0);

  lbl_boost_val = lv_label_create(t1);
  lv_obj_align_to(lbl_boost_val, sys_ctx->boost_meter, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
  lv_obj_set_style_text_color(lbl_boost_val, lv_color_white(), 0);

  lbl_temps_val = lv_label_create(t1);
  lv_obj_align(lbl_temps_val, LV_ALIGN_RIGHT_MID, -10, -5);
  lv_obj_set_style_text_color(lbl_temps_val, lv_color_white(), 0);

  // Vehicle speed readout (centre-left, below tachometer)
  lbl_speed_val = lv_label_create(t1);
  lv_obj_align(lbl_speed_val, LV_ALIGN_CENTER, -90, 30);
  lv_obj_set_style_text_color(lbl_speed_val, lv_color_white(), 0);

  // Throttle position readout (below speed)
  lbl_throttle_val = lv_label_create(t1);
  lv_obj_align(lbl_throttle_val, LV_ALIGN_CENTER, -90, 65);
  lv_obj_set_style_text_color(lbl_throttle_val, lv_color_white(), 0);

  // Add Interactive Touch Handler Reset Hook for Peak Value
  lv_obj_add_flag(sys_ctx->boost_meter, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(sys_ctx->boost_meter, handleBoostResetTouch, LV_EVENT_CLICKED, NULL);

  // =========================================================================
  // TAB 2: COMFORT — DOOR STATUS, HANDBRAKE, CLIMATE
  // =========================================================================
  label_comfort = lv_label_create(t2);
  lv_obj_align(label_comfort, LV_ALIGN_TOP_LEFT, 20, 20);
  lv_obj_set_style_text_color(label_comfort, lv_color_white(), 0);

  lbl_comfort_climate = lv_label_create(t2);
  lv_obj_align(lbl_comfort_climate, LV_ALIGN_TOP_LEFT, 20, 200);
  lv_obj_set_style_text_color(lbl_comfort_climate, lv_color_white(), 0);

  // =========================================================================
  // TAB 3: INFOTAINMENT — DECODED MMI + PLATFORM INFO
  // =========================================================================
  label_infotainment = lv_label_create(t3);
  lv_obj_align(label_infotainment, LV_ALIGN_TOP_LEFT, 20, 20);
  lv_obj_set_style_text_color(label_infotainment, lv_color_white(), 0);

  lbl_infomt_detail = lv_label_create(t3);
  lv_obj_align(lbl_infomt_detail, LV_ALIGN_TOP_LEFT, 20, 110);
  lv_obj_set_style_text_color(lbl_infomt_detail, lv_color_white(), 0);

  // =========================================================================
  // TAB 4: DIAGNOSTIC — VEHICLE IDENTITY + SYSTEM HEALTH
  // =========================================================================
  lbl_diag = lv_label_create(t4);
  lv_obj_align(lbl_diag, LV_ALIGN_TOP_LEFT, 20, 20);
  lv_obj_set_style_text_color(lbl_diag, lv_color_white(), 0);

  lv_obj_t *pwd_btn = lv_btn_create(t4);
  lv_obj_set_size(pwd_btn, 300, 60);
  lv_obj_align(pwd_btn, LV_ALIGN_BOTTOM_MID, 0, -24);
  lv_obj_add_event_cb(pwd_btn, handlePasswordButtonTouch, LV_EVENT_CLICKED, NULL);
  lv_obj_t *pwd_lbl = lv_label_create(pwd_btn);
  lv_label_set_text(pwd_lbl, "Set WiFi Password");
  lv_obj_center(pwd_lbl);

  // =========================================================================
  // DYNAMIC ARCHITECTURE LOOKUP IMPLEMENTATION
  // =========================================================================
  if (sys_ctx->interpreter != nullptr) {
    sys_ctx->interpreter->configureUiLimits();
    Serial.println("[UI ENGINE] Dynamic vehicle profile scales and color palettes applied.");
  }
}

// -------------------------------------------------------------
// METRIC EXTRACTION REDRAW LOGIC CYCLE
// -------------------------------------------------------------

// Rate-limit counters for expensive diagnostic panel refresh
static uint32_t ui_last_info_refresh = 0;
static uint32_t ui_last_diag_refresh = 0;

// Helper: decode MMI rotary key code to a human-readable action string
static const char* decodeMmiKey(uint8_t code) {
  switch (code) {
    case 0x01: return "VOL+";
    case 0x81: return "VOL-";
    case 0x02: return "TRACK+";
    case 0x82: return "TRACK-";
    case 0x04: return "MUTE";
    case 0x08: return "MEDIA";
    case 0x10: return "NAV";
    case 0x20: return "PHONE";
    case 0x40: return "VOICE";
    case 0x00: return "IDLE";
    default:   return "UNKNOWN";
  }
}

void updateUIElements() {
  char buf[64];

  // C-4: Snapshot the entire metrics struct under the spinlock so we never
  //      observe a torn value from Core 0's runBenchTelemetrySimulation.
  //      The peak update is also done under the lock for the same reason.
  LiveTelemetryMetrics m;
  portENTER_CRITICAL(&g_metrics_mux);
  m = sys_ctx->metrics;
  if (m.boost_bar > m.peak_boost_bar) {
    sys_ctx->metrics.peak_boost_bar = m.boost_bar;
    m.peak_boost_bar = m.boost_bar;
  }
  portEXIT_CRITICAL(&g_metrics_mux);

  // 1. Sync Live Engine RPM Dials
  if (sys_ctx->rpm_meter != nullptr) {
    lv_arc_set_value(sys_ctx->rpm_meter, (int)m.engine_rpm);
  }
  snprintf(buf, sizeof(buf), "%.0f RPM", m.engine_rpm);
  lv_label_set_text(lbl_rpm_val, buf);

  // 2. Boost pressure bar and peak label
  if (sys_ctx->boost_meter != nullptr) {
    lv_bar_set_value(sys_ctx->boost_meter, (int)(m.boost_bar * 100), LV_ANIM_OFF);
  }
  snprintf(buf, sizeof(buf), "%.2f Bar\nPK: %.2f B", m.boost_bar, m.peak_boost_bar);
  lv_label_set_text(lbl_boost_val, buf);

  // 3. Sync Dynamic Thermal Engine Arcs
  if (sys_ctx->oil_arc != nullptr) {
    lv_arc_set_value(sys_ctx->oil_arc, (int)m.oil_temp);
  }
  if (sys_ctx->coolant_arc != nullptr) {
    lv_arc_set_value(sys_ctx->coolant_arc, (int)m.coolant_temp);
  }
  snprintf(buf, sizeof(buf), "OIL: %.0f C\nH2O: %.0f C", m.oil_temp, m.coolant_temp);
  lv_label_set_text(lbl_temps_val, buf);

  // 4. Speed and throttle readouts (Tab 1)
  snprintf(buf, sizeof(buf), "SPD: %.1f km/h", m.vehicle_speed);
  lv_label_set_text(lbl_speed_val, buf);

  snprintf(buf, sizeof(buf), "THR: %.0f%%", m.throttle_pct);
  lv_label_set_text(lbl_throttle_val, buf);

  // 5. Comfort tab — door grid + handbrake + target temp (Tab 2)
  char comfort_buf[192];
  snprintf(comfort_buf, sizeof(comfort_buf),
    "DRIVER:    %s\n"
    "PASSENGER: %s\n"
    "REAR LEFT: %s\n"
    "REAR RIGHT:%s\n"
    "HANDBRAKE: %s\n"
    "TGT TEMP:  %.1f C",
    m.driver_door_open    ? "OPEN" : "CLOSED",
    m.passenger_door_open ? "OPEN" : "CLOSED",
    m.rear_left_door_open ? "OPEN" : "CLOSED",
    m.rear_right_door_open? "OPEN" : "CLOSED",
    m.handbrake_active    ? "ON"   : "OFF",
    m.target_temp);
  lv_label_set_text(label_comfort, comfort_buf);

  snprintf(buf, sizeof(buf), "EXT TEMP: %.1f C", m.exterior_temp);
  lv_label_set_text(lbl_comfort_climate, buf);

  // 6. Infotainment tab — decoded MMI + rate-limited platform info (Tab 3)
  snprintf(buf, sizeof(buf), "MMI: 0x%02X  [%s]", m.mmi_key_code, decodeMmiKey(m.mmi_key_code));
  lv_label_set_text(label_infotainment, buf);

  uint32_t now = millis();
  if (now - ui_last_info_refresh >= 2000) {
    ui_last_info_refresh = now;
    char info_buf[128];
    // Read model name under interpreter mutex (non-blocking trylock — skip if busy)
    if (xSemaphoreTake(g_interpreter_mutex, 0) == pdTRUE) {
      const char *mdl  = active_vehicle_profile.model_name[0] ? active_vehicle_profile.model_name : "UNKNOWN";
      const char *bus  = active_vehicle_profile.electrical_bus[0] ? active_vehicle_profile.electrical_bus : "---";
      snprintf(info_buf, sizeof(info_buf), "MODEL: %s\nBUS:   %s", mdl, bus);
      xSemaphoreGive(g_interpreter_mutex);
    } else {
      snprintf(info_buf, sizeof(info_buf), "MODEL: (updating)\nBUS:   ---");
    }
    lv_label_set_text(lbl_infomt_detail, info_buf);
  }

  // 7. Diagnostic tab — vehicle identity + heap stats (Tab 4, rate-limited 5 s)
  if (now - ui_last_diag_refresh >= 5000) {
    ui_last_diag_refresh = now;
    char diag_buf[256];
    uint32_t heap  = ESP.getFreeHeap();
    uint32_t psram = ESP.getFreePsram();
    uint32_t up    = millis() / 1000;
    if (xSemaphoreTake(g_interpreter_mutex, 0) == pdTRUE) {
      const char *brand = active_vehicle_profile.brand[0] ? active_vehicle_profile.brand : "GENERIC";
      const char *mdl   = active_vehicle_profile.model_name[0] ? active_vehicle_profile.model_name : "UNKNOWN";
      const char *bus   = active_vehicle_profile.electrical_bus[0] ? active_vehicle_profile.electrical_bus : "---";
      uint16_t    yr    = active_vehicle_profile.production_year;
      snprintf(diag_buf, sizeof(diag_buf),
        "BRAND:  %s\n"
        "MODEL:  %s\n"
        "BUS:    %s\n"
        "YEAR:   %u\n"
        "HEAP:   %lu B\n"
        "PSRAM:  %lu B\n"
        "UPTIME: %lus",
        brand, mdl, bus, yr, heap, psram, (unsigned long)up);
      xSemaphoreGive(g_interpreter_mutex);
    } else {
      snprintf(diag_buf, sizeof(diag_buf),
        "HEAP:   %lu B\nPSRAM:  %lu B\nUPTIME: %lus",
        heap, psram, (unsigned long)up);
    }
    lv_label_set_text(lbl_diag, diag_buf);
  }
}

// Remote touch data receiver
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  // Strict Null Pointer Safety Guard. Drop execution if invalid.
  if (client == NULL) return; 

  if (type == WS_EVT_DATA) { 
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info != NULL && info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      // C-3: Copy into a local buffer instead of writing data[len]=0, which
      //      writes one byte past the end of the AsyncTCP receive buffer and
      //      corrupts the heap.
      char cmd[96];
      size_t copy_len = (len < sizeof(cmd) - 1) ? len : (sizeof(cmd) - 1);
      memcpy(cmd, data, copy_len);
      cmd[copy_len] = '\0';

      if (strcmp(cmd, "RESET_PEAK") == 0) {
        // C-4: The peak field is also accessed from Core 1; protect with spinlock.
        portENTER_CRITICAL(&g_metrics_mux);
        sys_ctx->metrics.peak_boost_bar = 0.0;
        portEXIT_CRITICAL(&g_metrics_mux);
        Serial.println("[WEB EVENT] Peak Turbo metrics zeroed out via remote command.");
      } else if (strncmp(cmd, "SET_AP_PASSWORD ", 16) == 0) {
        const char* candidate = cmd + 16;
        if (queueApPasswordUpdate(candidate)) {
            client->text("{\"ok\":true,\"msg\":\"Password change queued. Hotspot restarting.\"}");
            Serial.println("[WEB EVENT] AP password update requested by web UI.");
        } else {
            client->text("{\"ok\":false,\"msg\":\"Invalid AP password. Use 8-63 printable ASCII chars.\"}");
            Serial.println("[WEB EVENT] AP password update rejected (invalid format).");
        }
      }
    }
  }
  else if (type == WS_EVT_CONNECT) {
    Serial.println("[WEB SERVER] Remote phone/tablet terminal device connected successfully.");
  }
  else if (type == WS_EVT_DISCONNECT) {
    Serial.println("[WEB SERVER] Remote phone/tablet terminal device disconnected.");
  }
}

// -------------------------------------------------------------
// EVENT CALLBACK REGISTER PATHS
// -------------------------------------------------------------
static void handleBoostResetTouch(lv_event_t * e) {
  // C-4: Called on Core 1 from the LVGL event loop; use spinlock consistent
  //      with all other peak_boost_bar writes.
  portENTER_CRITICAL(&g_metrics_mux);
  sys_ctx->metrics.peak_boost_bar = 0.0;
  portEXIT_CRITICAL(&g_metrics_mux);
  Serial.println("[UI EVENT] Historical Peak Boost memory register zeroed out.");
}

void showPasswordEditorOverlay() {
  if (g_pwd_modal != nullptr) return;

  g_pwd_modal = lv_obj_create(lv_scr_act());
  lv_obj_set_size(g_pwd_modal, DISP_HOR_RES - 120, DISP_VER_RES - 120);
  lv_obj_center(g_pwd_modal);

  lv_obj_t *title = lv_label_create(g_pwd_modal);
  lv_label_set_text(title, "Enter new WiFi AP password (8-63 chars)");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 16);

  g_pwd_textarea = lv_textarea_create(g_pwd_modal);
  lv_obj_set_width(g_pwd_textarea, DISP_HOR_RES - 220);
  lv_obj_align(g_pwd_textarea, LV_ALIGN_TOP_MID, 0, 56);
  lv_textarea_set_one_line(g_pwd_textarea, true);
  lv_textarea_set_password_mode(g_pwd_textarea, true);
  lv_textarea_set_max_length(g_pwd_textarea, 63);

  lv_obj_t *save_btn = lv_btn_create(g_pwd_modal);
  lv_obj_set_size(save_btn, 150, 50);
  lv_obj_align(save_btn, LV_ALIGN_TOP_LEFT, 40, 112);
  lv_obj_add_event_cb(save_btn, handlePasswordSaveTouch, LV_EVENT_CLICKED, NULL);
  lv_obj_t *save_lbl = lv_label_create(save_btn);
  lv_label_set_text(save_lbl, "Save");
  lv_obj_center(save_lbl);

  lv_obj_t *cancel_btn = lv_btn_create(g_pwd_modal);
  lv_obj_set_size(cancel_btn, 150, 50);
  lv_obj_align(cancel_btn, LV_ALIGN_TOP_RIGHT, -40, 112);
  lv_obj_add_event_cb(cancel_btn, handlePasswordCancelTouch, LV_EVENT_CLICKED, NULL);
  lv_obj_t *cancel_lbl = lv_label_create(cancel_btn);
  lv_label_set_text(cancel_lbl, "Cancel");
  lv_obj_center(cancel_lbl);

  g_pwd_keyboard = lv_keyboard_create(g_pwd_modal);
  lv_obj_set_size(g_pwd_keyboard, DISP_HOR_RES - 220, 320);
  lv_obj_align(g_pwd_keyboard, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_keyboard_set_textarea(g_pwd_keyboard, g_pwd_textarea);
}

void closePasswordEditorOverlay() {
  if (g_pwd_modal == nullptr) return;
  lv_obj_del(g_pwd_modal);
  g_pwd_modal = nullptr;
  g_pwd_textarea = nullptr;
  g_pwd_keyboard = nullptr;
}

static void handlePasswordButtonTouch(lv_event_t * e) {
  (void)e;
  showPasswordEditorOverlay();
}

static void handlePasswordSaveTouch(lv_event_t * e) {
  (void)e;
  if (g_pwd_textarea == nullptr) return;

  const char* entered = lv_textarea_get_text(g_pwd_textarea);
  if (queueApPasswordUpdate(entered)) {
    Serial.println("[UI EVENT] AP password update requested from touchscreen.");
    closePasswordEditorOverlay();
  } else {
    Serial.println("[UI EVENT] Invalid AP password. Use 8-63 printable ASCII chars.");
  }
}

static void handlePasswordCancelTouch(lv_event_t * e) {
  (void)e;
  closePasswordEditorOverlay();
}

// --- ACTIVE UDS DIAGNOSTIC VIN EXTRACTION ---
bool requestVehicleVIN(char* vinBuffer, size_t bufferSize) {
    if (bufferSize < 18) return false; // VIN is strictly 17 characters + null terminator
    
    twai_message_t tx_msg;
    tx_msg.identifier = 0x7E0;  // Standard Engine ECU Diagnostic Request ID
    tx_msg.extd = 0;
    tx_msg.rtr = 0;
    tx_msg.data_length_code = 8;
    
    // ISO-TP Single Frame: 3 bytes payload follow (Service 0x22, DID 0xF190)
    tx_msg.data[0] = 0x03; 
    tx_msg.data[1] = 0x22; 
    tx_msg.data[2] = 0xF1; 
    tx_msg.data[3] = 0x90; 
    for(int i=4; i<8; i++) tx_msg.data[i] = 0xAA; // Padding

    // Fire request out on Channel 0 (Drive Train Bus)
    if (twai_transmit_v2(twai_ports[0], &tx_msg, pdMS_TO_TICKS(100)) != ESP_OK) {
        Serial.println("[VIN DETECT] Failed to transmit diagnostic query frame.");
        return false;
    }

    twai_message_t rx_msg;
    uint32_t startTime = millis();
    int charsCollected = 0;
    bool flowControlSent = false;

    // Await multi-frame response payload block (1.5-second timeout gate)
    while (millis() - startTime < 1500 && charsCollected < 17) {
        if (twai_receive_v2(twai_ports[0], &rx_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
            if (rx_msg.identifier == 0x7E8) { // Engine ECU Diagnostic Response
                
                // Case A: ISO-TP First Frame (0x10) - Indicates data payload is splitting up
                if ((rx_msg.data[0] & 0xF0) == 0x10) {
                    // Extract initial string segment bytes safely starting at index 5
                    vinBuffer[0] = rx_msg.data[5]; // Stores 'W' (or 'T')
                    vinBuffer[1] = rx_msg.data[6]; // Stores 'A' (or 'R')
                    vinBuffer[2] = rx_msg.data[7]; // Stores 'U'
                    charsCollected = 3;

                    // Send the Mandatory ISO-TP Flow Control Frame immediately to unlock remaining frames
                    twai_message_t fc_msg;
                    fc_msg.identifier = 0x7E0;
                    fc_msg.extd = 0;
                    fc_msg.rtr = 0;
                    fc_msg.data_length_code = 8;
                    fc_msg.data[0] = 0x30; // Flow Control Clear To Send (CTS)
                    fc_msg.data[1] = 0x00; // Block Size = 0 (Send all)
                    fc_msg.data[2] = 0x00; // Separation Time = 0ms (Max Speed)
                    for(int i=3; i<8; i++) fc_msg.data[i] = 0xAA;
                    
                    twai_transmit_v2(twai_ports[0], &fc_msg, pdMS_TO_TICKS(50));
                    flowControlSent = true;
                }
                // Case B: ISO-TP Consecutive Frame (0x21, 0x22, etc.)
                else if ((rx_msg.data[0] & 0xF0) == 0x20 && flowControlSent) {
                    // Start reading at index 1 because index 0 is the sequence number (e.g. 0x21)
                    for (int i = 1; i < 8 && charsCollected < 17; i++) {
                        vinBuffer[charsCollected++] = rx_msg.data[i];
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (charsCollected == 17) {
      vinBuffer[charsCollected] = '\0'; 
      return true;
  }
    return false;
}

// -------------------------------------------------------------
// HARDWARE INITIALIZATION & PRE-FLIGHT TESTERS
// -------------------------------------------------------------
bool runBootDiagnostic(int port_idx, int tx_pin, int rx_pin, const char* label) {
  twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)tx_pin, (gpio_num_t)rx_pin, TWAI_MODE_NO_ACK);
  
  // Set the structural hardware controller index directly 
  g_cfg.controller_id = port_idx; 
  
  twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  
  twai_handle_t temp_handle;
  // Removed invalid (twai_port_num_t) typecast
  if (twai_driver_install_v2(&g_cfg, &t_cfg, &f_cfg, &temp_handle) != ESP_OK) return false;
  twai_start_v2(temp_handle);

  twai_status_info_t status;
  twai_get_status_info_v2(temp_handle, &status);
  
  twai_stop_v2(temp_handle);
  twai_driver_uninstall_v2(temp_handle);
  return (status.state != TWAI_STATE_BUS_OFF);
}

void decodeAndPrintVehicleIdentity(const char* vin) {
    if (strlen(vin) < 17) return;

    // =========================================================================
    // 1. EXTRACT WORLD MANUFACTURER IDENTIFIER (WMI) - POSITIONS 1-3
    // =========================================================================
    if (strncmp(vin, "WAU", 3) == 0)      active_vehicle_profile.brand = "AUDI AG (GERMANY)";
    else if (strncmp(vin, "TRU", 3) == 0) active_vehicle_profile.brand = "AUDI AG (HUNGARY)";
    else if (strncmp(vin, "WVW", 3) == 0) active_vehicle_profile.brand = "VOLKSWAGEN CARS";
    else if (strncmp(vin, "WVG", 3) == 0) active_vehicle_profile.brand = "VOLKSWAGEN SUV DIVISION";
    else if (strncmp(vin, "VSS", 3) == 0) active_vehicle_profile.brand = "SEAT / CUPRA";
    else if (strncmp(vin, "TMB", 3) == 0) active_vehicle_profile.brand = "SKODA AUTO";
    else if (strncmp(vin, "WP0", 3) == 0) active_vehicle_profile.brand = "PORSCHE STUTTGART";
    else                                  active_vehicle_profile.brand = "VAG MOTOR CORP";

    // =========================================================================
    // 2. EXTRACT CHASSIS GENERATION DICTIONARY - POSITIONS 7-8
    // =========================================================================
    char chassis[3] = { vin[6], vin[7], '\0' };
    active_vehicle_profile.network_generation = SERIES_UNKNOWN; // Reset baseline safety state

    // --- AUDI DIVISION MAPPINGS ---
    if (strcmp(chassis, "8P") == 0) { 
        active_vehicle_profile.model_name = "Audi A3 / S3 (Mk2 Platform)"; 
        active_vehicle_profile.electrical_bus = "CAN-TP2.0 LEGACY"; 
        active_vehicle_profile.network_generation = SERIES_PQ35_46_LEGACY; 
    }
    else if (strcmp(chassis, "8V") == 0) { 
        active_vehicle_profile.model_name = "Audi A3 / S3 / RS3 (MQB Matrix)"; 
        active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; 
        active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; 
    }
    else if (strcmp(chassis, "GY") == 0 || strcmp(chassis, "8Y") == 0) { 
        active_vehicle_profile.model_name = "Audi A3 / S3 / RS3 (MQB EVO 8Y)"; 
        active_vehicle_profile.electrical_bus = "MQB EVO CAN-FD/CAN"; 
        active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; 
    }
    else if (strcmp(chassis, "8K") == 0) { 
        active_vehicle_profile.model_name = "Audi A4 / S4 / RS4 (MLB B8)"; 
        active_vehicle_profile.electrical_bus = "MLB-INFRASTRUCTURE CAN"; 
        active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; 
    }
    else if (strcmp(chassis, "8W") == 0 || strcmp(chassis, "F4") == 0) { 
        active_vehicle_profile.model_name = "Audi A4 / S4 / A5 / RS5 (MLB B9)"; 
        active_vehicle_profile.electrical_bus = "MLB EVO FLEXRAY/CAN"; 
        active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; 
    }
    else if (strcmp(chassis, "4F") == 0) { 
        active_vehicle_profile.model_name = "Audi A6 / S6 / RS6 (C6 Era)"; 
        active_vehicle_profile.electrical_bus = "CAN-TP2.0 INFRASTRUCTURE"; 
        active_vehicle_profile.network_generation = SERIES_PQ35_46_LEGACY; 
    }
    else if (strcmp(chassis, "4G") == 0) { 
        active_vehicle_profile.model_name = "Audi A6 / S6 / A7 / RS7 (MLB C7)"; 
        active_vehicle_profile.electrical_bus = "MLB-INFRASTRUCTURE CAN"; 
        active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; 
    }
    else if (strcmp(chassis, "4K") == 0) { 
        active_vehicle_profile.model_name = "Audi A6 / A7 / RS6 / RS7 (MLB C8)"; 
        active_vehicle_profile.electrical_bus = "MLB EVO FLEXRAY/CAN"; 
        active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; 
    }
    else if (strcmp(chassis, "8T") == 0 || strcmp(chassis, "8F") == 0) { 
        active_vehicle_profile.model_name = "Audi A5 / S5 / RS5 (B8 Chassis)"; 
        active_vehicle_profile.electrical_bus = "MLB-INFRASTRUCTURE CAN"; 
        active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; 
    }
    else if (strcmp(chassis, "4H") == 0) { 
        active_vehicle_profile.model_name = "Audi A8 / S8 (D4 Luxury)"; 
        active_vehicle_profile.electrical_bus = "MLB-INFRASTRUCTURE CAN"; 
        active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; 
    }
    else if (strcmp(chassis, "4N") == 0) { 
        active_vehicle_profile.model_name = "Audi A8 / S8 (D5 Luxury)"; 
        active_vehicle_profile.electrical_bus = "MLB EVO FLEXRAY/CAN"; 
        active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; 
    }
    else if (strcmp(chassis, "8U") == 0) { 
        active_vehicle_profile.model_name = "Audi Q3 Compact SUV (PQ35)"; 
        active_vehicle_profile.electrical_bus = "HIGH-SPEED CAN-TP2.0"; 
        active_vehicle_profile.network_generation = SERIES_PQ35_46_LEGACY; 
    }
    else if (strcmp(chassis, "F3") == 0) { 
        active_vehicle_profile.model_name = "Audi Q3 / RS Q3 (MQB Crossover)"; 
        active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; 
        active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; 
    }
    else if (strcmp(chassis, "8R") == 0) { 
        active_vehicle_profile.model_name = "Audi Q5 SUV (Original 8R)"; 
        active_vehicle_profile.electrical_bus = "MLB-INFRASTRUCTURE CAN"; 
        active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; 
    }
    else if (strcmp(chassis, "FY") == 0) { 
        active_vehicle_profile.model_name = "Audi Q5 / SQ5 (MLB FY Platform)"; 
        active_vehicle_profile.electrical_bus = "MLB EVO FLEXRAY/CAN"; 
        active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; 
    }
    else if (strcmp(chassis, "4L") == 0) { 
        active_vehicle_profile.model_name = "Audi Q7 SUV (PQ47 4L Chassis)"; 
        active_vehicle_profile.electrical_bus = "CAN-TP2.0 INFRASTRUCTURE"; 
        active_vehicle_profile.network_generation = SERIES_PQ35_46_LEGACY; 
    }
    else if (strcmp(chassis, "4M") == 0) { 
        active_vehicle_profile.model_name = "Audi Q7 / SQ7 / Q8 / SQ8 / RSQ8"; 
        active_vehicle_profile.electrical_bus = "MLB EVO FLEXRAY/CAN"; 
        active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; 
    }
    else if (strcmp(chassis, "8J") == 0) { 
        active_vehicle_profile.model_name = "Audi TT / TTS / TT RS (Mk2)"; 
        active_vehicle_profile.electrical_bus = "CAN-TP2.0 MOTORWAY"; 
        active_vehicle_profile.network_generation = SERIES_PQ35_46_LEGACY; 
    }
    else if (strcmp(chassis, "8S") == 0) { 
        active_vehicle_profile.model_name = "Audi TT / TTS / TT RS (Mk3 MQB)"; 
        active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; 
        active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; 
    }
    else if (strcmp(chassis, "GA") == 0) { 
        active_vehicle_profile.model_name = "Audi Q2 Compact Crossover"; 
        active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; 
        active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; 
    }
    else if (strcmp(chassis, "8X") == 0) { 
        active_vehicle_profile.model_name = "Audi A1 Supermini (PQ25)"; 
        active_vehicle_profile.electrical_bus = "CAN-TP2.0 COMPACT"; 
        active_vehicle_profile.network_generation = SERIES_SMALL_PO_SKODA; 
    }
    else if (strcmp(chassis, "GB") == 0) { 
        active_vehicle_profile.model_name = "Audi A1 Sportback (MQB A0)"; 
        active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; 
        active_vehicle_profile.network_generation = SERIES_SMALL_PO_SKODA; 
    }

    // --- VOLKSWAGEN DIVISION MAPPINGS ---
    else if (strcmp(chassis, "1K") == 0 || strcmp(chassis, "5K") == 0 || strcmp(chassis, "AJ") == 0) { 
        active_vehicle_profile.model_name = "VW Golf Mk5 / Mk6 / Jetta"; 
        active_vehicle_profile.electrical_bus = "CAN-TP2.0 POWERTRAIN"; 
        active_vehicle_profile.network_generation = SERIES_PQ35_46_LEGACY; 
    }
    else if (strcmp(chassis, "5G") == 0 || strcmp(chassis, "BA") == 0 || strcmp(chassis, "AM") == 0 || strcmp(chassis, "AU") == 0) { 
        active_vehicle_profile.model_name = "VW Golf Mk7 / GTI / Golf R (MQB)"; 
        active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; 
        active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; 
    }
    else if (strcmp(chassis, "CD") == 0) { 
        active_vehicle_profile.model_name = "VW Golf Mk8 / GTI / Clubsport / R"; 
        active_vehicle_profile.electrical_bus = "MQB EVO HIGH-SPEED CAN"; 
        active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; 
    }
    else if (strcmp(chassis, "3C") == 0 || strcmp(chassis, "AN") == 0) { 
        active_vehicle_profile.model_name = "VW Passat B6 / B7 / CC"; 
        active_vehicle_profile.electrical_bus = "CAN-TP2.0 INFRASTRUCTURE"; 
        active_vehicle_profile.network_generation = SERIES_PQ35_46_LEGACY; 
    }
    else if (strcmp(chassis, "3G") == 0 || strcmp(chassis, "CB") == 0) { 
        active_vehicle_profile.model_name = "VW Passat B8 (MQB)"; 
        active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; 
        active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; 
    }
    else if (strcmp(chassis, "A3") == 0) { 
        active_vehicle_profile.model_name = "VW Passat B9 (MQB EVO)"; 
        active_vehicle_profile.electrical_bus = "MQB EVO HIGH-SPEED CAN"; 
        active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; 
    }
    else if (strcmp(chassis, "13") == 0) { 
        active_vehicle_profile.model_name = "VW Scirocco Coupe"; 
        active_vehicle_profile.electrical_bus = "CAN-TP2.0 INTERFACE"; 
        active_vehicle_profile.network_generation = SERIES_PQ35_46_LEGACY; 
    }
    else if (strcmp(chassis, "5N") == 0) { 
        active_vehicle_profile.model_name = "VW Tiguan SUV (Mk1)"; 
        active_vehicle_profile.electrical_bus = "CAN-TP2.0 INTERFACE"; 
        active_vehicle_profile.network_generation = SERIES_PQ35_46_LEGACY; 
    }

    else if (strcmp(chassis, "AD") == 0 || strcmp(chassis, "AX") == 0) { 
        active_vehicle_profile.model_name = "VW Tiguan Mk2 (MQB)"; 
        active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; 
        active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; 
    }
    else if (strcmp(chassis, "CT") == 0) { 
        active_vehicle_profile.model_name = "VW Tiguan Mk3 (MQB EVO)"; 
        active_vehicle_profile.electrical_bus = "MQB EVO HIGH-SPEED CAN"; 
        active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; 
    }
    else if (strcmp(chassis, "6R") == 0 || strcmp(chassis, "6C") == 0) { 
        active_vehicle_profile.model_name = "VW Polo Hatchback (PQ25)"; 
        active_vehicle_profile.electrical_bus = "CAN-TP2.0 COMPACT"; 
        active_vehicle_profile.network_generation = SERIES_SMALL_PO_SKODA; 
    }
    else if (strcmp(chassis, "AW") == 0) { 
        active_vehicle_profile.model_name = "VW Polo GTI / Hatch (MQB A0)"; 
        active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; 
        active_vehicle_profile.network_generation = SERIES_SMALL_PO_SKODA; 
    }
    else if (strcmp(chassis, "3H") == 0) { 
        active_vehicle_profile.model_name = "VW Arteon Fastback Coupe"; 
        active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; 
        active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; 
    }

    // --- SEAT / CUPRA DIVISION MAPPINGS ---
    else if (strcmp(chassis, "1P") == 0) { 
        active_vehicle_profile.model_name = "Seat Leon Cupra (Mk2 PQ35)"; 
        active_vehicle_profile.electrical_bus = "CAN-TP2.0 POWERTRAIN"; 
        active_vehicle_profile.network_generation = SERIES_PQ35_46_LEGACY; 
    }
    else if (strcmp(chassis, "5F") == 0) { 
        active_vehicle_profile.model_name = "Seat Leon FR / Cupra (Mk3 MQB)"; 
        active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; 
        active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; 
    }
    else if (strcmp(chassis, "KL") == 0) { 
        active_vehicle_profile.model_name = "Cupra Leon / Formentor (MQB EVO)"; 
        active_vehicle_profile.electrical_bus = "MQB EVO HIGH-SPEED CAN"; 
        active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; 
    }
    else if (strcmp(chassis, "KJ") == 0) { 
        active_vehicle_profile.model_name = "Seat Ibiza / Arona (MQB A0)"; 
        active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; 
        active_vehicle_profile.network_generation = SERIES_SMALL_PO_SKODA; 
    }

    // --- SKODA DIVISION MAPPINGS ---
    else if (strcmp(chassis, "1Z") == 0) { 
        active_vehicle_profile.model_name = "Skoda Octavia vRS (Mk2 PQ35)"; 
        active_vehicle_profile.electrical_bus = "CAN-TP2.0 POWERTRAIN"; 
        active_vehicle_profile.network_generation = SERIES_PQ35_46_LEGACY; 
    }
    else if (strcmp(chassis, "5E") == 0) { 
        active_vehicle_profile.model_name = "Skoda Octavia vRS (Mk3 MQB)"; 
        active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; 
        active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; 
    }
    else if (strcmp(chassis, "NX") == 0) { 
        active_vehicle_profile.model_name = "Skoda Octavia vRS (Mk4 MQB EVO)"; 
        active_vehicle_profile.electrical_bus = "MQB EVO HIGH-SPEED CAN"; 
        active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; 
    }
    else if (strcmp(chassis, "3T") == 0) { 
        active_vehicle_profile.model_name = "Skoda Superb Saloon (3T)"; 
        active_vehicle_profile.electrical_bus = "CAN-TP2.0 INFRASTRUCTURE"; 
        active_vehicle_profile.network_generation = SERIES_PQ35_46_LEGACY; 
    }
    else if (strcmp(chassis, "3V") == 0) { 
        active_vehicle_profile.model_name = "Skoda Superb (MQB Matrix)"; 
        active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; 
        active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; 
    }

    // --- PORSCHE DIVISION MAPPINGS ---
    else if (strcmp(chassis, "92") == 0) { 
        active_vehicle_profile.model_name = "Porsche Cayenne SUV (92A)"; 
        active_vehicle_profile.electrical_bus = "MLB-INFRASTRUCTURE CAN"; 
        active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; 
    }
    else if (strcmp(chassis, "9B") == 0) { 
        active_vehicle_profile.model_name = "Porsche Macan Crossover (95B)"; 
        active_vehicle_profile.electrical_bus = "MLB-INFRASTRUCTURE CAN"; 
        active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; 
    }
    else {
        active_vehicle_profile.model_name = "GENERIC MODEL ARCHITECTURE";
        active_vehicle_profile.electrical_bus = "STANDARD INFRASTRUCTURE CAN";
        active_vehicle_profile.network_generation = SERIES_UNKNOWN;
    }

    // =========================================================================
    // 3. UNIVERSAL MODEL YEAR LOOKUP ARRAY - POSITION 10 (1-based VIN index = vin[9])
    // =========================================================================
    char year_char = vin[9];
    active_vehicle_profile.production_year = 0; // Baseline safety state

    // 1-to-1 sequential string index covering years 2001 through 2030 perfectly
    const char year_tokens[] = "123456789ABCDEFGHJKLMNPRSTVWXY";
    const char* match = strchr(year_tokens, year_char);
    if (match != nullptr) {
        int token_index = match - year_tokens;
        active_vehicle_profile.production_year = 2001 + token_index;
    }

    // =========================================================================
    // 4. PRINT METADATA OUTPUT TO THE TERMINAL BUFFER
    // =========================================================================
    Serial.println("\n=======================================================");
    Serial.println("         DECODED VEHICLE TELEMETRY PROFILE             ");
    Serial.println("=======================================================");
    Serial.print("  MANUFACTURER ORIGIN : "); Serial.println(active_vehicle_profile.brand);
    Serial.print("  DESIGN PLATFORM LINE: "); Serial.println(active_vehicle_profile.model_name);
    Serial.print("  ELECTRICAL BUS TYPE : "); Serial.println(active_vehicle_profile.electrical_bus);
    Serial.print("  PRODUCTION YEAR     : "); Serial.println(active_vehicle_profile.production_year);
    Serial.println("=======================================================\n");

    // =========================================================================
    // 5. THE GLOBAL VAG SYSTEM DYNAMIC CLASS CONSTRUCTOR ROUTER MATRIX
    // =========================================================================
    // C-1/H-3: Take the interpreter mutex for the entire delete/new block.
    //          Core 1 holds this same mutex while calling interpreter methods,
    //          so we cannot delete the object while it is in use.
    //          Writes to active_vehicle_profile (brand, model_name, etc.) above
    //          are also protected by taking the mutex here before this function
    //          is called from loop() — the caller acquires it first (see loop()).
    if (g_interpreter_mutex != NULL) xSemaphoreTake(g_interpreter_mutex, portMAX_DELAY);

    if (sys_ctx->interpreter != nullptr) {
        delete sys_ctx->interpreter;
        sys_ctx->interpreter = nullptr;
    }

    // --- GROUP 1: MQB & MQB-EVO HIGH-SPEED TRANSLATION CLASS MATRIX ---
    if (active_vehicle_profile.network_generation == SERIES_MQB_A_CLASS) {
        if (strcmp(chassis, "8V") == 0)       sys_ctx->interpreter = new AudiS38VInterpreter();
        else if (strcmp(chassis, "GY") == 0 || strcmp(chassis, "8Y") == 0)  sys_ctx->interpreter = new AudiRS3GYInterpreter();
        else if (strcmp(chassis, "5G") == 0 || strcmp(chassis, "BA") == 0 || strcmp(chassis, "AM") == 0 || strcmp(chassis, "AU") == 0) sys_ctx->interpreter = new VwGolf7Interpreter();
        else if (strcmp(chassis, "CD") == 0)  sys_ctx->interpreter = new VwGolf8Interpreter();
        else if (strcmp(chassis, "3G") == 0 || strcmp(chassis, "CB") == 0) sys_ctx->interpreter = new VwPassatB8Interpreter();
        else if (strcmp(chassis, "A3") == 0)  sys_ctx->interpreter = new VwPassatB9Interpreter();
        else if (strcmp(chassis, "AD") == 0 || strcmp(chassis, "AX") == 0) sys_ctx->interpreter = new VwTiguanMk2Interpreter();
        else if (strcmp(chassis, "CT") == 0)  sys_ctx->interpreter = new VwTiguanMk3Interpreter();
        else if (strcmp(chassis, "3H") == 0)  sys_ctx->interpreter = new VwArteonInterpreter();
        else if (strcmp(chassis, "5F") == 0)  sys_ctx->interpreter = new SeatLeonMk3Interpreter();
        else if (strcmp(chassis, "KL") == 0)  sys_ctx->interpreter = new CupraLeonFormentorInterpreter();
        else if (strcmp(chassis, "5E") == 0)  sys_ctx->interpreter = new SkodaOctaviaMk3Interpreter();
        else if (strcmp(chassis, "NX") == 0)  sys_ctx->interpreter = new SkodaOctaviaMk4Interpreter();
        else if (strcmp(chassis, "3V") == 0)  sys_ctx->interpreter = new SkodaSuperbMQBInterpreter();
        else if (strcmp(chassis, "F3") == 0)  sys_ctx->interpreter = new AudiQ3MQBInterpreter();
        else if (strcmp(chassis, "8S") == 0)  sys_ctx->interpreter = new AudiTTMk3Interpreter();
        else if (strcmp(chassis, "GA") == 0)  sys_ctx->interpreter = new AudiQ2Interpreter();
        else                                  sys_ctx->interpreter = new GenericVehicleInterpreter();
        Serial.printf("[DECOUPLER] Dynamic Instance Allocation: Group 1 MQB Matrix class loaded for chassis %s\n", chassis);
    } 
    // --- GROUP 2: PQ LEGACY INFRASTRUCTURE TRANSLATION CLASS MATRIX ---
    else if (active_vehicle_profile.network_generation == SERIES_PQ35_46_LEGACY) {
        if (strcmp(chassis, "8P") == 0)       sys_ctx->interpreter = new AudiS38PInterpreter();
        else if (strcmp(chassis, "4F") == 0)  sys_ctx->interpreter = new AudiA6C6Interpreter();
        else if (strcmp(chassis, "8U") == 0)  sys_ctx->interpreter = new AudiQ3PQ35Interpreter();
        else if (strcmp(chassis, "4L") == 0)  sys_ctx->interpreter = new AudiQ74LInterpreter();
        else if (strcmp(chassis, "8J") == 0)  sys_ctx->interpreter = new AudiTTMk2Interpreter();
        else if (strcmp(chassis, "1K") == 0 || strcmp(chassis, "5K") == 0 || strcmp(chassis, "AJ") == 0) sys_ctx->interpreter = new VwGolf56Interpreter();
        else if (strcmp(chassis, "3C") == 0 || strcmp(chassis, "AN") == 0) sys_ctx->interpreter = new VwPassatB67Interpreter();
        else if (strcmp(chassis, "13") == 0)  sys_ctx->interpreter = new VwSciroccoInterpreter();
        else if (strcmp(chassis, "5N") == 0)  sys_ctx->interpreter = new VwTiguanMk1Interpreter();
        else if (strcmp(chassis, "1P") == 0)  sys_ctx->interpreter = new SeatLeonMk2Interpreter();
        else if (strcmp(chassis, "1Z") == 0)  sys_ctx->interpreter = new SkodaOctaviaMk2Interpreter();
        else if (strcmp(chassis, "3T") == 0)  sys_ctx->interpreter = new SkodaSuperb3TInterpreter();
        else                                  sys_ctx->interpreter = new GenericVehicleInterpreter();
        Serial.printf("[DECOUPLER] Dynamic Instance Allocation: Group 2 PQ Legacy class loaded for chassis %s\n", chassis);
    } 
    // --- GROUP 3: MLB LONGITUDINAL INFRASTRUCTURE CLASS MATRIX ---
    else if (active_vehicle_profile.network_generation == SERIES_MLB_LONG_CLASS) {
        if (strcmp(chassis, "8K") == 0)       sys_ctx->interpreter = new AudiA4MLB8KInterpreter();
        else if (strcmp(chassis, "8W") == 0 || strcmp(chassis, "F4") == 0)  sys_ctx->interpreter = new AudiA4MLB8WInterpreter();
        else if (strcmp(chassis, "4G") == 0)  sys_ctx->interpreter = new AudiA6MLBC7Interpreter();
        else if (strcmp(chassis, "4K") == 0)  sys_ctx->interpreter = new AudiA6MLBC8Interpreter();
        else if (strcmp(chassis, "8T") == 0 || strcmp(chassis, "8F") == 0) sys_ctx->interpreter = new AudiA5MLBB8Interpreter();
        else if (strcmp(chassis, "4H") == 0)  sys_ctx->interpreter = new AudiA8MLBD4Interpreter();
        else if (strcmp(chassis, "4N") == 0)  sys_ctx->interpreter = new AudiA8MLBD5Interpreter();
        else if (strcmp(chassis, "8R") == 0)  sys_ctx->interpreter = new AudiQ5MLB8RInterpreter();
        else if (strcmp(chassis, "FY") == 0)  sys_ctx->interpreter = new AudiQ5MLBFYInterpreter();
        else if (strcmp(chassis, "4M") == 0)  sys_ctx->interpreter = new AudiQ7MLB4MInterpreter();
        else if (strcmp(chassis, "92") == 0)  sys_ctx->interpreter = new PorscheCayenne92Interpreter();
        else if (strcmp(chassis, "9B") == 0)  sys_ctx->interpreter = new PorscheMacan9BInterpreter();
        else                                  sys_ctx->interpreter = new GenericVehicleInterpreter();
        Serial.printf("[DECOUPLER] Dynamic Instance Allocation: Group 3 MLB Longitudinal class loaded for chassis %s\n", chassis);
    }
    // --- GROUP 4: SMALL PO/SKODA COMPACT TRANSLATION CLASS MATRIX ---
    else if (active_vehicle_profile.network_generation == SERIES_SMALL_PO_SKODA) {
        if (strcmp(chassis, "8X") == 0)       sys_ctx->interpreter = new AudiA1PQ25Interpreter();
        else if (strcmp(chassis, "GB") == 0)  sys_ctx->interpreter = new AudiA1MQBA0Interpreter();
        else if (strcmp(chassis, "6R") == 0 || strcmp(chassis, "6C") == 0) sys_ctx->interpreter = new VwPoloPQ25Interpreter();
        else if (strcmp(chassis, "AW") == 0)  sys_ctx->interpreter = new VwPoloMQBA0Interpreter();
        else if (strcmp(chassis, "KJ") == 0)  sys_ctx->interpreter = new SeatIbizaMQBA0Interpreter();
        else                                  sys_ctx->interpreter = new GenericVehicleInterpreter();
        Serial.printf("[DECOUPLER] Dynamic Instance Allocation: Group 4 Compact platform class loaded for chassis %s\n", chassis);
    }
    // --- DEFAULT BENCH FALLBACK INTERFACE ---
    else {
        sys_ctx->interpreter = new GenericVehicleInterpreter();
        Serial.println("[DECOUPLER] Dynamic Instance Allocation: Reverted to Generic Baseline Interface.");
    }

    if (g_interpreter_mutex != NULL) xSemaphoreGive(g_interpreter_mutex);
}


void startTwaiChannel(int port_idx, int tx_pin, int rx_pin) {
  twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)tx_pin, (gpio_num_t)rx_pin, TWAI_MODE_NORMAL);
  
  // Explicitly map target layout channel assignment to hardware registers
  g_cfg.controller_id = port_idx;
  // Increase the RX queue from the default 5 frames to 64 to absorb burst
  // traffic on a live 500 Kbit/s vehicle CAN bus without losing frames.
  g_cfg.rx_queue_len  = 64;
  
  twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_cfg;

  // --- DYNAMIC HARDWARE ACCEPTANCE FILTERING ---
  if (port_idx == 0) {
      // M-3: Channel 0 carries both powertrain frames (0x000–0x3FF) and UDS
      //      diagnostic responses (0x7E8–0x7EF).  The two ranges are too far
      //      apart to express with a single SJA1000-compatible mask without
      //      inadvertently admitting unrelated IDs.  Accept-all is used and
      //      irrelevant IDs are dropped in software by the parser switch-cases.
      //
      //      The RX queue is now set to 64 entries (above) to absorb burst
      //      traffic without overrun.
      f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  } 
  else if (port_idx == 1) {
      // Channel 1: Comfort/Convenience bus — hardware-filter to 0x300–0x3FF.
      //
      // SJA1000 filter convention: mask bit = 1 → don't care; bit = 0 → must
      // match the corresponding acceptance_code bit.
      //
      // The top-3 bits of the 11-bit ID for 0x300–0x3FF are always 0b011 (hex
      // 0x3xx).  Setting those 3 bits in acceptance_code and clearing them in the
      // mask constrains the hardware to the entire 0x300–0x3FF block.  The
      // remaining 8 lower ID bits are left as don't-care (mask bits = 1) so that
      // every ID in the block is admitted — this is intentional and correct.
      f_cfg.acceptance_code = (0x300U << CAN_FILTER_ID_SHIFT);
      f_cfg.acceptance_mask = ~(0x700U << CAN_FILTER_ID_SHIFT);
      f_cfg.single_filter = true;
  } 
  else if (port_idx == 2) {
      // Channel 2: Infotainment bus — hardware-filter to 0x500–0x5FF.
      // Same three-bit masking strategy as Channel 1; top bits = 0b101 (0x5xx).
      // Lower 8 bits are intentionally don't-care to admit the full block.
      f_cfg.acceptance_code = (0x500U << CAN_FILTER_ID_SHIFT);
      f_cfg.acceptance_mask = ~(0x700U << CAN_FILTER_ID_SHIFT);
      f_cfg.single_filter = true;
  } 
  else {
      f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  }
  
  // L-4: Check the driver install return value; a silently ignored failure
  //      leaves twai_ports[port_idx] uninitialised, causing a crash on start.
  if (twai_driver_install_v2(&g_cfg, &t_cfg, &f_cfg, &twai_ports[port_idx]) != ESP_OK) {
      Serial.printf("[CRITICAL] Failed to install CAN Channel %d driver. Halting channel.\n", port_idx);
      return;
  }

  if (twai_start_v2(twai_ports[port_idx]) == ESP_OK) {
      Serial.printf("[SYSTEM] CAN Channel %d (TX:%d, RX:%d) safely isolated and started.\n", port_idx, tx_pin, rx_pin);
  } else {
      Serial.printf("[CRITICAL] Failed to activate hardware registers for CAN Channel %d\n", port_idx);
  }
}


 // -------------------------------------------------------------
// RAW NETWORK STREAM RECEPTION & VAG-SCALING TRANSLATION
// -------------------------------------------------------------
void processInboundFrames(int port_idx, const char* networkName) {
    twai_message_t msg;
    if (twai_receive_v2(twai_ports[port_idx], &msg, 0) == ESP_OK) {

        // M-4: Per-frame hex logging is high-bandwidth and causes UART contention
        //      between cores.  Enable only during bench-level debugging.
#ifdef DEBUG_CAN_FRAMES
        Serial.print("["); Serial.print(networkName); Serial.print("] ID: 0x");
        Serial.print(msg.identifier, HEX); Serial.print(" | HEX DATA PAYLOAD: ");
        for(int i = 0; i < msg.data_length_code; i++) {
            if(msg.data[i] < 0x10) Serial.print("0");
            Serial.print(msg.data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
#endif

        // C-1/H-3: Take the interpreter mutex before touching sys_ctx->interpreter
        //          and active_vehicle_profile to prevent use-after-free on Core 0
        //          delete/replace during VIN decode.
        //
        // C-4: The metrics spinlock is acquired per-dispatch branch to keep the
        //      critical section as short as possible.  The platform parse functions
        //      write to sys_ctx->metrics inside this critical section — they do NOT
        //      need their own spinlock calls because they already run here under it.
        if (g_interpreter_mutex != NULL) xSemaphoreTake(g_interpreter_mutex, portMAX_DELAY);
        if (sys_ctx->interpreter != nullptr && active_vehicle_profile.network_generation != SERIES_UNKNOWN) {
            if (port_idx == 0) {
                portENTER_CRITICAL(&g_metrics_mux);
                sys_ctx->interpreter->interpretDriveTrain(msg);
                portEXIT_CRITICAL(&g_metrics_mux);
            } else if (port_idx == 1) {
                portENTER_CRITICAL(&g_metrics_mux);
                sys_ctx->interpreter->interpretComfort(msg);
                portEXIT_CRITICAL(&g_metrics_mux);
            } else if (port_idx == 2) {
                portENTER_CRITICAL(&g_metrics_mux);
                sys_ctx->interpreter->interpretInfotainment(msg);
                portEXIT_CRITICAL(&g_metrics_mux);
            }
        }
        if (g_interpreter_mutex != NULL) xSemaphoreGive(g_interpreter_mutex);
    }
}

    // -------------------------------------------------------------// ASYNCHRONOUS ACOUSTIC AUDIO ENGINE (NON-BLOCKING)// -------------------------------------------------------------
void runAcousticAlertEngine() {
  // Pull thresholds using the system context metrics fields with pointer arrows (->)
  if (sys_ctx->metrics.oil_temp > MAX_SAFE_OIL_TEMP || sys_ctx->metrics.coolant_temp > MAX_SAFE_COOLANT_TEMP) {
    alarm_sounding = true;
    if (millis() - last_beep_time > 600) {
      last_beep_time = millis();
      tone(AUDIO_PWM_PIN, 2500, 150);
      Serial.println("[SAFETY ALERT] Thermal limit breached! Active warning sound output.");
    }
  } 
  else {
    if (alarm_sounding) {
      noTone(AUDIO_PWM_PIN);
      digitalWrite(AUDIO_PWM_PIN, LOW);
      alarm_sounding = false;
    }
  }
}
