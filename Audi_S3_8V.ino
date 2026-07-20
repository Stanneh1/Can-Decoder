#include "driver/twai.h"
#include <lvgl.h>


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
// --- WEB TRANSMISSION THREAD-SAFETY COUPLING ---

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// --- THREAD-SAFE FIXED CHAR BUFFER ARRAY ---
static char global_ws_buffer[256]; // Allocates a fixed memory space block
static volatile bool ws_payload_ready = false;

// --- ADVANCED VEHICLE DECODING BASES ---
enum MqbPlatformSeries {
    SERIES_UNKNOWN,
    SERIES_MQB_A_CLASS,    // Audi A3 8V/GY, Golf Mk7/Mk8, Leon, Octavia, Tiguan Mk2, etc.
    SERIES_MLB_LONG_CLASS, // Audi A4/A5/A6/A7/A8/Q5/Q7/Q8 Longitudinals (B8, B9, C7, C8)
    SERIES_PQ35_46_LEGACY, // Golf Mk5/Mk6, Audi A3 8P, Passat B6/B7, Scirocco
    SERIES_SMALL_PO_SKODA  // Polo, Ibiza, Fabia, Audi A1
};

struct DecodedVehicleMetrics {
    const char* brand = "VAG MOTOR CORP";
    const char* model_name = "GENERIC MODEL ARCHITECTURE";
    const char* electrical_bus = "STANDARD INFRASTRUCTURE CAN";
    int production_year = 0;
    MqbPlatformSeries network_generation = SERIES_UNKNOWN;
} active_vehicle_profile;

// --- WI-FI ACCESS POINT CREDENTIALS ---
const char* ap_ssid = "Audi_S3_Telemetry";
const char* ap_password = "Password123";
// Network Web Interface Handles
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
static uint32_t last_web_broadcast = 0;

// --- EMBEDDED DASHBOARD HTML PAGE ---
const char index_html[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Audi S3 8V Live Display</title>
    <style>
        body { background: #111; color: #fff; font-family: sans-serif; text-align: center; margin: 0; padding: 20px; }
        .grid { display: flex; flex-wrap: wrap; justify-content: center; gap: 20px; max-width: 1000px; margin: 0 auto; }
        .card { background: #222; border-radius: 15px; padding: 20px; min-width: 200px; box-shadow: 0 4px 10px rgba(0,0,0,0.5); position: relative; }
        .value { font-size: 2.5em; font-weight: bold; margin: 10px 0; transition: color 0.2s; }
        .label { font-size: 0.9em; color: #888; text-transform: uppercase; }
        .redline { color: #ff3e3e !important; animation: blink 0.3s infinite; }
        .normal { color: #32c832; }
        .cool { color: #0096ff; }
        button { background: #444; color: #fff; border: none; padding: 8px 15px; border-radius: 5px; cursor: pointer; margin-top: 10px; }
        button:hover { background: #555; }
        @keyframes blink { 0%, 100% { opacity: 1; } 50% { opacity: 0.5; } }
    </style>
</head>
<body>
    <h1 id="car_banner">VAG MQB TELEMETRY LINK</h1>
    <div class="grid">
        <div class="card"><div class="label">Engine Speed</div><div id="rpm" class="value normal">0</div><div class="label">RPM</div></div>
        <div class="card"><div class="label">Turbo Boost</div><div id="boost" class="value normal">0.00</div><div class="label">Bar</div><button onclick="resetPeak()">Reset Peak (<span id="peak">0.00</span>)</button></div>
        <div class="card"><div class="label">Engine Oil</div><div id="oil" class="value cool">0</div><div class="label">&deg;C</div></div>
        <div class="card"><div class="label">Coolant Temp</div><div id="coolant" class="value cool">0</div><div class="label">&deg;C</div></div>
    </div>
    <script>
        var gateway = `ws://${window.location.hostname}/ws`;
        var websocket;
        
        window.addEventListener('load', initWebSocket);

        function initWebSocket() {
            console.log("Attempting WebSocket linkage...");
            websocket = new WebSocket(gateway);
            websocket.onopen = onOpen;
            websocket.onclose = onClose;
            websocket.onmessage = onMessage;
        }

        function onOpen(event) {
            console.log("WebSocket Connection Verified OPEN.");
        }

        function onClose(event) {
            console.log("Connection closed abnormally. Re-linking in 2 seconds...");
            setTimeout(initWebSocket, 2000); // Auto-reconnect safety loop
        }

        function onMessage(event) {
            try {
                var data = JSON.parse(event.data);
                if(data.car) document.getElementById('car_banner').innerText = data.car;
                document.getElementById('rpm').innerText = data.rpm.toFixed(0);
                document.getElementById('rpm').className = (data.rpm >= 6500) ? "value redline" : "value normal";
                document.getElementById('boost').innerText = data.boost.toFixed(2);
                document.getElementById('peak').innerText = data.peak.toFixed(2);
                document.getElementById('oil').innerText = data.oil;
                document.getElementById('oil').className = (data.oil < 75) ? "value cool" : ((data.oil <= 115) ? "value normal" : "value redline");
                document.getElementById('coolant').innerText = data.h2o;
                document.getElementById('coolant').className = (data.h2o < 70) ? "value cool" : ((data.h2o <= 105) ? "value normal" : "value redline");
            } catch(e) {
                console.error("Data packet format error", e);
            }
        }

        function resetPeak() { 
            // FIX: Gated check to ensure the channel state is fully open before sending strings
            if (websocket && websocket.readyState === WebSocket.OPEN) {
                websocket.send("RESET_PEAK"); 
            } else {
                console.warn("Touch ignored: WebSocket is still initializing link state.");
            }
        }
    </script>
</body>
</html>
)rawhtml";


// Core Handles for the V2 Multi-Port API
twai_handle_t twai_ports[3];

// --- TELEMETRY RECORD STRUCTURES ---
struct VehicleData {
  float engine_rpm = 0;
  float boost_bar = 0;
  float peak_boost_bar = 0; 
  int oil_temp = 0;
  int coolant_temp = 0;
  bool driver_door_open = false;
  float target_temp = 22.0;
  int mmi_key_code = 0;
} s3_live_metrics;

// --- DYNAMIC GRAPHICAL UI OBJECT POINTERS ---
static lv_obj_t *tv; // Tabview base container
static lv_obj_t *rpm_meter;
static lv_obj_t *boost_meter;
static lv_obj_t *oil_arc;
static lv_obj_t *coolant_arc;

static lv_obj_t *lbl_rpm_val;
static lv_obj_t *lbl_boost_val;
static lv_obj_t *lbl_temps_val;
static lv_obj_t *label_comfort;
static lv_obj_t *label_infotainment;

// --- COLOR AND TIMER VARIABLES ---
static lv_color_t color_cool_blue;
static lv_color_t color_normal_green;
static lv_color_t color_alert_red;

static uint32_t last_beep_time = 0;
static bool alarm_sounding = false;

// Forward Declarations for LVGL Touch Callbacks
static void handleBoostResetTouch(lv_event_t * e);

// --- DISPLAY BUFFER BLOCK ALLOCATION FOR LVGL ---
// #define DISP_HOR_RES 800  // Set this to your specific Waveshare screen width
// #define DISP_VER_RES 480  // Set this to your specific Waveshare screen height

// --- DISPLAY RESOLUTION ADJUSTED FOR LANDSCAPE ROTATION ---
#define DISP_HOR_RES 1280 // Becomes your horizontal width when turned sideways
#define DISP_VER_RES 720  // Becomes your vertical height when turned sideways

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[DISP_HOR_RES * 10]; 
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

// Forward declaration of the custom thread runner
void CockpitCoreProcessor(void *pvParameters) {
  Serial.println("[SYSTEM] High-Memory Telemetry Task Thread bound to Core 1 successfully.");
  
  for(;;) {
    lv_timer_handler(); 
    
    processInboundFrames(0, "DRIVE TRAIN");
    processInboundFrames(1, "COMFORT");
    processInboundFrames(2, "INFOTAINMENT");
    
    updateUIElements();
    runAcousticAlertEngine();

    // Independent 100ms pacing timer for the web data flag
        // ASYNCHRONOUS TELEMETRY WEB STREAM OVERLAY (100ms / 10Hz)
    static uint32_t last_timer_tick = 0;
    if (millis() - last_timer_tick > 100) { 
      last_timer_tick = millis();
      
      // Only write to the buffer if Core 0 has dispatched the previous packet
      if (!ws_payload_ready) { 
        static JsonDocument doc; 
        doc.clear(); 
        
        doc["rpm"] = s3_live_metrics.engine_rpm;
        doc["boost"] = s3_live_metrics.boost_bar;
        doc["peak"] = s3_live_metrics.peak_boost_bar;
        doc["oil"] = s3_live_metrics.oil_temp;
        doc["h2o"] = s3_live_metrics.coolant_temp;
        doc["car"] = active_vehicle_profile.model_name;
        
        // FIX: Serialize directly into the fixed character array without dynamic memory growth
        serializeJson(doc, global_ws_buffer, sizeof(global_ws_buffer));
        
        ws_payload_ready = true; // Signal Core 0 that data is ready to send
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(1)); 
  }
}


void setup() {
  Serial.begin(921600);
  while(!Serial) delay(10);
  Serial.println("\n=== PILOT COCKPIT SAFETY PLATFORM INITIALISING ===");

  pinMode(AUDIO_PWM_PIN, OUTPUT);
  digitalWrite(AUDIO_PWM_PIN, LOW);

  // 1. RUN HARDWARE TRANSCIEVER ELECTRICAL DIAGNOSTICS ON BOOT
  bool system_safe = true;
  system_safe &= runBootDiagnostic(0, CH0_TX, CH0_RX, "Drive Train Transceiver");
  system_safe &= runBootDiagnostic(1, CH1_TX, CH1_RX, "Comfort Transceiver");
  system_safe &= runBootDiagnostic(2, CH2_TX, CH2_RX, "Infotainment Transceiver");

  if (!system_safe) {
    Serial.println("\n[CRITICAL ERROR] Transceiver hardware diagnostic check failed!");
    while(1) delay(1000); 
  }

  // 2. PRODUCTION TWAI INTERFACE STARTUP
  startTwaiChannel(0, CH0_TX, CH0_RX);
  startTwaiChannel(1, CH1_TX, CH1_RX);
  startTwaiChannel(2, CH2_TX, CH2_RX);

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
  WiFi.softAP(ap_ssid, ap_password);
  Serial.print("Access Point Launched. Connect to: "); Serial.println(ap_ssid);
  Serial.print("Dashboard Web URL Address: http://");  Serial.println(WiFi.softAPIP());

  // Bind and Link Web Server Routes
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  server.begin();


  // 3. GRAPHICS DISPLAY & TOUCH ENVIRONMENT ENVIRONMENT BINDING
  lv_init();

  // Initialize the drawing buffer structure safely
  lv_disp_draw_buf_init(&draw_buf, buf1, NULL, DISP_HOR_RES * 10);

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
    "CockpitTask",            // Descriptive tag for memory tracking logs
    32768,                    // Allocates a massive 32KB stack layout frame 
    NULL,                     // Task parameters input
    1,                        // Task Priority level execution ranking
    &CockpitTaskHandle,       // Thread handle tracking variable
    1                         // Pin the entire execution timeline strictly to Core 1
  );
  
  Serial.println("=== SYSTEM PRE-FLIGHT INITIALIZATION COMPLETED CLEANLY ===");
}

void loop() {
  static uint32_t last_cleanup = 0;
  if (millis() - last_cleanup > 1000) {
    last_cleanup = millis();
    ws.cleanupClients(); 
  }

  // If Core 1 has marked a packet as ready, transmit it safely here
  if (ws_payload_ready) {
    if (ws.count() > 0 && ws.availableForWriteAll()) {
      
      // FIX: Transmit the static character array cleanly as a raw text string
      ws.textAll(global_ws_buffer); 
    }
    ws_payload_ready = false; // Reset the flag so Core 1 can write the next frame
  }
  
  delay(1); 
}




// -------------------------------------------------------------
// RADIAL DASHBOARD CONTEXT MATRIX (LVGL ENGINE BUILD)
// -------------------------------------------------------------
void buildCockpitUI() {
  // Construct Custom Theme Profiles
  color_cool_blue = lv_color_make(0, 150, 255);
  color_normal_green = lv_color_make(50, 200, 50);
  color_alert_red = lv_color_make(255, 30, 30);

  // Create Horizontal 3-Tab Touch Panel Layout
  tv = lv_tabview_create(lv_scr_act(), LV_DIR_TOP, 40);
  lv_obj_t *tab1 = lv_tabview_add_tab(tv, "PERFORMANCE DIALS");
  lv_obj_t *tab2 = lv_tabview_add_tab(tv, "BODY & COMFORT");
  lv_obj_t *tab3 = lv_tabview_add_tab(tv, "RAW TRAFFIC MON");

  // --- 1. TACHOMETER DESIGN ELEMENT (RPM RADIAL SWEEP) ---
  rpm_meter = lv_arc_create(tab1);
  lv_obj_set_size(rpm_meter, 150, 150);
  lv_arc_set_rotation(rpm_meter, 135);
  lv_arc_set_bg_angles(rpm_meter, 0, 270);
  lv_arc_set_range(rpm_meter, 0, 8000); 
  lv_obj_align(rpm_meter, LV_ALIGN_LEFT_MID, 20, -20);
  lv_obj_remove_style(rpm_meter, NULL, LV_PART_KNOB); 

  lbl_rpm_val = lv_label_create(tab1);
  lv_obj_align_to(lbl_rpm_val, rpm_meter, LV_ALIGN_CENTER, 0, 0);
  lv_label_set_text(lbl_rpm_val, "0\nRPM");

  // --- 2. TURBOCHARGER PRESSURE ELEMENT (VERTICAL STATUS BAR) ---
  boost_meter = lv_bar_create(tab1);
  lv_obj_set_size(boost_meter, 30, 130);
  lv_bar_set_range(boost_meter, 0, 250); // Maps 0.00 to 2.50 bar absolute boost limits
  lv_obj_align(boost_meter, LV_ALIGN_CENTER, 0, -20);

  lbl_boost_val = lv_label_create(tab1);
  lv_obj_align_to(lbl_boost_val, boost_meter, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
  lv_label_set_text(lbl_boost_val, "0.00 Bar\nPK: 0.00 B");
  
  // Make the label interactive to clear Peak history memory when tapped
  lv_obj_add_flag(lbl_boost_val, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(lbl_boost_val, handleBoostResetTouch, LV_EVENT_CLICKED, NULL);

  // --- 3. DUAL THERMAL PASS METER (SPLIT OVERLAPPING COCKPIT ARCS) ---
  // Outer Layer Radial: Engine Oil Temperature Status
  oil_arc = lv_arc_create(tab1);
  lv_obj_set_size(oil_arc, 150, 150);
  lv_arc_set_rotation(oil_arc, 180);
  lv_arc_set_bg_angles(oil_arc, 0, 180); 
  lv_arc_set_range(oil_arc, 40, 150);    
  lv_obj_align(oil_arc, LV_ALIGN_RIGHT_MID, -20, -20);
  lv_obj_remove_style(oil_arc, NULL, LV_PART_KNOB);

  // Inner Layer Radial: Engine Engine Coolant Line Status
  coolant_arc = lv_arc_create(tab1);
  lv_obj_set_size(coolant_arc, 110, 110);
  lv_arc_set_rotation(coolant_arc, 180);
  lv_arc_set_bg_angles(coolant_arc, 0, 180);
  lv_arc_set_range(coolant_arc, 40, 120);
  lv_obj_align_to(coolant_arc, oil_arc, LV_ALIGN_CENTER, 0, 0);
  lv_obj_remove_style(coolant_arc, NULL, LV_PART_KNOB);

  lbl_temps_val = lv_label_create(tab1);
  lv_obj_align_to(lbl_temps_val, oil_arc, LV_ALIGN_OUT_BOTTOM_MID, 0, 15);
  lv_label_set_text(lbl_temps_val, "Oil: --°C | H2O: --°C");

  // --- TAB BACKLOG MONITORING FIELDS ---
  label_comfort = lv_label_create(tab2);
  lv_obj_align(label_comfort, LV_ALIGN_TOP_LEFT, 10, 10);

  label_infotainment = lv_label_create(tab3);
  lv_obj_align(label_infotainment, LV_ALIGN_TOP_LEFT, 10, 10);
}

// -------------------------------------------------------------
// METRIC EXTRACTION REDRAW LOGIC CYCLE
// -------------------------------------------------------------
void updateUIElements() {
  char buf[128];

  // 1. TACHOMETER REDLINE SWAP LOGIC 
  lv_arc_set_value(rpm_meter, (int)s3_live_metrics.engine_rpm);
  snprintf(buf, sizeof(buf), "%d\nRPM", (int)s3_live_metrics.engine_rpm);
  lv_label_set_text(lbl_rpm_val, buf);

  if (s3_live_metrics.engine_rpm >= 6500.0) {
    lv_obj_set_style_arc_color(rpm_meter, color_alert_red, LV_PART_INDICATOR);
  } else {
    lv_obj_set_style_arc_color(rpm_meter, color_normal_green, LV_PART_INDICATOR);
  }

  // 2. BOOST PRESSURE DISPLAY & HISTORICAL HIGHEST LOG
  if (s3_live_metrics.boost_bar > s3_live_metrics.peak_boost_bar) {
    s3_live_metrics.peak_boost_bar = s3_live_metrics.boost_bar;
  }
  
  lv_bar_set_value(boost_meter, (int)(s3_live_metrics.boost_bar * 100), LV_ANIM_OFF);
  snprintf(buf, sizeof(buf), "%.2f Bar\nPK: %.2f B", s3_live_metrics.boost_bar, s3_live_metrics.peak_boost_bar);
  lv_label_set_text(lbl_boost_val, buf);

  // 3. THERMAL MAP DYNAMIC GRADIENTS
  lv_arc_set_value(oil_arc, s3_live_metrics.oil_temp);
  lv_arc_set_value(coolant_arc, s3_live_metrics.coolant_temp);
  
  // Evaluate Engine Oil
  if (s3_live_metrics.oil_temp < 75) {
    lv_obj_set_style_arc_color(oil_arc, color_cool_blue, LV_PART_INDICATOR);
  } else if (s3_live_metrics.oil_temp >= 75 && s3_live_metrics.oil_temp <= 115) {
    lv_obj_set_style_arc_color(oil_arc, color_normal_green, LV_PART_INDICATOR);
  } else {
    lv_obj_set_style_arc_color(oil_arc, color_alert_red, LV_PART_INDICATOR);
  }

  // Evaluate Coolant System
  if (s3_live_metrics.coolant_temp < 70) {
    lv_obj_set_style_arc_color(coolant_arc, color_cool_blue, LV_PART_INDICATOR);
  } else if (s3_live_metrics.coolant_temp >= 70 && s3_live_metrics.coolant_temp <= 105) {
    lv_obj_set_style_arc_color(coolant_arc, color_normal_green, LV_PART_INDICATOR);
  } else {
    lv_obj_set_style_arc_color(coolant_arc, color_alert_red, LV_PART_INDICATOR);
  }

  snprintf(buf, sizeof(buf), "Oil: %d°C  |  H2O: %d°C", s3_live_metrics.oil_temp, s3_live_metrics.coolant_temp);
  lv_label_set_text(lbl_temps_val, buf);

  // 4. SECONDARY DOMAIN CONVERSION LOG STRINGS
  snprintf(buf, sizeof(buf), 
           "BCM CHASSIS DATA INTERFACE\n\n"
           "Driver Front Door State: %s\n"
           "Target Climatronic Air Volume: %.1f °C", 
           s3_live_metrics.driver_door_open ? "OPEN" : "CLOSED", s3_live_metrics.target_temp);
  lv_label_set_text(label_comfort, buf);

  snprintf(buf, sizeof(buf), 
           "LAST CAPTURED MEDIA DATA VECTOR\n\n"
           "MMI Steering Input Vector Code: 0x%X\n"
           "Diagnostic Status Matrix Link: ONLINE", s3_live_metrics.mmi_key_code);
  lv_label_set_text(label_infotainment, buf);
}

// Remote touch data receiver
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  // Strict Null Pointer Safety Guard. Drop execution if invalid.
  if (client == NULL) return; 

  if (type == WS_EVT_DATA) { 
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info != NULL && info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0; 
      
      if (strcmp((char*)data, "RESET_PEAK") == 0) {
        s3_live_metrics.peak_boost_bar = 0.0; 
        
        // FIX: Removed client->id() to prevent string conversion pointer faults
        Serial.println("[WEB EVENT] Peak Turbo metrics zeroed out via remote command.");
      }
    }
  }
  else if (type == WS_EVT_CONNECT) {
    // FIX: Using a clean, unparameterized text string for connection confirmations
    Serial.println("[WEB SERVER] Remote phone/tablet terminal device connected successfully.");
  }
  else if (type == WS_EVT_DISCONNECT) {
    // FIX: Clean unparameterized text logging string for link drops
    Serial.println("[WEB SERVER] Remote phone/tablet terminal device disconnected.");
  }
}

// -------------------------------------------------------------
// EVENT CALLBACK REGISTER PATHS
// -------------------------------------------------------------
static void handleBoostResetTouch(lv_event_t * e) {
  s3_live_metrics.peak_boost_bar = 0.0; 
  Serial.println("[UI EVENT] Historical Peak Boost memory register zeroed out.");
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

    // 1. EXTRACT WORLD MANUFACTURER IDENTIFIER (WMI)
    if (strncmp(vin, "WAU", 3) == 0) active_vehicle_profile.brand = "AUDI AG (GERMANY)";
    else if (strncmp(vin, "TRU", 3) == 0) active_vehicle_profile.brand = "AUDI AG (HUNGARY)";
    else if (strncmp(vin, "WVW", 3) == 0) active_vehicle_profile.brand = "VOLKSWAGEN CARS";
    else if (strncmp(vin, "WVG", 3) == 0) active_vehicle_profile.brand = "VOLKSWAGEN SUV DIVISION";
    else if (strncmp(vin, "VSS", 3) == 0) active_vehicle_profile.brand = "SEAT / CUPRA";
    else if (strncmp(vin, "TMB", 3) == 0) active_vehicle_profile.brand = "SKODA AUTO";
    else if (strncmp(vin, "WP0", 3) == 0) active_vehicle_profile.brand = "PORSCHE STUTTGART";

    // 2. DETECT CHASSIS GENERATION CODE (VIN CHARACTER POSITION 7 & 8)
    char chassis[3] = { vin[6], vin[7], '\0' };
    active_vehicle_profile.network_generation = SERIES_UNKNOWN; // Reset baseline

    // --- AUDI DIVISION ---
    if (strcmp(chassis, "8P") == 0) { active_vehicle_profile.model_name = "Audi A3 / S3 (PQ35 Platform)"; active_vehicle_profile.electrical_bus = "CAN-TP2.0 LEGACY"; active_vehicle_profile.network_generation = SERIES_PQ35_46_LEGACY; }
    else if (strcmp(chassis, "8V") == 0) { active_vehicle_profile.model_name = "Audi A3 / S3 / RS3 (MQB Matrix)"; active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; }
    else if (strcmp(chassis, "GY") == 0) { active_vehicle_profile.model_name = "Audi A3 / S3 / RS3 (MQB EVO 8Y)"; active_vehicle_profile.electrical_bus = "MQB EVO CAN-FD/CAN"; active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; }
    else if (strcmp(chassis, "8K") == 0) { active_vehicle_profile.model_name = "Audi A4 / S4 / RS4 (MLB B8)"; active_vehicle_profile.electrical_bus = "MLB-INFRASTRUCTURE CAN"; active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; }
    else if (strcmp(chassis, "8W") == 0) { active_vehicle_profile.model_name = "Audi A4 / S4 / A5 / RS5 (MLB B9)"; active_vehicle_profile.electrical_bus = "MLB EVO FLEXRAY/CAN"; active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; }
    else if (strcmp(chassis, "4F") == 0) { active_vehicle_profile.model_name = "Audi A6 / S6 / RS6 (C6 Era)"; active_vehicle_profile.electrical_bus = "CAN-TP2.0 INFRASTRUCTURE"; active_vehicle_profile.network_generation = SERIES_PQ35_46_LEGACY; }
    else if (strcmp(chassis, "4G") == 0) { active_vehicle_profile.model_name = "Audi A6 / S6 / A7 / RS7 (MLB C7)"; active_vehicle_profile.electrical_bus = "MLB-INFRASTRUCTURE CAN"; active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; }
    else if (strcmp(chassis, "4K") == 0) { active_vehicle_profile.model_name = "Audi A6 / A7 / RS6 / RS7 (MLB C8)"; active_vehicle_profile.electrical_bus = "MLB EVO FLEXRAY/CAN"; active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; }
    else if (strcmp(chassis, "8T") == 0 || strcmp(chassis, "8F") == 0) { active_vehicle_profile.model_name = "Audi A5 / S5 / RS5 (B8 Chassis)"; active_vehicle_profile.electrical_bus = "MLB-INFRASTRUCTURE CAN"; active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; }
    else if (strcmp(chassis, "4H") == 0) { active_vehicle_profile.model_name = "Audi A8 / S8 (D4 Luxury)"; active_vehicle_profile.electrical_bus = "MLB-INFRASTRUCTURE CAN"; active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; }
    else if (strcmp(chassis, "4N") == 0) { active_vehicle_profile.model_name = "Audi A8 / S8 (D5 Luxury)"; active_vehicle_profile.electrical_bus = "MLB EVO FLEXRAY/CAN"; active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; }
    else if (strcmp(chassis, "8U") == 0) { active_vehicle_profile.model_name = "Audi Q3 Compact SUV (PQ35)"; active_vehicle_profile.electrical_bus = "HIGH-SPEED CAN-TP2.0"; active_vehicle_profile.network_generation = SERIES_PQ35_46_LEGACY; }
    else if (strcmp(chassis, "F3") == 0) { active_vehicle_profile.model_name = "Audi Q3 / RS Q3 (MQB Sport Utility)"; active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; }
    else if (strcmp(chassis, "8R") == 0) { active_vehicle_profile.model_name = "Audi Q5 Crossover (8R)"; active_vehicle_profile.electrical_bus = "MLB-INFRASTRUCTURE CAN"; active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; }
    else if (strcmp(chassis, "FY") == 0) { active_vehicle_profile.model_name = "Audi Q5 / SQ5 (MLB FY)"; active_vehicle_profile.electrical_bus = "MLB EVO FLEXRAY/CAN"; active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; }
    else if (strcmp(chassis, "4M") == 0) { active_vehicle_profile.model_name = "Audi Q7 / SQ7 / Q8 / SQ8 (MLB 4M)"; active_vehicle_profile.electrical_bus = "MLB EVO FLEXRAY/CAN"; active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; }
    else if (strcmp(chassis, "8J") == 0) { active_vehicle_profile.model_name = "Audi TT / TTS / TT RS (Mk2)"; active_vehicle_profile.electrical_bus = "CAN-TP2.0 MOTORWAY"; active_vehicle_profile.network_generation = SERIES_PQ35_46_LEGACY; }
    else if (strcmp(chassis, "8S") == 0) { active_vehicle_profile.model_name = "Audi TT / TTS / TT RS (Mk3 MQB)"; active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; }
    else if (strcmp(chassis, "GA") == 0) { active_vehicle_profile.model_name = "Audi Q2 Compact Crossover"; active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; }
    else if (strcmp(chassis, "8X") == 0) { active_vehicle_profile.model_name = "Audi A1 Supermini (PQ25)"; active_vehicle_profile.electrical_bus = "CAN-TP2.0 COMPACT"; active_vehicle_profile.network_generation = SERIES_SMALL_PO_SKODA; }
    else if (strcmp(chassis, "GB") == 0) { active_vehicle_profile.model_name = "Audi A1 Sportback (MQB A0)"; active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; active_vehicle_profile.network_generation = SERIES_SMALL_PO_SKODA; }

    // --- VOLKSWAGEN DIVISION ---
    else if (strcmp(chassis, "1K") == 0 || strcmp(chassis, "5K") == 0 || strcmp(chassis, "AJ") == 0) { active_vehicle_profile.model_name = "VW Golf Mk5 / Mk6 / Jetta"; active_vehicle_profile.electrical_bus = "CAN-TP2.0 POWERTRAIN"; active_vehicle_profile.network_generation = SERIES_PQ35_46_LEGACY; }
    else if (strcmp(chassis, "5G") == 0 || strcmp(chassis, "BA") == 0 || strcmp(chassis, "AM") == 0) { active_vehicle_profile.model_name = "VW Golf Mk7 / GTI / Golf R (MQB)"; active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; }
    else if (strcmp(chassis, "CD") == 0) { active_vehicle_profile.model_name = "VW Golf Mk8 / GTI / Clubsport / R"; active_vehicle_profile.electrical_bus = "MQB EVO HIGH-SPEED CAN"; active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; }
    else if (strcmp(chassis, "3C") == 0 || strcmp(chassis, "AN") == 0) { active_vehicle_profile.model_name = "VW Passat B6 / B7 / CC"; active_vehicle_profile.electrical_bus = "CAN-TP2.0 INFRASTRUCTURE"; active_vehicle_profile.network_generation = SERIES_PQ35_46_LEGACY; }
    else if (strcmp(chassis, "3G") == 0 || strcmp(chassis, "CB") == 0) { active_vehicle_profile.model_name = "VW Passat B8 (MQB)"; active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; }
    else if (strcmp(chassis, "A3") == 0) { active_vehicle_profile.model_name = "VW Passat B9 (MQB EVO)"; active_vehicle_profile.electrical_bus = "MQB EVO HIGH-SPEED CAN"; active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; }
    else if (strcmp(chassis, "13") == 0) { active_vehicle_profile.model_name = "VW Scirocco Coupe"; active_vehicle_profile.electrical_bus = "CAN-TP2.0 INTERFACE"; active_vehicle_profile.network_generation = SERIES_PQ35_46_LEGACY; }
    else if (strcmp(chassis, "5N") == 0) { active_vehicle_profile.model_name = "VW Tiguan SUV (Mk1)"; active_vehicle_profile.electrical_bus = "CAN-TP2.0 INTERFACE"; active_vehicle_profile.network_generation = SERIES_PQ35_46_LEGACY; }
    else if (strcmp(chassis, "AD") == 0 || strcmp(chassis, "AX") == 0) { active_vehicle_profile.model_name = "VW Tiguan Mk2 (MQB)"; active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; }
    else if (strcmp(chassis, "CT") == 0) { active_vehicle_profile.model_name = "VW Tiguan Mk3 (MQB EVO)"; active_vehicle_profile.electrical_bus = "MQB EVO HIGH-SPEED CAN"; active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; }
    else if (strcmp(chassis, "6R") == 0 || strcmp(chassis, "6C") == 0) { active_vehicle_profile.model_name = "VW Polo Hatchback (PQ25)"; active_vehicle_profile.electrical_bus = "CAN-TP2.0 COMPACT"; active_vehicle_profile.network_generation = SERIES_SMALL_PO_SKODA; }
    else if (strcmp(chassis, "AW") == 0) { active_vehicle_profile.model_name = "VW Polo GTI / Hatch (MQB A0)"; active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; active_vehicle_profile.network_generation = SERIES_SMALL_PO_SKODA; }
    else if (strcmp(chassis, "AN") == 0) { active_vehicle_profile.model_name = "VW T-Roc Sport Crossover"; active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; }
    else if (strcmp(chassis, "3H") == 0) { active_vehicle_profile.model_name = "VW Arteon GranTurismo"; active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; }

    // --- SEAT / CUPRA ---
        else if (strcmp(chassis, "1P") == 0) { active_vehicle_profile.model_name = "Seat Leon Cupra (Mk2 PQ35)"; active_vehicle_profile.electrical_bus = "CAN-TP2.0 POWERTRAIN"; active_vehicle_profile.network_generation = SERIES_PQ35_46_LEGACY; }
    else if (strcmp(chassis, "5F") == 0) { active_vehicle_profile.model_name = "Seat Leon FR / Cupra (Mk3 MQB)"; active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; }
    else if (strcmp(chassis, "KL") == 0) { active_vehicle_profile.model_name = "Cupra Leon / Formentor (Mk4 MQB EVO)"; active_vehicle_profile.electrical_bus = "MQB EVO HIGH-SPEED CAN"; active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; }
    else if (strcmp(chassis, "KJ") == 0) { active_vehicle_profile.model_name = "Seat Ibiza / Arona (MQB A0)"; active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; active_vehicle_profile.network_generation = SERIES_SMALL_PO_SKODA; }
    
    // --- SKODA AUTOMOTIVE ---
    else if (strcmp(chassis, "1Z") == 0) { active_vehicle_profile.model_name = "Skoda Octavia vRS (Mk2 PQ35)"; active_vehicle_profile.electrical_bus = "CAN-TP2.0 POWERTRAIN"; active_vehicle_profile.network_generation = SERIES_PQ35_46_LEGACY; }
    else if (strcmp(chassis, "5E") == 0) { active_vehicle_profile.model_name = "Skoda Octavia vRS (Mk3 MQB)"; active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; }
    else if (strcmp(chassis, "NX") == 0) { active_vehicle_profile.model_name = "Skoda Octavia vRS (Mk4 MQB EVO)"; active_vehicle_profile.electrical_bus = "MQB EVO HIGH-SPEED CAN"; active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; }
    else if (strcmp(chassis, "3T") == 0) { active_vehicle_profile.model_name = "Skoda Superb Saloon (3T)"; active_vehicle_profile.electrical_bus = "CAN-TP2.0 INFRASTRUCTURE"; active_vehicle_profile.network_generation = SERIES_PQ35_46_LEGACY; }
    else if (strcmp(chassis, "3V") == 0) { active_vehicle_profile.model_name = "Skoda Superb (MQB Matrix)"; active_vehicle_profile.electrical_bus = "HIGH-SPEED MQB CAN"; active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS; }
    
    // --- PORSCHE MATRIX ---
    else if (strcmp(chassis, "92") == 0) { active_vehicle_profile.model_name = "Porsche Cayenne SUV (92A)"; active_vehicle_profile.electrical_bus = "MLB-INFRASTRUCTURE CAN"; active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; }
    else if (strcmp(chassis, "9B") == 0) { active_vehicle_profile.model_name = "Porsche Macan Crossover (95B)"; active_vehicle_profile.electrical_bus = "MLB-INFRASTRUCTURE CAN"; active_vehicle_profile.network_generation = SERIES_MLB_LONG_CLASS; }
    
    // 3. MAP MODEL YEAR CHARACTER INDICES (POSITION 10)
    char year_char = vin[9];
    if (year_char >= 'D' && year_char <= 'H') {
       // D=2013, E=2014, F=2015, G=2016, H=2017 
       active_vehicle_profile.production_year = 2013 + (year_char - 'D');
    } 
    else if (year_char >= 'J' && year_char <= 'N') {
       // J=2018, K=2019, L=2020, M=2021, N=2022 (Skips 'I') 
       active_vehicle_profile.production_year = 2018 + (year_char - 'J');
    }
    else if (year_char >= 'P' && year_char <= 'R') {
       // P=2023, R=2024 (Skips 'O', 'Q') 
       active_vehicle_profile.production_year = 2023 + (year_char - 'P');
    }
    else if (year_char == 'S') {
       active_vehicle_profile.production_year = 2025;
    }
    else if (year_char == 'T') {
       active_vehicle_profile.production_year = 2026;
    }

    // Print Detailed Metadata Trace out to the Console Buffer
    Serial.println("\n=======================================================");
    Serial.println("         DECODED VEHICLE TELEMETRY PROFILE             ");
    Serial.println("=======================================================");
    Serial.print("  MANUFACTURER ORIGIN : "); Serial.println(active_vehicle_profile.brand);
    Serial.print("  DESIGN PLATFORM LINE: "); Serial.println(active_vehicle_profile.model_name);
    Serial.print("  ELECTRICAL BUS TYPE : "); Serial.println(active_vehicle_profile.electrical_bus);
    Serial.print("  PRODUCTION YEAR     : "); Serial.println(active_vehicle_profile.production_year);
    Serial.println("=======================================================\n");
  }


void startTwaiChannel(int port_idx, int tx_pin, int rx_pin) {
  twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)tx_pin, (gpio_num_t)rx_pin, TWAI_MODE_NORMAL);
  
  // Explicitly map target layout channel assignment to hardware layout registers
  g_cfg.controller_id = port_idx; 
  
  twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  
  // Removed invalid (twai_port_num_t) typecast
  twai_driver_install_v2(&g_cfg, &t_cfg, &f_cfg, &twai_ports[port_idx]);
  
  // Check hardware initialization status explicitly on boot
  if (twai_start_v2(twai_ports[port_idx]) == ESP_OK) {
      Serial.printf("[SYSTEM] CAN Channel %d (TX:%d, RX:%d) initialized and started successfully.\n", port_idx, tx_pin, rx_pin);
  } else {
      Serial.printf("[CRITICAL] Failed to activate hardware registers for CAN Channel %d\n", port_idx);
  }
}

 // -------------------------------------------------------------// RAW NETWORK STREAM RECEPTION & VAG-SCALING TRANSLATION// -------------------------------------------------------------
 void processInboundFrames(int port_idx, const char* networkName)
  {
    twai_message_t msg;
    if (twai_receive_v2(twai_ports[port_idx], &msg, 0) == ESP_OK)
    {
      // Global Console System Logger Outlay (As requested)Serial.print("[");
       Serial.print(networkName); Serial.print("] ID: 0x");
       Serial.print(msg.identifier, HEX);
       Serial.print(" | HEX DATA PAYLOAD: ");
       for(int i = 0; i < msg.data_length_code; i++)
        {
          if(msg.data[i] < 0x10)
           Serial.print("0");
           Serial.print(msg.data[i], HEX);
           Serial.print(" ");
        }
        
        Serial.print("| -> ");
        // Redirect traffic vectors to corresponding layout decoders
        if (port_idx == 0) 
         decodeDriveTrain(msg);
        else if (port_idx == 1) 
         decodeComfort(msg);
        else if (port_idx == 2) decodeInfotainment(msg);
     }
  }
void decodeDriveTrain(twai_message_t &msg)
{
    switch(msg.identifier)
    {
    case 0x0FC:
        s3_live_metrics.engine_rpm = ((msg.data[1] << 8) | msg.data[0]) * 0.25;
        Serial.print("ENG DESCRIPTION: Engine Crankshaft Speed RPM | ACTUAL NUMBER: ");
        Serial.print(s3_live_metrics.engine_rpm, 1);
        Serial.println(" RPM");
        break;

    case 0x1A2:
        s3_live_metrics.oil_temp = msg.data[0] - 40;
        s3_live_metrics.coolant_temp = msg.data[1] - 40;
        Serial.print("ENG DESCRIPTION: Powertrain Thermal Mass Levels | ACTUAL NUMBER: Oil Temp: ");
        Serial.print(s3_live_metrics.oil_temp);
        Serial.print(" C, Coolant Temp: ");
        Serial.print(s3_live_metrics.coolant_temp);
        Serial.println(" C");
        break;

    case 0x28A:
        { // ◄ Bracket added to open a safe scoped block for local variables
            int raw_mbar = ((msg.data[1] << 8) | msg.data[0]) * 10;
            s3_live_metrics.boost_bar = (raw_mbar - 1013) / 1000.0;
            if (s3_live_metrics.boost_bar < 0) {
                s3_live_metrics.boost_bar = 0;
            }
            Serial.print("ENG DESCRIPTION: Turbocharger Boost Profile Pressure | ACTUAL NUMBER: Net Manifold Boost: ");
            Serial.print(s3_live_metrics.boost_bar, 2);
            Serial.println(" bar");
        } // ◄ Bracket added to cleanly isolate raw_mbar
        break;

    default:
        Serial.println("ENG DESCRIPTION: Base Engine Control Electronics Inter-Module Communication Frame");
        break;
    }
}
    void decodeComfort(twai_message_t &msg) 
    {
      switch(msg.identifier)
      {
        case 0x61C:s3_live_metrics.driver_door_open = msg.data[0] & 0x01;
         Serial.print("CMF DESCRIPTION: Body Control Module Structural Aperture State Matrix | ACTUAL NUMBER: Driver Front: ");
         Serial.println(s3_live_metrics.driver_door_open ? "OPEN" : "CLOSED");
        break;
         case 0x527:s3_live_metrics.target_temp = msg.data[0] * 0.5;
           Serial.print("CMF DESCRIPTION: Climatic Control Cabin Thermal Request Vector | ACTUAL NUMBER: Target Temp: ");
           Serial.print(s3_live_metrics.target_temp, 1);
           Serial.println(" C");
          break;
          default:
           Serial.println("CMF DESCRIPTION: Interior Body Convenience Systems Framework Bus Array Data Line");
          break;
       }
    }
    void decodeInfotainment(twai_message_t &msg) 
    {
      switch(msg.identifier) 
      {
        case 0x695:s3_live_metrics.mmi_key_code = msg.data[0];Serial.print("INF DESCRIPTION: Multi-Media Control Interface Wheel Key Input Matrix Vector | ACTUAL NUMBER: Button Array Hex: ");
         Serial.println(s3_live_metrics.mmi_key_code, HEX);
         break;
         default:
         Serial.println("INF DESCRIPTION: Multimedia Entertainment, High-Frequency Audio, or Sound Stage Processing Output");
         break;
      }
    }
    // -------------------------------------------------------------// ASYNCHRONOUS ACOUSTIC AUDIO ENGINE (NON-BLOCKING)// -------------------------------------------------------------
    void runAcousticAlertEngine() 
    {
      if (s3_live_metrics.oil_temp > MAX_SAFE_OIL_TEMP || s3_live_metrics.coolant_temp > MAX_SAFE_COOLANT_TEMP)
      {
        alarm_sounding = true;
       if (millis() - last_beep_time > 600) {last_beep_time = millis();tone(AUDIO_PWM_PIN, 2500, 150);
       // Sound aggressive 2.5kHz chirp for 150ms
       Serial.println("[SAFETY ALERT] Thermal limit breached! Active warning sound output.");
      }
    } 
    else 
    {
      if (alarm_sounding) {noTone(AUDIO_PWM_PIN);digitalWrite(AUDIO_PWM_PIN, LOW);
       // Kill residual amplifier hiss
       alarm_sounding = false;
    }
  }



}
