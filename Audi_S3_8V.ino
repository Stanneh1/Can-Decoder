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
// --- WI-FI ACCESS POINT CREDENTIALS ---
const char* ap_ssid = "Audi_S3_Telemetry";
const char* ap_password = "VAG_PERFORMANCE";

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
    <h1>AUDI S3 MQB TELEMETRY LINK</h1>
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

  // 2b. ACTIVATE ASYNCHRONOUS COCKPIT HOTSPOT AP NETWORK
  WiFi.softAP(ap_ssid, ap_password);
  Serial.print("Access Point Launched. Connect to: "); Serial.println(ap_ssid);
  Serial.print("Dashboard Web URL Address: http://"); Serial.println(WiFi.softAPIP());

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

// -------------------------------------------------------------
// HARDWARE INITIALIZATION & PRE-FLIGHT TESTERS
// -------------------------------------------------------------
// -------------------------------------------------------------
// HARDWARE INITIALIZATION & PRE-FLIGHT TESTERS (FIXED)
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

 void startTwaiChannel(int port_idx, int tx_pin, int rx_pin) {
  twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)tx_pin, (gpio_num_t)rx_pin, TWAI_MODE_NORMAL);
  
  // Explicitly map target layout channel assignment to hardware layout registers
  g_cfg.controller_id = port_idx; 
  
  twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  
  // Removed invalid (twai_port_num_t) typecast
  twai_driver_install_v2(&g_cfg, &t_cfg, &f_cfg, &twai_ports[port_idx]);
  twai_start_v2(twai_ports[port_idx]);
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
