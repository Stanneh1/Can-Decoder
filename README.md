# Universal CAN Bus Decoder & Telemetry Platform

A modular, extensible real-time CAN bus decoder and vehicle telemetry display system built for ESP32 microcontrollers. Designed to decode, interpret, and visualize automotive CAN bus messages from any CAN-equipped vehicle.

**Current Support:** Volkswagen Group vehicles (Audi, Volkswagen, Skoda, Seat/Cupra, Porsche)  
**Platform Coverage:** 38 Models vehicle models across 4 major electrical architectures (MQB, PQ-Legacy, MLB, Compact A0)  
**Future Support:** Extensible architecture enables support for any CAN-equipped platform

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Project Architecture](#project-architecture)
- [Supported Platforms & Vehicles](#supported-platforms--vehicles)
- [Hardware Requirements](#hardware-requirements)
- [Installation & Setup](#installation--setup)
- [CAN Decoder Architecture](#can-decoder-architecture)
- [Platform-Specific Signal Mappings](#platform-specific-signal-mappings)
- [UI & Display System](#ui--display-system)
- [Web Dashboard](#web-dashboard)
- [Extending to New Platforms](#extending-to-new-platforms)
- [Project Structure & File Organization](#project-structure--file-organization)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

This project implements a **universal CAN bus decoder** that captures raw CAN frames from vehicle electrical systems and translates them into human-readable telemetry metrics. The system features:

- **Multi-platform decoding engine** with 4 distinct electrical architecture interpreters (MQB Matrix, PQ-Legacy, MLB Longitudinal, Compact A0)
- **38 vehicle model coverage** with model-specific signal extraction and UI limits
- **Real-time metric parsing** at 1ms tick resolution across multiple independent CAN buses
- **Dynamic vehicle identification** via VIN decoding (automatically loads correct interpreter)
- **Modular interpreter pattern** - add new vehicles without touching core logic
- **Dual-interface display**: Local LVGL touchscreen UI + wireless web dashboard via WebSocket
- **Safety-critical alarm system** with thermal threshold monitoring and audio alerts
- **Bench testing simulator** for development without a live vehicle

The system is designed from the ground up for **platform-agnostic extensibility**. While currently focused on Volkswagen Group vehicles (which share the VAG electrical architecture), the modular interpreter pattern allows rapid addition of any CAN-equipped manufacturer.

---

## Features

### Core Decoding
- ✅ **Multi-frame CAN message assembly** (ISO-TP protocol for messages >8 bytes)
- ✅ **Platform-specific signal extraction** with byte-level precision
- ✅ **4 unified decoding cores** (MQB, PQ-Legacy, MLB, Compact) eliminate redundant code
- ✅ **Real-time metric updates** at 1ms resolution (Core 1 pinned task)
- ✅ **Bench telemetry simulator** with programmable ramp profiles for testing without vehicle

### Vehicle Platform Support
- ✅ **MQB Matrix** (17 vehicle models) - Modern transverse engines
- ✅ **PQ-Legacy** (12 vehicle models) - CAN-TP2.0 legacy platform
- ✅ **MLB Longitudinal** (12 vehicle models) - Premium/performance longitudinal engines
- ✅ **Compact A0** (5 vehicle models) - Small economy city cars
- ✅ **Generic fallback** - Unknown vehicles still display metrics

### Display & UI
- ✅ LVGL graphics engine (1280×720 landscape display with GT911 capacitive touch)
- ✅ Three tabbed dashboards: Performance, Convenience, Infotainment
- ✅ Dynamic gauge scaling per platform (RPM, boost, temperature limits)
- ✅ Color-coded status indicators (cool blue, normal green/brand color, alert red)
- ✅ Interactive peak boost reset button
- ✅ Brand-specific accent colors per interpreter

### Network & Remote Access
- ✅ WiFi Access Point hotspot (AP mode)
- ✅ Real-time WebSocket telemetry streaming (10 Hz broadcast)
- ✅ Responsive web dashboard with live metrics
- ✅ Auto-reconnect logic for reliable wireless connectivity

### Safety Features
- ✅ Dual thermal threshold monitoring (oil & coolant)
- ✅ Acoustic alarm (2500 Hz warning tone) on overheat
- ✅ Visual redline alerts on UI
- ✅ Hardware transceiver self-diagnostics on boot
- ✅ Peak boost tracking with manual reset

### Vehicle Identification
- ✅ Automatic VIN detection from vehicle ECU via UDS (0x22F190)
- ✅ 50+ Volkswagen Group model signatures with chassis codes
- ✅ Dynamic interpreter loading based on architecture
- ✅ Fallback to generic interpreter if VIN unavailable
- ✅ Bench VIN injection via serial console for desktop testing

---

## Project Architecture

### Multi-Platform Decoder Design

```
┌─────────────────────────────────────────────────────────────┐
│                  Vehicle CAN Buses (3x Independent)          │
│  (Drivetrain, Comfort, Infotainment / Entertainment)        │
└──────────────┬──────────────┬──────────────┬────────────────┘
               │              │              │
        ┌──────▼────┐  ┌──────▼────┐ ┌──────▼────┐
        │  CAN CH0  │  │  CAN CH1  │ │  CAN CH2  │
        │ (TWAI-0)  │  │ (TWAI-1)  │ │ (TWAI-2)  │
        └──────┬────┘  └──────┬────┘ └──────┬────┘
               │              │              │
               └──────────────┬──────────────┘
                              │
                    ┌─────────▼────────┐
                    │   Main Loop      │
                    │   (Core 0)       │
                    │  WebSocket Tx    │
                    └────────┬─────────┘
                             │
        ┌────────────────────┴──────────────────┐
        │                                       │
   ┌────▼─────────────┐          ┌─────────────▼──────┐
   │ CockpitProcessor │          │  Vehicle Interpreter│
   │   Task (Core 1)  │          │   (Platform-aware)  │
   │ 32KB stack, 1ms  │          │ ┌─────────────────┐ │
   │ - Frame Process  │          │ │ • MQB (Modern)  │ │
   │ - UI Updates     │          │ │ • PQ-Legacy     │ │
   │ - Alarm Engine   │          │ │ • MLB (Premium) │ │
   └────┬─────────────┘          │ │ • Compact A0    │ │
        │                        │ └─────────────────┘ │
   ┌────▼──────────────────┐    └─────────────────────┘
   │  LVGL Display Engine  │
   │  - Touch callbacks    │
   │  - Arc gauges (RPM)   │
   │  - Bar indicators     │
   │  - Label updates      │
   │  - Color-coded alerts │
   └──────────────────────┘
```

### Software Layering

1. **Hardware Abstraction** (TWAI driver, GPIO, I2C, SPI)
2. **CAN Frame Reception** (multi-frame assembly, ISO-TP handling)
3. **Platform Detection** (VIN decode → interpreter selection)
4. **Vehicle Interpreter Layer** (4 platform-specific decoders)
5. **Global Context** (shared metrics, UI pointers, vehicle state)
6. **UI Rendering** (LVGL, touchscreen, web broadcast)
7. **Safety Engine** (alarm monitoring, peak tracking)

---

## Supported Platforms & Vehicles

### Group 1: MQB Matrix (Modern Transverse Engines)
**17 Models** – Platform file: `Platform_MQB_Matrix.cpp`

**Audi:**
- S3 8V, RS3 GY, Q3 MQB, TT MK3, Q2

**Volkswagen:**
- Golf MK7, Golf MK8, Passat B8, Passat B9, Tiguan MK2, Tiguan MK3, Arteon

**Skoda:**
- Octavia MK3, Octavia MK4, Superb MQB

**Seat/Cupra:**
- Leon MK3, Cupra Leon/Formentor

### Group 2: PQ-Legacy (CAN-TP2.0 Legacy Vehicles)
**12 Models** – Platform file: `Platform_PQ_Legacy.cpp`

**Audi:**
- S3 8P, A6 C6, Q3 PQ35, Q7 4L, TT MK2

**Volkswagen:**
- Golf MK5/MK6, Passat B6/B7/CC, Scirocco, Tiguan MK1

**Skoda:**
- Octavia MK2, Superb 3T

**Seat/Cupra:**
- Leon MK2

### Group 3: MLB Longitudinal (Premium/Performance Cars)
**12 Models** – Platform file: `Platform_MLB_Longitudinal.cpp`

**Audi:**
- A4 MLB 8K/8W, A5 MLB B8, A6 MLB C7/C8, A8 MLB D4/D5, Q5 MLB 8R/FY, Q7 MLB 4M

**Porsche:**
- Cayenne 92, Macan 9B

### Group 4: Compact A0 (Economy & City Cars)
**5 Models** – Platform file: `Platform_Small_Compact.cpp`

**Audi:**
- A1 PQ25, A1 MQB A0

**Volkswagen:**
- Polo PQ25, Polo MQB A0

**Seat/Cupra:**
- Ibiza MQB A0

**Total Coverage: 38 Models Unique Vehicle Models across 4 Platforms**

---

## Hardware Requirements

### Microcontroller
- **ESP32** (dual-core 240 MHz, 520 KB SRAM)
- Recommended: Waveshare ESP32-S3-LCD-4.3" or compatible with integrated display

### CAN Interface
- **CAN Gateway Module** (MCP2515 or integrated TWAI on ESP32)
- **CAN Transceiver** (SN65HVD230 or TJA1050 recommended)
- **Dual/Triple CAN bus support** for modern vehicles

### Display & Touch
- **1280×720 LCD Display** (landscape orientation, 4:3 aspect)
- **GT911 Capacitive Touch Controller** (I2C interface)
- Display driver integration with LVGL 8.x

### Connectors
- **OBD2 adapter** for vehicle CAN bus connection
- **USB-C** for serial debugging and power
- **WiFi antenna** (built-in or external for range)

### Optional
- **Audio output** GPIO for alarm tone (PWM-capable, GPIO 45 recommended)
- **Debug LEDs** for hardware status indication
- **Real-time clock (RTC)** for timestamp accuracy

---

## Installation & Setup

### 1. Prerequisites

- **Arduino IDE 2.0+** or **PlatformIO**
- **ESP32 board support** installed
- **Libraries:**
  - `ESPAsyncWebServer` (async HTTP/WebSocket)
  - `AsyncTCP` (async TCP layer)
  - `ArduinoJson` (JSON serialization)
  - `LVGL` 8.x (graphics library)
  - ESP32 core (TWAI driver built-in)

### 2. Hardware Assembly

**CAN Bus Wiring (OBD2 → Transceiver → ESP32):**
```
Vehicle OBD2 Connector
├─ Pin 6  (GND)      → GND
├─ Pin 14 (CAN_H)    → CAN Transceiver CANH
└─ Pin 13 (CAN_L)    → CAN Transceiver CANL

CAN Transceiver (e.g., SN65HVD230)
├─ CANH  → Vehicle CAN_H
├─ CANL  → Vehicle CAN_L
├─ VCC   → +3.3V
├─ GND   → GND
├─ TXD   → ESP32 GPIO 4  (CH0_TX)
└─ RXD   → ESP32 GPIO 5  (CH0_RX)
```

**Display & Touch (LVGL):**
```
GT911 Capacitive Touch (I2C)
├─ SDA   → ESP32 GPIO 19 (I2C_SDA)
├─ SCL   → ESP32 GPIO 20 (I2C_SCL)
├─ INT   → GPIO 21 (optional interrupt)
├─ RST   → GPIO 22 (optional reset)
└─ VCC/GND → Power

LCD Display (1280×720)
├─ Data Pins (8-bit or SPI parallel)
├─ Control Pins (RS, RW, EN or CS, DC, SCLK, MOSI)
└─ Backlight PWM → GPIO 45 (brightness control)
```

### 3. Code Configuration

Edit the top of `Audi_S3_8V.ino`:

```cpp
// --- HARDWARE GPIO MAPPINGS ---
#define CH0_TX 4       // CAN Channel 0 TX
#define CH0_RX 5       // CAN Channel 0 RX
#define CH1_TX 6       // CAN Channel 1 TX (optional)
#define CH1_RX 7       // CAN Channel 1 RX (optional)
#define CH2_TX 8       // CAN Channel 2 TX (optional)
#define CH2_RX 9       // CAN Channel 2 RX (optional)

#define AUDIO_PWM_PIN 45   // Thermal alarm audio output

// --- SAFETY-CRITICAL THRESHOLDS ---
#define MAX_SAFE_OIL_TEMP 115      // °C for alarm trigger
#define MAX_SAFE_COOLANT_TEMP 105  // °C for alarm trigger

// --- DISPLAY RESOLUTION ---
#define DISP_HOR_RES 1280  // Landscape width
#define DISP_VER_RES 720   // Landscape height

// --- WIFI CREDENTIALS (AP MODE) ---
#define AP_SSID "Audi_S3_Telemetry"
#define AP_PASSWORD_DEFAULT "ChangeMe_S3AP!"
```

### 4. Upload & Test

```bash
# Using Arduino IDE
Tools > Board > ESP32 > ESP32-S3
Tools > Upload Speed > 921600
Tools > Serial Monitor > 921600 baud

# Monitor boot sequence:
# [BOOT] Initializing ESP32 CAN System...
# [SYSTEM] Interrogating Drivetrain Bus for Vehicle Identification...
# [SYSTEM] SUCCESS! Detected Car VIN: WVWZZZ3CZ...
# [DECOUPLER] Dynamic Instance Allocation: AudiS38VInterpreter loaded.
# [AP MODE] WiFi Broadcasting: Audi_S3_Telemetry
# Dashboard URL: http://192.168.4.1
```

---

## CAN Decoder Architecture

### The 4 Platform Decoders

Each platform has a **unified parsing core** that handles standard signal extraction, eliminating code duplication:

#### **Platform 1: MQB Matrix** (Modern)
```cpp
// parseStandardMqbFrame() - Shared by all MQB interpreters
case 0x0FC:    // Engine RPM
case 0x1A2:    // Oil & Coolant Temps  
case 0x28A:    // Turbo Boost Pressure
```
Used by: Audi S3 8V, VW Golf 7/8, Skoda Octavia MK3/4, etc.

#### **Platform 2: PQ-Legacy** (CAN-TP2.0)
```cpp
// parseStandardPqFrame() - Shared by all PQ interpreters
case 0x280:    // Engine RPM
case 0x288:    // Coolant & Oil Temps
case 0x380:    // Boost Pressure
```
Used by: Audi S3 8P, VW Golf MK5/6, Audi Q7 4L, etc.

#### **Platform 3: MLB Longitudinal** (Premium)
```cpp
// parseStandardMlbFrame() - Shared by all MLB interpreters
case 0x105:    // Engine RPM
case 0x1A4:    // Oil & Coolant Temps
case 0x2A2:    // Turbo Boost Pressure
```
Used by: Audi A4/A6/A8, Porsche Cayenne/Macan, etc.

#### **Platform 4: Compact A0** (Dual parsing)
```cpp
// parseCompactPq25Frame() - PQ25 legacy compact
case 0x280, 0x288, 0x380

// parseCompactMqba0Frame() - MQB A0 modern compact
case 0x0FC, 0x1A2, 0x28A
```
Used by: Audi A1, VW Polo, Seat Ibiza, etc.

### Signal Decoding Pipeline

```
Raw CAN Frame (8 bytes max, or ISO-TP multi-frame)
        │
        ▼
Frame Reception & Assembly
(multi-frame buffering for ISO-TP)
        │
        ▼
Router → Platform Dispatcher
    ├─ Channel 0 → interpretDriveTrain()
    ├─ Channel 1 → interpretComfort()
    └─ Channel 2 → interpretInfotainment()
        │
        ▼
Platform-Specific Parser
(MQB / PQ / MLB / Compact)
        │
        ▼
Byte Extraction & Scaling
(offset, bit mask, scale factor, unit conversion)
        │
        ▼
Update sys_ctx->metrics struct
(RPM, boost, temps, door, MMI code, etc.)
        │
        ▼
updateUIElements() reads fresh metrics
(LVGL arc/bar/label redraw triggered)
        │
        ▼
Every 100ms: Serialize metrics to JSON
(flag Core 0 to broadcast via WebSocket)
```

---

## Platform-Specific Signal Mappings

### MQB Matrix (Modern Cars)
```
Signal                  Frame ID    Bytes    Scale     Offset      Unit
─────────────────────────────────────────────────────────────────────
Engine RPM              0x0FC       0–1      0.25      0           RPM
Oil Temperature         0x1A2       0        1.0       −40°C       °C
Coolant Temperature     0x1A2       1        1.0       −40°C       °C
Turbo Boost             0x28A       0–1      10 mbar   −1013 mbar  bar
Driver Door             0x61C       0 (bit0) 1.0       —           bool
Climate Target          0x527       0        0.5       0           °C
MMI Key Code            0x695       0        1.0       0           hex
```

**Interpreters:** AudiS38VInterpreter, AudiRS3GYInterpreter, VwGolf7/8Interpreter, SkodaOctaviaMk3/4Interpreter, etc.

### PQ-Legacy (CAN-TP2.0)
```
Signal                  Frame ID    Bytes    Scale     Offset      Unit
─────────────────────────────────────────────────────────────────────
Engine RPM              0x280       0–1      0.25      0           RPM
Coolant Temperature     0x288       0        1.0       −40°C       °C
Oil Temperature         0x288       1        1.0       −40°C       °C
Boost Pressure          0x380       0        10 mbar   −1013 mbar  bar
Driver Door             0x351       0 (bit0) 1.0       —           bool
```

**Interpreters:** AudiS38PInterpreter, VwGolf56Interpreter, AudiQ74LInterpreter, SkodaOctaviaMk2Interpreter, etc.

### MLB Longitudinal (Premium/Performance)
```
Signal                  Frame ID    Bytes    Scale     Offset      Unit
─────────────────────────────────────────────────────────────────────
Engine RPM              0x105       0–1      0.25      0           RPM
Oil Temperature         0x1A4       0        1.0       −40°C       °C
Coolant Temperature     0x1A4       1        1.0       −40°C       °C
Turbo Boost             0x2A2       0–1      10 mbar   −1013 mbar  bar
Driver Door             0x3C3       0 (bit0) 1.0       —           bool
```

**Interpreters:** AudiA4MLB8KInterpreter, AudiA6MLBC7Interpreter, PorscheCayenne92Interpreter, etc.

### Compact A0 (Economy Cars)

**PQ25 Variant:**
```
Engine RPM              0x280       0–1      0.25      0           RPM
Coolant Temperature     0x288       0        1.0       −40°C       °C
Oil Temperature         0x288       1        1.0       −40°C       °C
Boost Pressure          0x380       0        10 mbar   −1013 mbar  bar
```

**MQB A0 Variant:**
```
Engine RPM              0x0FC       0–1      0.25      0           RPM
Oil Temperature         0x1A2       0        1.0       −40°C       °C
Coolant Temperature     0x1A2       1        1.0       −40°C       °C
Turbo Boost             0x28A       0–1      10 mbar   −1013 mbar  bar
```

**Interpreters:** AudiA1PQ25Interpreter, VwPoloPQ25Interpreter, AudiA1MQBA0Interpreter, VwPoloMQBA0Interpreter, etc.

---

## UI & Display System

### LVGL Architecture

- **Buffer Allocation:** 12,800 pixels (1280 × 10 line buffer, ~51 KB)
- **Orientation:** LV_DISP_ROT_90 (landscape, 90° clockwise rotation)
- **Touch Driver:** GT911 capacitive sensor with landscape axis transformation
- **Refresh Rate:** Continuous (LVGL timer every ~33 ms @ 30 FPS)
- **Color Scheme:** Platform-specific accent colors + status indicators

### UI Layout – Three Tabs

#### Tab 1: Performance Dashboard
```
┌────────────────────────────────────────────────────────────┐
│                                                            │
│   ┌─────────┐              ▲                              │
│   │   RPM   │            ┌─┴─┐  ┌──────────┐              │
│   │  Arc    │            │ ▲ │  │ Boost    │              │
│   │ Gauge   │            │ │ │  │ Bar      │              │
│   │  3000   │            │   │  │  1.23    │              │
│   └─────────┘            │   │  │  PK 1.45 │              │
│                          └───┘  └──────────┘              │
│                                                            │
│   ┌────────────────────────────────────────────────────┐ │
│   │ OIL: 92°C                                          │ │
│   │ H2O: 88°C                                          │ │
│   └────────────────────────────────────────────────────┘ │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

**Elements:**
- **RPM Arc** (left): 220×220 px, 135° sweep, center text overlay
- **Boost Bar** (right-top): 30×160 px vertical bar, peak value tracking
- **Temperature Arcs** (right-bottom): Oil and coolant separate 90×90 arcs
- **Touch Handler:** Boost bar clickable → reset peak value

#### Tab 2: Convenience Dashboard
```
┌────────────────────────────────────────────────────────────┐
│                                                            │
│  DRV DOOR: CLOSED        CLIMATE TARGET: 21.5°C           │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

**Signals:** Driver door status, climate control target

#### Tab 3: Infotainment Dashboard
```
┌────────────────────────────────────────────────────────────┐
│                                                            │
│  MMI WHEEL KEY CODE: 0x2C                                 │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

**Signals:** Steering wheel control codes

### Color Coding Scheme

| State | RGB Value | Trigger | Use |
|-------|-----------|---------|-----|
| Cool | `#0096FF` (blue) | <75°C (temp) | Cold startup indicator |
| Normal | Platform-specific* | 75–115°C (temp) | Operating range |
| Redline | `#FF3E3E` (red) | >115°C (temp) | Critical overheat |
| Blink | Animated | Alert state | Emphasize urgency |

**Platform-Specific Normal Colors:**
- Audi: `lv_color_make(180, 0, 0)` – Audi Performance Red
- VW: `lv_color_make(0, 100, 220)` – VW Racing Blue
- Skoda: `lv_color_make(0, 200, 0)` – vRS Motorsport Green
- Porsche: `lv_color_make(255, 200, 0)` – Porsche Amber
- Seat/Cupra: `lv_color_make(240, 20, 0)` – Spanish Red/Copper

### Dynamic UI Limits (Per-Platform)

**MQB (Modern):**
- RPM gauge: 0–8000 (redline 6500)
- Boost gauge: 0–250 (2.5 bar max)
- Temperature: −40 to +215°C

**PQ-Legacy (CAN-TP2.0):**
- RPM gauge: 0–7500 (redline 6000)
- Boost gauge: 0–200 (2.0 bar max)
- Temperature: −40 to +215°C

**MLB (Premium):**
- RPM gauge: 0–7000 (redline 5500)
- Boost gauge: 0–150 to 250 (platform-dependent)
- Temperature: −40 to +215°C

**Compact (Economy):**
- RPM gauge: 0–6500 (redline 5000)
- Boost gauge: 0–150 (1.5 bar max)
- Temperature: −40 to +215°C

Each interpreter's `configureUiLimits()` method sets these values at runtime.

---

## Web Dashboard

### Access

**WiFi Connection:**
```
SSID:     Audi_S3_Telemetry
Password: Saved AP password (default `ChangeMe_S3AP!`)
Gateway:  192.168.4.1
```

**Dashboard URL:**
```
http://192.168.4.1
```

### Features

- **Live Metrics Display:** RPM, boost, temperatures, door status, MMI key codes
- **Peak Boost Tracking:** Displayed value with reset button
- **Status-Based Coloring:** Same color scheme as local display
- **Auto-Reconnect:** 2-second retry loop if WebSocket drops
- **Responsive Design:** Works on mobile and desktop browsers

### WebSocket Protocol

- **Endpoint:** `ws://192.168.4.1/ws`
- **Update Rate:** 10 Hz (100 ms intervals)
- **Broadcast Mode:** All connected clients receive identical JSON payload
- **Bidirectional Commands:** `RESET_PEAK`, `SET_AP_PASSWORD <new_password>`

### Runtime Password Changes

- **Serial console:** `setpass` (interactive prompt) or `setpass <new_password>`
- **Web dashboard:** use the **Change AP Password** button in the Diagnostic tab
- **Touchscreen UI:** use **Set WiFi Password** on the Diagnostic tab
- New password is validated (8–63 printable ASCII chars), saved to NVS, and applied immediately by restarting the hotspot.

**Telemetry Payload (JSON):**
```json
{
  "rpm": 2850,
  "boost": 1.23,
  "peak": 1.45,
  "oil": 92,
  "h2o": 88,
  "car": "Audi A3 / S3 / RS3 (MQB Matrix)"
}
```

---

## Extending to New Platforms

### Step 1: Analyze Target Vehicle CAN Bus

Use a CAN analyzer (CANoe, PCAN-View, etc.) to identify signal frame IDs and byte offsets.

### Step 2: Create a New Interpreter Class

Create a new interpreter in the appropriate platform file:

```cpp
// In VehicleInterpreters.h (forward declaration)
class YourNewVehicleInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override;
    void interpretComfort(twai_message_t &msg) override;
    void interpretInfotainment(twai_message_t &msg) override;
    void configureUiLimits() override;
};

// In Platform_MQB_Matrix.cpp (example for MQB platform)
void YourNewVehicleInterpreter::interpretDriveTrain(twai_message_t &msg) { 
    parseStandardMqbFrame(msg);
}
void YourNewVehicleInterpreter::interpretComfort(twai_message_t &msg) { 
    parseStandardMqbComfort(msg);
}
void YourNewVehicleInterpreter::interpretInfotainment(twai_message_t &msg) {
    // Platform-specific infotainment signals
}
void YourNewVehicleInterpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(180, 0, 0);
    if (sys_ctx->rpm_meter != nullptr) 
        lv_arc_set_range(sys_ctx->rpm_meter, 0, 8000);
    if (sys_ctx->boost_meter != nullptr) 
        lv_bar_set_range(sys_ctx->boost_meter, 0, 250);
}
```

### Step 3: Integrate into VIN Decoding

Edit the VIN detection function in `Audi_S3_8V.ino`:

```cpp
// Add to chassis detection
else if (strcmp(chassis, "YOUR_CODE") == 0) { 
    active_vehicle_profile.model_name = "Your Vehicle Name";
    active_vehicle_profile.network_generation = SERIES_MQB_A_CLASS;
}

// Add to interpreter loading
else if (strcmp(chassis, "YOUR_CODE") == 0) 
    sys_ctx->interpreter = new YourNewVehicleInterpreter();
```

### Step 4: Test on Bench

1. Inject test CAN frames with known values
2. Verify metrics update correctly
3. Calibrate scale factors and offsets
4. Test on real vehicle

---

## Project Structure & File Organization

```
Can-Decoder/
│
├── Audi_S3_8V.ino                   # Main sketch - hardware config,
│                                    # WiFi, LVGL UI, VIN detection,
│                                    # main loop & CockpitTask
│
├── VehicleInterpreters.h            # Abstract base class, global
│                                    # context, forward declarations
│
├── VehicleInterpreters.cpp          # Global init, generic fallback
│
├── Platform_MQB_Matrix.cpp          # 17 MQB interpreters +
│                                    # parseStandardMqbFrame()
│
├── Platform_PQ_Legacy.cpp           # 12 PQ-Legacy interpreters +
│                                    # parseStandardPqFrame()
│
├── Platform_MLB_Longitudinal.cpp    # 12 MLB interpreters +
│                                    # parseStandardMlbFrame()
│
├── Platform_Small_Compact.cpp       # 5 Compact A0 interpreters +
│                                    # dual parsers
│
├── VehicleSimulator.h/cpp           # Bench simulator
│
└── README.md                        # This file
```

### Architecture Benefits

- **Unified Parsing Cores:** Each platform has single shared frame parser
- **Modular Interpreters:** Vehicle-specific logic in `configureUiLimits()` and colors
- **Extensibility:** Adding a new vehicle requires ~15 lines of code
- **Type Safety:** Abstract interface ensures all vehicles implement required methods

---

## Configuration

### Thermal Safety Thresholds
```cpp
#define MAX_SAFE_OIL_TEMP 115
#define MAX_SAFE_COOLANT_TEMP 105
```

### Audio Alert
```cpp
#define AUDIO_PWM_PIN 45      // 2500 Hz tone, 150 ms pulses
```

### WiFi Credentials
```cpp
#define AP_SSID "Audi_S3_Telemetry"
#define AP_PASSWORD_DEFAULT "ChangeMe_S3AP!"
```

### CAN Bus Baud Rate
```cpp
twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_500KBITS();
```

### Display Resolution
```cpp
#define DISP_HOR_RES 1280
#define DISP_VER_RES 720
```

---

## Troubleshooting

### Boot & Initialization
- **Transceiver failed:** Verify 3.3V power and GPIO connections
- **VIN timeout:** Vehicle must have ignition ON; check CAN bus activity
- **LVGL blank screen:** Check display power, I2C address (GT911 = 0x29), rotation setting

### CAN Reception
- **No frames received:** Verify vehicle CAN active with external analyzer
- **Metrics don't update:** Check interpreter loaded, frame IDs, byte offsets

### WebSocket & Web Dashboard
- **Connection drops:** Verify WiFi SSID broadcast, firewall allows WebSocket
- **High latency:** Ensure Core 0/Core 1 task separation, check buffer sizes

### Memory & Performance
- **Task watchdog timeout:** CockpitTask pinned to Core 1; monitor heap usage
- **Janky UI:** Ensure LVGL timer called each frame, reduce WebSocket rate if needed

---

## Contributing

### Adding a New Vehicle

1. Create interpreter class in appropriate platform file
2. Implement 4 virtual methods (drive-train, comfort, infotainment, UI limits)
3. Reuse shared platform parser where possible
4. Add to VIN decoding logic
5. Test on bench with CAN simulator
6. Submit PR with example logs and screenshots

### Code Style
- Use `snake_case` for variables/functions
- Use `UPPER_CASE` for `#define` constants
- Use `CamelCase` for class names
- Keep lines ≤100 characters

---

## License

[Specify your license: MIT, GPL-3.0, Apache-2.0, or proprietary]

---

## Contact & Support

- **GitHub Issues:** [https://github.com/Stanneh1/Can-Decoder/issues](https://github.com/Stanneh1/Can-Decoder/issues)
- **GitHub Discussions:** [Feature requests & collaboration]

---

**Last Updated:** July 22, 2026  
**Status:** Active Development  
**Supported Vehicles:** 38 Models across 4 Platforms  
**Lines of Code:** ~5,500 total
