# Universal CAN Bus Decoder & Telemetry Platform

A modular, extensible real-time CAN bus decoder and vehicle telemetry display system built for ESP32 microcontrollers. Designed to decode, interpret, and visualize automotive CAN bus messages from any vehicle platform with a compatible CAN gateway module.

**Current Support:** Volkswagen Group vehicles (Audi, Volkswagen, Skoda, Seat/Cupra, Porsche)  
**Future Support:** Extensible architecture enables support for any CAN-equipped platform

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Supported Platforms](#supported-platforms)
- [Hardware Requirements](#hardware-requirements)
- [Architecture](#architecture)
- [Installation & Setup](#installation--setup)
- [How It Works](#how-it-works)
- [CAN Message Decoding](#can-message-decoding)
- [UI & Display System](#ui--display-system)
- [Web Dashboard](#web-dashboard)
- [Extending to New Platforms](#extending-to-new-platforms)
- [Project Structure](#project-structure)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

This project implements a universal CAN bus decoder that captures raw CAN frames from vehicle electrical systems and translates them into human-readable telemetry metrics. It features:

- **Real-time CAN message parsing** across multiple independent CAN buses
- **Multi-platform support** with pluggable interpreter modules (MQB, PQ35, PQ47, MLB platforms)
- **Dual-interface display**: Local LVGL touchscreen UI + wireless web dashboard
- **Dynamic vehicle identification** via VIN decoding (automatically loads correct interpreter)
- **Safety-critical alarm system** with thermal threshold monitoring and audio alerts
- **Bench testing simulator** for development without a live vehicle

The system is designed from the ground up to be platform-agnostic. While currently focused on Volkswagen Group vehicles (which share the VAG electronic architecture), the modular interpreter pattern allows rapid addition of new manufacturers and platforms.

---

## Features

### Core Decoding
- ✅ Multi-frame CAN message assembly (ISO-TP protocol)
- ✅ Platform-specific signal extraction and scaling
- ✅ Real-time metric updates at 10Hz over WebSocket
- ✅ Bench telemetry simulator for testing without vehicle

### Display & UI
- ✅ LVGL graphics engine (1280×720 landscape display with GT911 capacitive touch)
- ✅ Three tabbed dashboards: Performance, Convenience, Infotainment
- ✅ Dynamic gauge scaling per vehicle platform
- ✅ Color-coded status indicators (cool blue, normal green, alert red)
- ✅ Interactive peak boost reset button

### Network & Remote Access
- ✅ WiFi Access Point hotspot (AP mode)
- ✅ Real-time WebSocket telemetry streaming
- ✅ Responsive web dashboard with live metrics
- ✅ Auto-reconnect logic for reliable connectivity

### Safety Features
- ✅ Dual thermal threshold monitoring (oil & coolant)
- ✅ Acoustic alarm (2500 Hz warning tone) on overheat
- ✅ Visual redline alerts on UI
- ✅ Hardware transceiver self-diagnostics on boot

### Vehicle Identification
- ✅ Automatic VIN detection from vehicle ECU
- ✅ 80+ Volkswagen Group model signatures
- ✅ Dynamic interpreter loading based on chassis code
- ✅ Fallback to generic interpreter if VIN unavailable
- ✅ Bench VIN injection via serial console for desktop testing

---

## Supported Platforms

### Current Implementation: Volkswagen Group (VAG)

#### Audi Models
- **8P** – A3 / S3 (PQ35 Platform, CAN-TP2.0)
- **8V** – A3 / S3 / RS3 (MQB Platform, HIGH-SPEED MQB CAN)
- **GY** – A3 / S3 / RS3 (MQB EVO 8Y, CAN-FD/CAN)
- **8K** – A4 / S4 / RS4 (MLB B8)
- **8W** – A4 / S4 / A5 / RS5 (MLB B9, EVO FlexRay/CAN)
- **4F** – A6 / S6 / RS6 (C6 Era, CAN-TP2.0)
- **4G** – A6 / S6 / A7 / RS7 (MLB C7)
- **4K** – A6 / A7 / RS6 / RS7 (MLB C8, EVO FlexRay/CAN)
- **4H** – A8 / S8 (D4 Luxury)
- **4N** – A8 / S8 (D5 Luxury, EVO FlexRay/CAN)
- **8T, 8F** – A5 / S5 / RS5 (B8 Chassis, MLB)
- **8U** – Q3 Compact SUV (PQ35, HIGH-SPEED CAN-TP2.0)
- **F3** – Q3 / RS Q3 (MQB)
- **8R** – Q5 Crossover (MLB)
- **FY** – Q5 / SQ5 (MLB FY, EVO FlexRay/CAN)
- **4M** – Q7 / SQ7 / Q8 / SQ8 (MLB 4M, EVO FlexRay/CAN)
- **8J** – TT / TTS / TT RS (Mk2, CAN-TP2.0)
- **8S** – TT / TTS / TT RS (Mk3 MQB)
- **GA** – Q2 Compact Crossover (MQB)
- **8X** – A1 Supermini (PQ25, CAN-TP2.0)
- **GB** – A1 Sportback (MQB A0)
- **4L** – Q7 SUV (PQ47 Platform, CAN-TP2.0)

#### Volkswagen Models
- **1K, 5K, AJ** – Golf Mk5 / Mk6 / Jetta (PQ35)
- **5G, BA, AM** – Golf Mk7 / GTI / Golf R (MQB)
- **CD** – Golf Mk8 / GTI / Clubsport / R (MQB EVO)
- **3C, AN** – Passat B6 / B7 / CC (CAN-TP2.0)
- **3G, CB** – Passat B8 (MQB)
- **A3** – Passat B9 (MQB EVO)
- **13** – Scirocco Coupe (CAN-TP2.0)
- **5N** – Tiguan SUV Mk1 (CAN-TP2.0)
- **AD, AX** – Tiguan Mk2 (MQB)
- **CT** – Tiguan Mk3 (MQB EVO)
- **6R, 6C** – Polo Hatchback (PQ25, CAN-TP2.0)
- **AW** – Polo GTI / Hatch (MQB A0)
- **3H** – Arteon GranTurismo (MQB)

#### Skoda Models
- **1Z** – Octavia vRS (Mk2 PQ35, CAN-TP2.0)
- **5E** – Octavia vRS (Mk3 MQB)
- **NX** – Octavia vRS (Mk4 MQB EVO)
- **3T** – Superb Saloon (CAN-TP2.0)
- **3V** – Superb (MQB)

#### Seat / Cupra Models
- **1P** – Leon Cupra (Mk2 PQ35, CAN-TP2.0)
- **5F** – Leon FR / Cupra (Mk3 MQB)
- **KL** – Cupra Leon / Formentor (Mk4 MQB EVO)
- **KJ** – Ibiza / Arona (MQB A0)

#### Porsche Models
- **92** – Cayenne SUV (92A, MLB)
- **9B** – Macan Crossover (95B, MLB)

---

## Hardware Requirements

### Microcontroller
- **ESP32** (dual-core, 240 MHz, 520 KB SRAM)
- Recommended: Waveshare ESP32-S3-LCD-4.3" or similar with integrated display

### CAN Interface
- **CAN Gateway Module** (e.g., MCP2515 or integrated TWAI)
- **CAN Transceiver** (SN65HVD230 or TJA1050 recommended)
- Dual CAN bus support (many newer vehicles use multiple CAN networks)

### Display & Touch
- **1280×720 LCD Display** (landscape orientation)
- **GT911 Capacitive Touch Controller** (I2C interface)
- Display driver integration with LVGL

### Connectors
- **CAN Bus connectors** (standard OBD2 adapter recommended)
- **USB-C** for serial debugging and power
- **WiFi antenna** (built-in or external for better range)

### Optional
- **Audio output** for alarm tone (PWM capable GPIO)
- **Debug LEDs** for hardware status indication
- **Real-time clock** (RTC) module for timestamp accuracy

---

## Architecture

### System Block Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                      Vehicle CAN Buses                       │
│  (Drive Train, Comfort, Infotainment / Entertainment)       │
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
                    │  WebSocket & ALM │
                    └────────┬─────────┘
                             │
        ┌────────────────────┴──────────────────┐
        │                                       │
   ┌────▼─────────────┐          ┌─────────────▼──────┐
   │ CockpitProcessor │          │  Vehicle Interpreter│
   │   Task (Core 1)  │          │   (Platform-aware)  │
   │ 32KB stack, 1ms  │          │   - MQB (8V)        │
   │ - Frame Process  │          │   - PQ35 (8P)       │
   │ - UI Updates     │          │   - PQ47 (4L)       │
   │ - Alarm Engine   │          │   - Generic         │
   └────┬─────────────┘          └─────────────────────┘
        │
   ┌────▼──────────────────┐
   │  LVGL Display Engine  │
   │  - Touch callbacks    │
   │  - Arc gauges (RPM)   │
   │  - Bar indicators     │
   │  - Label updates      │
   │  - Color-coded alerts │
   └──────────────────────┘
```

### Software Layers

1. **Hardware Abstraction** (TWAI driver, GPIO, I2C)
2. **CAN Frame Reception** (multi-frame assembly, ISO-TP handling)
3. **Vehicle Interpreter** (platform-specific signal extraction)
4. **Global Context** (shared metrics, UI pointers, state)
5. **UI Rendering** (LVGL, touchscreen, web broadcast)
6. **Application Logic** (alarm handling, telemetry streaming)

---

## Installation & Setup

### 1. Prerequisites

- Arduino IDE 2.0+ or PlatformIO
- ESP32 board support installed
- Libraries:
  - `ESPAsyncWebServer` (async HTTP/WebSocket)
  - `AsyncTCP` (async TCP client/server)
  - `ArduinoJson` (JSON serialization for telemetry)
  - `LVGL` v8.x (graphics library)
  - ESP32 core (TWAI CAN driver built-in)

### 2. Hardware Assembly

**CAN Bus Wiring:**
```
Vehicle OBD2 Connector
│
├─ Pin 6  (GND)      → GND
├─ Pin 14 (CAN_H)    → CAN Transceiver H
└─ Pin 13 (CAN_L)    → CAN Transceiver L

CAN Transceiver (e.g., SN65HVD230)
├─ CANH  → Vehicle CAN_H
├─ CANL  → Vehicle CAN_L
├─ VCC   → +3.3V
├─ GND   → GND
├─ TXD   → ESP32 GPIO 4  (CH0_TX)
└─ RXD   → ESP32 GPIO 5  (CH0_RX)
```

**Display & Touch:**
```
GT911 Capacitive Touch (I2C)
├─ SDA   → ESP32 GPIO 19 (I2C_SDA)
├─ SCL   → ESP32 GPIO 20 (I2C_SCL)
├─ INT   → GPIO 21 (optional interrupt)
├─ RST   → GPIO 22 (optional reset)
└─ VCC/GND → Power

LCD Display
├─ Data Pins (8-bit or SPI)
├─ Control Pins (RS, RW, EN or CS, DC, SCLK, MOSI)
└─ Backlight PWM → GPIO 45 (brightness control)
```

### 3. Code Configuration

Edit the top of the main `.ino` file:

```cpp
// --- HARDWARE CONFIGURATION MAPPINGS ---
#define CH0_TX 4       // CAN Channel 0 TX
#define CH0_RX 5       // CAN Channel 0 RX
#define CH1_TX 6       // CAN Channel 1 TX (optional)
#define CH1_RX 7       // CAN Channel 1 RX (optional)
#define CH2_TX 8       // CAN Channel 2 TX (optional)
#define CH2_RX 9       // CAN Channel 2 RX (optional)

#define AUDIO_PWM_PIN 45   // Thermal alarm audio output

// --- SAFETY CRITICAL THRESHOLDS ---
#define MAX_SAFE_OIL_TEMP 115      // °C threshold for oil alarm
#define MAX_SAFE_COOLANT_TEMP 105  // °C threshold for coolant alarm

// --- DISPLAY RESOLUTION ---
#define DISP_HOR_RES 1280  // Horizontal resolution (landscape)
#define DISP_VER_RES 720   // Vertical resolution (landscape)

// --- WI-FI CREDENTIALS ---
const char* ap_ssid = "Audi_S3_Telemetry";      // SSID (change as desired)
const char* ap_password = "Password123";         // Change for security
```

### 4. Upload & Test

```bash
# Using Arduino IDE
Tools > Board > esp32 > ESP32-S3
Tools > Upload Speed > 921600
Tools > Serial Monitor > 921600 baud

# Monitor boot sequence:
# [BOOT] Initializing terminal interface... Ready in 5 seconds.
# [SYSTEM] Interrogating Powertrain Bus for Vehicle Identification...
# [SYSTEM] SUCCESS! Detected Car VIN: WVWZZZ3CZ...
# [DECOUPLER] Dynamic Instance Allocation: AudiS38VInterpreter class loaded cleanly.
# Access Point Launched. Connect to: Audi_S3_Telemetry
# Dashboard Web URL Address: http://192.168.4.1
```

---

## How It Works

### Startup Sequence

1. **Serial Initialization** (921600 baud, 2 second stabilization)
2. **Hardware Diagnostics** (test all three CAN transceivers on boot)
3. **TWAI Driver Startup** (500 kbps baud rate, channel-specific acceptance filters)
4. **VIN Request** (send UDS diagnostic query 0x22F190 to engine ECU)
5. **Vehicle Profile Loading** (decode VIN, load appropriate interpreter)
6. **WiFi Access Point Launch** (broadcast SSID, start HTTP/WebSocket servers)
7. **Display System Init** (LVGL, touch driver, UI layout generation)
8. **Spawn High-Priority Task** (pin CockpitCoreProcessor to Core 1, 32 KB stack)
9. **Ready for Telemetry** (start receiving and decoding CAN frames)

### Runtime Loop

**Core 0 (Main Loop) – 1 ms tick:**
- Check WebSocket client count and dispatch buffered telemetry payload
- Clean up disconnected WebSocket clients every 1000 ms
- Accept serial VIN injection for bench testing
- Inject simulated telemetry for demo mode
- Delay 1 ms to yield to other tasks

**Core 1 (CockpitCoreProcessor Task) – 1 ms tick:**
- Invoke LVGL timer handler (UI redraw)
- Poll all three CAN channels for inbound frames (non-blocking)
- Route frames through active vehicle interpreter
- Update UI elements with fresh metrics
- Run acoustic alert engine (thermal monitoring)
- Every 100 ms: serialize metrics to JSON, flag Core 0 to broadcast

### CAN Frame Processing Pipeline

```
Raw CAN Frame (8 bytes)
       │
       ▼
┌─────────────────────────┐
│  processInboundFrames   │
│ (receives from TWAI)    │
└──────────┬──────────────┘
           │
           ▼
┌─────────────────────────────────────────┐
│ Log to serial (HEX dump, frame ID)      │
└──────────┬──────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────┐
│ Route via sys_ctx->interpreter          │
│  - Channel 0 → interpretDriveTrain()    │
│  - Channel 1 → interpretComfort()       │
│  - Channel 2 → interpretInfotainment()  │
└──────────┬──────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────┐
│ Extract Signal (byte offset, bit mask,  │
│  scale factor, physical unit)           │
└──────────┬──────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────┐
│ Update sys_ctx->metrics struct          │
│  - engine_rpm                           │
│  - boost_bar                            │
│  - oil_temp, coolant_temp               │
│  - door_open, target_temp               │
│  - mmi_key_code                         │
└──────────┬──────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────┐
│ updateUIElements() reads metrics,       │
│ updates LVGL arc/bar/label values       │
└─────────────────────────────────────────┘
```

---

## CAN Message Decoding

### Signal Format

Each signal is defined by:
- **Frame ID** (CAN identifier, 0x000–0x7FF)
- **Byte Offset** (0–7 within 8-byte payload)
- **Bit Mask** (e.g., 0xFF for full byte, 0x0F for lower nibble)
- **Scale Factor** (e.g., 0.25 RPM/LSB, 10 mbar/LSB)
- **Offset** (e.g., −40°C for temperature sensors)
- **Physical Unit** (RPM, bar, °C, mph, etc.)

### MQB Platform (Audi S3 8V) – Key Signals

| Signal | Frame ID | Bytes | Scale | Offset | Range | Unit |
|--------|----------|-------|-------|--------|-------|------|
| Engine RPM | 0x0FC | 0–1 | 0.25 | 0 | 0–8000 | RPM |
| Turbo Boost | 0x28A | 0–1 | 10 mbar | 1013 mbar ref | 0–2500 | mbar |
| Oil Temperature | 0x1A2 | 0 | 1 | −40 | −40 to +215 | °C |
| Coolant Temperature | 0x1A2 | 1 | 1 | −40 | −40 to +215 | °C |
| Driver Door | 0x61C | 0 | bit 0 | — | open/closed | bool |
| Climate Target | 0x527 | 0 | 0.5 | 0 | 16–32 | °C |
| MMI Key Code | 0x695 | 0 | 1 | 0 | 0–255 | code |

### PQ35 Platform (Audi A3 8P) – Key Signals

| Signal | Frame ID | Bytes | Scale | Offset | Unit |
|--------|----------|-------|-------|--------|------|
| Engine RPM | 0x280 | 0–1 | 0.25 | 0 | RPM |
| Turbo Boost | 0x380 | 2–3 | 10 mbar | 1013 mbar | mbar |
| Oil Temperature | 0x288 | 0 | 1 | −40 | °C |
| Coolant Temperature | 0x288 | 1 | 1 | −40 | °C |

### PQ47 Platform (Audi Q7 4L) – Key Signals

| Signal | Frame ID | Bytes | Scale | Offset | Unit |
|--------|----------|-------|-------|--------|------|
| Engine RPM | 0x120 | 0–1 | 0.5 | 0 | RPM |
| Boost Pressure | 0x3C0 | 4–5 | 5 mbar | 0 | mbar |
| Oil Temperature | 0x11E | 2 | 1 | −40 | °C |
| Coolant Temperature | 0x11E | 3 | 1 | −40 | °C |

### Multi-Frame Assembly (ISO-TP)

For messages longer than 8 bytes, the ISO-TP (ISO 15765-2) protocol is used:

```
First Frame:     0x10 [length_hi] [data_0–6]
Flow Control:    0x30 0x00 0x00 (clear-to-send)
Consecutive:     0x21 [data_7–14]
                 0x22 [data_15–22]
                 ...
```

**VIN Detection Example:**
```
TX: 0x7E0  03 22 F1 90 AA AA AA AA  (UDS ReadDataByIdentifier, DID 0xF190)
   ↓ (1.5 second timeout)
RX: 0x7E8  10 13 62 F1 90 57 41 55  (First Frame, 19 bytes, "WAU...")
TX: 0x7E0  30 00 00 AA AA AA AA AA  (Flow Control CTS)
RX: 0x7E8  21 56 5A 5A 5A 5A 5A 5A  (Consecutive #1, "VZZZZZZZ")
RX: 0x7E8  22 5A 5A 5A 5A 5A 5A 5A  (Consecutive #2, "ZZZZZZZ...")
```

Result: VIN extracted as "WVWZZZ3CZ..." (17 characters)

---

## UI & Display System

### LVGL Architecture

- **Buffer Allocation:** 12,800 pixels (1280 × 10 line buffer, ~51 KB)
- **Orientation:** LV_DISP_ROT_90 (landscape, 90° CW rotation)
- **Touch Driver:** GT911 capacitive sensor with axis transformation
- **Refresh Rate:** Continuous (LVGL timer every ~33 ms)

### UI Layout – Three Tabs

#### Tab 1: Performance
```
┌────────────────────────────────────────────────────────────┐
│                                                            │
│   ┌─────────┐              ▲                              │
│   │   RPM   │            ┌─┴─┐  ┌──────────┐              │
│   │  Arc    │            │ ▲ │  │ Boost    │              │
│   │ Gauge   │            │ │ │  │ Bar      │              │
│   │  3000   │            │   │  │  1.23    │              │
│   └─────────┘            └───┘  │  PK 1.45 │              │
│                                 └──────────┘              │
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
- **Touch Handler:** Boost bar clickable → reset peak value, log event

#### Tab 2: Convenience
```
┌────────────────────────────────────────────────────────────┐
│                                                            │
│                                                            │
│         DRV DOOR: CLOSED  |  TGT: 21.5°C                 │
│                                                            │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

**Signals:** Driver door status (open/closed), climate control target temperature

#### Tab 3: Infotainment
```
┌────────────────────────────────────────────────────────────┐
│                                                            │
│                                                            │
│         MMI VOL WHEEL HEX INPUT VECTOR: 0x2C              │
│                                                            │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

**Signals:** Steering wheel control input codes (hex)

### Color Coding Scheme

| State | Color | Trigger | Use Case |
|-------|-------|---------|----------|
| Cool | #0096FF (blue) | Below 75°C (temp) | Cold startup |
| Normal | #32C832 (green) | 75–115°C (temp), <6500 RPM | Operating range |
| Redline/Alert | #FF3E3E (red) | >115°C (temp), >6500 RPM | Critical condition |
| Blink Animation | — | 0.3 second cycle | Emphasize urgency |

### UI Limit Scaling (Per-Platform)

**MQB Interpreter:**
- RPM gauge: 0–8000 (redline 6500)
- Boost gauge: 0–250 (scale 1, maps to 2.5 bar max)
- Temperature: −40 to +215°C

**PQ35 Interpreter:**
- RPM gauge: 0–7500 (redline 6000)
- Boost gauge: 0–200 (scale 1, maps to 2.0 bar max)
- Temperature: −40 to +215°C

**Generic Interpreter:**
- RPM gauge: 0–6000 (redline 5000)
- Boost gauge: 0–150 (scale 1, maps to 1.5 bar max)
- Temperature: −40 to +215°C

---

## Web Dashboard

### Access

**WiFi Connection:**
```
SSID:     Audi_S3_Telemetry
Password: Password123
Gateway:  192.168.4.1
```

**Dashboard URL:**
```
http://192.168.4.1
```

### Features

- **Live Metrics Display:** RPM, boost, temperatures, door status, MMI code
- **Peak Boost Tracking:** Stored value with reset button
- **Status-Based Coloring:** Same color scheme as local display
- **Auto-Reconnect:** 2-second retry loop if connection drops
- **JSON Payload Format:**
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

### WebSocket Protocol

- **Endpoint:** `ws://192.168.4.1/ws`
- **Update Rate:** 10 Hz (100 ms)
- **Broadcast:** All connected clients receive identical payload
- **Commands:** `RESET_PEAK` (sent from web to device to zero peak boost)

---

## Extending to New Platforms

### Step 1: Analyze the Target Vehicle's CAN Bus

1. Capture raw CAN frames using a CAN analyzer (CANoe, PCAN-View, etc.)
2. Identify which frames contain the signals you need
3. Document:
   - Frame ID (hex)
   - Byte offset within frame (0–7)
   - Bit mask (if signal spans multiple bits)
   - Scale factor (e.g., 0.1 RPM/LSB)
   - Offset (e.g., −40°C baseline)
   - Physical unit
   - Valid range (min/max)

**Example CAN trace (hypothetical BMW vehicle):**
```
0x0F4  02 E0 [xx xx xx xx xx xx]  → RPM @ bytes 0-1, scale 0.25
0x1F4  [45 32 xx xx xx xx xx xx]  → Oil temp @ byte 0, offset -40
0x2F5  [xx 98 xx xx xx xx xx xx]  → Boost @ byte 1, scale 5 mbar
```

### Step 2: Create a New Interpreter Class

Create a new header file, e.g., `Model_BMW_F30.h`:

```cpp
#ifndef MODEL_BMW_F30_H
#define MODEL_BMW_F30_H

#include "VehicleInterpreters.h"

class BMWF30Interpreter : public BaseVehicleInterpreter {
public:
    BMWF30Interpreter() {
        platform_name = "BMW F30 (CAN-TP2.0)";
    }
    
    void interpretDriveTrain(const twai_message_t &msg) override {
        switch (msg.identifier) {
            case 0x0F4:  // Engine RPM
                if (sys_ctx->metrics.engine_rpm < 8000) {
                    sys_ctx->metrics.engine_rpm = 
                        ((msg.data[0] << 8) | msg.data[1]) * 0.25f;
                }
                break;
            case 0x1F4:  // Oil Temperature
                sys_ctx->metrics.oil_temp = msg.data[0] - 40;
                break;
            case 0x2F5:  // Boost Pressure
                sys_ctx->metrics.boost_bar = 
                    (msg.data[1] * 5 - 1013) / 1000.0f;
                break;
        }
    }
    
    void interpretComfort(const twai_message_t &msg) override {
        // Implement comfort signals (door, climate, etc.)
    }
    
    void interpretInfotainment(const twai_message_t &msg) override {
        // Implement infotainment signals (buttons, displays)
    }
    
    void configureUiLimits() override {
        if (sys_ctx->rpm_meter != nullptr) {
            lv_arc_set_range(sys_ctx->rpm_meter, 0, 8000);
        }
        if (sys_ctx->boost_meter != nullptr) {
            lv_bar_set_range(sys_ctx->boost_meter, 0, 200);
        }
    }
};

#endif
```

### Step 3: Integrate into VIN Decoding

Edit `decodeAndPrintVehicleIdentity()` in the main `.ino` file:

```cpp
// In the chassis detection section:
else if (strcmp(chassis, "F30") == 0) {
    active_vehicle_profile.model_name = "BMW 3 Series (F30)";
    active_vehicle_profile.electrical_bus = "CAN-TP2.0 FLEXRAY";
    active_vehicle_profile.network_generation = SERIES_BMW_F_CLASS;
}

// In the interpreter loading section:
else if (active_vehicle_profile.network_generation == SERIES_BMW_F_CLASS) {
    sys_ctx->interpreter = new BMWF30Interpreter();
    Serial.println("[DECOUPLER] Dynamic Instance Allocation: BMWF30Interpreter loaded.");
}
```

### Step 4: Include Header & Define Enum

In the main `.ino` file, add:

```cpp
#include "Model_BMW_F30.h"

enum NetworkGeneration {
    SERIES_UNKNOWN,
    SERIES_MQB_A_CLASS,
    SERIES_PQ35_46_LEGACY,
    SERIES_PQ47_4L,
    SERIES_BMW_F_CLASS,  // Add new platform
    // ... more platforms
};
```

### Step 5: Test on Bench

1. Connect bench CAN simulator (e.g., PEAK PCAN-View, CANoe)
2. Inject test frames via serial console VIN injection
3. Verify metrics update correctly
4. Calibrate scale factors and offsets against real vehicle data
5. Adjust UI limits (`configureUiLimits()`) for platform-specific ranges

---

## Project Structure

```
Can-Decoder/
├── Audi_S3_8V.ino                   # Main sketch (TODO: rename to universal name)
│
├── VehicleInterpreters.h            # Abstract base class + generic interpreter
├── VehicleInterpreters.cpp          # Generic interpreter implementation
│
├── Model_MQB_8V.h                   # Audi S3 8V / MQB platform decoder
├── Model_MQB_8V.cpp
│
├── Model_PQ35_8P.h                  # Audi A3 8P / PQ35 platform decoder
├── Model_PQ35_8P.cpp
│
├── Model_PQ47_4L.h                  # Audi Q7 4L / PQ47 platform decoder
├── Model_PQ47_4L.cpp
│
├── VehicleSimulator.h               # Bench CAN message simulator
├── VehicleSimulator.cpp             # Injects synthetic frames for testing
│
├── README.md                         # This file
└── LICENSE                           # Project license
```

### File Purposes

| File | Purpose |
|------|---------|
| **Audi_S3_8V.ino** | Entry point, hardware config, main/CockpitTask loops, LVGL UI builder, UDS VIN request, web server setup |
| **VehicleInterpreters.h/cpp** | `BaseVehicleInterpreter` interface (pure virtual methods), `GenericVehicleInterpreter` fallback, global `sys_ctx` pointer |
| **Model_MQB_8V.h/cpp** | `AudiS38VInterpreter`: decodes MQB platform CAN frames (0x0FC RPM, 0x28A boost, 0x1A2 temps, etc.) |
| **Model_PQ35_8P.h/cpp** | `AudiS38PInterpreter`: decodes PQ35 platform CAN frames (0x280 RPM, 0x288 temps, 0x380 boost, etc.) |
| **Model_PQ47_4L.h/cpp** | `AudiQ74LInterpreter`: decodes PQ47 platform CAN frames for Q7, Q5, Touareg |
| **VehicleSimulator.h/cpp** | `runBenchTelemetrySimulation()`: injects fake CAN frames with ramping RPM, static boost/temps for testing |

---

## Configuration

### Thermal Safety Thresholds

Edit at top of main `.ino`:

```cpp
#define MAX_SAFE_OIL_TEMP 115      // Triggers alarm when exceeded
#define MAX_SAFE_COOLANT_TEMP 105  // Triggers alarm when exceeded
```

### Audio Alert

```cpp
#define AUDIO_PWM_PIN 45           // GPIO pin for tone() output
```

Alarm specifications:
- **Frequency:** 2500 Hz
- **Duration:** 150 ms per pulse
- **Interval:** 600 ms between pulses (while over threshold)

### WiFi Credentials

```cpp
const char* ap_ssid = "Audi_S3_Telemetry";
const char* ap_password = "Password123";
```

**Security Note:** Change password before deploying; hardcoded credentials are not recommended for production.

### CAN Baud Rate

```cpp
twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_500KBITS();
```

Most vehicles use 500 kbps. Some older vehicles may use 250 kbps. Adjust if needed.

### Display Resolution

```cpp
#define DISP_HOR_RES 1280
#define DISP_VER_RES 720
```

Match your physical display. The UI is responsive and scales accordingly.

---

## Troubleshooting

### Boot Issues

**Problem:** "CRITICAL ERROR: Transceiver hardware diagnostic check failed!"

**Solution:**
- Check CAN transceiver power (3.3V)
- Verify TX/RX GPIO connections to transceiver
- Ensure CAN_H and CAN_L are not shorted
- Test with a multimeter: CAN_H and CAN_L should float around 2.5V at idle

**Problem:** VIN query times out ("WARNING: VIN query timed out")

**Solution:**
- Verify vehicle is powered on and ignition is in ACC or ON position
- Check CAN bus activity with a CAN analyzer; some vehicles may not respond to UDS on startup
- In `loop()`, use serial console to inject VIN: `WVWZZZ3CZ...` (17 chars exactly)

---

### Display Issues

**Problem:** LVGL displays garbage or nothing at all

**Solution:**
- Verify display controller power and I2C address (default 0x29 for GT911)
- Check backlight GPIO (GPIO 45) is toggled to HIGH
- Ensure LVGL frame buffer allocation succeeded (check serial debug output)
- Confirm display orientation matches `LV_DISP_ROT_90` setting

**Problem:** Touchscreen is unresponsive

**Solution:**
- Verify GT911 I2C communication: scan I2C bus with `Wire.beginTransmission(0x29)`
- The touch callback `landscape_touch_read_cb()` is stubbed; implement actual I2C reads
- Ensure axis transformation is correct for landscape mode

---

### CAN Frame Issues

**Problem:** No CAN frames received (silent)

**Solution:**
- Verify vehicle CAN bus is active (use an external CAN analyzer to confirm frames)
- Check acceptance filters; they may block your target frame IDs:
  ```cpp
  // Channel 0 accepts 0x000–0x3FF and 0x7E8–0x7EF (diagnostic)
  f_cfg.acceptance_code = (0x000 << 21);
  f_cfg.acceptance_mask = ~((0x7F0) << 21);
  ```
  Adjust the mask to allow your frame IDs if needed

**Problem:** Frames received but metrics not updating

**Solution:**
- Verify correct interpreter is loaded (`[DECOUPLER]` log message)
- Add debug `Serial.println()` in `interpretDriveTrain()` to confirm frame is parsed
- Check byte offsets and scale factors against vehicle-specific documentation

---

### WebSocket Issues

**Problem:** Web dashboard says "Connection closed, re-linking in 2 seconds..."

**Solution:**
- Verify ESP32 is broadcasting WiFi SSID: check phone WiFi list for `Audi_S3_Telemetry`
- Confirm firewall or router settings allow WebSocket upgrades
- Check serial log for `[WEB SERVER] Remote... connected` messages
- Verify browser supports WebSocket (all modern browsers do)

---

### Memory & Performance

**Problem:** Task watchdog timeout or reboot loop

**Solution:**
- CockpitCoreProcessor task is pinned to Core 1; Core 0 should remain responsive
- Increase TWAI RX buffer if frames are dropping: check `twai_driver_install_v2()` config
- Monitor heap usage with `heap_caps_get_free_size(MALLOC_CAP_SPIRAM)`
- Reduce LVGL draw buffer if SRAM is tight (currently 51 KB)

**Problem:** High latency or janky UI updates

**Solution:**
- LVGL timer handler runs every ~33 ms; ensure `lv_timer_handler()` is called each frame
- CAN processing (`processInboundFrames()`) is non-blocking; queued internally
- Consider reducing WebSocket broadcast rate from 10 Hz to 5 Hz if bandwidth is constrained

---

## Contributing

### Code Style

- Use `snake_case` for variables and functions
- Use `UPPER_CASE` for `#define` constants
- Use `CamelCase` for class names
- Keep lines ≤100 characters
- Comment non-obvious logic; platform-specific signal extraction deserves explanation

### Adding a New Platform

1. **Create `Model_<Manufacturer>_<Platform>.h/cpp`** following existing examples
2. **Implement `BaseVehicleInterpreter` virtual methods:**
   - `interpretDriveTrain()`
   - `interpretComfort()`
   - `interpretInfotainment()`
   - `configureUiLimits()`
3. **Add to VIN decoding logic** in main `.ino`
4. **Document signal mapping** in a table (Frame ID, Byte, Scale, Offset, Unit)
5. **Test on bench** with CAN simulator before real vehicle testing
6. **Submit pull request** with example CAN logs and screenshots

### Bug Reports

Include:
- Vehicle make/model/year
- Interpreter loaded (check serial boot log)
- Serial log output of the bug (paste relevant lines)
- Expected vs. actual behavior
- Steps to reproduce

---

## Future Roadmap

- [ ] **Rename main sketch** from `Audi_S3_8V.ino` to `VehicleTelemetryDecoder.ino` or `CanBusGateway.ino`
- [ ] **OBD2 Parameter Compliance:** Add standardized PID support (SAE J1979)
- [ ] **Data Logging:** SD card integration to record CAN traces and telemetry
- [ ] **Expanded Platforms:** BMW, Mercedes, Tesla, Japanese makers (Toyota, Honda, Nissan, Subaru)
- [ ] **Advanced Diagnostics:** DTC (Diagnostic Trouble Code) reading and clearing
- [ ] **Cloud Sync:** Optional cloud backend for historical data and analytics
- [ ] **Multi-vehicle Dashboard:** View multiple vehicles' telemetry simultaneously
- [ ] **Mobile App:** Native iOS/Android app instead of web browser
- [ ] **CAN Bus Simulation:** Virtual vehicle for training and development
- [ ] **Predictive Alerts:** Machine learning anomaly detection for engine/thermal issues

---

## License

[Specify your license here, e.g., MIT, GPL-3.0, Apache-2.0, or proprietary]

---

## Contact & Support

For questions, issues, or platform integration requests:

- **GitHub Issues:** [Link to your repo issues]
- **Email:** [Your contact email, if applicable]
- **Discord/Community:** [Link if applicable]

---

## Acknowledgments

- LVGL graphics team for the excellent embedded UI library
- Espressif for comprehensive ESP32 documentation and TWAI driver
- CAN signal documentation from automotive reverse-engineering community
- Contributors who have added platform support and bug fixes

---

**Last Updated:** 2026-07-22  
**Maintainer:** Stanneh1  
**Status:** Active Development
