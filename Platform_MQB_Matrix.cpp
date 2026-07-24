#include "VehicleInterpreters.h"

// =========================================================================
//  UNIFIED MQB DECODING CORE (USED BY ALL MQB ENGINE CLASSES)
// =========================================================================
static void parseStandardMqbFrame(twai_message_t &msg) {
    // M-6: Guard against an uninitialised context during early startup.
    if (sys_ctx == nullptr) return;

    switch(msg.identifier) {
        case 0x0FC: { // MQB Engine Speed (RPM)
            // H-1: Validate DLC before accessing data bytes to prevent out-of-bounds reads
            //      caused by malformed or remote frames (RTR) with DLC < 2.
            if (msg.data_length_code < 2) break;
            uint16_t low_byte  = (uint16_t)(*(msg.data + 0));
            uint16_t high_byte = (uint16_t)(*(msg.data + 1));
            sys_ctx->metrics.engine_rpm = ((high_byte << 8) | low_byte) * 0.25; 
            break;
        }
        case 0x1A2: { // MQB Drivetrain Thermal Indicators
            if (msg.data_length_code < 2) break;
            // M-1: Cast to int before subtracting 40 to prevent uint8_t underflow.
            //      A raw byte of 0 (representing -40°C) would wrap to 216 without this cast,
            //      falsely triggering the thermal alarm on cold start.
            sys_ctx->metrics.oil_temp     = decode_temperature_offset(msg.data[0]);
            sys_ctx->metrics.coolant_temp = decode_temperature_offset(msg.data[1]);
            break;
        }
        case 0x28A: { // MQB Turbocharger Absolute Manifold Pressure
            if (msg.data_length_code < 2) break;
            uint16_t low_b   = (uint16_t)(*(msg.data + 0));
            uint16_t high_b  = (uint16_t)(*(msg.data + 1));
            int raw_mbar     = ((high_b << 8) | low_b) * 10;
            sys_ctx->metrics.boost_bar = (raw_mbar - 1013) / 1000.0;
            if (sys_ctx->metrics.boost_bar < 0) sys_ctx->metrics.boost_bar = 0;
            break;
        }
        case 0x096: { // MQB Vehicle Speed (Kombi_1, 0.01 km/h per LSB)
            if (msg.data_length_code < 2) break;
            uint16_t raw_spd = (uint16_t)(*(msg.data + 0)) |
                               ((uint16_t)(*(msg.data + 1)) << 8);
            sys_ctx->metrics.vehicle_speed = raw_spd * 0.01f;
            break;
        }
        case 0x084: { // MQB Accelerator Pedal Position (0.4 % per LSB)
            if (msg.data_length_code < 1) break;
            float pct = *(msg.data + 0) * 0.4f;
            sys_ctx->metrics.throttle_pct = (pct > 100.0f) ? 100.0f : pct;
            break;
        }
        case 0x317: { // MQB Exterior Ambient Temperature (raw - 40 = °C)
            if (msg.data_length_code < 1) break;
            sys_ctx->metrics.exterior_temp = decode_temperature_offset(msg.data[0]);
            break;
        }
    }
}

static void parseStandardMqbComfort(twai_message_t &msg) {
    if (sys_ctx == nullptr) return;
    if (msg.identifier == 0x61C) {
        if (msg.data_length_code < 1) return;
        uint8_t db = *(msg.data + 0);
        sys_ctx->metrics.driver_door_open    = (db & 0x01) != 0;
        sys_ctx->metrics.passenger_door_open  = (db & 0x02) != 0;
        sys_ctx->metrics.rear_left_door_open  = (db & 0x04) != 0;
        sys_ctx->metrics.rear_right_door_open = (db & 0x08) != 0;
    }
    else if (msg.identifier == 0x527) {
        if (msg.data_length_code < 1) return;
        sys_ctx->metrics.target_temp = *(msg.data + 0) * 0.5;
    }
    else if (msg.identifier == 0x3BE) {
        // MQB EPB / Park-Brake status – bit 4 = electric handbrake applied
        if (msg.data_length_code < 1) return;
        sys_ctx->metrics.handbrake_active = (*(msg.data + 0) & 0x10) != 0;
    }
}

// =========================================================================
//  GROUP 1 IMPLEMENTATION MATRIX: PLATFORM METRICS AND VISUAL LIMITS
// =========================================================================

// --- 1. AUDI S3 8V ---
void AudiS38VInterpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMqbFrame(msg); }
void AudiS38VInterpreter::interpretComfort(twai_message_t &msg)    { parseStandardMqbComfort(msg); }
void AudiS38VInterpreter::interpretInfotainment(twai_message_t &msg) {
    if (msg.identifier == 0x695 && msg.data_length_code >= 1)
        sys_ctx->metrics.mmi_key_code = *(msg.data + 0);
}
void AudiS38VInterpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(180, 0, 0); // Audi Performance Red
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 8000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 250); // 2.5 Bar Max
}

// --- 2. AUDI RS3 GY (MQB EVO) ---
void AudiRS3GYInterpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMqbFrame(msg); }
void AudiRS3GYInterpreter::interpretComfort(twai_message_t &msg)    { parseStandardMqbComfort(msg); }
void AudiRS3GYInterpreter::interpretInfotainment(twai_message_t &msg) {
    if (msg.identifier == 0x695 && msg.data_length_code >= 1)
        sys_ctx->metrics.mmi_key_code = *(msg.data + 0);
}
void AudiRS3GYInterpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(200, 0, 0); // RS Crimson Red
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 8500); // Higher 5-Cylinder Redline
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 250);
}

// --- 3. VW GOLF MK7 / GTI / R ---
void VwGolf7Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMqbFrame(msg); }
void VwGolf7Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardMqbComfort(msg); }
void VwGolf7Interpreter::interpretInfotainment(twai_message_t &msg) {}
void VwGolf7Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(0, 100, 220); // VW Racing Signature Blue
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 7000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 200); // 2.0 Bar Max
}

// --- 4. VW GOLF MK8 / GTI / R ---
void VwGolf8Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMqbFrame(msg); }
void VwGolf8Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardMqbComfort(msg); }
void VwGolf8Interpreter::interpretInfotainment(twai_message_t &msg) {}
void VwGolf8Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(0, 240, 255); // Golf 8 Neon Cyan Digital Theme
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 7000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 250);
}

// --- 5. VW PASSAT B8 ---
void VwPassatB8Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMqbFrame(msg); }
void VwPassatB8Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardMqbComfort(msg); }
void VwPassatB8Interpreter::interpretInfotainment(twai_message_t &msg) {}
void VwPassatB8Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(255, 255, 255); // Clean Passat White Backlighting
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 6000); // Diesel/TDI Balance
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 150);
}

// --- 6. VW PASSAT B9 ---
void VwPassatB9Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMqbFrame(msg); }
void VwPassatB9Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardMqbComfort(msg); }
void VwPassatB9Interpreter::interpretInfotainment(twai_message_t &msg) {}
void VwPassatB9Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(200, 220, 255); // Soft Executive Ambient Blue
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 6000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 150);
}

// --- 7. VW TIGUAN MK2 ---
void VwTiguanMk2Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMqbFrame(msg); }
void VwTiguanMk2Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardMqbComfort(msg); }
void VwTiguanMk2Interpreter::interpretInfotainment(twai_message_t &msg) {}
void VwTiguanMk2Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(255, 255, 255); // Crisp Instrument White
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 6500);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 150);
}

// --- 8. VW TIGUAN MK3 ---
void VwTiguanMk3Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMqbFrame(msg); }
void VwTiguanMk3Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardMqbComfort(msg); }
void VwTiguanMk3Interpreter::interpretInfotainment(twai_message_t &msg) {}
void VwTiguanMk3Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(100, 255, 100); // Eco Active Green Ambient
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 6500);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 150);
}

// --- 9. VW ARTEON ---
void VwArteonInterpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMqbFrame(msg); }
void VwArteonInterpreter::interpretComfort(twai_message_t &msg)    { parseStandardMqbComfort(msg); }
void VwArteonInterpreter::interpretInfotainment(twai_message_t &msg) {}
void VwArteonInterpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(230, 230, 250); // Premium Soft Amber
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 7000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 200);
}

// --- 10. SEAT LEON MK3 ---
void SeatLeonMk3Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMqbFrame(msg); }
void SeatLeonMk3Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardMqbComfort(msg); }
void SeatLeonMk3Interpreter::interpretInfotainment(twai_message_t &msg) {}
void SeatLeonMk3Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(240, 20, 0); // Vibrant Spanish Red Theme
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 7500);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 220);
}

// --- 11. CUPRA LEON / FORMENTOR ---
void CupraLeonFormentorInterpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMqbFrame(msg); }
void CupraLeonFormentorInterpreter::interpretComfort(twai_message_t &msg)    { parseStandardMqbComfort(msg); }
void CupraLeonFormentorInterpreter::interpretInfotainment(twai_message_t &msg) {}
void CupraLeonFormentorInterpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(212, 115, 45); // Cupra Premium Copper Theme
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 8000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 250);
}

// --- 12. SKODA OCTAVIA MK3 ---
void SkodaOctaviaMk3Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMqbFrame(msg); }
void SkodaOctaviaMk3Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardMqbComfort(msg); }
void SkodaOctaviaMk3Interpreter::interpretInfotainment(twai_message_t &msg) {}
void SkodaOctaviaMk3Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(0, 200, 0); // Skoda vRS Motorsport Green
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 7000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 200);
}

// --- 13. SKODA OCTAVIA MK4 ---
void SkodaOctaviaMk4Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMqbFrame(msg); }
void SkodaOctaviaMk4Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardMqbComfort(msg); }
void SkodaOctaviaMk4Interpreter::interpretInfotainment(twai_message_t &msg) {}
void SkodaOctaviaMk4Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(50, 220, 50); // Neon vRS Green Layout
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 7000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 220);
}

// --- 14. SKODA SUPERB (MQB) ---
void SkodaSuperbMQBInterpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMqbFrame(msg); }
void SkodaSuperbMQBInterpreter::interpretComfort(twai_message_t &msg)    { parseStandardMqbComfort(msg); }
void SkodaSuperbMQBInterpreter::interpretInfotainment(twai_message_t &msg) {}
void SkodaSuperbMQBInterpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(220, 220, 200); // Superb Warm Executive White
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 6500);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 180);
}

// --- 15. AUDI Q3 (MQB) ---
void AudiQ3MQBInterpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMqbFrame(msg); }
void AudiQ3MQBInterpreter::interpretComfort(twai_message_t &msg)    { parseStandardMqbComfort(msg); }
void AudiQ3MQBInterpreter::interpretInfotainment(twai_message_t &msg) {}
void AudiQ3MQBInterpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(180, 0, 0); // Audi Red
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 7000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 180);
}

// --- 16. AUDI TT MK3 ---
void AudiTTMk3Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMqbFrame(msg); }
void AudiTTMk3Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardMqbComfort(msg); }
void AudiTTMk3Interpreter::interpretInfotainment(twai_message_t &msg) {}
void AudiTTMk3Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(180, 0, 0); // Audi Cockpit Red
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 8000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 240);
}

// --- 17. AUDI Q2 ---
void AudiQ2Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMqbFrame(msg); }
void AudiQ2Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardMqbComfort(msg); }
void AudiQ2Interpreter::interpretInfotainment(twai_message_t &msg) {}
void AudiQ2Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(180, 0, 0); // Standard Audi Red
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 7000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 160);
}
