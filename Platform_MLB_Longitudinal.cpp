#include "VehicleInterpreters.h"

// =========================================================================
//  UNIFIED MLB DECODING CORE (USED BY ALL LONGITUDINAL CARS)
// =========================================================================
static void parseStandardMlbFrame(twai_message_t &msg) {
    // M-6: Guard against an uninitialised context during early startup.
    if (sys_ctx == nullptr) return;

    switch(msg.identifier) {
        case 0x105: { // MLB Engine Speed (RPM variant slot)
            // H-1: DLC guard prevents out-of-bounds read on malformed/RTR frames.
            if (msg.data_length_code < 2) break;
            uint16_t low_byte  = (uint16_t)(*(msg.data + 0));
            uint16_t high_byte = (uint16_t)(*(msg.data + 1));
            sys_ctx->metrics.engine_rpm = ((high_byte << 8) | low_byte) * 0.25; 
            break;
        }
        case 0x1A4: { // MLB Drivetrain Thermal Indicators
            if (msg.data_length_code < 2) break;
            // M-1: int cast prevents uint8_t underflow on cold-start raw values < 40.
            sys_ctx->metrics.oil_temp     = decode_temperature_offset(msg.data[0]);
            sys_ctx->metrics.coolant_temp = decode_temperature_offset(msg.data[1]);
            break;
        }
        case 0x2A2: { // MLB Turbocharger Absolute Manifold Pressure
            if (msg.data_length_code < 2) break;
            uint16_t low_b   = (uint16_t)(*(msg.data + 0));
            uint16_t high_b  = (uint16_t)(*(msg.data + 1));
            int raw_mbar     = ((high_b << 8) | low_b) * 10;
            sys_ctx->metrics.boost_bar = (raw_mbar - 1013) / 1000.0;
            if (sys_ctx->metrics.boost_bar < 0) sys_ctx->metrics.boost_bar = 0;
            break;
        }
        case 0x096: { // MLB Vehicle Speed (0.01 km/h per LSB)
            if (msg.data_length_code < 2) break;
            uint16_t raw_spd = (uint16_t)(*(msg.data + 0)) |
                               ((uint16_t)(*(msg.data + 1)) << 8);
            sys_ctx->metrics.vehicle_speed = raw_spd * 0.01f;
            break;
        }
        case 0x084: { // MLB Accelerator Pedal Position (0.4 % per LSB)
            if (msg.data_length_code < 1) break;
            float pct = *(msg.data + 0) * 0.4f;
            sys_ctx->metrics.throttle_pct = (pct > 100.0f) ? 100.0f : pct;
            break;
        }
        case 0x317: { // MLB Exterior Ambient Temperature (raw - 40 = °C)
            if (msg.data_length_code < 1) break;
            sys_ctx->metrics.exterior_temp = decode_temperature_offset(msg.data[0]);
            break;
        }
    }
}

static void parseStandardMlbComfort(twai_message_t &msg) {
    if (sys_ctx == nullptr) return;
    if (msg.identifier == 0x3C3) {
        if (msg.data_length_code < 1) return;
        uint8_t db = *(msg.data + 0);
        sys_ctx->metrics.driver_door_open    = (db & 0x01) != 0;
        sys_ctx->metrics.passenger_door_open  = (db & 0x02) != 0;
        sys_ctx->metrics.rear_left_door_open  = (db & 0x04) != 0;
        sys_ctx->metrics.rear_right_door_open = (db & 0x08) != 0;
    }
    else if (msg.identifier == 0x317) {
        // MLB Exterior Ambient Temperature (raw - 40 = °C)
        if (msg.data_length_code < 1) return;
        sys_ctx->metrics.exterior_temp = decode_temperature_offset(msg.data[0]);
    }
    else if (msg.identifier == 0x3BE) {
        // MLB EPB / Park-Brake status – bit 4 = handbrake applied
        if (msg.data_length_code < 1) return;
        sys_ctx->metrics.handbrake_active = (*(msg.data + 0) & 0x10) != 0;
    }
}

// =========================================================================
//  GROUP 3 IMPLEMENTATION MATRIX: PLATFORM METRICS AND VISUAL LIMITS
// =========================================================================

// --- 1. AUDI A4 MLB 8K ---
void AudiA4MLB8KInterpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMlbFrame(msg); }
void AudiA4MLB8KInterpreter::interpretComfort(twai_message_t &msg)    { parseStandardMlbComfort(msg); }
void AudiA4MLB8KInterpreter::interpretInfotainment(twai_message_t &msg) {}
void AudiA4MLB8KInterpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(180, 0, 0); // Audi Red
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 7000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 150);
}

// --- 2. AUDI A4 MLB 8W ---
void AudiA4MLB8WInterpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMlbFrame(msg); }
void AudiA4MLB8WInterpreter::interpretComfort(twai_message_t &msg)    { parseStandardMlbComfort(msg); }
void AudiA4MLB8WInterpreter::interpretInfotainment(twai_message_t &msg) {}
void AudiA4MLB8WInterpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(220, 0, 0); // Bold Digital Red
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 8000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 200);
}

// --- 3. AUDI A6 MLB C7 ---
void AudiA6MLBC7Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMlbFrame(msg); }
void AudiA6MLBC7Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardMlbComfort(msg); }
void AudiA6MLBC7Interpreter::interpretInfotainment(twai_message_t &msg) {}
void AudiA6MLBC7Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(200, 30, 0); // C7 Amber/Red mix
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 7000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 180);
}

// --- 4. AUDI A6 MLB C8 ---
void AudiA6MLBC8Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMlbFrame(msg); }
void AudiA6MLBC8Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardMlbComfort(msg); }
void AudiA6MLBC8Interpreter::interpretInfotainment(twai_message_t &msg) {}
void AudiA6MLBC8Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(180, 0, 0); // Audi Red
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 8000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 250);
}

// --- 5. AUDI A5 MLB B8 ---
void AudiA5MLBB8Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMlbFrame(msg); }
void AudiA5MLBB8Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardMlbComfort(msg); }
void AudiA5MLBB8Interpreter::interpretInfotainment(twai_message_t &msg) {}
void AudiA5MLBB8Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(180, 0, 0); // Audi Red
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 7500);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 180);
}

// --- 6. AUDI A8 MLB D4 ---
void AudiA8MLBD4Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMlbFrame(msg); }
void AudiA8MLBD4Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardMlbComfort(msg); }
void AudiA8MLBD4Interpreter::interpretInfotainment(twai_message_t &msg) {}
void AudiA8MLBD4Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(240, 240, 240); // Luxury White Dashboard
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 6500);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 150);
}

// --- 7. AUDI A8 MLB D5 ---
void AudiA8MLBD5Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMlbFrame(msg); }
void AudiA8MLBD5Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardMlbComfort(msg); }
void AudiA8MLBD5Interpreter::interpretInfotainment(twai_message_t &msg) {}
void AudiA8MLBD5Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(255, 255, 255); // Clean White Matrix
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 6500);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 150);
}

// --- 8. AUDI Q5 MLB 8R ---
void AudiQ5MLB8RInterpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMlbFrame(msg); }
void AudiQ5MLB8RInterpreter::interpretComfort(twai_message_t &msg)    { parseStandardMlbComfort(msg); }
void AudiQ5MLB8RInterpreter::interpretInfotainment(twai_message_t &msg) {}
void AudiQ5MLB8RInterpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(180, 0, 0); // Audi Red
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 6500);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 150);
}

// --- 9. AUDI Q5 MLB FY ---
void AudiQ5MLBFYInterpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMlbFrame(msg); }
void AudiQ5MLBFYInterpreter::interpretComfort(twai_message_t &msg)    { parseStandardMlbComfort(msg); }
void AudiQ5MLBFYInterpreter::interpretInfotainment(twai_message_t &msg) {}
void AudiQ5MLBFYInterpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(180, 0, 0); // Audi Red
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 7000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 180);
}

// --- 10. AUDI Q7 MLB 4M ---
void AudiQ7MLB4MInterpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMlbFrame(msg); }
void AudiQ7MLB4MInterpreter::interpretComfort(twai_message_t &msg)    { parseStandardMlbComfort(msg); }
void AudiQ7MLB4MInterpreter::interpretInfotainment(twai_message_t &msg) {}
void AudiQ7MLB4MInterpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(180, 0, 0); // Audi Red
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 6500);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 200);
}

// --- 11. PORSCHE CAYENNE 92 ---
void PorscheCayenne92Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMlbFrame(msg); }
void PorscheCayenne92Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardMlbComfort(msg); }
void PorscheCayenne92Interpreter::interpretInfotainment(twai_message_t &msg) {}
void PorscheCayenne92Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(255, 200, 0); // Sporty Porsche Yellow/Amber accent
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 6500);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 180);
}

// --- 12. PORSCHE MACAN 9B ---
void PorscheMacan9BInterpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardMlbFrame(msg); }
void PorscheMacan9BInterpreter::interpretComfort(twai_message_t &msg)    { parseStandardMlbComfort(msg); }
void PorscheMacan9BInterpreter::interpretInfotainment(twai_message_t &msg) {}
void PorscheMacan9BInterpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(255, 200, 0); // Porsche Sport Amber
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 7500);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 220);
}
