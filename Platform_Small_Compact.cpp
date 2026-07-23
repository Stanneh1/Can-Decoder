#include "VehicleInterpreters.h"

// =========================================================================
//  UNIFIED COMPACT DECODING CORE (USED BY ALL SMALL COMPACT CARS)
// =========================================================================
static void parseCompactPq25Frame(twai_message_t &msg) {
    // M-6: Guard against an uninitialised context during early startup.
    if (sys_ctx == nullptr) return;

    switch(msg.identifier) {
        case 0x280: { // PQ25 Compact Engine Speed (RPM)
            // H-1: DLC guard prevents out-of-bounds read on malformed/RTR frames.
            if (msg.data_length_code < 2) break;
            uint16_t low_byte  = (uint16_t)(*(msg.data + 0));
            uint16_t high_byte = (uint16_t)(*(msg.data + 1));
            sys_ctx->metrics.engine_rpm = ((high_byte << 8) | low_byte) * 0.25;
            break;
        }
        case 0x288: { // PQ25 Thermal Indicators
            if (msg.data_length_code < 2) break;
            // M-1: int cast prevents uint8_t underflow on cold-start raw values < 40.
            sys_ctx->metrics.coolant_temp = (float)((int)msg.data[0] - 40);
            sys_ctx->metrics.oil_temp     = (float)((int)msg.data[1] - 40);
            break;
        }
        case 0x380: { // PQ25 Compact Manifold Pressure
            if (msg.data_length_code < 1) break;
            int absolute_mbar = (int)msg.data[0] * 10;
            sys_ctx->metrics.boost_bar = (absolute_mbar - 1013) / 1000.0;
            if (sys_ctx->metrics.boost_bar < 0) sys_ctx->metrics.boost_bar = 0;
            break;
        }
    }
}

static void parseCompactMqba0Frame(twai_message_t &msg) {
    if (sys_ctx == nullptr) return;

    switch(msg.identifier) {
        case 0x0FC: { // MQB A0 Engine Speed (RPM)
            if (msg.data_length_code < 2) break;
            uint16_t low_byte  = (uint16_t)(*(msg.data + 0));
            uint16_t high_byte = (uint16_t)(*(msg.data + 1));
            sys_ctx->metrics.engine_rpm = ((high_byte << 8) | low_byte) * 0.25; 
            break;
        }
        case 0x1A2: { // MQB A0 Thermal Indicators
            if (msg.data_length_code < 2) break;
            sys_ctx->metrics.oil_temp     = (float)((int)msg.data[0] - 40);
            sys_ctx->metrics.coolant_temp = (float)((int)msg.data[1] - 40);
            break;
        }
        case 0x28A: { // MQB A0 Turbocharger Pressure
            if (msg.data_length_code < 2) break;
            uint16_t low_b   = (uint16_t)(*(msg.data + 0));
            uint16_t high_b  = (uint16_t)(*(msg.data + 1));
            int raw_mbar     = ((high_b << 8) | low_b) * 10;
            sys_ctx->metrics.boost_bar = (raw_mbar - 1013) / 1000.0;
            if (sys_ctx->metrics.boost_bar < 0) sys_ctx->metrics.boost_bar = 0;
            break;
        }
    }
}

// =========================================================================
//  GROUP 4 IMPLEMENTATION MATRIX: PLATFORM METRICS AND VISUAL LIMITS
// =========================================================================

// --- 1. AUDI A1 PQ25 ---
void AudiA1PQ25Interpreter::interpretDriveTrain(twai_message_t &msg) { parseCompactPq25Frame(msg); }
void AudiA1PQ25Interpreter::interpretComfort(twai_message_t &msg) {
    if (msg.identifier == 0x351 && msg.data_length_code >= 1)
        sys_ctx->metrics.driver_door_open = (*(msg.data + 0) & 0x01);
}
void AudiA1PQ25Interpreter::interpretInfotainment(twai_message_t &msg) {}
void AudiA1PQ25Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(180, 0, 0); // Audi Red
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 7000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 150);
}

// --- 2. AUDI A1 MQB A0 ---
void AudiA1MQBA0Interpreter::interpretDriveTrain(twai_message_t &msg) { parseCompactMqba0Frame(msg); }
void AudiA1MQBA0Interpreter::interpretComfort(twai_message_t &msg) {
    if (msg.identifier == 0x61C && msg.data_length_code >= 1)
        sys_ctx->metrics.driver_door_open = (*(msg.data + 0) & 0x01);
}
void AudiA1MQBA0Interpreter::interpretInfotainment(twai_message_t &msg) {}
void AudiA1MQBA0Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(220, 0, 0); // Modern Audi Digital Red
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 7000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 180);
}

// --- 3. VW POLO PQ25 ---
void VwPoloPQ25Interpreter::interpretDriveTrain(twai_message_t &msg) { parseCompactPq25Frame(msg); }
void VwPoloPQ25Interpreter::interpretComfort(twai_message_t &msg) {
    if (msg.identifier == 0x351 && msg.data_length_code >= 1)
        sys_ctx->metrics.driver_door_open = (*(msg.data + 0) & 0x01);
}
void VwPoloPQ25Interpreter::interpretInfotainment(twai_message_t &msg) {}
void VwPoloPQ25Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(255, 255, 255); // Clean Instrument White
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 6500);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 150);
}

// --- 4. VW POLO MQB A0 ---
void VwPoloMQBA0Interpreter::interpretDriveTrain(twai_message_t &msg) { parseCompactMqba0Frame(msg); }
void VwPoloMQBA0Interpreter::interpretComfort(twai_message_t &msg) {
    if (msg.identifier == 0x61C && msg.data_length_code >= 1)
        sys_ctx->metrics.driver_door_open = (*(msg.data + 0) & 0x01);
}
void VwPoloMQBA0Interpreter::interpretInfotainment(twai_message_t &msg) {}
void VwPoloMQBA0Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(0, 100, 220); // VW Racing Blue
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 7000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 200); // GTI/Polo R Bounds
}

// --- 5. SEAT IBIZA MQB A0 ---
void SeatIbizaMQBA0Interpreter::interpretDriveTrain(twai_message_t &msg) { parseCompactMqba0Frame(msg); }
void SeatIbizaMQBA0Interpreter::interpretComfort(twai_message_t &msg) {
    if (msg.identifier == 0x61C && msg.data_length_code >= 1)
        sys_ctx->metrics.driver_door_open = (*(msg.data + 0) & 0x01);
}
void SeatIbizaMQBA0Interpreter::interpretInfotainment(twai_message_t &msg) {}
void SeatIbizaMQBA0Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(240, 20, 0); // Seat Sport Red
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 7000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 180);
}
