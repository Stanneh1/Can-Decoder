#include "VehicleInterpreters.h"

// =========================================================================
//  UNIFIED PQ DECODING CORE (USED BY ALL LEGACY TP2.0 ENGINE CLASSES)
// =========================================================================
static void parseStandardPqFrame(twai_message_t &msg) {
    switch(msg.identifier) {
        case 0x280: { // PQ Engine Speed (RPM)
            uint16_t low_byte  = (uint16_t)(*(msg.data + 0));
            uint16_t high_byte = (uint16_t)(*(msg.data + 1));
            sys_ctx->metrics.engine_rpm = ((high_byte << 8) | low_byte) * 0.25;
            break;
        }
        case 0x288: { // PQ Drivetrain Thermal Indicators
            sys_ctx->metrics.coolant_temp = (float)(*(msg.data + 0) - 40);
            sys_ctx->metrics.oil_temp     = (float)(*(msg.data + 1) - 40); 
            break;
        }
        case 0x380: { // PQ Absolute Manifold Pressure (Boost)
            int absolute_mbar = *(msg.data + 0) * 10; 
            sys_ctx->metrics.boost_bar = (absolute_mbar - 1013) / 1000.0;
            if (sys_ctx->metrics.boost_bar < 0) sys_ctx->metrics.boost_bar = 0;
            break;
        }
    }
}

static void parseStandardPqComfort(twai_message_t &msg) {
    if (msg.identifier == 0x351) { 
        sys_ctx->metrics.driver_door_open = (*(msg.data + 0) & 0x01);
    }
}

// =========================================================================
//  GROUP 2 IMPLEMENTATION MATRIX: PLATFORM METRICS AND VISUAL LIMITS
// =========================================================================

// --- 1. AUDI S3 8P ---
void AudiS38PInterpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardPqFrame(msg); }
void AudiS38PInterpreter::interpretComfort(twai_message_t &msg)    { parseStandardPqComfort(msg); }
void AudiS38PInterpreter::interpretInfotainment(twai_message_t &msg) { if (msg.identifier == 0x5C1) sys_ctx->metrics.mmi_key_code = *(msg.data + 0); }
void AudiS38PInterpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(220, 0, 0); // Bold Audi Instrument Red
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 8000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 200); // 2.0 Bar Max
}

// --- 2. AUDI A6 C6 ---
void AudiA6C6Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardPqFrame(msg); }
void AudiA6C6Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardPqComfort(msg); }
void AudiA6C6Interpreter::interpretInfotainment(twai_message_t &msg) {}
void AudiA6C6Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(200, 30, 0); // Warm C6 Cabin Amber/Red
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 7000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 150);
}

// --- 3. AUDI Q3 (PQ35) ---
void AudiQ3PQ35Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardPqFrame(msg); }
void AudiQ3PQ35Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardPqComfort(msg); }
void AudiQ3PQ35Interpreter::interpretInfotainment(twai_message_t &msg) {}
void AudiQ3PQ35Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(220, 0, 0); // Audi Red
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 7000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 150);
}

// --- 4. AUDI Q7 4L ---
void AudiQ74LInterpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardPqFrame(msg); }
void AudiQ74LInterpreter::interpretComfort(twai_message_t &msg)    { parseStandardPqComfort(msg); }
void AudiQ74LInterpreter::interpretInfotainment(twai_message_t &msg) { if (msg.identifier == 0x5C1) sys_ctx->metrics.mmi_key_code = *(msg.data + 0); }
void AudiQ74LInterpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(200, 50, 0); // Traditional Audi Amber/Red mix
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 6000); // Diesel Utility bounds
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 150);
}

// --- 5. AUDI TT MK2 ---
void AudiTTMk2Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardPqFrame(msg); }
void AudiTTMk2Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardPqComfort(msg); }
void AudiTTMk2Interpreter::interpretInfotainment(twai_message_t &msg) {}
void AudiTTMk2Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(220, 0, 0); // Sport Cockpit Red
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 8000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 200);
}

// --- 6. VW GOLF MK5 / MK6 / JETTA ---
void VwGolf56Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardPqFrame(msg); }
void VwGolf56Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardPqComfort(msg); }
void VwGolf56Interpreter::interpretInfotainment(twai_message_t &msg) {}
void VwGolf56Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(255, 255, 255); // Clean White/Blue backlight look
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 7000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 200);
}

// --- 7. VW PASSAT B6 / B7 / CC ---
void VwPassatB67Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardPqFrame(msg); }
void VwPassatB67Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardPqComfort(msg); }
void VwPassatB67Interpreter::interpretInfotainment(twai_message_t &msg) {}
void VwPassatB67Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(255, 255, 255); // Passat White Inst Panel
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 6000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 150);
}

// --- 8. VW SCIROCCO ---
void VwSciroccoInterpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardPqFrame(msg); }
void VwSciroccoInterpreter::interpretComfort(twai_message_t &msg)    { parseStandardPqComfort(msg); }
void VwSciroccoInterpreter::interpretInfotainment(twai_message_t &msg) {}
void VwSciroccoInterpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(255, 255, 255); // Scirocco White Instrumentation
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 7000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 200);
}

// --- 9. VW TIGUAN MK1 ---
void VwTiguanMk1Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardPqFrame(msg); }
void VwTiguanMk1Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardPqComfort(msg); }
void VwTiguanMk1Interpreter::interpretInfotainment(twai_message_t &msg) {}
void VwTiguanMk1Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(255, 255, 255); // Crisp White Dial Plates
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 6500);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 150);
}

// --- 10. SEAT LEON MK2 ---
void SeatLeonMk2Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardPqFrame(msg); }
void SeatLeonMk2Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardPqComfort(msg); }
void SeatLeonMk2Interpreter::interpretInfotainment(twai_message_t &msg) {}
void SeatLeonMk2Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(240, 20, 0); // Sporty Seat Red
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 7500);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 200);
}

// --- 11. SKODA OCTAVIA MK2 ---
void SkodaOctaviaMk2Interpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardPqFrame(msg); }
void SkodaOctaviaMk2Interpreter::interpretComfort(twai_message_t &msg)    { parseStandardPqComfort(msg); }
void SkodaOctaviaMk2Interpreter::interpretInfotainment(twai_message_t &msg) {}
void SkodaOctaviaMk2Interpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(0, 200, 0); // Classic vRS Green
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 7000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 200);
}

// --- 12. SKODA SUPERB 3T ---
void SkodaSuperb3TInterpreter::interpretDriveTrain(twai_message_t &msg) { parseStandardPqFrame(msg); }
void SkodaSuperb3TInterpreter::interpretComfort(twai_message_t &msg)    { parseStandardPqComfort(msg); }
void SkodaSuperb3TInterpreter::interpretInfotainment(twai_message_t &msg) {}
void SkodaSuperb3TInterpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(220, 220, 200); // Executive Warm White
    if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 6500);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 150);
}
