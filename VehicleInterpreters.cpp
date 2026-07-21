#include "VehicleInterpreters.h"

// =========================================================================
//  CLASS METHOD DEFINITIONS - ISOLATED MATH PROCESSING STAGE
// =========================================================================

// --- AUDI S3 8V (MQB PLATFORM ERA) ---
void AudiS38VInterpreter::interpretDriveTrain(twai_message_t &msg) {
    switch(msg.identifier) {
        case 0x0FC: s3_live_metrics.engine_rpm = ((msg.data[1] << 8) | msg.data[0]) * 0.25; break;
        case 0x1A2: s3_live_metrics.oil_temp = msg.data[0] - 40; s3_live_metrics.coolant_temp = msg.data[1] - 40; break;
        case 0x28A: {
            int raw_mbar = ((msg.data[1] << 8) | msg.data[0]) * 10;
            s3_live_metrics.boost_bar = (raw_mbar - 1013) / 1000.0;
            if (s3_live_metrics.boost_bar < 0) s3_live_metrics.boost_bar = 0;
            break;
        }
    }
}
void AudiS38VInterpreter::interpretComfort(twai_message_t &msg) {
    if (msg.identifier == 0x61C) s3_live_metrics.driver_door_open = msg.data[0] & 0x01;
    else if (msg.identifier == 0x527) s3_live_metrics.target_temp = msg.data[0] * 0.5;
}
void AudiS38VInterpreter::interpretInfotainment(twai_message_t &msg) {
    if (msg.identifier == 0x695) s3_live_metrics.mmi_key_code = msg.data[0];
}
void AudiS38VInterpreter::configureUiLimits() {
    color_normal_green = lv_color_make(180, 0, 0); // Audi Performance Red Theme
    if (rpm_meter != NULL) lv_arc_set_range(rpm_meter, 0, 8000);
    if (boost_meter != NULL) lv_bar_set_range(boost_meter, 0, 250);
}

// --- AUDI S3 8P (PQ35 PLATFORM ERA) ---
void AudiS38PInterpreter::interpretDriveTrain(twai_message_t &msg) {
    switch(msg.identifier) {
        case 0x280: s3_live_metrics.engine_rpm = ((msg.data[1] << 8) | msg.data[0]) * 0.25; break;
        case 0x288: s3_live_metrics.coolant_temp = msg.data[0] - 40; s3_live_metrics.oil_temp = msg.data[0] - 40; break;
        case 0x380: {
            int absolute_mbar = msg.data[0] * 10;
            s3_live_metrics.boost_bar = (absolute_mbar - 1013) / 1000.0;
            if (s3_live_metrics.boost_bar < 0) s3_live_metrics.boost_bar = 0;
            break;
        }
    }
}
void AudiS38PInterpreter::interpretComfort(twai_message_t &msg) {
    if (msg.identifier == 0x351) s3_live_metrics.driver_door_open = msg.data[0] & 0x01;
}
void AudiS38PInterpreter::interpretInfotainment(twai_message_t &msg) {
    if (msg.identifier == 0x5C1) s3_live_metrics.mmi_key_code = msg.data[0];
}
void AudiS38PInterpreter::configureUiLimits() {
    color_normal_green = lv_color_make(220, 0, 0); // Bold Audi Red Theme
    if (rpm_meter != NULL) lv_arc_set_range(rpm_meter, 0, 8000);
    if (boost_meter != NULL) lv_bar_set_range(boost_meter, 0, 200);
}

// --- BENCH COMPACT GENERIC FALLBACK ---
void GenericVehicleInterpreter::configureUiLimits() {
    color_normal_green = lv_color_make(0, 180, 0); // Default Baseline Green Theme
    if (rpm_meter != NULL) lv_arc_set_range(rpm_meter, 0, 6000);
    if (boost_meter != NULL) lv_bar_set_range(boost_meter, 0, 150);
}
