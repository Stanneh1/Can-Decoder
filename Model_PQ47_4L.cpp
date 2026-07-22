#include "Model_PQ47_4L.h"

// =========================================================================
//  CLASS METHOD DEFINITIONS: AUDI Q7 (PQ47 4L LEGACY PLATFORM TRANSLATIONS)
// =========================================================================

void AudiQ74LInterpreter::interpretDriveTrain(twai_message_t &msg) {
    switch(msg.identifier) {
        case 0x280: // Legacy Powertrain Engine RPM Frame
            sys_ctx->metrics.engine_rpm = ((msg.data[2] << 8) | msg.data[1]) * 0.25;
            break;
            
        case 0x288: // Legacy Engine Coolant Thermal Mass Levels
            sys_ctx->metrics.coolant_temp = msg.data[0] - 40;
            sys_ctx->metrics.oil_temp = msg.data[1] - 40; // Variant dependent sensor stream
            break;
            
        case 0x380: { // Legacy Manifold Absolute Pressure (MAP) Frame
            int absolute_mbar = msg.data[1] * 10; 
            sys_ctx->metrics.boost_bar = (absolute_mbar - 1013) / 1000.0;
            if (sys_ctx->metrics.boost_bar < 0) {
                sys_ctx->metrics.boost_bar = 0;
            }
            break;
        }
    }
}

void AudiQ74LInterpreter::interpretComfort(twai_message_t &msg) {
    // 4L Legacy Comfort Convenience status mapping
    if (msg.identifier == 0x351) { 
        sys_ctx->metrics.driver_door_open = msg.data[0] & 0x01;
    }
}

void AudiQ74LInterpreter::interpretInfotainment(twai_message_t &msg) {
    // Legacy 2G/3G High MMI volume control wheel tracking
    if (msg.identifier == 0x5C1) {
        sys_ctx->metrics.mmi_key_code = msg.data[0];
    }
}

void AudiQ74LInterpreter::configureUiLimits() {
    // Enforce massive SUV instrument panel aesthetics (lower RPM limits!)
    sys_ctx->normal_green = lv_color_make(200, 50, 0); // Traditional Audi Amber/Red mix illumination
    
    if (sys_ctx->rpm_meter != nullptr) {
        lv_arc_set_range(sys_ctx->rpm_meter, 0, 6000); // 6000 RPM Max (Diesel / Utility baseline limits)
    }
    if (sys_ctx->boost_meter != nullptr) {
        lv_bar_set_range(sys_ctx->boost_meter, 0, 150); // 1.5 Bar Gauge Max Max
    }
}