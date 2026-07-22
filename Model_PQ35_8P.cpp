#include "Model_PQ35_8P.h"

// =========================================================================
//  CLASS METHOD DEFINITIONS: AUDI S3 (PQ35 8P PLATFORM ERA ERASE MATH)
// =========================================================================

void AudiS38PInterpreter::interpretDriveTrain(twai_message_t &msg) {
    switch(msg.identifier) {
        case 0x280: { // PQ35 Engine RPM Frame
            uint16_t low_byte  = (uint16_t)(*(msg.data + 0));
            uint16_t high_byte = (uint16_t)(*(msg.data + 1));
            sys_ctx->metrics.engine_rpm = ((high_byte << 8) | low_byte) * 0.25;
            break;
        }
            
        case 0x288: // PQ35 Coolant / Engine Temps Frame
            sys_ctx->metrics.coolant_temp = *(msg.data + 0) - 40;
            sys_ctx->metrics.oil_temp     = *(msg.data + 1) - 40; 
            break;
            
        case 0x380: { // PQ35 Boost / Ambient Context Frame
            int absolute_mbar = *(msg.data + 0) * 10; 
            sys_ctx->metrics.boost_bar = (absolute_mbar - 1013) / 1000.0;
            if (sys_ctx->metrics.boost_bar < 0) {
                sys_ctx->metrics.boost_bar = 0;
            }
            break;
        }
    }
}

void AudiS38PInterpreter::interpretComfort(twai_message_t &msg) {
    if (msg.identifier == 0x351) { 
        sys_ctx->metrics.driver_door_open = (*(msg.data + 0) & 0x01);
    }
}

void AudiS38PInterpreter::interpretInfotainment(twai_message_t &msg) {
    if (msg.identifier == 0x5C1) {
        sys_ctx->metrics.mmi_key_code = *(msg.data + 0);
    }
}

void AudiS38PInterpreter::configureUiLimits() {
    sys_ctx->normal_green = lv_color_make(220, 0, 0); // Bold Audi Instrument Red Theme
    
    if (sys_ctx->rpm_meter != nullptr) {
        lv_arc_set_range(sys_ctx->rpm_meter, 0, 8000); 
    }
    if (sys_ctx->boost_meter != nullptr) {
        lv_bar_set_range(sys_ctx->boost_meter, 0, 200); 
    }
}