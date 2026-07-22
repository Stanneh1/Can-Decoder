#include "Model_MQB_8V.h"

// =========================================================================
//  CLASS METHOD DEFINITIONS: AUDI S3 8V / RECENT MQB PLATFORM CODES
// =========================================================================

void AudiS38VInterpreter::interpretDriveTrain(twai_message_t &msg) {
    switch(msg.identifier) {
        case 0x0FC: {
            // MQB Engine RPM Frame (Byte 1 and Byte 0 combined * 0.25)
            // Explicitly cast to uint16_t using the pointer offsets to defeat signed bit promotion!
            uint16_t low_byte  = (uint16_t)(*(msg.data + 0));
            uint16_t high_byte = (uint16_t)(*(msg.data + 1));
            
            sys_ctx->metrics.engine_rpm = ((high_byte << 8) | low_byte) * 0.25; 
            break;
        }
            
        case 0x1A2: 
            // MQB Powertrain Thermal Mass Levels (Byte 0 = Oil Temp, Byte 1 = Coolant Temp)
            sys_ctx->metrics.oil_temp     = (float)(*(msg.data + 0) - 40); 
            sys_ctx->metrics.coolant_temp = (float)(*(msg.data + 1) - 40); 
            break;
            
        case 0x28A: {
            // MQB Turbocharger Boost Pressure Frame
            uint16_t low_b   = (uint16_t)(*(msg.data + 0));
            uint16_t high_b  = (uint16_t)(*(msg.data + 1));
            int raw_mbar     = ((high_b << 8) | low_b) * 10;
            
            sys_ctx->metrics.boost_bar = (raw_mbar - 1013) / 1000.0;
            if (sys_ctx->metrics.boost_bar < 0) {
                sys_ctx->metrics.boost_bar = 0;
            }
            break;
        }
    }
}

void AudiS38VInterpreter::interpretComfort(twai_message_t &msg) {
    // MQB Body Control Module / Climate convenience data
    if (msg.identifier == 0x61C) {
        sys_ctx->metrics.driver_door_open = (*(msg.data + 0) & 0x01);
    }
    else if (msg.identifier == 0x527) {
        sys_ctx->metrics.target_temp = *(msg.data + 0) * 0.5;
    }
}

void AudiS38VInterpreter::interpretInfotainment(twai_message_t &msg) {
    // MQB Multi-Media MMI scroll wheel input codes
    if (msg.identifier == 0x695) {
        sys_ctx->metrics.mmi_key_code = *(msg.data + 0);
    }
}

void AudiS38VInterpreter::configureUiLimits() {
    // Enforce dynamic MQB high-performance instrument parameters
    sys_ctx->normal_green = lv_color_make(180, 0, 0); // Dynamic Audi Red Illumination Theme!
    
    if (sys_ctx->rpm_meter != nullptr) {
        lv_arc_set_range(sys_ctx->rpm_meter, 0, 8000); // 8000 RPM Max
    }
    if (sys_ctx->boost_meter != nullptr) {
        lv_bar_set_range(sys_ctx->boost_meter, 0, 250); // 2.5 Bar Gauge Max
    }
}