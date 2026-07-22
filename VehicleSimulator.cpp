#include "VehicleSimulator.h"

// =========================================================================
//  CLASS METHOD DEFINITIONS: PLATFORM-AWARE TELEMETRY SIMULATOR CORE
// =========================================================================

void runBenchTelemetrySimulation(float target_rpm, float target_boost, float target_oil, float target_h2o) {
    if (sys_ctx == nullptr) return;

    // 1. Direct variable injection (Guarantees the smartphone browser stays buttery smooth)
    sys_ctx->metrics.engine_rpm   = target_rpm;
    sys_ctx->metrics.boost_bar    = target_boost;
    sys_ctx->metrics.oil_temp     = target_oil;
    sys_ctx->metrics.coolant_temp = target_h2o;

    // 2. Safely process hardware frame simulation ONLY if a real vehicle network is locked!
    // This stops the hardware registers from filling up and freezing the chip on your open desk.
    if (active_vehicle_profile.network_generation != SERIES_UNKNOWN) {
        twai_message_t tx_msg;
        tx_msg.extd = 0;
        tx_msg.rtr = 0;
        tx_msg.data_length_code = 8;

        if (active_vehicle_profile.network_generation == SERIES_MQB_A_CLASS) {
            // A. Pack Engine Speed to MQB Bus standard (ID: 0x0FC)
            tx_msg.identifier = 0x0FC;
            uint16_t raw_rpm = (uint16_t)(target_rpm / 0.25);
            *(tx_msg.data + 0) = (uint8_t)(raw_rpm & 0xFF);        
            *(tx_msg.data + 1) = (uint8_t)((raw_rpm >> 8) & 0xFF); 
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0); // Set timeout to 0 (NON-BLOCKING)

            // B. Pack Thermal Statistics to MQB Bus standard (ID: 0x1A2)
            tx_msg.identifier = 0x1A2;
            *(tx_msg.data + 0) = (uint8_t)(target_oil + 40);
            *(tx_msg.data + 1) = (uint8_t)(target_h2o + 40);
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // C. Pack Boost Pressure to MQB Bus standard (ID: 0x28A)
            tx_msg.identifier = 0x28A;
            int absolute_mbar = (int)((target_boost * 1000.0) + 1013.0);
            uint16_t raw_mbar = (uint16_t)(absolute_mbar / 10);
            *(tx_msg.data + 0) = (uint8_t)(raw_mbar & 0xFF);        
            *(tx_msg.data + 1) = (uint8_t)((raw_mbar >> 8) & 0xFF); 
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);
        } 
        else {
            // A. Pack Engine Speed to PQ Bus standard (ID: 0x280)
            tx_msg.identifier = 0x280;
            uint16_t raw_rpm = (uint16_t)(target_rpm / 0.25);
            *(tx_msg.data + 0) = (uint8_t)(raw_rpm & 0xFF);        
            *(tx_msg.data + 1) = (uint8_t)((raw_rpm >> 8) & 0xFF); 
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // B. Pack Thermal Channels to PQ Bus standard (ID: 0x288)
            tx_msg.identifier = 0x288;
            *(tx_msg.data + 0) = (uint8_t)(target_oil + 40);
            *(tx_msg.data + 1) = (uint8_t)(target_h2o + 40);
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // C. Pack Boost Pressure to PQ Bus standard (ID: 0x380)
            tx_msg.identifier = 0x380;
            int absolute_mbar = (int)((target_boost * 1000.0) + 1013.0);
            *(tx_msg.data + 0) = (uint8_t)(absolute_mbar / 10);
            for(int i = 1; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);
        }
    }
}
