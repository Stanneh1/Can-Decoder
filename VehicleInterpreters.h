#ifndef VEHICLE_INTERPRETERS_H
#define VEHICLE_INTERPRETERS_H

#include <Arduino.h>
#include "driver/twai.h"
#include <lvgl.h>

// --- ADVANCED VEHICLE DECODING BASES ---
enum MqbPlatformSeries {
    SERIES_UNKNOWN,
    SERIES_MQB_A_CLASS,    
    SERIES_MLB_LONG_CLASS, 
    SERIES_PQ35_46_LEGACY, 
    SERIES_SMALL_PO_SKODA  
};

// --- GLOBAL TELEMETRY STORAGE STRUCTURE ---
struct LiveTelemetryMetrics {
    float engine_rpm = 0.0;
    float boost_bar = 0.0;
    float peak_boost_bar = 0.0;
    float oil_temp = 0.0;
    float coolant_temp = 0.0;
    bool driver_door_open = false;
    float target_temp = 0.0;
    uint8_t mmi_key_code = 0x00;
};

struct DecodedVehicleMetrics {
    const char* brand = "VAG MOTOR CORP";
    const char* model_name = "GENERIC MODEL ARCHITECTURE";
    const char* electrical_bus = "STANDARD INFRASTRUCTURE CAN";
    int production_year = 0;
    MqbPlatformSeries network_generation = SERIES_UNKNOWN;
};

// Share variable definitions safely across files without duplicates
extern LiveTelemetryMetrics s3_live_metrics;
extern DecodedVehicleMetrics active_vehicle_profile;
extern lv_obj_t *rpm_meter;
extern lv_obj_t *boost_meter;
extern lv_color_t color_normal_green;

// --- ABSTRACT MULTI-VEHICLE PARSER INTERFACE BLUEPRINT ---
class BaseVehicleInterpreter {
public:
    virtual ~BaseVehicleInterpreter() {}
    virtual void interpretDriveTrain(twai_message_t &msg) = 0;
    virtual void interpretComfort(twai_message_t &msg) = 0;
    virtual void interpretInfotainment(twai_message_t &msg) = 0;
    virtual void configureUiLimits() = 0;
};

// Global unified tracking instance pointer allocation handle
extern BaseVehicleInterpreter* currentCarInterpreter;

// =========================================================================
//  VEHICLE SUB-CLASS CONFIGURATION PROFILES
// =========================================================================
class AudiS38VInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override;
    void interpretComfort(twai_message_t &msg) override;
    void interpretInfotainment(twai_message_t &msg) override;
    void configureUiLimits() override;
};

class AudiS38PInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override;
    void interpretComfort(twai_message_t &msg) override;
    void interpretInfotainment(twai_message_t &msg) override;
    void configureUiLimits() override;
};

class GenericVehicleInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override {}
    void interpretComfort(twai_message_t &msg) override {}
    void interpretInfotainment(twai_message_t &msg) override {}
    void configureUiLimits() override;
};

#endif // VEHICLE_INTERPRETERS_H
