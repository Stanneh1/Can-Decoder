#ifndef VEHICLE_INTERPRETERS_H
#define VEHICLE_INTERPRETERS_H

#include <Arduino.h>
#include "driver/twai.h"
#include <lvgl.h>
#include "freertos/portmacro.h"
#include <atomic>

// --- CAN FILTER HELPER ---
// SJA1000/ESP32 TWAI: standard 11-bit IDs are stored in bits [31:21] of the
// 32-bit acceptance_code/acceptance_mask register.
static constexpr int CAN_FILTER_ID_SHIFT = 21;

// Decode raw VW/Audi temperature byte (value = Celsius + 40 offset, uint8_t).
// Promotes to int before subtracting to avoid uint8_t underflow at cold start
// (raw < 40 would wrap to 216+ as unsigned).
static inline float decode_temperature_offset(uint8_t raw) {
    return (float)((int)raw - 40);
}

// --- ADVANCED VEHICLE DECODING ENUMS ---
enum MqbPlatformSeries {
    SERIES_UNKNOWN,
    SERIES_MQB_A_CLASS,    
    SERIES_MLB_LONG_CLASS, 
    SERIES_PQ35_46_LEGACY, 
    SERIES_SMALL_PO_SKODA  
};

// --- GLOBAL TELEMETRY STRUCT LAYOUT TEMPLATES ---
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

// --- ABSTRACT MULTI-VEHICLE PARSER INTERFACE BLUEPRINT ---
class BaseVehicleInterpreter {
public:
    virtual ~BaseVehicleInterpreter() {}
    virtual void interpretDriveTrain(twai_message_t &msg) = 0;
    virtual void interpretComfort(twai_message_t &msg) = 0;
    virtual void interpretInfotainment(twai_message_t &msg) = 0;
    virtual void configureUiLimits() = 0;
};

// =========================================================================
//  CLEAN GLOBAL STORAGE CONFIGURATIONS
// =========================================================================
struct GlobalFrameworkContext {
    LiveTelemetryMetrics metrics;
    DecodedVehicleMetrics profile;
    BaseVehicleInterpreter* interpreter = nullptr;
    
    // UI Layout tracking pointer references
    lv_obj_t* tv = nullptr;
    lv_obj_t* rpm_meter = nullptr;
    lv_obj_t* boost_meter = nullptr;
    lv_obj_t* oil_arc = nullptr;
    lv_obj_t* coolant_arc = nullptr;
    
    lv_color_t normal_green;
};

// --- GLOBAL EXTERN LINKS SHARED ACROSS ALL WORKSPACES ---
extern GlobalFrameworkContext* sys_ctx;
extern DecodedVehicleMetrics active_vehicle_profile;
extern twai_handle_t twai_ports[]; 

extern lv_obj_t *rpm_meter;
extern lv_obj_t *boost_meter;
extern lv_color_t color_normal_green;

// --- MULTICORE SYNCHRONISATION PRIMITIVES ---
// g_metrics_mux: spinlock protecting sys_ctx->metrics fields against Core-0/Core-1 races.
// g_interpreter_mutex: FreeRTOS mutex protecting sys_ctx->interpreter and active_vehicle_profile.
extern portMUX_TYPE      g_metrics_mux;
extern SemaphoreHandle_t g_interpreter_mutex;

// g_twai0_valid: atomic flag that is cleared to false before the bus-off recovery on Core 1
// uninstalls the TWAI port-0 driver, and restored to true once the reinstall completes.
// Core 0's runBenchTelemetrySimulation checks this flag before every twai_transmit_v2 call
// on port 0, preventing a use-after-free crash when the handle is momentarily invalid.
extern std::atomic<bool> g_twai0_valid;

// =========================================================================
//  BENCH COMPACT GENERIC FALLBACK INTERPRETER BLUEPRINT
// =========================================================================
class GenericVehicleInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override {}
    void interpretComfort(twai_message_t &msg) override {}
    void interpretInfotainment(twai_message_t &msg) override {}
    void configureUiLimits() override;
};

// =========================================================================
//  FORWARD DECLARATIONS: GROUP 1 - MQB MATRIX VEHICLES (Platform_MQB_Matrix.h)
// =========================================================================
class AudiS38VInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiRS3GYInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwGolf7Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwGolf8Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwPassatB8Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwPassatB9Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwTiguanMk2Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwTiguanMk3Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwArteonInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class SeatLeonMk3Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class CupraLeonFormentorInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class SkodaOctaviaMk3Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class SkodaOctaviaMk4Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class SkodaSuperbMQBInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiQ3MQBInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiTTMk3Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiQ2Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};

// =========================================================================
//  FORWARD DECLARATIONS: GROUP 2 - PQ LEGACY INFRASTRUCTURE (Platform_PQ_Legacy.h)
// =========================================================================
class AudiS38PInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiA6C6Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiQ3PQ35Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiQ74LInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiTTMk2Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwGolf56Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwPassatB67Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwSciroccoInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwTiguanMk1Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class SeatLeonMk2Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class SkodaOctaviaMk2Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class SkodaSuperb3TInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};

// =========================================================================
//  FORWARD DECLARATIONS: GROUP 3 - MLB LONGITUDINAL BUS (Platform_MLB_Longitudinal.h)
// =========================================================================
class AudiA4MLB8KInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiA4MLB8WInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiA6MLBC7Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiA6MLBC8Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiA5MLBB8Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiA8MLBD4Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiA8MLBD5Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiQ5MLB8RInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiQ5MLBFYInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiQ7MLB4MInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class PorscheCayenne92Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class PorscheMacan9BInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};

// =========================================================================
//  FORWARD DECLARATIONS: GROUP 4 - SMALL COMPACT COMPACT PLATFORMS (Platform_Small_Compact.h)
// =========================================================================
class AudiA1PQ25Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiA1MQBA0Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwPoloPQ25Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwPoloMQBA0Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class SeatIbizaMQBA0Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};

#endif // VEHICLE_INTERPRETERS_H
