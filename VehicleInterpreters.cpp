#include "VehicleInterpreters.h"

// Initialize as a completely safe, empty memory pointer layout first
GlobalFrameworkContext* sys_ctx = nullptr;

void GenericVehicleInterpreter::configureUiLimits() {
    if (sys_ctx != nullptr) {
        sys_ctx->normal_green = lv_color_make(0, 180, 0); 
        if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 6000);
        if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 150);
    }
}
