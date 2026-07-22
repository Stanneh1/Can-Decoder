#ifndef VEHICLE_SIMULATOR_H
#define VEHICLE_SIMULATOR_H

#include <Arduino.h>
#include "driver/twai.h"
#include "VehicleInterpreters.h"

// --- PUBLIC SIMULATOR IGNITION CONTROLS ---
void runBenchTelemetrySimulation(float target_rpm, float target_boost, float target_oil, float target_h2o);

#endif // VEHICLE_SIMULATOR_H