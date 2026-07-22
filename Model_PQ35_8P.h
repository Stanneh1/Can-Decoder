#ifndef MODEL_PQ35_8P_H
#define MODEL_PQ35_8P_H

#include "VehicleInterpreters.h"

class AudiS38PInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override;
    void interpretComfort(twai_message_t &msg) override;
    void interpretInfotainment(twai_message_t &msg) override;
    void configureUiLimits() override;
};

#endif // MODEL_PQ35_8P_H
