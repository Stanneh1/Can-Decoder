#ifndef MODEL_MQB_8V_H
#define MODEL_MQB_8V_H

#include "VehicleInterpreters.h"

class AudiS38VInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override;
    void interpretComfort(twai_message_t &msg) override;
    void interpretInfotainment(twai_message_t &msg) override;
    void configureUiLimits() override;
};

#endif // MODEL_MQB_8V_H
