#ifndef MODEL_PQ47_4L_H
#define MODEL_PQ47_4L_H

#include "VehicleInterpreters.h"

class AudiQ74LInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override;
    void interpretComfort(twai_message_t &msg) override;
    void interpretInfotainment(twai_message_t &msg) override;
    void configureUiLimits() override;
};

#endif // MODEL_PQ47_4L_H