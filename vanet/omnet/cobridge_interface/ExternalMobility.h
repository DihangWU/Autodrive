#pragma once
#include "veins/base/modules/BaseMobility.h"

class ExternalMobility : public veins::BaseMobility {
public:
    void setPosition(double x, double y, double z, double heading);
};
