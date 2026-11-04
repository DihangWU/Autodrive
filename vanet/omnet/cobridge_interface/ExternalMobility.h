#pragma once
#include "veins/base/modules/BaseMobility.h"

class ExternalMobility : public veins::BaseMobility {
protected:
    void initialize(int stage) override;
public:
    void setPosition(double x, double y, double z, double heading);
};
