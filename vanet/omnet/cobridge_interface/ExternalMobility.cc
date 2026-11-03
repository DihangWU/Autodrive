#include "ExternalMobility.h"
#include <cmath>
Define_Module(ExternalMobility);

void ExternalMobility::setPosition(double x, double y, double z, double heading) {
    Enter_Method_Silent();
    // Freeze the last committed traffic position during the network window:
    // extrapolation would invent traffic motion not yet committed by CoBridge.
    move.setStart(veins::Coord(x, y, z), simTime());
    move.setSpeed(0);
    move.setOrientationByVector(veins::Coord(std::cos(heading), std::sin(heading), 0));
    move.setDirectionByTarget(veins::Coord(x + std::cos(heading), y + std::sin(heading), z));
    updatePosition();
}
