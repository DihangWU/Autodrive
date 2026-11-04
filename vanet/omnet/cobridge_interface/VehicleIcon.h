#pragma once
#include <omnetpp.h>

// Top-down car, front along local +X; coordinates and heading are CARLA metres/degrees.
omnetpp::cGroupFigure* createVehicleIcon(const char* name, bool lincoln);
void positionVehicleIcon(omnetpp::cGroupFigure* icon, double x, double y, double heading);
