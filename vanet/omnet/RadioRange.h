#pragma once
#include <omnetpp.h>
#include "veins/base/utils/Coord.h"

// Coverage circles are measured in the map's XY plane. Antenna height still
// contributes to the existing Veins propagation/path-loss calculation.
inline bool isRsuNode(omnetpp::cModule* node) {
    auto* app = node->getSubmodule("appl");
    return app && app->par("isRsu").boolValue();
}
inline double linkRange(omnetpp::cModule* a, omnetpp::cModule* b) {
    return a->getParentModule()->par(isRsuNode(a) || isRsuNode(b) ? "rsuRange" : "vehicleRange").doubleValue();
}
inline bool withinRange(omnetpp::cModule* a, const veins::Coord& pa,
                        omnetpp::cModule* b, const veins::Coord& pb) {
    const double dx = pa.x - pb.x, dy = pa.y - pb.y;
    const double radius = linkRange(a, b);
    return dx * dx + dy * dy <= radius * radius + 1e-8;
}
