#pragma once
#include <omnetpp.h>
#include <algorithm>
#include "veins/base/utils/Coord.h"

// Coverage circles are measured in the map's XY plane. Antenna height still
// contributes to the existing Veins propagation/path-loss calculation.
inline bool isRsuNode(omnetpp::cModule* node) {
    auto* app = node->getSubmodule("appl");
    return app && app->par("isRsu").boolValue();
}
inline double nodeRange(omnetpp::cModule* node) {
    double radius = node->par("communicationRange").doubleValue();
    return radius > 0 ? radius : node->getParentModule()->par(isRsuNode(node) ? "rsuRange" : "vehicleRange").doubleValue();
}
inline double linkRange(bool aRsu, double aRadius, bool bRsu, double bRadius) {
    if (aRsu != bRsu) return aRsu ? aRadius : bRadius;
    return std::min(aRadius,bRadius);
}
inline double linkRange(omnetpp::cModule* a, omnetpp::cModule* b) {
    return linkRange(isRsuNode(a),nodeRange(a),isRsuNode(b),nodeRange(b));
}
inline bool withinRange(omnetpp::cModule* a, const veins::Coord& pa,
                        omnetpp::cModule* b, const veins::Coord& pb) {
    const double dx = pa.x - pb.x, dy = pa.y - pb.y;
    const double radius = linkRange(a, b);
    return dx * dx + dy * dy <= radius * radius + 1e-8;
}
