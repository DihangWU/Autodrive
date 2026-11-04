#include "ExternalMobility.h"
#include <cmath>
Define_Module(ExternalMobility);

void ExternalMobility::initialize(int stage) {
    if(stage!=1||!hasGUI()) {BaseMobility::initialize(stage);return;}
    // Veins treats bgb canvas size as physical playground scaling. Our canvas is
    // the cropped road map, while radio coordinates stay unscaled in metres.
    auto& display=getSimulation()->getSystemModule()->getDisplayString();
    std::string width=display.getTagArg("bgb",0),height=display.getTagArg("bgb",1);
    display.setTagArg("bgb",0,"0");display.setTagArg("bgb",1,"0");
    try {BaseMobility::initialize(stage);}
    catch(...) {display.setTagArg("bgb",0,width.c_str());display.setTagArg("bgb",1,height.c_str());throw;}
    display.setTagArg("bgb",0,width.c_str());display.setTagArg("bgb",1,height.c_str());
}

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
