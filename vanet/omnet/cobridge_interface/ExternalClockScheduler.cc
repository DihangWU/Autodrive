#include "NetworkGateway.h"
#include <omnetpp.h>
#include <chrono>
using namespace omnetpp;

// A GUI stop returns from the scheduler without releasing the external clock gate.
class ExternalClockScheduler : public cSequentialScheduler {
    std::chrono::steady_clock::time_point nextGuiRefresh{};
public:
    std::string str() const override { return "CoBridge external clock"; }
    cEvent* takeNextEvent() override {
        auto* gateway=check_and_cast<NetworkGateway*>(getSimulation()->getSystemModule()->getSubmodule("gateway"));
        while(gateway->waitingForTraffic()) {
            gateway->pollExternalInput(20);
            // idle() redraws the entire Qtenv inspector. Never put that redraw
            // in front of a newly received advance; idle GUI refresh is 20 Hz.
            if(!gateway->waitingForTraffic())break;
            auto now=std::chrono::steady_clock::now();
            if(!getEnvir()->isGUI()||now>=nextGuiRefresh) {
                nextGuiRefresh=now+std::chrono::milliseconds(50);
                if(getEnvir()->idle())return nullptr;
            }
        }
        return cSequentialScheduler::takeNextEvent();
    }
};
Register_Class(ExternalClockScheduler);
