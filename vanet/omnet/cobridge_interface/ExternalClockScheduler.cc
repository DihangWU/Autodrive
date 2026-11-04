#include "NetworkGateway.h"
#include <omnetpp.h>
using namespace omnetpp;

// A GUI stop returns from the scheduler without releasing the external clock gate.
class ExternalClockScheduler : public cSequentialScheduler {
public:
    std::string str() const override { return "CoBridge external clock"; }
    cEvent* takeNextEvent() override {
        auto* gateway=check_and_cast<NetworkGateway*>(getSimulation()->getSystemModule()->getSubmodule("gateway"));
        while(gateway->waitingForTraffic()) {
            gateway->pollExternalInput(20);
            if(getEnvir()->idle())return nullptr;
        }
        return cSequentialScheduler::takeNextEvent();
    }
};
Register_Class(ExternalClockScheduler);
