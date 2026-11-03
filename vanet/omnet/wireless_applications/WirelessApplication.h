#pragma once
#include "veins/base/modules/BaseApplLayer.h"
#include "WirelessFragment.h"
#include <map>
#include <vector>
class NetworkGateway;

class WirelessApplication : public veins::BaseApplLayer {
    struct Assembly { std::vector<std::string> parts; std::vector<bool> present; double sentAt, created; std::string sender; };
    std::map<std::string, Assembly> assemblies;
    NetworkGateway* gateway = nullptr;
    std::string nodeId;
protected:
    void initialize(int stage) override;
    void handleLowerMsg(cMessage* message) override;
    void handleLowerControl(cMessage* message) override;
public:
    void transmit(const std::string& sender, const std::string& recipient,
                  const std::string& content, const std::string& identity, veins::LAddress::L2Type address);
};
