#include "WirelessApplication.h"
#include "../cobridge_interface/NetworkGateway.h"
#include "veins/modules/utility/Consts80211p.h"
#include <algorithm>
Define_Module(WirelessApplication);

void WirelessApplication::initialize(int stage) {
    BaseApplLayer::initialize(stage);
    if (stage == 0) {
        nodeId = getParentModule()->par("externalId").stdstringValue();
        gateway = check_and_cast<NetworkGateway*>(getSimulation()->getSystemModule()->getSubmodule("gateway"));
    }
}
void WirelessApplication::transmit(const std::string& sender, const std::string& recipient,
                                  const std::string& content, const std::string& identity, veins::LAddress::L2Type address) {
    Enter_Method_Silent();
    constexpr size_t fragmentSize = 1200;
    int count = std::max<size_t>(1, (content.size() + fragmentSize - 1) / fragmentSize);
    for (int i = 0; i < count; ++i) {
        auto* frame = new WirelessFragment();
        frame->sender = sender; frame->recipient = recipient; frame->transmission = identity;
        frame->index = i; frame->count = count; frame->sentAt = simTime().dbl();
        frame->content = content.substr(i * fragmentSize, fragmentSize);
        frame->setBitLength(8 * (48 + sender.size() + recipient.size() + identity.size() + frame->content.size()));
        frame->setChannelNumber(static_cast<int>(veins::Channel::cch));
        frame->setUserPriority(5);
        frame->setRecipientAddress(address);
        gateway->fragment(false);
        gateway->event("fragment.tx", {{"sender",sender},{"recipient",recipient},{"transmission",identity},{"fragment",i},{"count",count},{"bits",frame->getBitLength()}});
        sendDown(frame);
    }
}
void WirelessApplication::handleLowerMsg(cMessage* message) {
    auto* frame = dynamic_cast<WirelessFragment*>(message);
    if (!frame) { delete message; return; }
    gateway->fragment(true);
    gateway->event("fragment.rx", {{"sender",frame->sender},{"recipient",nodeId},{"transmission",frame->transmission},{"fragment",frame->index}});
    if (frame->recipient != "broadcast" && frame->recipient != nodeId) { delete frame; return; }
    for (auto it = assemblies.begin(); it != assemblies.end();) {
        if (simTime().dbl() - it->second.created > 10) {
            gateway->event("reassembly.expired", {{"recipient",nodeId},{"transmission",it->first}});
            it = assemblies.erase(it);
        } else ++it;
    }
    auto found = assemblies.find(frame->transmission);
    if (found == assemblies.end()) {
        if (assemblies.size() >= 512) { gateway->event("reassembly.capacity", {{"recipient",nodeId}}); delete frame; return; }
        found = assemblies.emplace(frame->transmission, Assembly{std::vector<std::string>(frame->count),std::vector<bool>(frame->count,false),frame->sentAt,simTime().dbl(),frame->sender}).first;
    }
    auto& assembly = found->second;
    if (frame->index >= 0 && frame->index < (int)assembly.parts.size()) {
        assembly.parts[frame->index] = frame->content; assembly.present[frame->index] = true;
    }
    if (std::all_of(assembly.present.begin(), assembly.present.end(), [](bool b) {return b;})) {
        std::string content;
        for (const auto& part : assembly.parts) content += part;
        gateway->delivered(nodeId, assembly.sender, content, assembly.sentAt);
        assemblies.erase(found);
    }
    delete frame;
}
void WirelessApplication::handleLowerControl(cMessage* message) {
    gateway->event("mac.control", {{"node",nodeId},{"kind",message->getKind()},{"name",message->getName()}});
    delete message;
}
