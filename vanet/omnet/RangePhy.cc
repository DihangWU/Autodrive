// Engineering-local hard radio coverage limit, preserving the Veins MAC/PHY.
#include "veins/modules/phy/PhyLayer80211p.h"
#include "veins/base/messages/AirFrame_m.h"
#include "RadioRange.h"

class RangePhy80211p : public veins::PhyLayer80211p {
    long outsideRange = 0;
protected:
    void handleAirFrameStartReceive(veins::AirFrame* frame) override {
        // The existing model gives out-of-range frames exactly zero power:
        // they cannot be decoded or cause interference/CCA. Drop them before
        // ChannelInfo/Decider scheduling instead of tracking zero-power signals.
        // Keep the reference path switchable for deterministic equivalence tests.
        auto* senderPhy = frame->getSenderModule();
        if (par("earlyRangeFilter").boolValue() && senderPhy &&
            !withinRange(senderPhy->getParentModule()->getParentModule(),
                         frame->getPoa().pos.getPositionAt(),
                         getParentModule()->getParentModule(), antennaPosition.getPositionAt())) {
            ++outsideRange;
            delete frame;
            return;
        }
        PhyLayer80211p::handleAirFrameStartReceive(frame);
    }
    void filterSignal(veins::AirFrame* frame) override {
        PhyLayer80211p::filterSignal(frame);
        auto& signal = frame->getSignal();
        auto* sender = frame->getSenderModule()->getParentModule()->getParentModule();
        auto* receiver = getParentModule()->getParentModule();
        if (!withinRange(sender, signal.getSenderPoa().pos.getPositionAt(),
                         receiver, signal.getReceiverPoa().pos.getPositionAt())) {
            // Zero power before decoding/CCA: an out-of-range frame cannot
            // be received or interfere with an in-range transmission.
            signal *= 0;
            ++outsideRange;
        }
    }
    void finish() override {
        PhyLayer80211p::finish();
        recordScalar("outsideRangeFrames", outsideRange);
    }
};
Define_Module(RangePhy80211p);
