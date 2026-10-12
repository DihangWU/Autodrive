// Engineering-local hard radio coverage limit, preserving the Veins MAC/PHY.
#include "veins/modules/phy/PhyLayer80211p.h"
#include "veins/base/messages/AirFrame_m.h"
#include "RadioRange.h"

class RangePhy80211p : public veins::PhyLayer80211p {
    long outsideRange = 0;
protected:
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
