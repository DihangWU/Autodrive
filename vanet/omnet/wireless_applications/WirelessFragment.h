#pragma once
#include <string>
#include "veins/modules/messages/BaseFrame1609_4_m.h"

// Application fragmentation keeps each transmitted frame within an explicit size.
// The payload bytes below are accounted for in bitLength and carried by the real PHY.
class WirelessFragment : public veins::BaseFrame1609_4 {
public:
    std::string sender, recipient, transmission, content;
    int index = 0, count = 1;
    double sentAt = 0;
    WirelessFragment(const char* name = "vanet-fragment") : BaseFrame1609_4(name) {}
    WirelessFragment(const WirelessFragment& other) = default;
    WirelessFragment* dup() const override { return new WirelessFragment(*this); }
};
