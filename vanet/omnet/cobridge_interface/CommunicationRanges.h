#pragma once
#include <nlohmann/json.hpp>
#include <map>
#include <string>
#include <cmath>
using nlohmann::json;

class CommunicationRanges {
    std::map<std::string,json> positions;
public:
    double v2v=50,rsu=170;
    void synchronize(const json& nodes) {
        positions.clear();
        for(const auto& node:nodes)positions[node.at("id").get<std::string>()]=node;
    }
    void clear() {positions.clear();}
    json describe(const std::string& sender,const std::string& receiver) const {
        auto a=positions.find(sender),b=positions.find(receiver);
        if(a==positions.end()||b==positions.end())return {{"allowed",false},{"reason","node_absent"}};
        bool roadside=a->second.value("role","npc")=="rsu"||b->second.value("role","npc")=="rsu";
        double distance=std::hypot(a->second.at("x").get<double>()-b->second.at("x").get<double>(),
                                   a->second.at("y").get<double>()-b->second.at("y").get<double>());
        double limit=roadside?rsu:v2v;
        return {{"allowed",distance<=limit+1e-9},{"distance_metres",distance},{"limit_metres",limit},{"type",roadside?"V2I":"V2V"}};
    }
    json links() const {
        json value=json::object();
        for(const auto& receiver:positions) {
            value[receiver.first]=json::array();
            for(const auto& sender:positions)
                if(sender.first!=receiver.first&&describe(sender.first,receiver.first).at("allowed").get<bool>())
                    value[receiver.first].push_back(sender.first);
        }
        return value;
    }
    json configuration() const {return {{"v2v_metres",v2v},{"rsu_metres",rsu}};}
};
