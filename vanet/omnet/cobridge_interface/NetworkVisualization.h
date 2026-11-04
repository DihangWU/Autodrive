#pragma once
#include <omnetpp.h>
#include <nlohmann/json.hpp>
#include <map>
#include <string>
using nlohmann::json;
using namespace omnetpp;

class NetworkVisualization {
    struct Link {cLineFigure* figure;std::string sender,receiver;double until;};
    cModule* root;
    cGroupFigure *roads,*coverage,*messages,*vehicles;
    cTextFigure* status;
    json scene;
    std::map<std::string,json> positions;
    std::map<std::string,cOvalFigure*> circles;
    std::map<std::string,cGroupFigure*> vehicleIcons;
    std::map<std::string,double> pulses;
    std::map<std::string,Link> links;
    cFigure::Point position(const std::string& id) const;
public:
    NetworkVisualization(cModule* root,const std::string& sceneFile);
    void synchronize(const json& values,const std::map<std::string,cModule*>& modules);
    void transmitted(const std::string& sender,const std::string& kind,double now);
    void received(const std::string& sender,const std::string& receiver,const std::string& kind,double now);
    void refresh(double now,long sent,long received);
    void reset();
};
