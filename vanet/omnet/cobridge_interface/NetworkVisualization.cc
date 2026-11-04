#include "NetworkVisualization.h"
#include "VehicleIcon.h"
#include <fstream>
#include <set>
#include <sstream>

NetworkVisualization::NetworkVisualization(cModule* module,const std::string& path):root(module) {
    std::ifstream input(path);if(!input)throw cRuntimeError("Cannot read Qtenv road scene");input>>scene;
    auto* canvas=root->getCanvas();
    auto* background=new cRectangleFigure("map-background");
    background->setBounds({0,0,scene["width"].get<double>()+190,scene["height"].get<double>()});
    background->setFilled(true);background->setOutlined(false);
    background->setFillColor(cFigure::Color(32,36,43));background->setZIndex(-100);
    canvas->addFigure(background);
    roads=new cGroupFigure("SUMO-road-map");roads->setZIndex(-10);canvas->addFigure(roads);
    coverage=new cGroupFigure("effective-radio-ranges");coverage->setZIndex(-2);canvas->addFigure(coverage);
    messages=new cGroupFigure("actual-wireless-receptions");messages->setZIndex(2);canvas->addFigure(messages);
    vehicles=new cGroupFigure("vehicle-icons");vehicles->setZIndex(3);canvas->addFigure(vehicles);
    for(const auto& lane:scene["lanes"]) {
        auto* line=new cPolylineFigure(lane["id"].get<std::string>().c_str());
        std::vector<cFigure::Point> points;
        for(const auto& p:lane["points"])points.emplace_back(p[0].get<double>(),p[1].get<double>());
        line->setPoints(points);line->setLineWidth(lane["width"].get<double>());
        line->setZoomLineWidth(true);line->setLineColor(cFigure::Color(95,100,110));
        roads->addFigure(line);
        if(!lane.value("internal",false)) {
            auto* center=new cPolylineFigure();center->setPoints(points);center->setLineWidth(.25);
            center->setZoomLineWidth(true);center->setLineColor(cFigure::Color(175,180,185));
            center->setLineStyle(cFigure::LINE_DASHED);roads->addFigure(center);
        }
    }
    double width=scene["width"],height=scene["height"];
    root->getDisplayString().setTagArg("bgb",0,std::to_string(width+190).c_str());
    root->getDisplayString().setTagArg("bgb",1,std::to_string(height).c_str());
    root->getDisplayString().setTagArg("bgc",0,"#20242b");
    int row=0;
    for(const auto* name:{"world","connectionManager","gateway"}) {
        auto& display=root->getSubmodule(name)->getDisplayString();
        display.setTagArg("p",0,std::to_string(width+75).c_str());display.setTagArg("p",1,std::to_string(100+row++*80).c_str());
    }
    status=new cTextFigure("communication-status");status->setPosition(cFigure::Point(35,15));
    status->setColor(cFigure::Color(235,240,245));status->setFont(cFigure::Font("Arial",11));canvas->addFigure(status);
    refresh(0,0,0);
}
cFigure::Point NetworkVisualization::position(const std::string& id) const {
    const auto& value=positions.at(id);
    return {value.at("x").get<double>()+scene["coordinate_offset_x"].get<double>(),
            value.at("y").get<double>()+scene["coordinate_offset_y"].get<double>()};
}
void NetworkVisualization::synchronize(const json& values,const std::map<std::string,cModule*>& modules) {
    positions.clear();
    for(const auto& node:values) {
        std::string id=node["id"],role=node.value("role","npc");positions[id]=node;
        if((role=="rsu"||role=="lincoln")&&!circles.count(id)) {
            auto* circle=new cOvalFigure(id.c_str());circle->setFilled(role=="rsu");
            circle->setFillColor(cFigure::Color(45,210,105));circle->setFillOpacity(.035);
            circle->setLineWidth(1.1);circle->setZoomLineWidth(false);
            circle->setLineColor(role=="rsu"?cFigure::Color(35,225,100):cFigure::Color(100,170,250));
            coverage->addFigure(circle);circles[id]=circle;
        }
        auto& display=modules.at(id)->getDisplayString();
        display.setTagArg("b",0,role=="rsu"?"12":"0");display.setTagArg("b",1,role=="rsu"?"12":"0");
        display.setTagArg("b",2,role=="rsu"?"oval":"rect");
        display.setTagArg("b",3,role=="rsu"?"#23df66":(role=="lincoln"?"#ff5349":"#4aa8ff"));
        display.setTagArg("tt",0,id.c_str());display.setTagArg("t",0,id.c_str());
        display.setTagArg("t",2,"#eeeeee");
        if(role!="rsu") {
            if(!vehicleIcons.count(id)) {
                auto* icon=createVehicleIcon(id.c_str(),role=="lincoln");
                icon->setTooltip(id.c_str());vehicles->addFigure(icon);vehicleIcons[id]=icon;
            }
            auto point=position(id);
            positionVehicleIcon(vehicleIcons.at(id),point.x,point.y,node.value("heading",0.));
        }
    }
    for(auto it=circles.begin();it!=circles.end();) {
        if(!positions.count(it->first)) {delete coverage->removeFigure(it->second);pulses.erase(it->first);it=circles.erase(it);}
        else ++it;
    }
    for(auto it=vehicleIcons.begin();it!=vehicleIcons.end();) {
        if(!positions.count(it->first)) {delete vehicles->removeFigure(it->second);it=vehicleIcons.erase(it);}
        else ++it;
    }
}
void NetworkVisualization::transmitted(const std::string& sender,const std::string&,double now) {pulses[sender]=now+.35;}
void NetworkVisualization::received(const std::string& sender,const std::string& receiver,const std::string&,double now) {
    if(!positions.count(sender)||!positions.count(receiver))return;
    auto a=positions.at(sender).value("role","npc"),b=positions.at(receiver).value("role","npc");
    auto pair=[&](const char* first,const char* second) {return (a==first&&b==second)||(a==second&&b==first);};
    cFigure::Color color;
    if(pair("lincoln","rsu"))color=cFigure::Color(90,255,85);
    else if(pair("lincoln","npc"))color=cFigure::Color(70,150,255);
    else if(pair("rsu","npc"))color=cFigure::Color(30,145,65);
    else return;
    auto key=sender+">"+receiver;
    if(!links.count(key)) {
        if(links.size()>=128) {auto it=links.begin();delete messages->removeFigure(it->second.figure);links.erase(it);}
        auto* line=new cLineFigure();line->setLineWidth(2.);line->setZoomLineWidth(false);line->setEndArrowhead(cFigure::ARROW_SIMPLE);
        line->setLineColor(color);
        messages->addFigure(line);links.emplace(key,Link{line,sender,receiver,now+.5});
    }
    links.at(key).until=now+.5;
}
void NetworkVisualization::refresh(double now,long sent,long received) {
    for(const auto& entry:circles) {
        const auto& id=entry.first;const auto& node=positions.at(id);
        bool rsu=node.value("role","npc")=="rsu",lincoln=node.value("role","npc")=="lincoln";
        bool active=pulses.count(id)&&pulses.at(id)>now;
        double radius=scene["communication_ranges"][rsu?"rsu_metres":"v2v_metres"];
        auto point=position(id);entry.second->setBounds({point.x-radius,point.y-radius,2*radius,2*radius});
        entry.second->setVisible(rsu||lincoln);entry.second->setLineOpacity(rsu?.95:(active?.9:.35));
    }
    for(auto it=links.begin();it!=links.end();) {
        if(it->second.until<now||!positions.count(it->second.sender)||!positions.count(it->second.receiver)) {
            delete messages->removeFigure(it->second.figure);it=links.erase(it);
        } else {
            it->second.figure->setStart(position(it->second.sender));it->second.figure->setEnd(position(it->second.receiver));++it;
        }
    }
    std::ostringstream label;
    label << "Town04 | V2V " << scene["communication_ranges"]["v2v_metres"] << " m | RSU " << scene["communication_ranges"]["rsu_metres"]
          << " m (green) | sent " << sent << " | received " << received << "\nArrows = actual complete receptions; same north/up orientation as SUMO";
    status->setText(label.str().c_str());
}
void NetworkVisualization::reset() {
    for(auto& entry:vehicleIcons)delete vehicles->removeFigure(entry.second);
    vehicleIcons.clear();
    for(auto& entry:circles)delete coverage->removeFigure(entry.second);
    for(auto& entry:links)delete messages->removeFigure(entry.second.figure);
    circles.clear();positions.clear();links.clear();pulses.clear();
}
