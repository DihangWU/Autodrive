#include "VehicleIcon.h"
#include <cmath>
using namespace omnetpp;

cGroupFigure* createVehicleIcon(const char* name, bool lincoln) {
    auto* car=new cGroupFigure(name);
    // Four tyres, tapered bonnet, cabin glazing, headlights and tail lights.
    auto rectangle=[&](double x,double y,double w,double h,cFigure::Color color) {
        auto* part=new cRectangleFigure();part->setBounds({x,y,w,h});
        part->setFilled(true);part->setOutlined(false);part->setFillColor(color);car->addFigure(part);
    };
    for(double x:{-1.55,.9})for(double y:{-1.14,.88})rectangle(x,y,.65,.26,cFigure::Color(10,12,16));
    auto* body=new cPolygonFigure();
    body->setPoints({{-2.4,-.7},{-2.15,-1.},{1.55,-1.},{2.4,-.72},{2.4,.72},{1.55,1.},{-2.15,1.},{-2.4,.7}});
    body->setFilled(true);body->setFillColor(lincoln?cFigure::Color(255,83,73):cFigure::Color(74,168,255));
    body->setLineColor(cFigure::Color(15,20,25));body->setLineWidth(.13);body->setZoomLineWidth(true);car->addFigure(body);
    auto* windshield=new cPolygonFigure();windshield->setPoints({{.55,-.8},{1.18,-.65},{1.18,.65},{.55,.8}});
    windshield->setFilled(true);windshield->setOutlined(false);windshield->setFillColor(cFigure::Color(175,225,245));car->addFigure(windshield);
    rectangle(-1.4,-.7,.45,1.4,cFigure::Color(55,85,105));
    for(double y:{-.75,.5}) {
        rectangle(2.12,y,.2,.25,cFigure::Color(255,250,205));
        rectangle(-2.28,y,.16,.25,cFigure::Color(180,25,35));
    }
    return car;
}

void positionVehicleIcon(cGroupFigure* icon,double x,double y,double heading) {
    cFigure::Transform transform;
    transform.rotate(heading*M_PI/180.);transform.translate(x,y);icon->setTransform(transform);
}
