// Local adapter for Veins 5.3.1; wireless packets traverse its 802.11p MAC/PHY.
#include <omnetpp.h>
#include <map>
#include <set>
#include <sstream>
#include <fstream>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include "veins/base/modules/BaseMobility.h"
#include "veins/base/connectionManager/BaseConnectionManager.h"
#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
using namespace omnetpp;
using namespace veins;

// SUMO uses y-up; Qtenv uses y-down. Apply one rigid reflection to roads,
// mobility and hence radio/annotation positions. Wire coordinates stay SUMO.
struct MapCoordinates {
    double minX, minY, maxX, maxY;
    static constexpr double margin = 40;
    MapCoordinates() {
        std::ifstream input("roads.txt");
        std::string tag;
        if (!(input >> tag >> minX >> minY >> maxX >> maxY) || tag != "BOUNDS")
            throw cRuntimeError("Invalid roads.txt header; run vanet/build.sh to export Town04");
    }
    Coord convert(const Coord& p) const {
        return Coord(p.x - minX + margin, maxY - p.y + margin, p.z);
    }
};
static const MapCoordinates& mapCoordinates() {
    static const MapCoordinates coordinates;
    return coordinates;
}

class ExternalMobility : public BaseMobility {
protected:
    void initialize(int stage) override {
        BaseMobility::initialize(stage);
        if (stage == 0)
            move.setStart(mapCoordinates().convert(move.getStartPos()), simTime());
    }
public:
    void setPosition(double x, double y, double z) {
        move.setStart(mapCoordinates().convert(Coord(x,y,z)), simTime());
        move.setSpeed(0); // sample-and-hold between CARLA ticks
        updatePosition();
    }
};
Define_Module(ExternalMobility);

class ConnectivityApp : public DemoBaseApplLayer {
    cMessage* timer = nullptr;
    long tx = 0, rx = 0, relayed = 0;
    int seq = 0;
    cOutVector delay;
    std::ofstream csv;
    std::set<BaseFrame1609_4*> pendingRelays;
    bool isRsu() const { return par("isRsu").boolValue(); }
    Coord position() { return check_and_cast<ExternalMobility*>(getParentModule()->getSubmodule("veinsmobility"))->getPositionAt(simTime()); }
    void sendProbe(BaseFrame1609_4* original = nullptr) {
        auto* p = new BaseFrame1609_4(original ? "RSU relay" : "connectivity probe");
        populateWSM(p);
        Coord pos = position();
        p->addPar("sx") = pos.x; p->addPar("sy") = pos.y; p->addPar("sz") = pos.z;
        p->addPar("source") = getParentModule()->getName();
        p->addPar("origin") = original ? original->par("origin").stringValue() : getParentModule()->getName();
        p->addPar("sequence") = original ? original->par("sequence").longValue() : ++seq;
        p->addPar("sentAt") = original ? original->par("sentAt").doubleValue() : simTime().dbl();
        p->addPar("relay") = original != nullptr;
        p->addPar("rsu") = isRsu();
        ++tx;
        csv << simTime() << ",TX," << p->par("source").stringValue() << ",*," << p->par("origin").stringValue() << "," << p->par("sequence").longValue() << "," << originalToDelay(p) << "," << (original ? "relay" : "direct") << std::endl;
        getParentModule()->bubble(original ? "RSU forwarding" : "TX probe");
        sendDown(p);
    }
    double originalToDelay(BaseFrame1609_4* p) { return simTime().dbl()-p->par("sentAt").doubleValue(); }
protected:
    void initialize(int stage) override {
        DemoBaseApplLayer::initialize(stage);
        if (stage == 0) {
            delay.setName("endToEndDelay");
            std::string name = std::string(par("csvDir").stringValue()) + "/" + getParentModule()->getName() + "-" + std::to_string(getParentModule()->getId()) + ".csv";
            csv.open(name);
            if (!csv) throw cRuntimeError("Cannot open packet CSV: %s", name.c_str());
            csv << "time,event,sender,receiver,origin,sequence,delay,path" << std::endl;
        } else if (stage == 1) {
            timer = new cMessage("probe timer");
            scheduleAt(simTime()+uniform(.05,.15), timer);
        }
    }
    void handleSelfMsg(cMessage* msg) override {
        if (auto* original = dynamic_cast<BaseFrame1609_4*>(msg)) {
            pendingRelays.erase(original);
            ++relayed;
            sendProbe(original);
            delete original;
            return;
        }
        if (msg != timer) { DemoBaseApplLayer::handleSelfMsg(msg); return; }
        sendProbe();
        scheduleAt(simTime()+par("interval").doubleValue(), timer);
    }
    void onWSM(BaseFrame1609_4* p) override {
        if (!p->hasPar("origin")) return;
        ++rx;
        double d = originalToDelay(p);
        delay.record(d);
        bool relay = p->par("relay").boolValue();
        csv << simTime() << ",RX," << p->par("source").stringValue() << "," << getParentModule()->getName() << "," << p->par("origin").stringValue() << "," << p->par("sequence").longValue() << "," << d << "," << (relay ? "relay" : "direct") << std::endl;
        EV_INFO << "RX " << p->par("source").stringValue() << " -> " << getParentModule()->getName() << " seq=" << p->par("sequence").longValue() << " delay=" << d << " relay=" << relay << endl;
        Coord from(p->par("sx").doubleValue(),p->par("sy").doubleValue(),p->par("sz").doubleValue());
        auto* line = annotations->drawLine(from, position(), relay ? "orange" : "green");
        annotations->scheduleErase(.25, line);
        getParentModule()->bubble(relay ? "RX via RSU" : "RX direct");
        if (isRsu() && !relay && !p->par("rsu").boolValue()) {
            // Separate RSUs can receive the same broadcast almost simultaneously.
            // Jitter avoids deterministic collisions between their immediate replies.
            auto* original = p->dup();
            pendingRelays.insert(original);
            scheduleAt(simTime()+uniform(0,par("relayJitter").doubleValue()),original);
        }
    }
    void finish() override {
        DemoBaseApplLayer::finish();
        recordScalar("probeTx",tx); recordScalar("probeRx",rx); recordScalar("relayTx",relayed);
    }
public:
    ~ConnectivityApp() override {
        if (timer) cancelAndDelete(timer);
        for (auto* original : pendingRelays) cancelAndDelete(original);
    }
};
Define_Module(ConnectivityApp);

class CarlaManager : public cSimpleModule {
    int fd = -1;
    cMessage* tick = nullptr;
    std::map<long,cModule*> nodes;
    long frame = 0;
    double step = .05;
    std::string readLine() {
        std::string line; char ch;
        while (true) {
            ssize_t n=recv(fd,&ch,1,0);
            if (n!=1) throw cRuntimeError("VANET socket closed; stop experiment or reconnect by restarting OMNeT++");
            if(ch=='\n') return line;
            line += ch;
            if(line.size()>4096) throw cRuntimeError("Oversized VANET frame");
        }
    }
    void update(long id,const std::string& name,double x,double y,double z) {
        cModule* m;
        if(!nodes.count(id)) {
            m=cModuleType::get("Obu")->create(name.c_str(),getParentModule());
            m->finalizeParameters(); m->buildInside();
            auto* mobility=check_and_cast<ExternalMobility*>(m->getSubmodule("veinsmobility"));
            mobility->setStartPosition(Coord(x,y,z));
            m->scheduleStart(simTime()); m->callInitialize(); nodes[id]=m;
            EV_INFO << "OBU CREATED actor=" << id << endl;
        } else m=nodes[id];
        check_and_cast<ExternalMobility*>(m->getSubmodule("veinsmobility"))->setPosition(x,y,z);
    }
    void apply() {
        std::set<long> present;
        if(par("smoke").boolValue()) {
            frame++; step=.05;
            update(101,"lincoln0",700+simTime().dbl(),625,2); present.insert(101);
            if(simTime()<5 || simTime()>=6) { update(102,"lincoln1",745,600+simTime().dbl(),2); present.insert(102); }
        } else {
            std::istringstream header(readLine()); std::string tag; int count;
            if(!(header>>tag>>frame>>step>>count) || tag!="FRAME" || step<=0 || count<0 || count>10000) throw cRuntimeError("Invalid VANET frame header");
            for(int i=0;i<count;++i) {
                long id;std::string name;double x,y,z;std::istringstream row(readLine());
                if(!(row>>id>>name>>x>>y>>z)) throw cRuntimeError("Invalid VANET vehicle row");
                update(id,name,x,y,z); present.insert(id);
            }
        }
        for(auto it=nodes.begin();it!=nodes.end();) {
            if(!present.count(it->first)) { EV_INFO << "OBU DELETED actor=" << it->first << endl; check_and_cast<BaseConnectionManager*>(getParentModule()->getSubmodule("connectionManager"))->unregisterNic(it->second->getSubmodule("nic")); it->second->callFinish(); it->second->deleteModule(); it=nodes.erase(it); }
            else ++it;
        }
    }
protected:
    int numInitStages() const override { return 2; }
    void initialize(int stage) override {
        if (stage == 1) {
            std::istringstream definitions(par("rsuDefinitions").stringValue());
            std::string row;
            while (std::getline(definitions,row)) {
                if (row.empty()) continue;
                std::istringstream input(row);
                std::string name; double x,y,z;
                if (!(input >> name >> x >> y >> z)) throw cRuntimeError("Invalid RSU definition");
                if (getParentModule()->getSubmodule(name.c_str())) throw cRuntimeError("Duplicate RSU: %s",name.c_str());
                auto* rsu=cModuleType::get("Obu")->create(name.c_str(),getParentModule());
                rsu->finalizeParameters(); rsu->buildInside();
                rsu->getDisplayString().setTagArg("i",0,"device/antennatower");
                rsu->getDisplayString().setTagArg("t",0,name.c_str());
                check_and_cast<ExternalMobility*>(rsu->getSubmodule("veinsmobility"))->setStartPosition(Coord(x,y,z));
                rsu->scheduleStart(simTime()); rsu->callInitialize();
                EV_INFO << "RSU CREATED " << name << " SUMO=" << x << "," << y << "," << z << endl;
            }
            return;
        }
        const auto& coordinates = mapCoordinates();
        auto* roads = new cGroupFigure("Town04 roads");
        getParentModule()->getCanvas()->addFigure(roads,0);
        std::ifstream roadfile("roads.txt");
        std::string row; int roadNumber=0;
        while(std::getline(roadfile,row)) {
            std::istringstream input(row); double width,x,y;
            if(!(input>>width)) continue;
            auto* road=new cPolylineFigure(("lane"+std::to_string(roadNumber++)).c_str());
            while(input>>x>>y) {
                const Coord p = coordinates.convert(Coord(x,y,0));
                road->addPoint(cFigure::Point(p.x,p.y));
            }
            road->setLineColor(cFigure::Color(100,100,100)); road->setLineWidth(width);
            roads->addFigure(road);
        }
        if(!par("smoke").boolValue()) {
            fd=socket(AF_INET,SOCK_STREAM,0);
            timeval timeout{10,0}; setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
            sockaddr_in a{};a.sin_family=AF_INET;a.sin_port=htons(par("port").intValue());a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
            if(connect(fd,reinterpret_cast<sockaddr*>(&a),sizeof(a))) throw cRuntimeError("Cannot connect to shared VANET bridge on loopback; start vanet/run_lincoln.sh first");
        }
        tick=new cMessage("CARLA network boundary");
        tick->setSchedulingPriority(100); // finish radio/app events at the boundary before ACK
        scheduleAt(simTime(),tick);
    }
    void handleMessage(cMessage*) override {
        if(frame && fd>=0) {
            std::string ack="ACK "+std::to_string(frame)+"\n";
            if(::send(fd,ack.c_str(),ack.size(),MSG_NOSIGNAL)!=static_cast<ssize_t>(ack.size())) throw cRuntimeError("Failed VANET ACK");
        }
        apply();
        scheduleAt(simTime()+step,tick);
    }
    void finish() override {
        // Qtenv keeps the network alive after reaching the time limit.
        // Release the CARLA bridge even while the result view stays open.
        if(fd>=0) { ::shutdown(fd,SHUT_RDWR); ::close(fd); fd=-1; }
    }
public:
    ~CarlaManager() override { if(tick) cancelAndDelete(tick); if(fd>=0) close(fd); }
};
Define_Module(CarlaManager);
