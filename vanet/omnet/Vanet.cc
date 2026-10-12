// Local adapter for Veins 5.3.1; wireless packets traverse its 802.11p MAC/PHY.
#include <omnetpp.h>
#include <map>
#include <set>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <deque>
#include <cerrno>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include "veins/base/modules/BaseMobility.h"
#include "veins/base/connectionManager/BaseConnectionManager.h"
#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "RadioRange.h"
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
    cMessage* helloTimer = nullptr;
    long tx = 0, rx = 0, relayed = 0;
    int seq = 0;
    cOutVector delay;
    std::ofstream csv;
    std::set<BaseFrame1609_4*> pendingRelays;
    std::set<BaseFrame1609_4*> pendingHandshakeAcks;
    struct Peer { simtime_t ack = -1; simtime_t received = -1; };
    std::map<int,Peer> peers;
    long helloNonce = 0;
    bool isRsu() const { return par("isRsu").boolValue(); }
    Coord position() { return check_and_cast<ExternalMobility*>(getParentModule()->getSubmodule("veinsmobility"))->getPositionAt(simTime()); }
    void sendHandshake(BaseFrame1609_4* hello = nullptr) {
        auto* p = new BaseFrame1609_4(hello ? "link HELLO ACK" : "link HELLO");
        populateWSM(p,hello ? hello->par("address").longValue() : LAddress::L2BROADCAST());
        p->addPar("kind") = hello ? "helloAck" : "hello";
        p->addPar("source") = getParentModule()->getName();
        p->addPar("sourceId") = getParentModule()->getId();
        p->addPar("address") = long(myId);
        p->addPar("nonce") = hello ? hello->par("nonce").longValue() : ++helloNonce;
        p->addPar("targetId") = hello ? hello->par("sourceId").longValue() : -1;
        csv << simTime() << (hello ? ",ACK_TX," : ",HELLO_TX,") << getParentModule()->getName()
            << "," << (hello ? hello->par("source").stringValue() : "*") << ",,0,0,handshake" << std::endl;
        sendDown(p);
    }
    void sendProbe(BaseFrame1609_4* original = nullptr) {
        auto* p = new BaseFrame1609_4(original ? "RSU relay" : "connectivity probe");
        populateWSM(p);
        Coord pos = position();
        p->addPar("sx") = pos.x; p->addPar("sy") = pos.y; p->addPar("sz") = pos.z;
        p->addPar("source") = getParentModule()->getName();
        p->addPar("sourceId") = getParentModule()->getId();
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
            scheduleAt(simTime()+uniform(.15,.25), timer);
            if (par("sendHello").boolValue()) {
                helloTimer = new cMessage("HELLO timer");
                scheduleAt(simTime()+uniform(.01,.03),helloTimer);
            }
        }
    }
    void handleSelfMsg(cMessage* msg) override {
        if (auto* original = dynamic_cast<BaseFrame1609_4*>(msg)) {
            if (original->hasPar("kind")) {
                pendingHandshakeAcks.erase(original);
                sendHandshake(original);
                delete original;
                return;
            }
            pendingRelays.erase(original);
            ++relayed;
            sendProbe(original);
            delete original;
            return;
        }
        if (msg == helloTimer) {
            sendHandshake();
            // Keep HELLO/ACK exchanges separate from data and de-synchronize
            // retries, avoiding repeatable half-duplex collisions.
            scheduleAt(simTime()+par("interval").doubleValue()*(1+uniform(-.15,.15)),helloTimer);
            return;
        }
        if (msg != timer) { DemoBaseApplLayer::handleSelfMsg(msg); return; }
        if (par("sendProbes").boolValue()) sendProbe();
        for (auto it=peers.begin(); it!=peers.end();) {
            if (simTime() - std::max(it->second.ack,it->second.received) > par("handshakeTimeout").doubleValue()) it=peers.erase(it);
            else ++it;
        }
        scheduleAt(simTime()+par("interval").doubleValue(), timer);
    }
    void onWSM(BaseFrame1609_4* p) override {
        if (!p->hasPar("source")) return;
        auto* sender = getParentModule()->getParentModule()->getSubmodule(p->par("source").stringValue());
        if (!sender) return; // The source may have been removed in a CARLA snapshot.
        if (!p->hasPar("sourceId") || sender->getId()!=p->par("sourceId").longValue()) return;
        auto* senderMobility = check_and_cast<ExternalMobility*>(sender->getSubmodule("veinsmobility"));
        if (!withinRange(sender, senderMobility->getPositionAt(simTime()), getParentModule(), position())) return;
        if (p->hasPar("kind")) {
            std::string kind = p->par("kind").stringValue();
            if (kind=="hello") {
                csv << simTime() << ",HELLO_RX," << sender->getName() << "," << getParentModule()->getName() << ",,0,0,handshake" << std::endl;
                // Vehicles outside V2V range cannot hear one another's CCA.
                // Spread their HELLO responses to avoid hidden-terminal ACK
                // collisions at a shared RSU.
                auto* hello = p->dup();
                pendingHandshakeAcks.insert(hello);
                scheduleAt(simTime()+uniform(0,.02),hello);
            } else if (kind=="helloAck" && p->par("targetId").longValue()==getParentModule()->getId()
                       && p->par("nonce").longValue()==helloNonce && helloNonce>0) {
                peers[sender->getId()].ack = simTime();
                csv << simTime() << ",ACK_RX," << sender->getName() << "," << getParentModule()->getName() << ",,0,0,handshake" << std::endl;
            }
            return; // Handshake packets neither tint the link nor get relayed.
        }
        if (!p->hasPar("origin")) return;
        peers[sender->getId()].received = simTime();
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
    bool handshakeWith(cModule* peer) const {
        auto it=peers.find(peer->getId());
        return it!=peers.end() && it->second.ack>=0 && simTime()-it->second.ack<=par("handshakeTimeout").doubleValue();
    }
    bool receivedFrom(cModule* peer, double window) const {
        auto it=peers.find(peer->getId());
        return it!=peers.end() && it->second.received>=0 && simTime()-it->second.received<=window;
    }
    ~ConnectivityApp() override {
        if (timer) cancelAndDelete(timer);
        if (helloTimer) cancelAndDelete(helloTimer);
        for (auto* original : pendingRelays) cancelAndDelete(original);
        for (auto* hello : pendingHandshakeAcks) cancelAndDelete(hello);
    }
};
Define_Module(ConnectivityApp);

class CarlaManager : public cSimpleModule {
    int fd = -1;
    cMessage* tick = nullptr;
    std::map<long,cModule*> nodes;
    std::map<std::string,cModule*> rsus;
    cGroupFigure* links = nullptr;
    std::map<std::string,cLineFigure*> rangeLinks;
    long frame = 0;
    double step = .05;
    bool waitingForFrame = false;
    std::string receiveBuffer;
    std::deque<std::string> frameLines;
    size_t expectedLines = 0;
    std::string feedback() {
        std::vector<cModule*> all;
        for (auto& entry: rsus) all.push_back(entry.second);
        for (auto& entry: nodes) all.push_back(entry.second);
        std::ostringstream rows;
        int count=0;
        for (size_t i=0;i<all.size();++i) {
            auto* a=all[i];
            auto* aa=check_and_cast<ConnectivityApp*>(a->getSubmodule("appl"));
            Coord pa=check_and_cast<ExternalMobility*>(a->getSubmodule("veinsmobility"))->getPositionAt(simTime());
            for (size_t j=i+1;j<all.size();++j) {
                auto* b=all[j];
                auto* ba=check_and_cast<ConnectivityApp*>(b->getSubmodule("appl"));
                Coord pb=check_and_cast<ExternalMobility*>(b->getSubmodule("veinsmobility"))->getPositionAt(simTime());
                if (!withinRange(a,pa,b,pb) || !aa->handshakeWith(b) || !ba->handshakeWith(a)) continue;
                int directions=(ba->receivedFrom(a,par("activityWindow").doubleValue()) ? 1 : 0)
                             | (aa->receivedFrom(b,par("activityWindow").doubleValue()) ? 2 : 0);
                rows << a->getName() << ' ' << b->getName() << ' ' << directions << '\n';
                ++count;
            }
        }
        std::ostringstream header;
        header << "VIS " << frame << ' ' << simTime() << ' ' << count << '\n';
        return header.str()+rows.str()+"ACK "+std::to_string(frame)+"\n";
    }
    void updateLinks() {
        std::vector<cModule*> all;
        for (auto& entry : rsus) all.push_back(entry.second);
        for (auto& entry : nodes) all.push_back(entry.second);
        std::set<std::string> present;
        for (size_t i=0; i<all.size(); ++i) {
            auto* a = all[i];
            Coord pa = check_and_cast<ExternalMobility*>(a->getSubmodule("veinsmobility"))->getPositionAt(simTime());
            for (size_t j=i+1; j<all.size(); ++j) {
                auto* b = all[j];
                Coord pb = check_and_cast<ExternalMobility*>(b->getSubmodule("veinsmobility"))->getPositionAt(simTime());
                if (!withinRange(a,pa,b,pb)) continue;
                std::string key = std::to_string(a->getId()) + "_" + std::to_string(b->getId());
                present.insert(key);
                if (!rangeLinks.count(key)) {
                    auto* line = new cLineFigure(("link_" + key).c_str());
                    line->setLineColor(cFigure::Color(isRsuNode(a) || isRsuNode(b) ? "purple" : "steelblue"));
                    line->setLineStyle(cFigure::LINE_DASHED);
                    line->setLineWidth(1.5);
                    std::string tooltip = std::string(a->getName()) + " <-> " + b->getName() +
                                          " (coverage " + std::to_string(int(linkRange(a,b))) + "m)";
                    line->setTooltip(tooltip.c_str());
                    links->addFigure(line);
                    rangeLinks[key] = line;
                    EV_INFO << "COVERAGE LINK CREATED " << a->getName() << " <-> " << b->getName() << endl;
                }
                rangeLinks[key]->setStart(cFigure::Point(pa.x,pa.y));
                rangeLinks[key]->setEnd(cFigure::Point(pb.x,pb.y));
            }
        }
        for (auto it=rangeLinks.begin(); it!=rangeLinks.end();) {
            if (!present.count(it->first)) {
                EV_INFO << "COVERAGE LINK REMOVED " << it->second->getTooltip() << endl;
                delete links->removeFigure(it->second);
                it = rangeLinks.erase(it);
            } else ++it;
        }
    }
    bool receiveFrame() {
        // A scene update may take longer than one CARLA step. Keep Qtenv
        // responsive while waiting, without advancing simulation time or
        // applying a partially received snapshot. Stop/Run resumes this frame.
        while (true) {
            size_t newline;
            while ((newline=receiveBuffer.find('\n'))!=std::string::npos) {
                if (newline>4096) throw cRuntimeError("Oversized VANET frame line");
                frameLines.push_back(receiveBuffer.substr(0,newline));
                receiveBuffer.erase(0,newline+1);
                if (frameLines.size()==1) {
                    std::istringstream header(frameLines.front());
                    std::string tag; long nextFrame; double nextStep; int count,rsuCount=0;
                    if (!(header>>tag>>nextFrame>>nextStep>>count) ||
                        (tag!="FRAME" && tag!="FRAME2") || !std::isfinite(nextStep) || nextStep<=0 || count<0 || count>10000)
                        throw cRuntimeError("Invalid VANET frame header");
                    if (tag=="FRAME2" && (!(header>>rsuCount) || rsuCount<0 || rsuCount>10000))
                        throw cRuntimeError("Invalid VANET RSU count");
                    expectedLines=1+count+rsuCount;
                }
                if (frameLines.size()==expectedLines) return true;
            }
            if (receiveBuffer.size()>4096) throw cRuntimeError("Oversized VANET frame line");
            char buffer[4096];
            ssize_t n=recv(fd,buffer,sizeof(buffer),MSG_DONTWAIT);
            if (n>0) { receiveBuffer.append(buffer,size_t(n)); continue; }
            if (n==0) throw cRuntimeError("VANET bridge disconnected");
            if (errno==EINTR) continue;
            if (errno!=EAGAIN && errno!=EWOULDBLOCK) throw cRuntimeError("VANET socket receive failed");
            pollfd socket{fd,POLLIN,0};
            if (::poll(&socket,1,20)<0 && errno!=EINTR) throw cRuntimeError("VANET socket poll failed");
            if (getEnvir()->idle()) return false;
        }
    }
    std::string readLine() {
        if (frameLines.empty()) throw cRuntimeError("Incomplete VANET snapshot");
        std::string line=std::move(frameLines.front());
        frameLines.pop_front();
        return line;
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
    void updateRsu(const std::string& name,double x,double y,double z) {
        cModule* rsu;
        if (!rsus.count(name)) {
            if (getParentModule()->getSubmodule(name.c_str())) throw cRuntimeError("Duplicate RSU: %s",name.c_str());
            rsu=cModuleType::get("Obu")->create(name.c_str(),getParentModule());
            rsu->finalizeParameters(); rsu->buildInside();
            // *.rsu*.appl.isRsu is finalized by the configuration when the
            // module is built; non-mutable parameters cannot be set later.
            rsu->getDisplayString().setTagArg("i",0,"device/antennatower");
            rsu->getDisplayString().setTagArg("t",0,name.c_str());
            check_and_cast<ExternalMobility*>(rsu->getSubmodule("veinsmobility"))->setStartPosition(Coord(x,y,z));
            rsu->scheduleStart(simTime()); rsu->callInitialize();
            rsus[name] = rsu;
            EV_INFO << "RSU CREATED " << name << " SUMO=" << x << "," << y << "," << z << endl;
        } else rsu = rsus[name];
        check_and_cast<ExternalMobility*>(rsu->getSubmodule("veinsmobility"))->setPosition(x,y,z);
    }
    void deleteNode(cModule* node) {
        check_and_cast<BaseConnectionManager*>(getParentModule()->getSubmodule("connectionManager"))->unregisterNic(node->getSubmodule("nic"));
        node->callFinish();
        node->deleteModule();
    }
    void apply() {
        std::set<long> present;
        if(par("smoke").boolValue()) {
            frame++; step=.05;
            update(101,"lincoln0",700+simTime().dbl(),625,2); present.insert(101);
            if(simTime()<5 || simTime()>=6) { update(102,"lincoln1",725,600+simTime().dbl(),2); present.insert(102); }
        } else {
            std::istringstream header(readLine()); std::string tag; int count,rsuCount=0;
            if(!(header>>tag>>frame>>step>>count) || (tag!="FRAME" && tag!="FRAME2") || step<=0 || count<0 || count>10000) throw cRuntimeError("Invalid VANET frame header");
            if(tag=="FRAME2" && (!(header>>rsuCount) || rsuCount<0 || rsuCount>10000)) throw cRuntimeError("Invalid VANET RSU count");
            for(int i=0;i<count;++i) {
                long id;std::string name;double x,y,z;std::istringstream row(readLine());
                if(!(row>>id>>name>>x>>y>>z)) throw cRuntimeError("Invalid VANET vehicle row");
                update(id,name,x,y,z); present.insert(id);
            }
            if (tag=="FRAME2") {
                std::set<std::string> presentRsus;
                for (int i=0;i<rsuCount;++i) {
                    std::string name; double x,y,z; std::istringstream row(readLine());
                    if (!(row>>name>>x>>y>>z) || name.rfind("rsu",0)!=0 || !presentRsus.insert(name).second)
                        throw cRuntimeError("Invalid VANET RSU row");
                    updateRsu(name,x,y,z);
                }
                for (auto it=rsus.begin();it!=rsus.end();) {
                    if (!presentRsus.count(it->first)) {
                        EV_INFO << "RSU DELETED " << it->first << endl;
                        deleteNode(it->second);
                        it=rsus.erase(it);
                    } else ++it;
                }
            }
        }
        for(auto it=nodes.begin();it!=nodes.end();) {
            if(!present.count(it->first)) { EV_INFO << "OBU DELETED actor=" << it->first << endl; deleteNode(it->second); it=nodes.erase(it); }
            else ++it;
        }
        updateLinks();
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
                updateRsu(name,x,y,z);
            }
            return;
        }
        const auto& coordinates = mapCoordinates();
        for (const char* name : {"vehicleRange", "rsuRange"}) {
            double radius = getParentModule()->par(name).doubleValue();
            if (!std::isfinite(radius) || radius <= 0) throw cRuntimeError("%s must be positive and finite",name);
        }
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
        links = new cGroupFigure("Communication links");
        getParentModule()->getCanvas()->addFigure(links,1);
        if(!par("smoke").boolValue()) {
            fd=socket(AF_INET,SOCK_STREAM,0);
            sockaddr_in a{};a.sin_family=AF_INET;a.sin_port=htons(par("port").intValue());a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
            if(connect(fd,reinterpret_cast<sockaddr*>(&a),sizeof(a))) throw cRuntimeError("Cannot connect to shared VANET bridge on loopback; start vanet/run_lincoln.sh first");
        }
        tick=new cMessage("CARLA network boundary");
        tick->setSchedulingPriority(100); // finish radio/app events at the boundary before ACK
        scheduleAt(simTime(),tick);
    }
    void handleMessage(cMessage*) override {
        if(frame && fd>=0 && !waitingForFrame) {
            std::string ack=feedback();
            size_t written=0;
            while (written<ack.size()) {
                ssize_t n=::send(fd,ack.data()+written,ack.size()-written,MSG_NOSIGNAL);
                if(n<=0) throw cRuntimeError("Failed VANET visualization/ACK");
                written+=size_t(n);
            }
        }
        if (fd>=0) {
            waitingForFrame=true;
            if (!receiveFrame()) {
                // Stop is handled by Qtenv after this event returns. Resume
                // at the same boundary, with no duplicate ACK or radio step.
                scheduleAt(simTime(),tick);
                return;
            }
        }
        apply();
        waitingForFrame=false;
        expectedLines=0;
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
