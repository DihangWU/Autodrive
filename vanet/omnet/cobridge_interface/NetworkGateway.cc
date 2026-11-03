#include "NetworkGateway.h"
#include "ExternalMobility.h"
#include "../wireless_applications/WirelessApplication.h"
#include "veins/modules/mac/ieee80211p/Mac1609_4.h"
#include "veins/base/connectionManager/ChannelAccess.h"
#include "veins/base/connectionManager/BaseConnectionManager.h"
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <set>
#include <cmath>
#include <limits>
#include <chrono>
#include <filesystem>
Define_Module(NetworkGateway);

void NetworkGateway::initialize() {
    socketPath = par("socketPath").stdstringValue();
    if (socketPath.size() >= sizeof(sockaddr_un::sun_path)) throw cRuntimeError("Unix socket path too long");
    server = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server < 0) throw cRuntimeError("socket failed: %s", strerror(errno));
    sockaddr_un address{}; address.sun_family = AF_UNIX;
    std::strcpy(address.sun_path, socketPath.c_str());
    // Launcher owns a process lock and removes a stale socket before invocation.
    if (bind(server, reinterpret_cast<sockaddr*>(&address), sizeof(address)) || listen(server, 1))
        throw cRuntimeError("bind/listen failed: %s", strerror(errno));
    events.open(par("eventFile").stringValue(), std::ios::app);
    if (!events) throw cRuntimeError("Cannot open network events log");
    stepStatistics.open(par("stepStatisticsFile").stringValue(), std::ios::app);
    if (!stepStatistics) throw cRuntimeError("Cannot open network boundary statistics log");
    boundary = new cMessage("external-time-boundary");
    boundary->setSchedulingPriority(std::numeric_limits<short>::max());
    scheduleAt(simTime(), boundary);
    event("network.ready", {{"socket",socketPath},{"radio","Veins IEEE 802.11p"}});
    EV_INFO << "VANET network ready: " << socketPath << endl;
}
void NetworkGateway::sendResponse(const json& value) {
    std::string data = value.dump() + "\n"; size_t offset = 0;
    while (offset < data.size()) {
        auto count = ::send(client, data.data()+offset, data.size()-offset, MSG_NOSIGNAL);
        if (count <= 0) throw cRuntimeError("CoBridge connection lost while sending result");
        offset += count;
    }
}
json NetworkGateway::readCommand() {
    constexpr size_t maxLine = 8 * 1024 * 1024;
    while (true) {
        auto end = buffer.find('\n');
        if (end != std::string::npos) {
            auto line = buffer.substr(0, end); buffer.erase(0, end+1);
            return json::parse(line);
        }
        char chunk[65536]; auto count = recv(client, chunk, sizeof(chunk), 0);
        if (count <= 0) throw cRuntimeError("CoBridge connection lost; network state cannot silently restart");
        buffer.append(chunk, count);
        if (buffer.size() > maxLine) throw cRuntimeError("CoBridge message exceeds 8 MiB limit");
    }
}
void NetworkGateway::synchronizeNodes(const json& values) {
    if (!values.is_array() || values.size() > 256) throw std::runtime_error("nodes must be array of at most 256 nodes");
    std::set<std::string> present;
    for (const auto& state : values) {
        std::string id = state.at("id");
        if (id.empty() || id.size()>128 || !present.insert(id).second) throw std::runtime_error("invalid or duplicate node id");
        double x = state.at("x").get<double>() + par("coordinateOffsetX").doubleValue();
        double y = state.at("y").get<double>() + par("coordinateOffsetY").doubleValue();
        double z = state.at("z").get<double>() + 100.0 + (state.value("role", "npc") == "rsu" ? 0.0 : par("vehicleAntennaOffsetZ").doubleValue());
        if (!std::isfinite(x+y+z) || x<0 || y<0 || z<0 || x>=10000 || y>=10000 || z>=1000) throw std::runtime_error("node outside configured 3D playground");
        if (!nodes.count(id)) {
            auto* node = cModuleType::get("vanet.radio.ExternalNode")->create(("radio"+std::to_string(nextNode++)).c_str(),getParentModule());
            node->par("externalId").setStringValue(id.c_str());
            node->finalizeParameters(); node->buildInside();
            auto* mobility = check_and_cast<ExternalMobility*>(node->getSubmodule("veinsmobility"));
            mobility->setStartPosition(veins::Coord(x,y,z));
            node->scheduleStart(simTime()); node->callInitialize(); nodes[id] = node;
            event("node.add", {{"node",id},{"role",state.value("role","npc")}});
        }
        check_and_cast<ExternalMobility*>(nodes.at(id)->getSubmodule("veinsmobility"))->setPosition(x,y,z,state.value("heading",0.0)*M_PI/180);
    }
    for (auto it=nodes.begin(); it!=nodes.end();) {
        if (!present.count(it->first)) {
            event("node.remove", {{"node",it->first}});
            removeNode(it->second); it=nodes.erase(it);
        } else ++it;
    }
}
void NetworkGateway::removeNode(cModule* node) {
    auto* nic = node->getSubmodule("nic");
    veins::ChannelAccess::getConnectionManager(nic)->unregisterNic(nic);
    node->callFinish(); node->deleteModule();
}
json NetworkGateway::artifactManifest() const {
    namespace fs = std::filesystem;
    auto processDirectory=fs::path(par("eventFile").stringValue()).parent_path();
    return {{"process_scalar_file",(processDirectory/"network.sca").string()},
            {"process_vector_file",(processDirectory/"network.vec").string()},
            {"process_configuration_file",(processDirectory/"omnetpp.ini").string()},
            {"scalar_vector_scope","shared_network_process"},
            {"results_directory",resultsDirectory},
            {"wireless_events_file",resultsDirectory.empty()?par("eventFile").stdstringValue():(fs::path(resultsDirectory)/"wireless_events.jsonl").string()},
            {"step_statistics_file",resultsDirectory.empty()?par("stepStatisticsFile").stdstringValue():(fs::path(resultsDirectory)/"network_steps.jsonl").string()}};
}
void NetworkGateway::writeArchiveManifest(bool closed) const {
    if(resultsDirectory.empty()) return;
    auto manifest=artifactManifest();
    manifest["network_start_time"]=archiveStartTime;
    manifest["network_last_boundary_time"]=simTime().dbl();
    manifest["archive_closed"]=closed;
    if(closed)manifest["network_end_time"]=simTime().dbl();
    manifest["note"]="Raw JSONL belongs to this archive interval. OMNeT++ sca/vec files are shared process-level artifacts and finalize when the network process exits.";
    std::ofstream file(std::filesystem::path(resultsDirectory)/"network_manifest.json");
    if(!file)throw cRuntimeError("Cannot write per-run network manifest");
    file << manifest.dump(2) << '\n' << std::flush;
    if(!file)throw cRuntimeError("Cannot flush per-run network manifest");
}
void NetworkGateway::selectResultsDirectory(const std::string& directory) {
    namespace fs = std::filesystem;
    if(directory.empty()) return;
    auto root=fs::weakly_canonical(par("allowedResultsRoot").stringValue());
    auto target=fs::weakly_canonical(directory);
    auto inside=[&](const fs::path& path) {
        auto canonical=fs::weakly_canonical(path);
        auto r=root.begin(), p=canonical.begin();
        for(;r!=root.end() && p!=canonical.end() && *r==*p;++r,++p) {}
        return r==root.end() && p!=canonical.end();
    };
    if(!inside(target)) throw std::runtime_error("results_directory must be a subdirectory of vanet/results");
    fs::create_directories(target);
    auto eventPath=target/"wireless_events.jsonl";
    auto statisticsPath=target/"network_steps.jsonl";
    auto manifestPath=target/"network_manifest.json";
    if(!inside(eventPath)||!inside(statisticsPath)||!inside(manifestPath))
        throw std::runtime_error("result file resolves outside allowed results root");
    std::ofstream newEvents(eventPath,std::ios::app),newStatistics(statisticsPath,std::ios::app);
    if(!newEvents||!newStatistics) throw std::runtime_error("cannot open per-run network logs");
    event("network.archive_end",{{"next_directory",target.string()}});
    writeArchiveManifest(true);
    events=std::move(newEvents);stepStatistics=std::move(newStatistics);resultsDirectory=target.string();
    archiveStartTime=simTime().dbl();
    writeArchiveManifest();
}
void NetworkGateway::resetNetwork(int seed, const std::string& directory) {
    if (seed < 0) throw std::runtime_error("seed must be a nonnegative integer");
    selectResultsDirectory(directory);
    for (auto& entry : nodes) removeNode(entry.second);
    nodes.clear(); deliveries=json::array();
    nextNode=nextTransmission=0;txCount=rxCount=fragmentTx=fragmentRx=0;stepId=-1;
    auto* environment=getEnvir();
    for(int i=0;i<environment->getNumRNGs();++i)
        environment->getRNG(i)->initialize(seed,i,environment->getNumRNGs(),0,1,environment->getConfig());
    event("network.reset",{{"seed",seed},{"time_continues",true}});
}
void NetworkGateway::advance(const json& command) {
    double until = command.at("until"); long next = command.at("step_id");
    if (!std::isfinite(until) || until<=simTime().dbl() || until-simTime().dbl()>1.0+1e-9 || next<=stepId)
        throw std::runtime_error("advance requires increasing step_id and time, window <= 1s");
    synchronizeNodes(command.at("nodes"));
    const auto& transmissions = command.at("transmissions");
    if (!transmissions.is_array() || transmissions.size()>2048) throw std::runtime_error("invalid transmission batch");
    for (const auto& tx : transmissions) {
        std::string sender=tx.at("sender"),recipient=tx.value("recipient","broadcast");
        if (!nodes.count(sender)) throw std::runtime_error("unknown sender: "+sender);
        if (recipient!="broadcast" && !nodes.count(recipient)) {
            event("message.target_absent",{{"sender",sender},{"recipient",recipient}}); continue;
        }
        std::string content=tx.at("payload").dump();
        if (content.size()>1024*1024) throw std::runtime_error("wireless payload exceeds 1 MiB");
        veins::LAddress::L2Type address=veins::LAddress::L2BROADCAST();
        if (recipient!="broadcast") address=check_and_cast<veins::BaseMacLayer*>(nodes.at(recipient)->getSubmodule("nic")->getSubmodule("mac1609_4"))->getMACAddress();
        check_and_cast<WirelessApplication*>(nodes.at(sender)->getSubmodule("appl"))->transmit(sender,recipient,content,std::to_string(nextTransmission++),address);
        ++txCount;
        event("message.tx", {{"sender",sender},{"recipient",recipient},{"bytes",content.size()},{"payload",tx.at("payload")}});
    }
    stepId=next; pending=true; scheduleAt(SimTime(until),boundary);
}
void NetworkGateway::handleMessage(cMessage*) {
    if (pending) {
        json stats={{"messages_tx",txCount},{"messages_rx",rxCount},{"fragments_tx",fragmentTx},{"fragments_rx",fragmentRx},{"nodes",nodes.size()}};
        json record={{"step_id",stepId},{"sim_time",simTime().dbl()},{"stats",stats},{"delivered_count",deliveries.size()},
                     {"wall_time_unix",std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count()}};
        stepStatistics << record.dump() << '\n' << std::flush;
        if(!stepStatistics)throw cRuntimeError("Cannot flush network boundary statistics");
        writeArchiveManifest();
        sendResponse({{"ok",true},{"step_id",stepId},{"time",simTime().dbl()},{"deliveries",deliveries},{"stats",stats}});
        deliveries=json::array(); pending=false;
    }
    if (client<0) {
        client=accept(server,nullptr,nullptr);
        if (client<0) throw cRuntimeError("accept failed: %s",strerror(errno));
    }
    while (true) {
        try {
            auto command=readCommand(); auto action=command.value("action","");
            if (action=="hello" || action=="status") {
                sendResponse({{"ok",true},{"protocol",1},{"capabilities",{{"per_run_archives",true}}},{"radio","Veins IEEE 802.11p"},{"time",simTime().dbl()},{"nodes",nodes.size()}});
            } else if (action=="reset") {
                resetNetwork(command.at("seed").get<int>(),command.value("results_directory",std::string()));
                sendResponse({{"ok",true},{"reset",true},{"time",simTime().dbl()},{"seed",command.at("seed")},{"artifacts",artifactManifest()}});
            } else if (action=="shutdown") {
                sendResponse({{"ok",true},{"stopped",true}}); endSimulation(); return;
            } else if (action=="advance") {advance(command);return;}
            else sendResponse({{"ok",false},{"error","unknown action"}});
        } catch (const cTerminationException&) { throw; }
          catch (const cRuntimeError&) { throw; }
          catch (const std::exception& error) {
            event("protocol.error",{{"error",error.what()}});
            sendResponse({{"ok",false},{"error",error.what()},{"time",simTime().dbl()}});
        }
    }
}
void NetworkGateway::delivered(const std::string& receiver,const std::string& sender,const std::string& content,double sentAt) {
    Enter_Method_Silent();
    json packet={{"sender",sender},{"recipient",receiver},{"payload",json::parse(content)},{"sent_at",sentAt},{"received_at",simTime().dbl()}};
    deliveries.push_back(packet); ++rxCount; event("message.rx",packet);
}
void NetworkGateway::event(const std::string& type,const json& fields) {
    json value=fields; value["type"]=type;value["sim_time"]=simTime().dbl();
    value["wall_time_unix"]=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    value["module"]="OMNeT++";
    if(events) events << value.dump() << '\n' << std::flush;
}
void NetworkGateway::fragment(bool received) { if(received)++fragmentRx;else ++fragmentTx; }
void NetworkGateway::finish() {
    recordScalar("messagesTransmitted",txCount);recordScalar("messagesDelivered",rxCount);
    recordScalar("fragmentsTransmitted",fragmentTx);recordScalar("fragmentsReceived",fragmentRx);
    event("network.finished",{{"steps",stepId},{"messages_tx",txCount},{"messages_rx",rxCount}});
    writeArchiveManifest(true);
}
NetworkGateway::~NetworkGateway() {
    if(boundary)cancelAndDelete(boundary);
    if(client>=0)::close(client);if(server>=0)::close(server);
    if(!socketPath.empty())unlink(socketPath.c_str());
}
