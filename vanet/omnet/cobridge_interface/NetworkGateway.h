#pragma once
#include <omnetpp.h>
#include <nlohmann/json.hpp>
#include <map>
#include <string>
#include <fstream>
using nlohmann::json;
using namespace omnetpp;

class NetworkGateway : public cSimpleModule {
    int server = -1, client = -1;
    std::string socketPath, buffer;
    cMessage* boundary = nullptr;
    std::map<std::string, cModule*> nodes;
    std::ofstream events, stepStatistics;
    std::string resultsDirectory;
    double archiveStartTime = 0;
    json deliveries = json::array();
    long stepId = -1, txCount = 0, rxCount = 0, fragmentTx = 0, fragmentRx = 0;
    unsigned long nextNode = 0, nextTransmission = 0;
    bool pending = false;
    void sendResponse(const json& value);
    json readCommand();
    void synchronizeNodes(const json& values);
    void removeNode(cModule* node);
    void resetNetwork(int seed, const std::string& directory);
    void selectResultsDirectory(const std::string& directory);
    json artifactManifest() const;
    void writeArchiveManifest(bool closed = false) const;
    void advance(const json& command);
protected:
    void initialize() override;
    void handleMessage(cMessage* message) override;
    void finish() override;
public:
    ~NetworkGateway() override;
    void delivered(const std::string& receiver, const std::string& sender,
                   const std::string& content, double sentAt);
    void event(const std::string& type, const json& fields);
    void fragment(bool received);
};
