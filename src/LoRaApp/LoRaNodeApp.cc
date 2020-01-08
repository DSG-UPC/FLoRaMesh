//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "LoRaNodeApp.h"

#include "inet/mobility/static/StationaryMobility.h"
namespace inet {

Define_Module(LoRaNodeApp);

void LoRaNodeApp::initialize(int stage)
{
    cSimpleModule::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        std::pair<double,double> coordsValues = std::make_pair(-1, -1);
        cModule *host = getContainingNode(this);
        // Generate random location for nodes if circle deployment type
        if (strcmp(host->par("deploymentType").stringValue(), "circle")==0) {
           coordsValues = generateUniformCircleCoordinates(host->par("maxGatewayDistance").doubleValue(), host->par("gatewayX").doubleValue(), host->par("gatewayY").doubleValue());
           StationaryMobility *mobility = check_and_cast<StationaryMobility *>(host->getSubmodule("mobility"));
           mobility->par("initialX").setDoubleValue(coordsValues.first);
           mobility->par("initialY").setDoubleValue(coordsValues.second);
        }
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        bool isOperational;
        NodeStatus *nodeStatus = dynamic_cast<NodeStatus *>(findContainingNode(this)->getSubmodule("status"));
        isOperational = (!nodeStatus) || nodeStatus->getState() == NodeStatus::UP;
        if (!isOperational)
            throw cRuntimeError("This module doesn't support starting in node DOWN state");
        do {
            timeToFirstPacket = par("timeToFirstPacket");
            EV << "Wylosowalem czas :" << timeToFirstPacket << endl;
            //if(timeToNextPacket < 5) error("Time to next packet must be grater than 3");
        } while(timeToFirstPacket <= 5);

        //timeToFirstPacket = par("timeToFirstPacket");
        selfDataPacket = new cMessage("selfDataPacket");
        scheduleAt(simTime()+timeToFirstPacket, selfDataPacket);

        sentPackets = 0;
        receivedPackets = 0;
        receivedADRCommands = 0;
        numberOfPacketsToSend = par("numberOfPacketsToSend");

        LoRa_AppPacketSent = registerSignal("LoRa_AppPacketSent");

        //LoRa physical layer parameters
        loRaTP = par("initialLoRaTP").doubleValue();
        loRaCF = units::values::Hz(par("initialLoRaCF").doubleValue());
        loRaSF = par("initialLoRaSF");
        loRaBW = inet::units::values::Hz(par("initialLoRaBW").doubleValue());
        loRaCR = par("initialLoRaCR");
        loRaUseHeader = par("initialUseHeader");
        evaluateADRinNode = par("evaluateADRinNode");
        sfVector.setName("SF Vector");
        tpVector.setName("TP Vector");

        //Current network settings
        numberOfNodes = par("numberOfNodes");

        //Routing variables
        packetForwarding = par("packetForwarding");
        numberOfHops = par("numberOfHops");

        neighbourNodes = {};
        LoRaPacketBuffer = {};

        //Node identifier
        nodeId = getContainingNode(this)->getIndex();

    }
}

std::pair<double,double> LoRaNodeApp::generateUniformCircleCoordinates(double radius, double gatewayX, double gatewayY)
{
    double randomValueRadius = uniform(0,(radius*radius));
    double randomTheta = uniform(0,2*M_PI);

    // generate coordinates for circle with origin at 0,0
    double x = sqrt(randomValueRadius) * cos(randomTheta);
    double y = sqrt(randomValueRadius) * sin(randomTheta);
    // Change coordinates based on coordinate system used in OMNeT, with origin at top left
    x = x + gatewayX;
    y = gatewayY - y;
    std::pair<double,double> coordValues = std::make_pair(x,y);
    return coordValues;
}

void LoRaNodeApp::finish()
{
    cModule *host = getContainingNode(this);
    StationaryMobility *mobility = check_and_cast<StationaryMobility *>(host->getSubmodule("mobility"));
    Coord coord = mobility->getCurrentPosition();
    recordScalar("positionX", coord.x);
    recordScalar("positionY", coord.y);
    recordScalar("finalTP", loRaTP);
    recordScalar("finalSF", loRaSF);
    recordScalar("sentPackets", sentPackets);
    recordScalar("receivedPackets", receivedPackets);
    recordScalar("receivedADRCommands", receivedADRCommands);
}

void LoRaNodeApp::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) {

        if (msg == selfDataPacket) {

            sendDataPacket();

            if (simTime() >= getSimulation()->getWarmupPeriod())
                sentPackets++;

            delete msg;

            if (numberOfPacketsToSend == 0 || sentPackets < numberOfPacketsToSend || LoRaPacketBuffer.size() > 0) {
                double time;
                if(loRaSF == 7) time = 7.808;
                if(loRaSF == 8) time = 13.9776;
                if(loRaSF == 9) time = 24.6784;
                if(loRaSF == 10) time = 49.3568;
                if(loRaSF == 11) time = 85.6064;
                if(loRaSF == 12) time = 171.2128;

            do {
                //(ToDo) Warning: too small a par("timeToNextPacket") might cause a lock here
                timeToNextPacket = par("timeToNextPacket");
            } while(timeToNextPacket <= time);

            selfDataPacket = new cMessage("selfDataPacket");
            scheduleAt(simTime() + timeToNextPacket, selfDataPacket);
            }
        }
    }
    else
    {
        handleMessageFromLowerLayer(msg);
        delete msg;
    }
}

void LoRaNodeApp::handleMessageFromLowerLayer(cMessage *msg)
{
    receivedPackets++;

    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    //Keep track of neighbouring nodes
    if (!isNeighbour(packet->getVia())){
        bubble ("New neighbour!");
        neighbourNodes.push_back(packet->getVia());
    }

    //Check if the packet is for the current node
    if (packet->getDestination() == nodeId) {
        bubble("I received a LoRa packet for me!");
    }
    //Check for retransmissions of packages originally sent by this node
    else if (packet->getSource() == nodeId) {
        bubble("I received a LoRa packet originally sent by me!");
    }
    //Forward packet
    else {
        switch(packetForwarding) {

        // No forwarding
        case 0 :
            bubble("Packet forwarding disabled!");
            break;

        //N-hop broadcast forwarding
        case 1 :
            if ( packet->getHops() > 0 ) {
                bubble("I received a LoRa packet to retransmit!");

                LoRaAppPacket *dataPacket = new LoRaAppPacket("DataFrame");
                dataPacket->setKind(packet->getKind());

                dataPacket->setDataInt(packet->getDataInt());

                dataPacket->setSource(packet->getSource());
                dataPacket->setDestination(packet->getDestination());
                dataPacket->setVia(nodeId);

                dataPacket->setHops(packet->getHops() -1 );

                LoRaPacketBuffer.push_back(*dataPacket);
            }
            else {
                bubble("I received a LoRa packet that has reached the maximum hop count!");
            }

        }
    }
}

bool LoRaNodeApp::handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback)
{
    Enter_Method_Silent();

    throw cRuntimeError("Unsupported lifecycle operation '%s'", operation->getClassName());
    return true;
}

void LoRaNodeApp::sendDataPacket()
{
    LoRaAppPacket *dataPacket = new LoRaAppPacket("DataFrame");

    if (LoRaPacketBuffer.size() > 0 ) {
        bubble("Forwarding packet!");
        LoRaAppPacket *frontDataPacket = &LoRaPacketBuffer.front();
        dataPacket = frontDataPacket->dup();
        LoRaPacketBuffer.erase (LoRaPacketBuffer.begin());
    }
    else {

        dataPacket->setKind(DATA);


        dataPacket->setDataInt(sentPackets);

        dataPacket->setSource(nodeId);
        dataPacket->setVia(nodeId);

        do dataPacket->setDestination(intuniform(0, numberOfNodes-1));
        while (dataPacket->getDestination() == nodeId);

        if ( isNeighbour(dataPacket->getDestination())){
            dataPacket->setHops(0);
        }

        else {
            dataPacket->setHops(numberOfHops);
        }
    }


    //add LoRa control info
    LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;
    cInfo->setLoRaTP(loRaTP);
    cInfo->setLoRaCF(loRaCF);
    cInfo->setLoRaSF(loRaSF);
    cInfo->setLoRaBW(loRaBW);
    cInfo->setLoRaCR(loRaCR);

    dataPacket->setControlInfo(cInfo);

    sfVector.record(loRaSF);
    tpVector.record(loRaTP);

    send(dataPacket, "appOut");

    emit(LoRa_AppPacketSent, loRaSF);
}

void LoRaNodeApp::increaseSFIfPossible()
{
    if(loRaSF < 12) loRaSF++;
}

bool LoRaNodeApp::isNeighbour(int neighbourId)
{
    for (std::vector<int>::iterator nbptr = neighbourNodes.begin(); nbptr < neighbourNodes.end(); nbptr++) {
        if ( neighbourId == *nbptr ) {
            return true;
        }
    }
    return false;
}

} //end namespace inet
