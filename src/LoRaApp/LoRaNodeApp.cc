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
        forwardedPackets = 0;
        receivedPackets = 0;
        receivedACKs = 0;
        receivedOwnACKs = 0;
        receivedADRCommands = 0;

        numberOfDestinationsPerNode = par("numberOfDestinationsPerNode");
        numberOfPacketsPerDestination = par("numberOfPacketsPerDestination");

        numberOfPacketsToForward = par("numberOfPacketsToForward");
        maxHops = par("maxHops");

        LoRa_AppPacketSent = registerSignal("LoRa_AppPacketSent");

        //LoRa physical layer parameters
        loRaTP = par("initialLoRaTP").doubleValue();
        loRaCF = units::values::Hz(par("initialLoRaCF").doubleValue());
        loRaSF = par("initialLoRaSF");
        loRaBW = inet::units::values::Hz(par("initialLoRaBW").doubleValue());
        loRaCR = par("initialLoRaCR");
        loRaUseHeader = par("initialUseHeader");
        loRaCAD = par("initialLoRaCAD");
        loRaCADatt = par("initialLoRaCADatt").doubleValue();
        evaluateADRinNode = par("evaluateADRinNode");
        sfVector.setName("SF Vector");
        tpVector.setName("TP Vector");

        //Current network settings
        numberOfNodes = par("numberOfNodes");

        //Routing variables
        packetForwarding = par("packetForwarding");
        maxHops = par("maxHops");

        neighbourNodes = {};
        LoRaPacketsToSend = {};
        LoRaPacketsToForward = {};
        LoRaPacketsForwarded = {};
        ACKedNodes = {};

        //Node identifier
        nodeId = getContainingNode(this)->getIndex();

        //Application acknowledgment
        requestACKfromApp = par("requestACKfromApp");
        stopOnACK = par("stopOnACK");
        AppACKReceived = false;
        firstACK = 0;

        //Spreading factor
        increaseSF = par("increaseSF");
        firstACKSF = 0;
        packetsPerSF = par("packetsPerSF");
        packetsInSF = 0;

        //WATCHES only for GUI
        if (getEnvir()->isGUI()) {
            WATCH(sentPackets);
            WATCH(forwardedPackets);
            WATCH(receivedPackets);
            WATCH(receivedACKs);
            WATCH(receivedOwnACKs);
            WATCH(AppACKReceived);
            WATCH(firstACK);

            WATCH(loRaSF);
            WATCH(packetsInSF);

            WATCH_VECTOR(neighbourNodes);
            WATCH_VECTOR(ACKedNodes);

            WATCH_VECTOR(LoRaPacketsToSend);
            WATCH_VECTOR(LoRaPacketsToForward);
            WATCH_VECTOR(LoRaPacketsForwarded);
        }

        generateDataPackets();
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
    recordScalar("forwardedPackets", forwardedPackets);
    recordScalar("receivedPackets", receivedPackets);
    recordScalar("receivedACKs", receivedACKs);
    recordScalar("receivedOwnACKs", receivedOwnACKs);
    recordScalar("receivedADRCommands", receivedADRCommands);
    recordScalar("AppACKReceived", AppACKReceived);
    recordScalar("firstACK", firstACK);
    recordScalar("firstACKSF", firstACKSF);

    for (std::vector<LoRaAppPacket>::iterator lbptr = LoRaPacketsToSend.begin(); lbptr < LoRaPacketsToSend.end(); lbptr++) {
        LoRaPacketsToSend.erase(lbptr);
    }

    for (std::vector<LoRaAppPacket>::iterator lbptr = LoRaPacketsToForward.begin(); lbptr < LoRaPacketsToForward.end(); lbptr++) {
        LoRaPacketsToForward.erase(lbptr);
    }

    for (std::vector<LoRaAppPacket>::iterator lbptr = LoRaPacketsForwarded.begin(); lbptr < LoRaPacketsForwarded.end(); lbptr++) {
        LoRaPacketsForwarded.erase(lbptr);
    }
}

void LoRaNodeApp::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) {

        if (msg == selfDataPacket) {

            if (simTime() >= getSimulation()->getWarmupPeriod())
            {
                bool schedule = false;

                // Check conditions for sending own data packet
                if ( LoRaPacketsToSend.size() > 0 ) // || sentPackets < numberOfPacketsToSend) && (!AppACKReceived || !stopOnACK))
                {
                    sendDataPacket();
                    schedule = true;
                }
                // Check conditions for forwarding own data packet
                else if ( forwardedPackets < numberOfPacketsToForward || numberOfPacketsToForward == 0)
                {
                    // Only go to the sendDataPacket() function if there is something to be forwarded
                    if ( LoRaPacketsToForward.size() > 0 )
                        sendDataPacket();
                    // Schedule a self-message, since we have room for forwarding
                    schedule = true;
                }

                // Schedule the next self-message
                if (schedule)
                {
                    double time;
                    if(loRaSF == 7) time = 7.808;
                    if(loRaSF == 8) time = 13.9776;
                    if(loRaSF == 9) time = 24.6784;
                    if(loRaSF == 10) time = 49.3568;
                    if(loRaSF == 11) time = 85.6064;
                    if(loRaSF == 12) time = 171.2128;

                    do {
                        //(ToDo) Warning: too small a par("timeToNextPacket") might cause a lock here
                        //(ToDo) Note: simple workaround
                        // timeToNextPacket = par("timeToNextPacket");
                           timeToNextPacket = math::max(time, par("timeToNextPacket"));
                    } while(timeToNextPacket <= time);

                    selfDataPacket = new cMessage("selfDataPacket");
                    scheduleAt(simTime() + timeToNextPacket, selfDataPacket);
                }
            }
        }
        delete msg;
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

    switch(packetForwarding) {

        // Dummy broadcast flooding. Packets for other nodes are broadcast once if their TTL is > 0
        case 0:
            // DATA and ACK packets are handled separately
            switch (packet->getMsgType()) {
                // DATA packets
                case DATA:
                    bubble("DATA packet received!");

                    // Check if the packet is for the current node
                    if (packet->getDestination() == nodeId) {
                        bubble("I received a data packet for me!");
                        // ToDo delete packet;
                        // ToDo count this packet
                    }
                    // Packet for another node
                    else {
                        // Check if the packet is from another node to avoid loops of own packets
                        if (packet->getSource() == nodeId) {
                            bubble("I received a LoRa packet originally sent by me!");
                            // ToDo delete packet
                            // ToDo count this packet
                        }
                        // Packet from another node
                        else {
                            // Check for too old packets with TTL == 0
                            if ( packet->getTtl() == 0 ) {
                                bubble("This packet has reached TTL expiration!");
                                // ToDo delete packet
                                // ToDo count this packet
                            }
                            // Packet has not reached its maximum TTL
                            else {
                                // Check if the packet has already been forwarded
                                if ( isPacketForwarded(packet) ) {
                                    bubble("This packet has already been forwarded!");
                                    // ToDo delete packet
                                }
                                // Cool, it can be forwarded!
                                else {
                                    if ( isPacketToBeForwarded(packet) ) {
                                        bubble("This packet is already scheduled to be forwarded!");
                                        // ToDo delete packet
                                    }
                                    else {
                                        bubble("Saving packet to forward it later!");

                                        // Duplicate the packet
                                        LoRaAppPacket *dataPacket = new LoRaAppPacket("DataFrame");
                                        dataPacket->setMsgType(packet->getMsgType());

                                        dataPacket->setDataInt(packet->getDataInt());

                                        dataPacket->setSource(packet->getSource());
                                        dataPacket->setDestination(packet->getDestination());
                                        dataPacket->setVia(nodeId);

                                        dataPacket->getOptions().setAppACKReq(packet->getOptions().getAppACKReq());
                                        dataPacket->getOptions().setADRACKReq(packet->getOptions().getADRACKReq());

                                        dataPacket->setTtl(packet->getTtl() -1 );

                                        LoRaPacketsToForward.push_back(*dataPacket);
                                        // ToDo delete packet
                                    }
                                }
                            }
                        }
                    }
                    break;

                case ACK:
                    break;

                default:
                    bubble("Packet of unknown type received!");
                    // ToDo delete packet
                    break;

            }// Closes: "switch (packet->getMsgType())"

        case 1:
            break;

        default:
           break;

    }// Closes: "switch (forwarding)"
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
    bool transmit = false;

    int packetPos = 0;

    // Own packets are [generated and] sent first, then we may forward others'
    if (LoRaPacketsToSend.size() > 0) {

        LoRaAppPacket *firstDataPacket = &LoRaPacketsToSend.front();
        dataPacket = firstDataPacket->dup();
        LoRaPacketsToSend.erase(LoRaPacketsToSend.begin());

        transmit= true;
        sentPackets++;
    }

    // Forward other nodes' packets
    else {
        if (LoRaPacketsToForward.size() > 0) {

            // bubble("Forwarding packet!");

            LoRaAppPacket *firstDataPacket = &LoRaPacketsToForward.at(0);
            dataPacket = firstDataPacket->dup();
            LoRaPacketsToForward.erase(LoRaPacketsToForward.begin());

            // ToDo
            switch(packetForwarding) {
                // Dummy broadcast flooding
                case 0:
                {
                    break;
                }

                default:
                    break;
            }

            transmit= true;
            forwardedPackets++;
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

    if (transmit) {
        sfVector.record(loRaSF);
        tpVector.record(loRaTP);
        send(dataPacket, "appOut");
        emit(LoRa_AppPacketSent, loRaSF);
    }
}

void LoRaNodeApp::generateDataPackets() {

    std::vector<int> destinations = {};

    while (destinations.size() < numberOfDestinationsPerNode) {

        int destination = intuniform(0,  numberOfNodes-1);

        if (destination != nodeId) {
            bool newDestination = true;

            for (int i = 0; i< destinations.size(); i++) {
                if (destination == destinations[i]) {
                    newDestination = false;
                    break;
                }
            }

            if (newDestination){
                destinations.push_back(destination);
            }

        }
    }

    for (int j = 0; j < destinations.size(); j++) {
        for (int k = 0; k < numberOfPacketsPerDestination; k++) {
            LoRaAppPacket *dataPacket = new LoRaAppPacket("DataFrame");

            dataPacket->setMsgType(DATA);
            dataPacket->setDataInt(k);
            dataPacket->setSource(nodeId);
            dataPacket->setVia(nodeId);
            dataPacket->setDestination(j);
            dataPacket->getOptions().setAppACKReq(requestACKfromApp);

            switch (packetForwarding) {
            case 0:
                dataPacket->setTtl(maxHops);
                break;
            default:
                dataPacket->setTtl(maxHops);
                break;
            }

            LoRaPacketsToSend.push_back(*dataPacket);
         }
    }
}

void LoRaNodeApp::increaseSFIfPossible()
{
    if(loRaSF < 12) {
        // char text[32];
        // sprintf(text, "Increasing SF from %d to %d", loRaSF, loRaSF+1);
        // bubble(text);
        loRaSF++;
    }
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

bool LoRaNodeApp::isACKed(int nodeId)
{
    for (std::vector<int>::iterator nbptr = ACKedNodes.begin(); nbptr < ACKedNodes.end(); nbptr++) {
        if ( nodeId == *nbptr ) {
            return true;
        }
    }
    return false;
}

bool LoRaNodeApp::isPacketForwarded(cMessage *msg)
{
    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    for (std::vector<LoRaAppPacket>::iterator lbptr = LoRaPacketsForwarded.begin(); lbptr < LoRaPacketsForwarded.end(); lbptr++) {
        if ( packet->getMsgType() == lbptr->getMsgType() &&
                packet->getDataInt() == lbptr->getDataInt() &&
                packet->getSource() == lbptr->getSource() &&
                packet->getDestination() == lbptr->getDestination() ) {
            return true;
        }
    }
    return false;
}

bool LoRaNodeApp::isPacketToBeForwarded(cMessage *msg)
{
    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    for (std::vector<LoRaAppPacket>::iterator lbptr = LoRaPacketsToForward.begin(); lbptr < LoRaPacketsToForward.end(); lbptr++) {
        if ( packet->getMsgType() == lbptr->getMsgType() &&
                packet->getDataInt() == lbptr->getDataInt() &&
                packet->getSource() == lbptr->getSource() &&
                packet->getDestination() == lbptr->getDestination() ) {
            return true;
        }
    }
    return false;
}

} //end namespace inet

