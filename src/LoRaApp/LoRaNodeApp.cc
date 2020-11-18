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
#include "inet/common/FSMA.h"
#include "../LoRa/LoRaMac.h"


#include "inet/mobility/static/StationaryMobility.h"
namespace inet {

#define BROADCAST_ADDRESS   16777215

#define NO_FORWARDING               0
#define DUMMY_BROADCAST_SINGLE_SF   1
#define SMART_BROADCAST_SINGLE_SF   2
#define HOP_COUNT_SINGLE_SF         3
#define RSSI_SINGLE_SF              4
#define TIME_ON_AIR_CAD             5

Define_Module (LoRaNodeApp);

void LoRaNodeApp::initialize(int stage) {
    cSimpleModule::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        // Get this node's ID
        nodeId = getContainingNode(this)->getIndex();
        std::pair<double, double> coordsValues = std::make_pair(-1, -1);
        cModule *host = getContainingNode(this);

        //EV << "AAAAAAAAAAAAAAAAAAAAAA" << getParentModule()->getSubmodule("LoRaNodeNic")->getSubmodule("mac")->getDisplayString() << endl;


        // Generate random location for nodes if circle deployment type
        if (strcmp(host->par("deploymentType").stringValue(), "circle") == 0) {
            coordsValues = generateUniformCircleCoordinates(
                    host->par("maxGatewayDistance").doubleValue(),
                    host->par("gatewayX").doubleValue(),
                    host->par("gatewayY").doubleValue());
            StationaryMobility *mobility = check_and_cast<StationaryMobility *>(
                    host->getSubmodule("mobility"));
            mobility->par("initialX").setDoubleValue(coordsValues.first);
            mobility->par("initialY").setDoubleValue(coordsValues.second);
        } else if (strcmp(host->par("deploymentType").stringValue(), "edges")
                == 0) {
            int minX = host->par("minX");
            int maxX = host->par("maxX");
            int minY = host->par("minY");
            int maxY = host->par("maxY");
            StationaryMobility *mobility = check_and_cast<StationaryMobility *>(
                    host->getSubmodule("mobility"));
            mobility->par("initialX").setDoubleValue(
                    minX + maxX * (((nodeId + 1) % 4 / 2) % 2));
            mobility->par("initialY").setDoubleValue(
                    minY + maxY * (((nodeId) % 4 / 2) % 2));
        } else if (strcmp(host->par("deploymentType").stringValue(), "grid")
                == 0) {
            int minX = host->par("minX");
            int sepX = host->par("sepX");
            int minY = host->par("minY");
            int sepY = host->par("sepY");
            int cols = host->par("cols");
            StationaryMobility *mobility = check_and_cast<StationaryMobility *>(
                    host->getSubmodule("mobility"));
            mobility->par("initialX").setDoubleValue(
                    minX + sepX * (nodeId % cols));
            mobility->par("initialY").setDoubleValue(
                    minY + sepY * ((int) nodeId / cols));
        }
    } else if (stage == INITSTAGE_APPLICATION_LAYER) {
        bool isOperational;

        NodeStatus *nodeStatus = dynamic_cast<NodeStatus *>(findContainingNode(this)->getSubmodule("status"));
        isOperational = (!nodeStatus) || nodeStatus->getState() == NodeStatus::UP;

        if (!isOperational)
        {
            throw cRuntimeError("This module doesn't support starting in node DOWN state");
        }

        timeToFirstPacket = math::max(5, par("timeToFirstPacket"));
        timeToFirstRoutingPacket = math::max(5, par("timeToFirstRoutingPacket"));

        EV << "Time to first data packet: " << timeToFirstPacket << endl;
        EV << "Time to first routing packet: " << timeToFirstRoutingPacket << endl;

        // Timer for sending local data packets or forwarding them
        selfDataPacket = new cMessage("selfDataPacket");
        scheduleAt(simTime() + timeToFirstPacket, selfDataPacket);

        // Timer for sending routing protocol packets
        selfRoutingPacket = new cMessage("selfRoutingPacket");
        scheduleAt(simTime() + timeToFirstRoutingPacket, selfRoutingPacket);

        sentPackets = 0;
        sentDataPackets = 0;
        sentRoutingPackets = 0;
        sentAckPackets = 0;
        receivedPackets = 0;
        receivedPacketsForMe = 0;
        receivedPacketsFromMe = 0;
        receivedPacketsToForward = 0;
        receivedDataPackets = 0;
        receivedDataPacketsForMe = 0;
        receivedDataPacketsForMeUnique = 0;
        receivedDataPacketsFromMe = 0;
        receivedDataPacketsToForward = 0;
        receivedDataPacketsToForwardCorrect = 0;
        receivedDataPacketsToForwardExpired = 0;
        receivedDataPacketsToForwardUnique = 0;
        receivedAckPackets = 0;
        receivedAckPacketsForMe = 0;
        receivedAckPacketsFromMe = 0;
        receivedAckPacketsToForward = 0;
        receivedAckPacketsToForwardCorrect = 0;
        receivedAckPacketsToForwardExpired = 0;
        receivedAckPacketsToForwardUnique = 0;
        receivedADRCommands = 0;
        forwardedPackets = 0;
        forwardedDataPackets = 0;
        forwardedAckPackets = 0;

        numberOfDestinationsPerNode = par("numberOfDestinationsPerNode");
        numberOfPacketsPerDestination = par("numberOfPacketsPerDestination");

        numberOfPacketsToForward = par("numberOfPacketsToForward");
        maxHops = par("maxHops");

        LoRa_AppPacketSent = registerSignal("LoRa_AppPacketSent");

        //LoRa physical layer parameters
        loRaTP = par("initialLoRaTP").doubleValue();
        loRaCF = units::values::Hz(par("initialLoRaCF").doubleValue());
        loRaSF = par("initialLoRaSF");
        minLoRaSF = par("minLoRaSF");
        maxLoRaSF = par("maxLoRaSF");
        loRaBW = inet::units::values::Hz(par("initialLoRaBW").doubleValue());
        loRaCR = par("initialLoRaCR");
        loRaUseHeader = par("initialUseHeader");
        loRaCAD = par("initialLoRaCAD");
        loRaCADatt = par("initialLoRaCADatt").doubleValue();
        evaluateADRinNode = par("evaluateADRinNode");
        txSfVector.setName("Tx SF Vector");
        txTpVector.setName("Tx TP Vector");
        rxRssiVector.setName("Rx RSSI Vector");
        rxSfVector.setName("Rx SF Vector");

        //Current network settings
        numberOfNodes = par("numberOfNodes");

        //Routing variables
        packetForwarding = par("packetForwarding");
        ownDataPriority = par("ownDataPriority");
        maxHops = par("maxHops");

        neighbourNodes = {};
        knownNodes = {};
        LoRaPacketsToSend = {};
        LoRaPacketsToForward = {};
        LoRaPacketsForwarded = {};
        DataPacketsForMe = {};
        ACKedNodes = {};

        //Routing table
        singleMetricRoutingTable = {};
        dualMetricRoutingTable = {};

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
            WATCH(sentDataPackets);
            WATCH(sentRoutingPackets);
            WATCH(sentAckPackets);
            WATCH(receivedPackets);
            WATCH(receivedPacketsForMe);
            WATCH(receivedPacketsFromMe);
            WATCH(receivedPacketsToForward);
            WATCH(receivedDataPackets);
            WATCH(receivedDataPacketsForMe);
            WATCH(receivedDataPacketsForMeUnique);
            WATCH(receivedDataPacketsFromMe);
            WATCH(receivedDataPacketsToForward);
            WATCH(receivedDataPacketsToForwardCorrect);
            WATCH(receivedDataPacketsToForwardExpired);
            WATCH(receivedDataPacketsToForwardUnique);
            WATCH(receivedAckPackets);
            WATCH(receivedAckPacketsForMe);
            WATCH(receivedAckPacketsFromMe);
            WATCH(receivedAckPacketsToForward);
            WATCH(receivedAckPacketsToForwardCorrect);
            WATCH(receivedAckPacketsToForwardExpired);
            WATCH(receivedAckPacketsToForwardUnique);
            WATCH(receivedADRCommands);
            WATCH(forwardedPackets);
            WATCH(forwardedDataPackets);
            WATCH(forwardedAckPackets);

            WATCH(AppACKReceived);
            WATCH(firstACK);

            WATCH(loRaSF);
            WATCH(packetsInSF);

            WATCH_VECTOR(neighbourNodes);
            WATCH_VECTOR(knownNodes);
            WATCH_VECTOR(ACKedNodes);

            //WATCH_VECTOR(singleMetricRoutingTable);
            //WATCH_VECTOR(dualMetreicRoutingTable);

            WATCH_VECTOR(LoRaPacketsToSend);
            WATCH_VECTOR(LoRaPacketsToForward);
            WATCH_VECTOR(LoRaPacketsForwarded);
            WATCH_VECTOR(DataPacketsForMe);
        }

        generateDataPackets();
    }
}

std::pair<double, double> LoRaNodeApp::generateUniformCircleCoordinates(
        double radius, double gatewayX, double gatewayY) {
    double randomValueRadius = uniform(0, (radius * radius));
    double randomTheta = uniform(0, 2 * M_PI);

    // generate coordinates for circle with origin at 0,0
    double x = sqrt(randomValueRadius) * cos(randomTheta);
    double y = sqrt(randomValueRadius) * sin(randomTheta);
    // Change coordinates based on coordinate system used in OMNeT, with origin at top left
    x = x + gatewayX;
    y = gatewayY - y;
    std::pair<double, double> coordValues = std::make_pair(x, y);
    return coordValues;
}

void LoRaNodeApp::finish() {
    cModule *host = getContainingNode(this);
    StationaryMobility *mobility = check_and_cast<StationaryMobility *>(
            host->getSubmodule("mobility"));
    Coord coord = mobility->getCurrentPosition();
    recordScalar("positionX", coord.x);
    recordScalar("positionY", coord.y);
    recordScalar("finalTP", loRaTP);
    recordScalar("finalSF", loRaSF);

    recordScalar("sentPackets", sentPackets);
    recordScalar("sentDataPackets", sentDataPackets);
    recordScalar("sentRoutingPackets", sentRoutingPackets);
    recordScalar("sentAckPackets", sentAckPackets);
    recordScalar("receivedPackets", receivedPackets);
    recordScalar("receivedPacketsForMe", receivedPacketsForMe);
    recordScalar("receivedPacketsFromMe", receivedPacketsFromMe);
    recordScalar("receivedPacketsToForward", receivedPacketsToForward);
    recordScalar("receivedDataPackets", receivedDataPackets);
    recordScalar("receivedDataPacketsForMe", receivedDataPacketsForMe);
    recordScalar("receivedDataPacketsForMeUnique", receivedDataPacketsForMeUnique);
    recordScalar("receivedDataPacketsFromMe", receivedDataPacketsFromMe);
    recordScalar("receivedDataPacketsToForward", receivedDataPacketsToForward);
    recordScalar("receivedDataPacketsToForwardCorrect",
            receivedDataPacketsToForwardCorrect);
    recordScalar("receivedDataPacketsToForwardExpired",
            receivedDataPacketsToForwardExpired);
    recordScalar("receivedDataPacketsToForwardUnique",
            receivedDataPacketsToForwardUnique);
    recordScalar("receivedAckPacketsToForward", receivedAckPacketsToForward);
    recordScalar("receivedAckPacketsToForwardCorrect",
            receivedAckPacketsToForwardCorrect);
    recordScalar("receivedAckPacketsToForwardExpired",
            receivedAckPacketsToForwardExpired);
    recordScalar("receivedAckPacketsToForwardUnique",
            receivedAckPacketsToForwardUnique);
    recordScalar("receivedAckPackets", receivedAckPackets);
    recordScalar("receivedAckPacketsForMe", receivedAckPacketsForMe);
    recordScalar("receivedAckPacketsFromMe", receivedAckPacketsFromMe);
    recordScalar("receivedADRCommands", receivedADRCommands);
    recordScalar("forwardedPackets", forwardedPackets);
    recordScalar("forwardedDataPackets", forwardedDataPackets);
    recordScalar("forwardedAckPackets", forwardedAckPackets);

    recordScalar("receivedADRCommands", receivedADRCommands);
    recordScalar("AppACKReceived", AppACKReceived);
    recordScalar("firstACK", firstACK);
    recordScalar("firstACKSF", firstACKSF);

    for (std::vector<LoRaAppPacket>::iterator lbptr = LoRaPacketsToSend.begin();
            lbptr < LoRaPacketsToSend.end(); lbptr++) {
        LoRaPacketsToSend.erase(lbptr);
    }

    for (std::vector<LoRaAppPacket>::iterator lbptr =
            LoRaPacketsToForward.begin(); lbptr < LoRaPacketsToForward.end();
            lbptr++) {
        LoRaPacketsToForward.erase(lbptr);
    }

    for (std::vector<LoRaAppPacket>::iterator lbptr =
            LoRaPacketsForwarded.begin(); lbptr < LoRaPacketsForwarded.end();
            lbptr++) {
        LoRaPacketsForwarded.erase(lbptr);
    }

    for (std::vector<LoRaAppPacket>::iterator lbptr =
            DataPacketsForMe.begin(); lbptr < DataPacketsForMe.end();
            lbptr++) {
        DataPacketsForMe.erase(lbptr);
    }
}

void LoRaNodeApp::handleMessage(cMessage *msg) {

    if (msg->isSelfMessage()) {
        if (msg == selfDataPacket) {
            handleSelfDataPacket();
        }
        else if (msg == selfRoutingPacket) {
            handleSelfRoutingPacket();
        }
    } else {
        handleMessageFromLowerLayer(msg);
    }

    delete msg;
}


void LoRaNodeApp::handleSelfDataPacket() {

    LoRaMac *lrmc = (LoRaMac *)getParentModule()->getSubmodule("LoRaNic")->getSubmodule("mac");

    bool schedule = false;

    // Only proceed to send a data packet if the 'mac' module in 'LoRaNic' is IDLE and the warmup period is due
    if ( lrmc->fsm.getState() == IDLE && simTime() >= getSimulation()->getWarmupPeriod() ) {

        // Check conditions for sending own data packet
        if (LoRaPacketsToSend.size() > 0)  // || sentPackets < numberOfPacketsToSend) && (!AppACKReceived || !stopOnACK))
        {
            sendPacket();
            schedule = true;
        }
        // Check conditions for forwarding own data packet
        else if (forwardedPackets < numberOfPacketsToForward
                || numberOfPacketsToForward == 0) {
            // Only go to the sendDataPacket() function if there is something to be forwarded
            if (LoRaPacketsToForward.size() > 0)
                sendPacket();
            // Schedule a self-message, since we have room for forwarding
            schedule = true;
        }
    }
    else {
        schedule = true;
    }

    // Schedule the next self-message
        if (schedule) {
            double time;
            if (loRaSF == 7)
                time = 7.808;
            if (loRaSF == 8)
                time = 13.9776;
            if (loRaSF == 9)
                time = 24.6784;
            if (loRaSF == 10)
                time = 49.3568;
            if (loRaSF == 11)
                time = 85.6064;
            if (loRaSF == 12)
                time = 171.2128;

        do {
            // Warning: too small a par("timeToNextPacket") causes a lock here.
            // timeToNextPacket = par("timeToNextPacket");
            //
            // Workaround:
            timeToNextPacket = math::max(time,
                    par("timeToNextPacket"));
        } while (timeToNextPacket <= time);

        selfDataPacket = new cMessage("selfDataPacket");
        scheduleAt(simTime() + timeToNextPacket, selfDataPacket);
    }
}

void LoRaNodeApp::handleSelfRoutingPacket() {

    LoRaMac *lrmc = (LoRaMac *)getParentModule()->getSubmodule("LoRaNic")->getSubmodule("mac");

    // Only proceed to send a routing packet if the 'mac' module in 'LoRaNic' is IDLE
    if ( lrmc->fsm.getState() == IDLE ) {

        sendRoutingPacket();
        }

    double time;
            if (loRaSF == 7)
                time = 7.808;
            if (loRaSF == 8)
                time = 13.9776;
            if (loRaSF == 9)
                time = 24.6784;
            if (loRaSF == 10)
                time = 49.3568;
            if (loRaSF == 11)
                time = 85.6064;
            if (loRaSF == 12)
                time = 171.2128;

    selfRoutingPacket = new cMessage("selfRoutingPacket");
    timeToNextPacket = math::max(time, par("timeToNextRoutingPacket"));

    scheduleAt(simTime() + timeToNextPacket, selfRoutingPacket);
}


void LoRaNodeApp::handleMessageFromLowerLayer(cMessage *msg) {
    receivedPackets++;

    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    // Check if the packet is from this node (i.e., a packet that some
    // other node is broadcasting which we have happened to receive). We
    // count it and discard it immediately.
    if (packet->getSource() == nodeId) {
        receivedDataPacketsFromMe++;
        bubble("I received a LoRa packet originally sent by me!");
    }
    // Else, check if the packet is for this node (i.e., a packet directly
    // received from the origin or relayed by a neighbour)
    else if (packet->getDestination() == nodeId) {
        manageReceivedPacketForMe(packet);
    }
    // Else it can be a routing protocol broadcast message
    else if (packet->getDestination() == BROADCAST_ADDRESS) {
        manageReceivedRoutingPacket(packet);
    }
    // Else it can be a data packet from and to other nodes...
    else {
        // which we may forward, if it is being broadcast
        if (packet->getVia() == BROADCAST_ADDRESS) {
            manageReceivedDataPacketToForward(packet);
        }
        // or not, if it's a unicast packet we justhappened to receive.
        else{
            receivedDataPackets++;
        }
    }

    // *msg is deleted by parent handleMessage() function
}

void LoRaNodeApp::manageReceivedRoutingPacket(cMessage *msg) {


    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    // Check it actually is a routing message
    if (packet->getMsgType() == ROUTING) {

        receivedRoutingPackets++;

        switch (packetForwarding) {

            case NO_FORWARDING:
                bubble("Discarding routing packet as forwarding is disabled");
                break;

            case DUMMY_BROADCAST_SINGLE_SF:
                bubble("Discarding routing packet as forwarding is dummy broadcast");
                break;

            case SMART_BROADCAST_SINGLE_SF:
                bubble("Processing routing packet");
                if ( !isSMRTNeighbour(packet->getSource())) {
                    singleMetricRoute newRoute;
                    newRoute.id = packet->getSource();
                    newRoute.via = packet->getSource();
                    newRoute.metric = 1;

                    singleMetricRoutingTable.push_back(newRoute);
                }
                break;

            case HOP_COUNT_SINGLE_SF:
                bubble("Processing routing packet");

                EV << endl;
                EV << "Processing routing packet in node " << nodeId << endl;
                EV << "Routing table size: " << end(singleMetricRoutingTable) - begin(singleMetricRoutingTable) << endl;

                if ( !isSMRTNeighbour(packet->getSource())) {

                     EV << "Adding neighbour " << packet->getSource() << endl;
                     singleMetricRoute newNeighbour;
                     newNeighbour.id = packet->getSource();
                     newNeighbour.via = packet->getSource();
                     newNeighbour.metric = 1;

                     singleMetricRoutingTable.push_back(newNeighbour);
                }

                for (int i = 0; i < packet->getRoutingTableArraySize(); i++) {
                    LoRaRoute thisRoute = packet->getRoutingTable(i);

                    if (!isSMRTNeighbour(thisRoute.getId()) && thisRoute.getId() != nodeId) {
                        EV << "Adding route to node " << thisRoute.getId() << endl;
                        singleMetricRoute newRoute;
                        newRoute.id = thisRoute.getId();
                        newRoute.via = packet->getSource();
                        newRoute.metric = thisRoute.getMetric()+1;

                        singleMetricRoutingTable.push_back(newRoute);
                    }
                }

                EV << "Routing table size: " << end(singleMetricRoutingTable) - begin(singleMetricRoutingTable) << endl;
                break;

            case RSSI_SINGLE_SF:
                bubble("Processing routing packet");

                EV << endl;
                EV << "Processing routing packet in node " << nodeId << endl;
                EV << "Routing table size: " << end(singleMetricRoutingTable) - begin(singleMetricRoutingTable) << endl;

                if ( !isSMRTNeighbour(packet->getSource())) {

                     EV << "Adding neighbour " << packet->getSource() << endl;
                     singleMetricRoute newNeighbour;
                     newNeighbour.id = packet->getSource();
                     newNeighbour.via = packet->getSource();
                     newNeighbour.metric = 1;

                     singleMetricRoutingTable.push_back(newNeighbour);
                }

                for (int i = 0; i < packet->getRoutingTableArraySize(); i++) {
                    LoRaRoute thisRoute = packet->getRoutingTable(i);

                    if (!isSMRTNeighbour(thisRoute.getId()) && thisRoute.getId() != nodeId) {
                        EV << "Adding route to node " << thisRoute.getId() << endl;
                        singleMetricRoute newRoute;
                        newRoute.id = thisRoute.getId();
                        newRoute.via = packet->getSource();
                        newRoute.metric = thisRoute.getMetric()+std::abs(packet->getOptions().getRSSI());
                    }
                }

                EV << "Routing table size: " << end(singleMetricRoutingTable) - begin(singleMetricRoutingTable) << endl;
                break;

            case TIME_ON_AIR_CAD:
                EV << endl;
                EV << "Processing routing packet in node " << nodeId << endl;
                EV << "Routing table size: " << end(dualMetricRoutingTable) - begin(dualMetricRoutingTable) << endl;

                if ( !isRouteInDualMetricRoutingTableNeighbour(packet->getSource(), packet->getSource(), packet->getOptions().getLoRaSF())) {
                    EV << "Adding neighbour " << packet->getSource() << " with SF " << packet->getOptions().getLoRaSF() << endl;

                    dualMetricRoute newNeighbour;
                    newNeighbour.id = packet->getSource();
                    newNeighbour.via = packet->getSource();
                    newNeighbour.sf = packet->getOptions().getLoRaSF();
                    newNeighbour.metric = pow(2, packet->getOptions().getLoRaSF() - 7);
                    dualMetricRoutingTable.push_back(newNeighbour);
                }



                for (int i = 0; i < packet->getRoutingTableArraySize(); i++) {
                    LoRaRoute thisRoute = packet->getRoutingTable(i);

                    if (  thisRoute.getId() != nodeId && !isRouteInDualMetricRoutingTableNeighbour(packet->getSource(), packet->getVia(), packet->getOptions().getLoRaSF())) {
                        EV << "Adding route to node " << thisRoute.getId() << endl;
                        dualMetricRoute newRoute;
                        newRoute.id = thisRoute.getId();
                        newRoute.via = packet->getSource();
                        newRoute.sf = packet->getOptions().getLoRaSF();
                        newRoute.metric = thisRoute.getMetric() + pow(2, packet->getOptions().getLoRaSF());
                    }
                }

                EV << "Routing table size: " << end(dualMetricRoutingTable) - begin(dualMetricRoutingTable) << endl;
                break;

            default:
                break;
        }
    }
}

void LoRaNodeApp::manageReceivedPacketToForward(cMessage *msg) {
    receivedPacketsToForward++;

    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    switch (packet->getMsgType()) {
    // DATA packet
    case DATA:
        manageReceivedDataPacketToForward(packet);
        break;
    // ACK packet
    case ACK:
        manageReceivedAckPacketToForward(packet);
        break;
    default:
        break;
    }
}

void LoRaNodeApp::manageReceivedAckPacketToForward(cMessage *msg) {
    receivedAckPackets++;
    receivedAckPacketsToForward++;
}

void LoRaNodeApp::manageReceivedDataPacketToForward(cMessage *msg) {
    receivedDataPackets++;
    receivedDataPacketsToForward++;

    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    // Check for too old packets with TTL <= 1
    if (packet->getTtl() <= 1) {
        bubble("This packet has reached TTL expiration!");
        receivedDataPacketsToForwardExpired++;
    }

    // Packet has not reached its maximum TTL
    else {
        receivedDataPacketsToForwardCorrect++;

        switch (packetForwarding) {
        // Forwarding disabled
        case 0:
            bubble("Discarding packet as forwarding is disabled");
            break;
            // Forwarding enabled
        default:
            // Check if the packet has already been forwarded
            if (isPacketForwarded(packet)) {
                bubble("This packet has already been forwarded!");
            }
            // Check if the packet is buffered to be forwarded
            else if (isPacketToBeForwarded(packet)) {
                bubble("This packet is already scheduled to be forwarded!");
            } else {
                bubble("Saving packet to forward it later!");
                receivedDataPacketsToForwardUnique++;

                // Duplicate the packet
                // ToDo: maybe "LoRaAppPacket *dataPacket = packet->dup();" instead?

                LoRaAppPacket *dataPacket = new LoRaAppPacket("DataFrame");
                dataPacket->setMsgType(packet->getMsgType());

                dataPacket->setDataInt(packet->getDataInt());

                dataPacket->setSource(packet->getSource());
                dataPacket->setDestination(packet->getDestination());

                dataPacket->getOptions().setAppACKReq(
                        packet->getOptions().getAppACKReq());
                dataPacket->getOptions().setADRACKReq(
                        packet->getOptions().getADRACKReq());

                dataPacket->setTtl(packet->getTtl() - 1);

                LoRaPacketsToForward.push_back(*dataPacket);
            }
        }
    }
}

void LoRaNodeApp::manageReceivedPacketForMe(cMessage *msg) {
    receivedPacketsForMe++;

    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    switch (packet->getMsgType()) {
    // DATA packet
    case DATA:
        manageReceivedDataPacketForMe(packet);
        break;
        // ACK packet
    case ACK:
        manageReceivedAckPacketForMe(packet);
        break;
        // Other type
    default:
        break;
    }
}

void LoRaNodeApp::manageReceivedDataPacketForMe(cMessage *msg) {
    receivedDataPackets++;
    receivedDataPacketsForMe++;

    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    if (isDataPacketForMeUnique(packet)) {
        DataPacketsForMe.push_back(*packet);
        receivedDataPacketsForMeUnique++;
    }
}

void LoRaNodeApp::manageReceivedAckPacketForMe(cMessage *msg) {
    receivedAckPackets++;
    receivedAckPacketsForMe++;

    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    // Optional: do something with the packet
}

bool LoRaNodeApp::handleOperationStage(LifecycleOperation *operation, int stage,
        IDoneCallback *doneCallback) {
    Enter_Method_Silent();

    throw cRuntimeError("Unsupported lifecycle operation '%s'",
            operation->getClassName());
    return true;
}

void LoRaNodeApp::sendPacket() {
    LoRaAppPacket *dataPacket = new LoRaAppPacket("DataFrame");
    bool transmit = false;

    // Send local data packets with a configurable ownDataPriority priority over packets to forward, if there is any
    if ((LoRaPacketsToSend.size() > 0 && uniform(0, 1) < ownDataPriority)
            || (LoRaPacketsToSend.size() > 0 && LoRaPacketsToForward.size() == 0)) {

        // Get the first data packet in the buffer to send it
        LoRaAppPacket *firstDataPacket = &LoRaPacketsToSend.front();
        dataPacket = firstDataPacket->dup();
        LoRaPacketsToSend.erase(LoRaPacketsToSend.begin());

        transmit = true;
        sentPackets++;
        sentDataPackets++;
    }

    // Forward other nodes' packets, if any
    else if (LoRaPacketsToForward.size() > 0) {

        switch (packetForwarding) {

        case 1:
            while (LoRaPacketsToForward.size() > 0) {

                // Get the first packet in the forwarding buffer to send it
                LoRaAppPacket *firstDataPacket = &LoRaPacketsToForward.at(0);
                dataPacket = firstDataPacket->dup();
                LoRaPacketsToForward.erase(LoRaPacketsToForward.begin());

                // Check that the packet has not been forwarded in the mean time, this should never occur
                if (isPacketForwarded(dataPacket)) {
                    delete dataPacket;
                    break;
                } else {
                    transmit = true;
                    sentPackets++;
                    forwardedPackets++;
                    bubble("Forwarding packet!");

                    // Keep a copy of the forwarded packet to avoid sending it again if received later on
                    LoRaPacketsForwarded.push_back(*dataPacket->dup());
                }
            }
            break;

        default:
            // Get the first packet in the forwarding buffer to delete it
            LoRaPacketsToForward.erase(LoRaPacketsToForward.begin());
            transmit = false;
            break;
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
        txSfVector.record(loRaSF);
        txTpVector.record(loRaTP);
        send(dataPacket, "appOut");
        emit(LoRa_AppPacketSent, loRaSF);
    }
}

void LoRaNodeApp::sendRoutingPacket() {

    bool transmit = false;
    LoRaAppPacket *routingPacket = new LoRaAppPacket("DataFrame");

    std::vector<LoRaRoute> theseLoRaRoutes;
    int singleMetricRoutesCount = end(singleMetricRoutingTable) - begin(singleMetricRoutingTable);
    int dualMetricRoutesCount = end(dualMetricRoutingTable) - begin(dualMetricRoutingTable);

    switch (packetForwarding) {

        case NO_FORWARDING:
            break;

        case DUMMY_BROADCAST_SINGLE_SF:
            break;

        case SMART_BROADCAST_SINGLE_SF:
            break;

        case HOP_COUNT_SINGLE_SF:
        case RSSI_SINGLE_SF:
           transmit = true;

           routingPacket->setRoutingTableArraySize(singleMetricRoutesCount);

           for (int i = 0; i < singleMetricRoutesCount; i++) {
               LoRaRoute thisLoRaRoute;
               thisLoRaRoute.setId(singleMetricRoutingTable[i].id);
               thisLoRaRoute.setMetric(singleMetricRoutingTable[i].metric);
               routingPacket->setRoutingTable(i, thisLoRaRoute);
           }

           break;

        case TIME_ON_AIR_CAD:
            transmit = true;

            std::vector<LoRaRoute> allLoRaRoutes;
//            std::vector<LoRaRoute> bestLoRaRoutes;

            routingPacket->setRoutingTableArraySize(dualMetricRoutesCount);

           for (int i = 0; i < dualMetricRoutesCount; i++) {
               LoRaRoute thisLoRaRoute;
               thisLoRaRoute.setId(dualMetricRoutingTable[i].id);
               thisLoRaRoute.setMetric(dualMetricRoutingTable[i].metric);
               allLoRaRoutes.push_back(thisLoRaRoute);
               routingPacket->setRoutingTable(i, thisLoRaRoute);
           }

//           for (int j = 0; j < routesCount; j++) {
//               LoRaRoute thisLoRaRoute;
//
//
//               thisLoRaRoute.setId(dualMetricRoutingTable[i].id);
//               thisLoRaRoute.setMetric(dualMetricRoutingTable[i].metric);
//               allLoRaRoutes.push_back(thisLoRaRoute);
//           }

          //routingPacket->setRoutingTableArraySize(routesCount);
       //   routingPacket->setRoutingTable(i, allLoRaRoutes);

            break;
    }


    if (transmit) {
        sentPackets++;
        sentRoutingPackets++;
        //add LoRa control info
        LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;
        cInfo->setLoRaTP(loRaTP);
        cInfo->setLoRaCF(loRaCF);
        cInfo->setLoRaSF(loRaSF);
        cInfo->setLoRaBW(loRaBW);
        cInfo->setLoRaCR(loRaCR);

        routingPacket->setControlInfo(cInfo);

        routingPacket->setMsgType(ROUTING);
        routingPacket->setDataInt(nodeId);
        routingPacket->setSource(nodeId);
        routingPacket->setVia(nodeId);
        routingPacket->setDestination(BROADCAST_ADDRESS);
        routingPacket->getOptions().setAppACKReq(false);

        txSfVector.record(loRaSF);
        txTpVector.record(loRaTP);
        send(routingPacket, "appOut");
        bubble("Sending routing packet");
        emit(LoRa_AppPacketSent, loRaSF);
    }
    else {
        delete routingPacket;
    }
}

void LoRaNodeApp::generateDataPackets() {

    std::vector<int> destinations = { };

    while (destinations.size() < numberOfDestinationsPerNode
            && numberOfNodes - 1 - destinations.size() > 0) {

        int destination = intuniform(0, numberOfNodes - 1);

        if (destination != nodeId) {
            bool newDestination = true;

            for (int i = 0; i < destinations.size(); i++) {
                if (destination == destinations[i]) {
                    newDestination = false;
                    break;
                }
            }

            if (newDestination) {
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
            dataPacket->setDestination(destinations[j]);
            dataPacket->getOptions().setAppACKReq(requestACKfromApp);

            switch (packetForwarding) {
//            case 0:
//                dataPacket->setTtl(1);
//                break;
            default:
                dataPacket->setTtl(maxHops);
                break;
            }

            LoRaPacketsToSend.push_back(*dataPacket);
            delete dataPacket;
        }
    }
}

void LoRaNodeApp::increaseSFIfPossible() {
    if (loRaSF < 12) {
        // char text[32];
        // sprintf(text, "Increasing SF from %d to %d", loRaSF, loRaSF+1);
        // bubble(text);
        loRaSF++;
    }
}

bool LoRaNodeApp::isNeighbour(int neighbourId) {
    for (std::vector<int>::iterator nbptr = neighbourNodes.begin();
            nbptr < neighbourNodes.end(); nbptr++) {
        if (neighbourId == *nbptr) {
            return true;
        }
    }
    return false;
}

bool LoRaNodeApp::isSMRTNeighbour(int neighbourId) {
    for (std::vector<singleMetricRoute>::iterator nbptr = singleMetricRoutingTable.begin();
            nbptr < singleMetricRoutingTable.end(); nbptr++) {

        if (nbptr->id == neighbourId) {
            return true;
        }
    }
    return false;
}

bool LoRaNodeApp::isRouteInDualMetricRoutingTableNeighbour(int id, int via, int sf) {
    for (std::vector<dualMetricRoute>::iterator nbptr = dualMetricRoutingTable.begin();
            nbptr < dualMetricRoutingTable.end(); nbptr++) {

        if (nbptr->id == id && nbptr->sf == sf && nbptr->via == via) {
            return true;
        }
    }
    return false;
}

bool LoRaNodeApp::isKnownNode(int knownNodeId) {
    for (std::vector<int>::iterator nbptr = knownNodes.begin();
            nbptr < knownNodes.end(); nbptr++) {
        if (knownNodeId == *nbptr) {
            return true;
        }
    }
    return false;
}

bool LoRaNodeApp::isACKed(int nodeId) {
    for (std::vector<int>::iterator nbptr = ACKedNodes.begin();
            nbptr < ACKedNodes.end(); nbptr++) {
        if (nodeId == *nbptr) {
            return true;
        }
    }
    return false;
}

bool LoRaNodeApp::isPacketForwarded(cMessage *msg) {
    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    for (std::vector<LoRaAppPacket>::iterator lbptr =
            LoRaPacketsForwarded.begin(); lbptr < LoRaPacketsForwarded.end();
            lbptr++) {
        if (packet->getMsgType() == lbptr->getMsgType()
                && packet->getDataInt() == lbptr->getDataInt()
                && packet->getSource() == lbptr->getSource()
                && packet->getDestination() == lbptr->getDestination()) {
            return true;
        }
    }
    return false;
}

bool LoRaNodeApp::isPacketToBeForwarded(cMessage *msg) {
    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    for (std::vector<LoRaAppPacket>::iterator lbptr =
            LoRaPacketsToForward.begin(); lbptr < LoRaPacketsToForward.end();
            lbptr++) {
        if (packet->getMsgType() == lbptr->getMsgType()
                && packet->getDataInt() == lbptr->getDataInt()
                && packet->getSource() == lbptr->getSource()
                && packet->getDestination() == lbptr->getDestination()) {
            return true;
        }
    }
    return false;
}

bool LoRaNodeApp::isDataPacketForMeUnique(cMessage *msg) {
    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    for (std::vector<LoRaAppPacket>::iterator lbptr =
            DataPacketsForMe.begin(); lbptr < DataPacketsForMe.end();
            lbptr++) {
        if (packet->getMsgType() == lbptr->getMsgType()
                && packet->getDataInt() == lbptr->getDataInt()
                && packet->getSource() == lbptr->getSource()
                && packet->getDestination() == lbptr->getDestination()) {
            return false;
        }
    }
    return true;
}

} //end namespace inet

