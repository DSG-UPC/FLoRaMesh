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
#define RSSI_SUM_SINGLE_SF          4
#define RSSI_PROD_SINGLE_SF         5
#define ETX_SINGLE_SF               6
#define TIME_ON_AIR_HC_CAD          7
#define TIME_ON_AIR_SF_CAD          8

Define_Module (LoRaNodeApp);

void LoRaNodeApp::initialize(int stage) {
    cSimpleModule::initialize(stage);

    //Current network settings
    numberOfNodes = par("numberOfNodes");

    if (stage == INITSTAGE_LOCAL) {
        // Get this node's ID
        nodeId = getContainingNode(this)->getIndex();
        std::pair<double, double> coordsValues = std::make_pair(-1, -1);
        cModule *host = getContainingNode(this);

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
        } else if (strcmp(host->par("deploymentType").stringValue(), "grid") == 0) {
            int minX = host->par("minX");
            int sepX = host->par("sepX");
            int minY = host->par("minY");
            int sepY = host->par("sepY");
            int cols = int(sqrt(numberOfNodes));
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

        // Initialize counters
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

        firstDataPacketTransmissionTime = 0;
        lastDataPacketTransmissionTime = 0;
        firstDataPacketReceptionTime = 0;
        lastDataPacketReceptionTime = 0;

        dataPacketsDue = false;
        routingPacketsDue = false;

        sendPacketsContinuously = par("sendPacketsContinuously");
        enforceDutyCycle = par("enforceDutyCycle");
        dutyCycle = par("dutyCycle");
        numberOfDestinationsPerNode = par("numberOfDestinationsPerNode");
        numberOfPacketsPerDestination = par("numberOfPacketsPerDestination");

        numberOfPacketsToForward = par("numberOfPacketsToForward");

        LoRa_AppPacketSent = registerSignal("LoRa_AppPacketSent");

        currDataInt = 0;

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

        //Routing variables
        routingMetric = par("routingMetric");
        routeDiscovery = par("routeDiscovery");
        routingPacketPriority = par("routingPacketPriority");
        ownDataPriority = par("ownDataPriority");
        routeTimeout = par("routeTimeout");
        storeBestRoutesOnly = par("storeBestRouteOnly");
        getRoutesFromDataPackets = par("getRoutesFromDataPackets");
        packetTTL = par("packetTTL");
        if ( packetTTL == 0 ) {
            if (strcmp(getContainingNode(this)->par("deploymentType").stringValue(), "grid") == 0) {
                packetTTL = 2*(sqrt(numberOfNodes)-1);
            }
            else {
                packetTTL = 2*(sqrt(numberOfNodes));
            }
        }

        //Packet sizes
        dataPacketSize = par("dataPacketDefaultSize");
        routingPacketMaxSize = par("routingPacketMaxSize");

        // Packet timings
        timeToNextDataPacket = par("timeToNextDataPacket");
        timeToNextRoutingPacket = par("timeToNextRoutingPacket");

        simTimeResolution = pow(10, simTimeResolution.getScaleExp());

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

            WATCH(firstDataPacketTransmissionTime);
            WATCH(lastDataPacketTransmissionTime);
            WATCH(firstDataPacketReceptionTime);
            WATCH(lastDataPacketReceptionTime);

            //WATCH_VECTOR(singleMetricRoutingTable);
            //WATCH_VECTOR(dualMetreicRoutingTable);

            WATCH_VECTOR(LoRaPacketsToSend);
            WATCH_VECTOR(LoRaPacketsToForward);
            WATCH_VECTOR(LoRaPacketsForwarded);
            WATCH_VECTOR(DataPacketsForMe);
        }

        if (numberOfDestinationsPerNode == 0 ) {
            numberOfDestinationsPerNode = numberOfNodes-1;
        }
        generateDataPackets();

        // Routing packets timer
        timeToFirstRoutingPacket = math::max(5, par("timeToFirstRoutingPacket"));
        switch (routingMetric) {
            // No routing packets are to be sent
            case NO_FORWARDING:
            case DUMMY_BROADCAST_SINGLE_SF:
            case SMART_BROADCAST_SINGLE_SF:
                break;
            // Schedule selfRoutingPackets
            default:
                routingPacketsDue = true;
                nextRoutingPacketTransmissionTime = timeToFirstRoutingPacket;
                EV << "Time to first routing packet: " << timeToFirstRoutingPacket << endl;
                break;
        }

        // Data packets timer
        timeToFirstDataPacket = math::max(5, par("timeToFirstDataPacket"));
        if (LoRaPacketsToSend.size() > 0) {
                    dataPacketsDue = true;
                    nextDataPacketTransmissionTime = timeToFirstDataPacket;
                    EV << "Time to first data packet: " << timeToFirstDataPacket << endl;
        }

        selfPacket = new cMessage("selfPacket");

        if (dataPacketsDue || routingPacketsDue) {

            if (dataPacketsDue && !routingPacketsDue) {
                scheduleAt(simTime() + timeToFirstDataPacket, selfPacket);
                EV << "Self packet triggered by due data packet" << endl;
            }
            else if (routingPacketsDue && !dataPacketsDue) {
                scheduleAt(simTime() + timeToFirstRoutingPacket, selfPacket);
                EV << "Self packet triggered by due routing packet" << endl;
            }
            else if (timeToFirstDataPacket < timeToFirstRoutingPacket) {
                scheduleAt(simTime() + timeToFirstDataPacket, selfPacket);
                EV << "Self packet triggered by due data packet before due routing packet" << endl;
            }
            else {
                scheduleAt(simTime() + timeToFirstRoutingPacket, selfPacket);
                EV << "Self packet triggered by due routing packet before due data packet" << endl;
            }
        }

        dutyCycleEnd = simTime();
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

    recordScalar("firstDataPacketTransmissionTime", firstDataPacketTransmissionTime);
    recordScalar("lastDataPacketTransmissionTime", lastDataPacketTransmissionTime);
    recordScalar("firstDataPacketReceptionTime", firstDataPacketReceptionTime);
    recordScalar("lastDataPacketReceptionTime", lastDataPacketReceptionTime);

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
        handleSelfMessage(msg);
    } else {
        handleMessageFromLowerLayer(msg);
    }
}


void LoRaNodeApp::handleSelfMessage(cMessage *msg) {

    //LoRaMac *lrmc = (LoRaMac *)getParentModule()->getSubmodule("LoRaNic")->getSubmodule("mac");

    simtime_t txDuration = 0;
    simtime_t nextScheduleTime = 0;
    bool sendRouting = false;
    bool sendData = false;

    // Check if there are routing packets to send
    if ( routingPacketsDue && simTime() >= nextRoutingPacketTransmissionTime ) {
        sendRouting = true;
    }
    // Check if there are data packets to send or forward, and if it is time to send them
    if ( (LoRaPacketsToSend.size() > 0 || LoRaPacketsToForward.size() > 0 ) && simTime() >= nextDataPacketTransmissionTime ) {
        sendData = true;
    }

    // If both types of packets are pending, decide which one will be sent
    if (sendRouting && sendData) {
        // Either send a routing packet...
        if (bernoulli(routingPacketPriority)) {
            sendData = false;
        }
        else {
            sendRouting = false;
        }
    }

    // Send routing packet
    if (sendRouting) {
        txDuration = sendRoutingPacket();
        if (enforceDutyCycle) {
            // Update duty cycle end
            dutyCycleEnd = simTime() + txDuration/dutyCycle;
            // Update next routing packet transmission time, taking the duty cycle into account
            nextRoutingPacketTransmissionTime = simTime() + math::max(timeToNextRoutingPacket.dbl(), txDuration.dbl()/dutyCycle);
        }
        else {
            // Update next routing packet transmission time
            nextRoutingPacketTransmissionTime = simTime() + math::max(timeToNextRoutingPacket.dbl(), txDuration.dbl());
        }
    }

    // Send or forward data packet
    else if (sendData) {
        txDuration = sendDataPacket();
        if (enforceDutyCycle) {
            // Update duty cycle end
            dutyCycleEnd = simTime() + txDuration/dutyCycle;
            // Update next routing packet transmission time, taking the duty cycle into account
            nextDataPacketTransmissionTime = simTime() + math::max(timeToNextDataPacket.dbl(), txDuration.dbl()/dutyCycle);
        }
        else {
            // Update next routing packet transmission time
            nextDataPacketTransmissionTime = simTime() + math::max(timeToNextDataPacket.dbl(), txDuration.dbl());
        }
        if ( LoRaPacketsToSend.size() > 0 || LoRaPacketsToForward.size() > 0 ) {
            dataPacketsDue = true;
        }
        else {
            dataPacketsDue = false;
        }
    }

    // Schedule selfPacket to whatever is to happen sooner, either routing packet transmission...
    if (routingPacketsDue) {
        nextScheduleTime = math::max(simTime().dbl(), nextRoutingPacketTransmissionTime.dbl());
    }

    // ... or data packet transmission.
    if (dataPacketsDue) {
        if (nextScheduleTime==0 || (nextScheduleTime >= simTime() && nextDataPacketTransmissionTime < simTime())) {
            nextScheduleTime = math::max(simTime().dbl(), nextDataPacketTransmissionTime.dbl());
        }
    }

    // Take the duty cycle into account
    if (enforceDutyCycle) {
        nextScheduleTime = math::max(nextScheduleTime.dbl(), dutyCycleEnd.dbl());
    }

    // Last, check the schedule time is in the future, otherwise just add a 1s delay
    if (! (nextScheduleTime > simTime()) ) {
        nextScheduleTime = simTime() + 1;
    }

    // Schedule a self message to send routing or data packets. Add a
    // simulation time delta (i.e., a simtime-resolution unit) to avoid
    // timing conflicts in the LoRaMac layer
    if (routingPacketsDue || dataPacketsDue) {
        scheduleAt(nextScheduleTime + 10*simTimeResolution, selfPacket);
    }
}

//
//void LoRaNodeApp::handleSelfDataPacket() {
//
//    LoRaMac *lrmc = (LoRaMac *)getParentModule()->getSubmodule("LoRaNic")->getSubmodule("mac");
//
//    bool schedule = false;
//    simtime_t txDuration = 0;
//
//    // Only proceed to send a data packet if the 'mac' module in 'LoRaNic' is IDLE and the warmup period is due
//    if ( (lrmc->fsm.getState() == IDLE
//// ToDo: check if all states are acceptable
////            || lrmc->fsm.getState() == WAIT_DELAY_1 || lrmc->fsm.getState() == LISTENING_1 || lrmc->fsm.getState() == WAIT_DELAY_2 || lrmc->fsm.getState() == LISTENING_2
//            ) && simTime() >= getSimulation()->getWarmupPeriod() ) {
//
//        // Generate more packets if needed
//        if (sendPacketsContinuously && LoRaPacketsToSend.size() == 0){
//            generateDataPackets();
//        }
//
//        // Check conditions for sending own data packet
//        if (LoRaPacketsToSend.size() > 0)  // || sentPackets < numberOfPacketsToSend) && (!AppACKReceived || !stopOnACK))
//        {
//            txDuration = sendPacket();
//            schedule = true;
//        }
//        // Check conditions for forwarding own data packet
//        else if (forwardedPackets < numberOfPacketsToForward
//                || numberOfPacketsToForward == 0) {
//            // Only go to the sendDataPacket() function if there is something to be forwarded
//            if (LoRaPacketsToForward.size() > 0)
//                txDuration = sendPacket();
//            // Schedule a self-message, since we have room for forwarding
//            schedule = true;
//        }
//    }
//    else {
//        schedule = true;
//    }
//
//    if (schedule) {
////        double time;
////        if (loRaSF == 7)
////            time = 7.808;
////        if (loRaSF == 8)
////            time = 13.9776;
////        if (loRaSF == 9)
////            time = 24.6784;
////        if (loRaSF == 10)
////            time = 49.3568;
////        if (loRaSF == 11)
////            time = 85.6064;
////        if (loRaSF == 12)
////            time = 171.2128;
//
//        if (enforceDutyCycle) {
//            txDuration = txDuration/dutyCycle;
//        }
//
//        selfDataPacket = new cMessage("selfDataPacket");
//        scheduleAt(simTime() + txDuration + 0.0000000001, selfDataPacket);
//    }
//}

void LoRaNodeApp::handleMessageFromLowerLayer(cMessage *msg) {
    receivedPackets++;

    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    // Check if the packet is from this node (i.e., a packet that some
    // other node is broadcasting which we have happened to receive). We
    // count it and discard it immediately.
    if (packet->getSource() == nodeId) {
        receivedDataPacketsFromMe++;
        bubble("I received a LoRa packet originally sent by me!");
        if (firstDataPacketReceptionTime == 0) {
            firstDataPacketReceptionTime = simTime();
        }
        lastDataPacketReceptionTime = simTime();
    }
    // Else, check if the packet is for this node (i.e., a packet directly
    // received from the origin or relayed by a neighbour)
    else if (packet->getDestination() == nodeId) {
        manageReceivedPacketForMe(packet);
        if (firstDataPacketReceptionTime == 0) {
            firstDataPacketReceptionTime = simTime();
        }
        lastDataPacketReceptionTime = simTime();
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
            if (firstDataPacketReceptionTime == 0) {
                firstDataPacketReceptionTime = simTime();
            }
            lastDataPacketReceptionTime = simTime();
        }
        // or not, if it's a unicast packet we justhappened to receive.
        else{
            receivedDataPackets++;
            lastDataPacketReceptionTime = simTime();
        }
    }

    delete msg;
}

void LoRaNodeApp::manageReceivedRoutingPacket(cMessage *msg) {


    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    // Check it actually is a routing message
    if (packet->getMsgType() == ROUTING) {

        receivedRoutingPackets++;

        sanitizeRoutingTable();

        switch (routingMetric) {

            case NO_FORWARDING:
                bubble("Discarding routing packet as forwarding is disabled");
                break;

            case DUMMY_BROADCAST_SINGLE_SF:
                bubble("Discarding routing packet as forwarding is dummy broadcast");
                break;

            case SMART_BROADCAST_SINGLE_SF:
                bubble("Processing routing packet");
                if ( !isRouteInSingleMetricRoutingTable(packet->getSource(), packet->getSource())) {
                    singleMetricRoute newRoute;
                    newRoute.id = packet->getSource();
                    newRoute.via = packet->getSource();
                    newRoute.metric = 1;
                    newRoute.valid = simTime() + routeTimeout;
                    singleMetricRoutingTable.push_back(newRoute);
                }
                break;

            case HOP_COUNT_SINGLE_SF:
                bubble("Processing routing packet");

//                EV << "Processing routing packet in node " << nodeId << endl;
//                EV << "Routing table size: " << end(singleMetricRoutingTable) - begin(singleMetricRoutingTable) << endl;

                if ( !isRouteInSingleMetricRoutingTable(packet->getSource(), packet->getSource())) {

                     EV << "Adding neighbour " << packet->getSource() << endl;
                     singleMetricRoute newNeighbour;
                     newNeighbour.id = packet->getSource();
                     newNeighbour.via = packet->getSource();
                     newNeighbour.metric = 1;
                     newNeighbour.valid = simTime() + routeTimeout;

                     singleMetricRoutingTable.push_back(newNeighbour);
                }

                for (int i = 0; i < packet->getRoutingTableArraySize(); i++) {
                    LoRaRoute thisRoute = packet->getRoutingTable(i);

                    if (thisRoute.getId() != nodeId) {
                        // Add new route
                        if (!isRouteInSingleMetricRoutingTable(thisRoute.getId(), packet->getSource()) && thisRoute.getId() != nodeId) {
                            EV << "Adding route to node " << thisRoute.getId() << " via " << packet->getSource() << endl;
                            singleMetricRoute newRoute;
                            newRoute.id = thisRoute.getId();
                            newRoute.via = packet->getSource();
                            newRoute.metric = thisRoute.getPriMetric()+1;
                            newRoute.valid = simTime() + routeTimeout;

                            singleMetricRoutingTable.push_back(newRoute);
                        }
                        // Or update known one
                        else {
                            int routeIndex = getRouteIndexInRouteInSingleMetricRoutingTable(thisRoute.getId(), packet->getSource());
                            if (routeIndex >= 0) {
                                singleMetricRoutingTable[routeIndex].metric = thisRoute.getPriMetric()+1;
                                singleMetricRoutingTable[routeIndex].valid = simTime() + routeTimeout;
                            }
                        }
                    }
                }

                EV << "Routing table size: " << end(singleMetricRoutingTable) - begin(singleMetricRoutingTable) << endl;
                break;

            case RSSI_SUM_SINGLE_SF:
                bubble("Processing routing packet");

//                EV << "Processing routing packet in node " << nodeId << endl;
//                EV << "Routing table size: " << end(singleMetricRoutingTable) - begin(singleMetricRoutingTable) << endl;

                if ( !isRouteInSingleMetricRoutingTable(packet->getSource(), packet->getSource())) {

                     EV << "Adding neighbour " << packet->getSource() << endl;
                     singleMetricRoute newNeighbour;
                     newNeighbour.id = packet->getSource();
                     newNeighbour.via = packet->getSource();
                     newNeighbour.metric = std::abs(packet->getOptions().getRSSI());
                     newNeighbour.valid = simTime() + routeTimeout;

                     singleMetricRoutingTable.push_back(newNeighbour);
                }

                for (int i = 0; i < packet->getRoutingTableArraySize(); i++) {
                    LoRaRoute thisRoute = packet->getRoutingTable(i);

                    if (thisRoute.getId() != nodeId) {
                        // Add new route
                        if (!isRouteInSingleMetricRoutingTable(thisRoute.getId(), packet->getSource()) && thisRoute.getId() != nodeId) {
                            EV << "Adding route to node " << thisRoute.getId() << " via " << packet->getSource() << endl;
                            singleMetricRoute newRoute;
                            newRoute.id = thisRoute.getId();
                            newRoute.via = packet->getSource();
                            newRoute.metric = thisRoute.getPriMetric()+std::abs(packet->getOptions().getRSSI());
                            newRoute.valid = simTime() + routeTimeout;

                            singleMetricRoutingTable.push_back(newRoute);
                        }
                        // Or update known one
                        else {
                            int routeIndex = getRouteIndexInRouteInSingleMetricRoutingTable(thisRoute.getId(), packet->getSource());
                            if (routeIndex >= 0) {
                                singleMetricRoutingTable[routeIndex].metric = thisRoute.getPriMetric()+std::abs(packet->getOptions().getRSSI());
                                singleMetricRoutingTable[routeIndex].valid = simTime() + routeTimeout;
                            }
                        }
                    }
                }
                break;


            case RSSI_PROD_SINGLE_SF:
                bubble("Processing routing packet");

//                EV << "Processing routing packet in node " << nodeId << endl;
//                EV << "Routing table size: " << end(singleMetricRoutingTable) - begin(singleMetricRoutingTable) << endl;

                if ( !isRouteInSingleMetricRoutingTable(packet->getSource(), packet->getSource())) {

//                     EV << "Adding neighbour " << packet->getSource() << endl;
                     singleMetricRoute newNeighbour;
                     newNeighbour.id = packet->getSource();
                     newNeighbour.via = packet->getSource();
                     newNeighbour.metric = std::abs(packet->getOptions().getRSSI());
                     newNeighbour.valid = simTime() + routeTimeout;

                     singleMetricRoutingTable.push_back(newNeighbour);
                }

                for (int i = 0; i < packet->getRoutingTableArraySize(); i++) {
                    LoRaRoute thisRoute = packet->getRoutingTable(i);

                    if (thisRoute.getId() != nodeId) {
                        // Add new route
                        if (!isRouteInSingleMetricRoutingTable(thisRoute.getId(), packet->getSource())) {
//                            EV << "Adding route to node " << thisRoute.getId() << " via " << packet->getSource() << endl;
                            singleMetricRoute newRoute;
                            newRoute.id = thisRoute.getId();
                            newRoute.via = packet->getSource();
                            newRoute.metric = thisRoute.getPriMetric()*std::abs(packet->getOptions().getRSSI());
                            newRoute.valid = simTime() + routeTimeout;

                            singleMetricRoutingTable.push_back(newRoute);
                        }
                        // Or update known one
                        else {
                            int routeIndex = getRouteIndexInRouteInSingleMetricRoutingTable(thisRoute.getId(), packet->getSource());
                            if (routeIndex >= 0) {
                                singleMetricRoutingTable[routeIndex].metric = thisRoute.getPriMetric()*std::abs(packet->getOptions().getRSSI());
                                singleMetricRoutingTable[routeIndex].valid = simTime() + routeTimeout;
                            }
                        }
                    }
                }
                break;

            case ETX_SINGLE_SF:
                bubble("Processing routing packet");

//                EV << "Processing routing packet in node " << nodeId << endl;
//                EV << "Routing table size: " << end(dualMetricRoutingTable) - begin(dualMetricRoutingTable) << endl;

                if ( !isRouteInDualMetricRoutingTable(packet->getSource(), packet->getSource(), packet->getOptions().getLoRaSF())) {
//                    EV << "Adding neighbour " << packet->getSource() << " with SF " << packet->getOptions().getLoRaSF() << endl;

                    dualMetricRoute newNeighbour;
                    newNeighbour.id = packet->getSource();
                    newNeighbour.via = packet->getSource();
                    newNeighbour.sf = packet->getOptions().getLoRaSF();
                    newNeighbour.priMetric = 1;
                    newNeighbour.secMetric = packet->getDataInt();
                    newNeighbour.valid = simTime() + routeTimeout;
                    dualMetricRoutingTable.push_back(newNeighbour);
                }

                for (int i = 0; i < packet->getRoutingTableArraySize(); i++) {
                    LoRaRoute thisRoute = packet->getRoutingTable(i);

                    if (thisRoute.getId() != nodeId ) {
                        // Add new route
                        if ( !isRouteInDualMetricRoutingTable(packet->getSource(), packet->getVia(), packet->getOptions().getLoRaSF())) {
//                            EV << "Adding route to node " << thisRoute.getId() << " via " << packet->getSource() << " with SF " << packet->getOptions().getLoRaSF() << endl;
                            dualMetricRoute newRoute;
                            newRoute.id = thisRoute.getId();
                            newRoute.via = packet->getSource();
                            newRoute.sf = packet->getOptions().getLoRaSF();
                            newRoute.priMetric = thisRoute.getPriMetric() + 1;
                            newRoute.secMetric = packet->getDataInt();
                            newRoute.valid = simTime() + routeTimeout;
                        }
                    }
                    else {
                        int routeIndex = getRouteIndexInDualMetricRoutingTable(thisRoute.getId(), packet->getSource(), packet->getOptions().getLoRaSF());
                        if (routeIndex >= 0) {
                            dualMetricRoutingTable[routeIndex].priMetric = thisRoute.getPriMetric() * (packet->getDataInt() - thisRoute.getSecMetric())/2;
                            dualMetricRoutingTable[routeIndex].secMetric = packet->getDataInt();
                            dualMetricRoutingTable[routeIndex].valid = simTime() + routeTimeout;
                        }
                    }
                }

//                EV << "Routing table size: " << end(dualMetricRoutingTable) - begin(dualMetricRoutingTable) << endl;
                break;

            case TIME_ON_AIR_HC_CAD:
                bubble("Processing routing packet");

//                EV << "Processing routing packet in node " << nodeId << endl;
//                EV << "Routing table size: " << end(dualMetricRoutingTable) - begin(dualMetricRoutingTable) << endl;

                if ( !isRouteInDualMetricRoutingTable(packet->getSource(), packet->getSource(), packet->getOptions().getLoRaSF())) {
//                    EV << "Adding neighbour " << packet->getSource() << " with SF " << packet->getOptions().getLoRaSF() << endl;

                    dualMetricRoute newNeighbour;
                    newNeighbour.id = packet->getSource();
                    newNeighbour.via = packet->getSource();
                    newNeighbour.sf = packet->getOptions().getLoRaSF();
                    newNeighbour.priMetric = pow(2, packet->getOptions().getLoRaSF() - 7);
                    newNeighbour.secMetric = 1;
                    newNeighbour.valid = simTime() + routeTimeout;
                    dualMetricRoutingTable.push_back(newNeighbour);
                }

                for (int i = 0; i < packet->getRoutingTableArraySize(); i++) {
                    LoRaRoute thisRoute = packet->getRoutingTable(i);

                    if (thisRoute.getId() != nodeId ) {
                        // Add new route
                        if ( !isRouteInDualMetricRoutingTable(packet->getSource(), packet->getVia(), packet->getOptions().getLoRaSF())) {
//                            EV << "Adding route to node " << thisRoute.getId() << " via " << packet->getSource() << " with SF " << packet->getOptions().getLoRaSF() << endl;
                            dualMetricRoute newRoute;
                            newRoute.id = thisRoute.getId();
                            newRoute.via = packet->getSource();
                            newRoute.sf = packet->getOptions().getLoRaSF();
                            newRoute.priMetric = thisRoute.getPriMetric() + pow(2, packet->getOptions().getLoRaSF());
                            newRoute.secMetric = thisRoute.getSecMetric() + 1;
                            newRoute.valid = simTime() + routeTimeout;
                        }
                    }
                    // Or update known one
                    else {
                        int routeIndex = getRouteIndexInDualMetricRoutingTable(thisRoute.getId(), packet->getSource(), packet->getOptions().getLoRaSF());
                        if (routeIndex >= 0) {
                            dualMetricRoutingTable[routeIndex].priMetric = thisRoute.getPriMetric() + pow(2, packet->getOptions().getLoRaSF());
                            dualMetricRoutingTable[routeIndex].secMetric = thisRoute.getSecMetric() + 1;
                            dualMetricRoutingTable[routeIndex].valid = simTime() + routeTimeout;
                        }
                    }
                }

//                EV << "Routing table size: " << end(dualMetricRoutingTable) - begin(dualMetricRoutingTable) << endl;
                break;

            case TIME_ON_AIR_SF_CAD:
                bubble("Processing routing packet");

//                EV << "Processing routing packet in node " << nodeId << endl;
//                EV << "Routing table size: " << end(dualMetricRoutingTable) - begin(dualMetricRoutingTable) << endl;

                if ( !isRouteInDualMetricRoutingTable(packet->getSource(), packet->getSource(), packet->getOptions().getLoRaSF())) {
//                    EV << "Adding neighbour " << packet->getSource() << " with SF " << packet->getOptions().getLoRaSF() << endl;

                    dualMetricRoute newNeighbour;
                    newNeighbour.id = packet->getSource();
                    newNeighbour.via = packet->getSource();
                    newNeighbour.sf = packet->getOptions().getLoRaSF();
                    newNeighbour.priMetric = pow(2, packet->getOptions().getLoRaSF() - 7);
                    newNeighbour.secMetric = packet->getOptions().getLoRaSF() - 7;
                    newNeighbour.valid = simTime() + routeTimeout;
                    dualMetricRoutingTable.push_back(newNeighbour);
                }

                for (int i = 0; i < packet->getRoutingTableArraySize(); i++) {
                    LoRaRoute thisRoute = packet->getRoutingTable(i);

                    if (thisRoute.getId() != nodeId ) {
                        // Add new route
                        if ( !isRouteInDualMetricRoutingTable(packet->getSource(), packet->getVia(), packet->getOptions().getLoRaSF())) {
//                            EV << "Adding route to node " << thisRoute.getId() << " via " << packet->getSource() << " with SF " << packet->getOptions().getLoRaSF() << endl;
                            dualMetricRoute newRoute;
                            newRoute.id = thisRoute.getId();
                            newRoute.via = packet->getSource();
                            newRoute.sf = packet->getOptions().getLoRaSF();
                            newRoute.priMetric = thisRoute.getPriMetric() + pow(2, packet->getOptions().getLoRaSF());
                            newRoute.secMetric = thisRoute.getSecMetric() + packet->getOptions().getLoRaSF() - 7;
                            newRoute.valid = simTime() + routeTimeout;
                        }
                        // Or update known one
                        else {
                            int routeIndex = getRouteIndexInDualMetricRoutingTable(thisRoute.getId(), packet->getSource(), packet->getOptions().getLoRaSF());
                            if (routeIndex >= 0) {
                                dualMetricRoutingTable[routeIndex].priMetric = thisRoute.getPriMetric() + pow(2, packet->getOptions().getLoRaSF());
                                dualMetricRoutingTable[routeIndex].secMetric = thisRoute.getSecMetric() + packet->getOptions().getLoRaSF() - 7;
                                dualMetricRoutingTable[routeIndex].valid = simTime() + routeTimeout;
                            }
                        }
                    }
                }

//                EV << "Routing table size: " << end(dualMetricRoutingTable) - begin(dualMetricRoutingTable) << endl;
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
    LoRaAppPacket *dataPacket = packet->dup();

    // Check for too old packets with TTL <= 1
    if (packet->getTtl() <= 1) {
        bubble("This packet has reached TTL expiration!");
        receivedDataPacketsToForwardExpired++;
    }

    // Packet has not reached its maximum TTL
    else {
        receivedDataPacketsToForwardCorrect++;

        switch (routingMetric) {
            case NO_FORWARDING:
                bubble("Discarding packet as forwarding is disabled");
                break;

            case DUMMY_BROADCAST_SINGLE_SF:
                receivedDataPacketsToForwardUnique++;
                dataPacket->setTtl(packet->getTtl() - 1);
                LoRaPacketsToForward.push_back(*dataPacket);
                break;

            case HOP_COUNT_SINGLE_SF:
            case RSSI_SUM_SINGLE_SF:
            case RSSI_PROD_SINGLE_SF:
            case ETX_SINGLE_SF:
            case TIME_ON_AIR_HC_CAD:
            case TIME_ON_AIR_SF_CAD:
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

                        dataPacket->setTtl(packet->getTtl() - 1);
                        LoRaPacketsToForward.push_back(*dataPacket);
                    }
        }

    }

    delete dataPacket;
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

simtime_t LoRaNodeApp::sendDataPacket() {
    LoRaAppPacket *dataPacket = new LoRaAppPacket("DataFrame");

    bool transmit = false;
    simtime_t txDuration = 0;

    // Send local data packets with a configurable ownDataPriority priority over packets to forward, if there is any
    if (
            (LoRaPacketsToSend.size() > 0 && bernoulli(ownDataPriority))
            || (LoRaPacketsToSend.size() > 0 && LoRaPacketsToForward.size() == 0)) {

        bubble("Sending a local data packet!");

        // Name packets to ease tracking
        std::string fullName = dataPacket->getName();;
        const char* addName = "Orig";
        fullName += addName;
        fullName += std::to_string(nodeId);

        dataPacket->setName(fullName.c_str());

        // Get the data from the first packet in the data buffer to send it
        dataPacket->setMsgType(LoRaPacketsToSend.front().getMsgType());
        dataPacket->setDataInt(LoRaPacketsToSend.front().getDataInt());
        dataPacket->setSource(LoRaPacketsToSend.front().getSource());
        dataPacket->setVia(LoRaPacketsToSend.front().getSource());
        dataPacket->setDestination(LoRaPacketsToSend.front().getDestination());
        dataPacket->setTtl(LoRaPacketsToSend.front().getTtl());
        dataPacket->getOptions().setAppACKReq(LoRaPacketsToSend.front().getOptions().getAppACKReq());
        dataPacket->setByteLength(LoRaPacketsToSend.front().getByteLength());

        LoRaPacketsToSend.erase(LoRaPacketsToSend.begin());

        transmit = true;

        sentDataPackets++;
        if (firstDataPacketTransmissionTime == 0)
            firstDataPacketTransmissionTime = simTime();
        lastDataPacketTransmissionTime = simTime();
    }

    // Forward other nodes' packets, if any
    else if (LoRaPacketsToForward.size() > 0) {

        bubble("Forwarding a packet!");

        std::string fullName = dataPacket->getName();;
        const char* addName = "Fwd";
        fullName += addName;
        dataPacket->setName(fullName.c_str());
        fullName += std::to_string(nodeId);

        switch (routingMetric) {
            case NO_FORWARDING:
                // This should never happen
                bubble("Forwarding disabled!");
                break;

            case DUMMY_BROADCAST_SINGLE_SF:
                // Get the first packet in the forwarding buffer

                addName = "Dummy";
                fullName += addName;
                dataPacket->setName(fullName.c_str());


                // Get the data from the first packet in the forwarding buffer to send it
                dataPacket->setMsgType(LoRaPacketsToForward.front().getMsgType());
                dataPacket->setDataInt(LoRaPacketsToForward.front().getDataInt());
                dataPacket->setSource(LoRaPacketsToForward.front().getSource());
                dataPacket->setVia(LoRaPacketsToForward.front().getSource());
                dataPacket->setDestination(LoRaPacketsToForward.front().getDestination());
                dataPacket->setTtl(LoRaPacketsToForward.front().getTtl());
                dataPacket->getOptions().setAppACKReq(LoRaPacketsToForward.front().getOptions().getAppACKReq());
                dataPacket->setByteLength(LoRaPacketsToForward.front().getByteLength());

                LoRaPacketsToForward.erase(LoRaPacketsToForward.begin());

                transmit = true;

                forwardedPackets++;
                bubble("Forwarding packet!");

                break;

            case SMART_BROADCAST_SINGLE_SF:
            case HOP_COUNT_SINGLE_SF:
            case RSSI_SUM_SINGLE_SF:
            case RSSI_PROD_SINGLE_SF:
            case ETX_SINGLE_SF:
            case TIME_ON_AIR_HC_CAD:
            case TIME_ON_AIR_SF_CAD:
            default:
                while (LoRaPacketsToForward.size() > 0) {

                    addName = "Other";
                    fullName += addName;
                    dataPacket->setName(fullName.c_str());

                    // Get the data from the first packet in the forwarding buffer to send it
                    dataPacket->setMsgType(LoRaPacketsToForward.front().getMsgType());
                    dataPacket->setDataInt(LoRaPacketsToForward.front().getDataInt());
                    dataPacket->setSource(LoRaPacketsToForward.front().getSource());
                    dataPacket->setVia(LoRaPacketsToForward.front().getSource());
                    dataPacket->setDestination(LoRaPacketsToForward.front().getDestination());
                    dataPacket->setTtl(LoRaPacketsToForward.front().getTtl());
                    dataPacket->getOptions().setAppACKReq(LoRaPacketsToForward.front().getOptions().getAppACKReq());
                    dataPacket->setByteLength(LoRaPacketsToForward.front().getByteLength());

                    LoRaPacketsToForward.erase(LoRaPacketsToForward.begin());

                    // Check that the packet has not been forwarded in the mean time, this should never occur
                    if (!isPacketForwarded(dataPacket)) {
                        bubble("Forwarding packet!");
                        forwardedPackets++;
                        transmit = true;

                        // Keep a copy of the forwarded packet to avoid sending it again if received later on
                        LoRaPacketsForwarded.push_back(*dataPacket);
                        break;
                    }
                }
                break;

        }
    }

    if (transmit) {
        sentPackets++;

        std::string fullName = dataPacket->getName();;
        const char* ownName = "Tx";
        fullName += ownName;
        dataPacket->setName(fullName.c_str());

        //add LoRa control info
        LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;
        cInfo->setLoRaTP(loRaTP);
        cInfo->setLoRaCF(loRaCF);
        cInfo->setLoRaSF(loRaSF);
        cInfo->setLoRaBW(loRaBW);
        cInfo->setLoRaCR(loRaCR);

        sanitizeRoutingTable();

        int routeIndex = getBestRouteIndexTo(dataPacket->getDestination());

        switch (routingMetric) {
            case DUMMY_BROADCAST_SINGLE_SF:
                dataPacket->setVia(BROADCAST_ADDRESS);
                break;
            case SMART_BROADCAST_SINGLE_SF:
            case HOP_COUNT_SINGLE_SF:
            case RSSI_SUM_SINGLE_SF:
            case RSSI_PROD_SINGLE_SF:
            case ETX_SINGLE_SF:
                if ( routeIndex >= 0 ) {
                    dataPacket->setVia(singleMetricRoutingTable[routeIndex].via);
                }
                else{
                    dataPacket->setVia(BROADCAST_ADDRESS);
                }
                break;
            case TIME_ON_AIR_HC_CAD:
            case TIME_ON_AIR_SF_CAD:
                if ( routeIndex >= 0 ) {
                    dataPacket->setVia(dualMetricRoutingTable[routeIndex].via);
                    cInfo->setLoRaSF(dualMetricRoutingTable[routeIndex].sf);
                }
                else{
                    dataPacket->setVia(BROADCAST_ADDRESS);
                }
                break;
        }

        dataPacket->setControlInfo(cInfo);

        txDuration = calculateTransmissionDuration(dataPacket);

        send(dataPacket, "appOut");
        txSfVector.record(loRaSF);
        txTpVector.record(loRaTP);
        emit(LoRa_AppPacketSent, loRaSF);
    }
    else {
        delete dataPacket;
    }

    // Generate more packets if needed
    if (sendPacketsContinuously && LoRaPacketsToSend.size() == 0) {
        generateDataPackets();
    }

    return txDuration;
}

simtime_t LoRaNodeApp::sendRoutingPacket() {

    bool transmit = false;
    simtime_t txDuration = 0;

    LoRaAppPacket *routingPacket = new LoRaAppPacket("RoutingPacket");

    LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;

    cInfo->setLoRaTP(loRaTP);
    cInfo->setLoRaCF(loRaCF);
    cInfo->setLoRaSF(loRaSF);
    cInfo->setLoRaBW(loRaBW);
    cInfo->setLoRaCR(loRaCR);

    sanitizeRoutingTable();

    std::vector<LoRaRoute> theseLoRaRoutes;
    int singleMetricRoutesCount = end(singleMetricRoutingTable) - begin(singleMetricRoutingTable);
    int dualMetricRoutesCount = end(dualMetricRoutingTable) - begin(dualMetricRoutingTable);

    switch (routingMetric) {

        case NO_FORWARDING:
            break;

        case DUMMY_BROADCAST_SINGLE_SF:
            break;

        case SMART_BROADCAST_SINGLE_SF:
            break;

        case HOP_COUNT_SINGLE_SF:
        case RSSI_SUM_SINGLE_SF:
           transmit = true;

           routingPacket->setRoutingTableArraySize(singleMetricRoutesCount);

           for (int i = 0; i < singleMetricRoutesCount; i++) {
               LoRaRoute thisLoRaRoute;
               thisLoRaRoute.setId(singleMetricRoutingTable[i].id);
               thisLoRaRoute.setPriMetric(singleMetricRoutingTable[i].metric);
               routingPacket->setRoutingTable(i, thisLoRaRoute);
           }

           break;

        case TIME_ON_AIR_HC_CAD:
        case TIME_ON_AIR_SF_CAD:
            transmit = true;

            cInfo->setLoRaSF(pickCADSF());

            std::vector<LoRaRoute> allLoRaRoutes;

            routingPacket->setRoutingTableArraySize(dualMetricRoutesCount);

           for (int i = 0; i < dualMetricRoutesCount; i++) {
               LoRaRoute thisLoRaRoute;
               thisLoRaRoute.setId(dualMetricRoutingTable[i].id);
               thisLoRaRoute.setPriMetric(dualMetricRoutingTable[i].priMetric);
               thisLoRaRoute.setSecMetric(dualMetricRoutingTable[i].secMetric);
               allLoRaRoutes.push_back(thisLoRaRoute);
               routingPacket->setRoutingTable(i, thisLoRaRoute);
           }

            break;
    }


    if (transmit) {
        sentPackets++;
        sentRoutingPackets++;
        //add LoRa control info


        routingPacket->setControlInfo(cInfo);

        routingPacket->setMsgType(ROUTING);
        routingPacket->setDataInt(nodeId);
        routingPacket->setSource(nodeId);
        routingPacket->setVia(nodeId);
        routingPacket->setDestination(BROADCAST_ADDRESS);
        routingPacket->getOptions().setAppACKReq(false);
        routingPacket->setByteLength(routingPacketMaxSize);

        txSfVector.record(loRaSF);
        txTpVector.record(loRaTP);

        txDuration = calculateTransmissionDuration(routingPacket);

        send(routingPacket, "appOut");
        bubble("Sending routing packet");
        emit(LoRa_AppPacketSent, loRaSF);
    }
    else {
        delete routingPacket;
    }
    return txDuration;
}

void LoRaNodeApp::generateDataPackets() {

    if (true) {
    //if (nodeId == 0) {
        std::vector<int> destinations = { };

        if (numberOfDestinationsPerNode == 0 )
            numberOfDestinationsPerNode = numberOfNodes-1;

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

        for (int k = 0; k < numberOfPacketsPerDestination; k++) {
            for (int j = 0; j < destinations.size(); j++) {
                LoRaAppPacket *dataPacket = new LoRaAppPacket("DataPacket");

                dataPacket->setMsgType(DATA);
                dataPacket->setDataInt(currDataInt+k);
                dataPacket->setSource(nodeId);
                dataPacket->setVia(nodeId);
                dataPacket->setDestination(destinations[j]);
                dataPacket->getOptions().setAppACKReq(requestACKfromApp);
                dataPacket->setByteLength(dataPacketSize);

                switch (routingMetric) {
    //            case 0:
    //                dataPacket->setTtl(1);
    //                break;
                default:
                    dataPacket->setTtl(packetTTL);
                    break;
                }

                LoRaPacketsToSend.push_back(*dataPacket);
                delete dataPacket;
            }
            currDataInt++;
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

bool LoRaNodeApp::isRouteInSingleMetricRoutingTable(int id, int via) {
    if (getRouteIndexInRouteInSingleMetricRoutingTable(id, via) >= 0) {
        return true;
    }
    return false;
}

int LoRaNodeApp::getRouteIndexInRouteInSingleMetricRoutingTable(int id, int via) {
    int singleMetricRoutesCount = end(singleMetricRoutingTable) - begin(singleMetricRoutingTable);

    for (int i = 0; i < singleMetricRoutesCount; i++) {
        if (singleMetricRoutingTable[i].id == id && singleMetricRoutingTable[i].via == via) {
            return i;
        }
    }

    return -1;
}

bool LoRaNodeApp::isRouteInDualMetricRoutingTable(int id, int via, int sf) {
    if (getRouteIndexInDualMetricRoutingTable(id, via, sf) >= 0) {
        return true;
    }
    return false;
}

int LoRaNodeApp::getRouteIndexInDualMetricRoutingTable(int id, int via, int sf) {
    int dualMetricRoutesCount = end(dualMetricRoutingTable) - begin(dualMetricRoutingTable);

    int singleMetricRoutesCount = end(singleMetricRoutingTable) - begin(singleMetricRoutingTable);

    for (int i = 0; i < singleMetricRoutesCount; i++) {
        if (dualMetricRoutingTable[i].id == id && dualMetricRoutingTable[i].via == via && dualMetricRoutingTable[i].sf == sf) {
            return i;
        }
    }

    return -1;
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

int LoRaNodeApp::pickCADSF() {
    do {
        int thisSF = intuniform(minLoRaSF,maxLoRaSF);
        if (bernoulli(pow(0.5, thisSF-minLoRaSF+1)))
            return thisSF;
    } while (true);
}

int LoRaNodeApp::getBestRouteIndexTo(int destination) {
    if (singleMetricRoutingTable.size() > 0) {

        int singleMetricRoutesCount = end(singleMetricRoutingTable) - begin(singleMetricRoutingTable);

        std::vector<singleMetricRoute> availableRoutes;

        for (int i = 0; i < singleMetricRoutesCount; i++) {
            if (singleMetricRoutingTable[i].id == destination) {
                availableRoutes.push_back(singleMetricRoutingTable[i]);
            }
        }

        if (availableRoutes.size() > 0) {
            int bestRoute = 0;
            int bestMetric = availableRoutes[0].metric;

            int availableRoutesCount = end(availableRoutes) - begin(availableRoutes);
            for (int j = 0; j < availableRoutesCount; j++) {
                if (availableRoutes[j].metric < bestMetric) {
                    bestMetric = availableRoutes[j].metric;
                }
            }

            simtime_t lastMetric = 0;

            for (int k = 0; k < availableRoutesCount; k++) {
                if (availableRoutes[k].metric == bestMetric) {
                    if (availableRoutes[k].valid >= lastMetric) {
                        bestRoute = k;
                        lastMetric = availableRoutes[k].valid;
                    }
                }
            }
            return bestRoute;
        }
    }
    else if (dualMetricRoutingTable.size() > 0) {
        int dualMetricRoutesCount = end(dualMetricRoutingTable) - begin(dualMetricRoutingTable);

        std::vector<dualMetricRoute> availableRoutes;

        for (int i = 0; i < dualMetricRoutesCount; i++) {
            if (dualMetricRoutingTable[i].id == destination) {
                availableRoutes.push_back(dualMetricRoutingTable[i]);
            }
        }

        if (availableRoutes.size() > 0) {
            int bestRoute = 0;

            int availableRoutesCount = end(availableRoutes) - begin(availableRoutes);

            for (int j = 0; j < availableRoutesCount; j++) {
                if (availableRoutes[j].priMetric < availableRoutes[bestRoute].priMetric ||
                        ( availableRoutes[j].priMetric == availableRoutes[bestRoute].priMetric &&
                                availableRoutes[j].secMetric < availableRoutes[bestRoute].secMetric) ||
                                ( availableRoutes[j].priMetric == availableRoutes[bestRoute].priMetric &&
                                        availableRoutes[j].secMetric == availableRoutes[bestRoute].secMetric &&
                                            availableRoutes[j].valid > availableRoutes[bestRoute].valid)) {
                    bestRoute = j;
                }
            }
            EV << "Best available route to " << destination << " is via " << availableRoutes[bestRoute].via << " with SF " << "availableRoutes[bestRoute].sf" << endl;
            return bestRoute;
        }
    }

    return -1;
}

void LoRaNodeApp::sanitizeRoutingTable() {
    bool routeDeleted = false;

    if (singleMetricRoutingTable.size() > 0) {

        do {
            routeDeleted = false;

            for (std::vector<singleMetricRoute>::iterator smr =
                    singleMetricRoutingTable.begin(); smr < singleMetricRoutingTable.end();
                    smr++) {
                if (smr->valid < simTime()) {
                    singleMetricRoutingTable.erase(smr);
                    routeDeleted = true;
                    break;
                }
            }
        } while (routeDeleted);
    }
    else if (dualMetricRoutingTable.size() > 0) {

        do {
            routeDeleted = false;

            for (std::vector<dualMetricRoute>::iterator dmr =
                    dualMetricRoutingTable.begin(); dmr < dualMetricRoutingTable.end();
                    dmr++) {
                if (dmr->valid < simTime()) {
                    dualMetricRoutingTable.erase(dmr);
                    routeDeleted = true;
                    break;
                }
            }
        } while (routeDeleted);
    }
}

int LoRaNodeApp::getSFTo(int destination) {
    if (dualMetricRoutingTable.size() > 0) {
        int dualMetricRoutesCount = end(dualMetricRoutingTable) - begin(dualMetricRoutingTable);

        std::vector<dualMetricRoute> availableRoutes;

        for (int i = 0; i < dualMetricRoutesCount; i++) {
            if (dualMetricRoutingTable[i].id == destination) {
                availableRoutes.push_back(dualMetricRoutingTable[i]);
            }
        }

        dualMetricRoutesCount = end(dualMetricRoutingTable) - begin(dualMetricRoutingTable);

        if (availableRoutes.size() > 0) {
            int bestRoute = 0;

            int availableRoutesCount = end(availableRoutes) - begin(availableRoutes);

            for (int j = 0; j < availableRoutesCount; j++) {
                if (availableRoutes[j].sf < availableRoutes[bestRoute].sf) {
                    bestRoute = j;
                }
            }

            if ( availableRoutes[bestRoute].sf >= minLoRaSF && availableRoutes[bestRoute].sf <= maxLoRaSF) {
                return availableRoutes[bestRoute].sf;
            }
        }
    }

    return minLoRaSF;
}


simtime_t LoRaNodeApp::calculateTransmissionDuration(cMessage *msg) {

    TransmissionRequest *controlInfo = dynamic_cast<TransmissionRequest *>(msg->getControlInfo());

    const LoRaAppPacket *frame = check_and_cast<const LoRaAppPacket *>(msg);
    const LoRaMacControlInfo *cInfo = check_and_cast<const LoRaMacControlInfo *>(frame->getControlInfo());

    int nPreamble = 8;
    simtime_t Tsym = (pow(2, cInfo->getLoRaSF()))/(cInfo->getLoRaBW().get()/1000);
    simtime_t Tpreamble = (nPreamble + 4.25) * Tsym / 1000;

    int payloadBytes = frame->getByteLength()+8; //+8 bytes for headers

    int payloadSymbNb = 8 + math::max(ceil((8*payloadBytes - 4*cInfo->getLoRaSF() + 28 + 16 - 20*0)/(4*(cInfo->getLoRaSF()-2*0)))*(cInfo->getLoRaCR() + 4), 0);

    simtime_t Theader = 0.5 * (8+payloadSymbNb) * Tsym / 1000;
    simtime_t Tpayload = 0.5 * (8+payloadSymbNb) * Tsym / 1000;

    const simtime_t duration = Tpreamble + Theader + Tpayload;
    return duration;
}

} //end namespace inet

