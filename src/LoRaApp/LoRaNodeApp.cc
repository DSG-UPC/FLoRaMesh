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
        if (!isOperational) {
            throw cRuntimeError("This module doesn't support starting in node DOWN state");
        }

        timeToFirstDataPacket = math::max(5, par("timeToFirstCalibrationPacket"));

        sentPackets = 0;
        sentCalibrationPackets = 0;
        sentDataPackets = 0;
        sentForwardedPackets = 0;
        receivedPackets = 0;
        receivedCalibrationPackets = 0;
        receivedDataPackets = 0;
        receivedForwardedPackets = 0;
        receivedACKs = 0;
        receivedADRs = 0;
        receivedOwnACKs = 0;
        receivedOwnADRs = 0;

        numberOfDataPacketsToSend = par("numberOfDataPacketsToSend");
        numberOfCalibrationPacketsToSend = par("numberOfCalibrationPacketsToSend");
        numberOfForwardedPacketsToSend = par("numberOfForwardedPacketsToSend");

        calibrationPeriod = par("calibrationPeriod");
        timeToFirstCalibrationPacket = par("timeToFirstCalibrationPacket");
        timeToNextCalibrationPacket = par("timeToNextCalibrationPacket");

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
        ACKedNodes = {};

        //Node identifier
        nodeId = getContainingNode(this)->getIndex();

        //Application acknowledgment and ADR
        requestACKfromApp = par("requestACKfromApp");
        requestADRfromApp = par("requestADRfromApp");
        stopOnACK = par("stopOnACK");
        stopOnADR = par("stopOnADR");
        AppACKReceived = false;
        AppADRReceived = false;
        firstACK = 0;
        firstADR = 0;

        //Spreading factor
        increaseSF = par("increaseSF");
        firstACKSF = 0;
        firstADRSF = 0;
        packetsPerSF = par("packetsPerSF");
        packetsInSF = 0;

        //WATCHES only for GUI
        if (getEnvir()->isGUI()) {
            WATCH(sentPackets);
            WATCH(sentCalibrationPackets);
            WATCH(sentDataPackets);
            WATCH(sentForwardedPackets);
            WATCH(receivedPackets);
            WATCH(receivedACKs);
            WATCH(receivedOwnACKs);
            WATCH(receivedADRs);
            WATCH(receivedOwnADRs);
            WATCH(AppADRReceived);
            WATCH(AppACKReceived);
            WATCH(firstACK);
            WATCH(firstACKSF);
            WATCH(firstADR);
            WATCH(firstADRSF);

            WATCH(loRaSF);
            WATCH(loRaTP);
            WATCH(packetsInSF);

            WATCH_VECTOR(neighbourNodes);
            WATCH_VECTOR(ACKedNodes);

            WATCH_VECTOR(LoRaPacketBuffer);

            WATCH(requestADRfromApp);
        }

        if (requestADRfromApp) {
            selfCalibrationPacket = new cMessage("selfCalibrationPacket");
            scheduleAt(simTime()+getSimulation()->getWarmupPeriod()+timeToFirstCalibrationPacket, selfCalibrationPacket);
        }

        selfDataPacket = new cMessage("selfDataPacket");
        scheduleAt(simTime()+getSimulation()->getWarmupPeriod()+calibrationPeriod+timeToFirstDataPacket, selfDataPacket);
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
    recordScalar("sentCalibrationPackets", sentCalibrationPackets);
    recordScalar("sentDataPackets", sentDataPackets);
    recordScalar("sentForwardedPackets", sentForwardedPackets);
    recordScalar("receivedPackets", receivedPackets);
    recordScalar("receivedACKs", receivedACKs);
    recordScalar("receivedOwnACKs", receivedOwnACKs);
    recordScalar("receivedADRs", receivedADRs);
    recordScalar("receivedOwnADRs", receivedOwnADRs);
    recordScalar("AppACKReceived", AppACKReceived);
    recordScalar("firstACK", firstACK);
    recordScalar("firstACKSF", firstACKSF);
    recordScalar("firstADR", firstADR);
    recordScalar("firstADRSF", firstADRSF);

    for (std::vector<LoRaAppPacket>::iterator lbptr = LoRaPacketBuffer.begin(); lbptr < LoRaPacketBuffer.end(); lbptr++) {
        LoRaPacketBuffer.erase(lbptr);
    }
}

void LoRaNodeApp::handleMessage(cMessage *msg)
{
    // Handle selfMessage
    if (msg->isSelfMessage()) {

        // selfMessage for sending data packets
        if (msg == selfDataPacket) {

            if (simTime() >= getSimulation()->getWarmupPeriod()+calibrationPeriod)
            {
                bool schedule = false;
                // Check conditions for sending own data packet
                if ((numberOfDataPacketsToSend == 0 || sentDataPackets < numberOfDataPacketsToSend) && (!AppACKReceived || !stopOnACK))
                {
                    sendDataPacket();
                    schedule = true;
                }
                // Check conditions for forwarding own data packet
                else if ( sentForwardedPackets < numberOfForwardedPacketsToSend)
                {
                    // Only go to the sendDataPacket() function if there is something to be forwarded
                    if ( LoRaPacketBuffer.size() > 0 )
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

                    timeToNextDataPacket = math::max(time, par("timeToNextDataPacket"));

                    selfDataPacket = new cMessage("selfDataPacket");
                    scheduleAt(simTime() + timeToNextDataPacket, selfDataPacket);
                }
            }
        }
        // selfMessage for sending calibration packets
        else if (msg == selfCalibrationPacket) {

            // Check that we are in the calibration period
            if (simTime() >= getSimulation()->getWarmupPeriod() && simTime() < getSimulation()->getWarmupPeriod() + calibrationPeriod)
            {
                // Check conditions for sending calibration packet: infinite packets to send or maximum packets not yet reached, and calibration not yet achieved
                if ((numberOfCalibrationPacketsToSend == 0 || sentCalibrationPackets < numberOfCalibrationPacketsToSend) && (!AppADRReceived || !stopOnADR)) {
                    sentCalibrationPackets++;
                    sentPackets++;
                    sendCalibrationPacket();
                }

                // Minimum delay until next self-message, depending on the SF in use
                double time;
                if(loRaSF == 7) time = 7.808;
                if(loRaSF == 8) time = 13.9776;
                if(loRaSF == 9) time = 24.6784;
                if(loRaSF == 10) time = 49.3568;
                if(loRaSF == 11) time = 85.6064;
                if(loRaSF == 12) time = 171.2128;

                timeToNextCalibrationPacket = math::max(time, par("timeToNextCalibrationPacket"));

                // Conditions for scheduling next selfCalibrationPacket: infinite packets to send or maximum packets not yet reached, and calibration period still going on
                if ( (numberOfCalibrationPacketsToSend == 0 || sentCalibrationPackets < numberOfCalibrationPacketsToSend) &&
                     (simTime() + timeToNextCalibrationPacket < getSimulation()->getWarmupPeriod() + calibrationPeriod) && (!AppADRReceived || !stopOnADR) )
                {
                    selfCalibrationPacket = new cMessage("selfCalibrationPacket");
                    scheduleAt(simTime() + timeToNextCalibrationPacket, selfCalibrationPacket);
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

    switch (packet->getMsgType()) {
        case ACK :
            receivedACKs++;
            //Check if the packet is for the current node
                if (packet->getDestination() == nodeId) {
                    receivedOwnACKs++;
                    // bubble("I received an ACK packet from the app for me!");
                    AppACKReceived = true;

                    // Keep track of first ACKed packet via the data returned by the app
                    // if (firstACK == 0) {
                    //    firstACK = packet->getDataInt();
                    //    firstACKSF = loRaSF;
                    // }

                    // Keep track of first ACKed packet via the sentPackets count
                    if (firstACK == 0) {
                       firstACK = sentPackets;
                       firstACKSF = loRaSF;
                    }

                }
                else {
                    // bubble("I received an ACK packet from the app for another node!");
                    if (!isACKed(packet->getDestination())) {
                        ACKedNodes.push_back(packet->getDestination());
                    }

                }
            break;
        case DATA:
            // bubble("DATA packet received!");

            //Check if the packet is for the current node
            if (packet->getDestination() == nodeId) {
                // bubble("I received a data packet for me!");
                //TODO delete packet;
            }
            else {
                //Keep track of neighbouring nodes
                if (!isNeighbour(packet->getVia())){
                    // bubble ("New neighbour!");
                    neighbourNodes.push_back(packet->getVia());
                }

                // Check for retransmissions of packages originally sent by this node
                if (packet->getSource() == nodeId) {
                    // bubble("I received a LoRa packet originally sent by me!");
                    //TODO delete packet
                }
                //Forward packet
                else {
                    switch(packetForwarding) {
                        // No forwarding
                        case 0 :
                            // bubble("Packet forwarding disabled!");
                            break;

                        //N-hop broadcast forwarding
                        case 1 :
                            if ( packet->getHops() > 0 ) {
                                // bubble("I received a LoRa packet to retransmit!");

                                LoRaAppPacket *dataPacket = new LoRaAppPacket("DataFrame");
                                dataPacket->setMsgType(packet->getMsgType());

                                dataPacket->setDataInt(packet->getDataInt());

                                dataPacket->setSource(packet->getSource());
                                dataPacket->setDestination(packet->getDestination());
                                dataPacket->setVia(nodeId);

                                dataPacket->getOptions().setAppACKReq(packet->getOptions().getAppACKReq());
                                dataPacket->getOptions().setAppADRReq(packet->getOptions().getAppADRReq());

                                dataPacket->setHops(packet->getHops() -1 );

                                LoRaPacketBuffer.push_back(*dataPacket);
                            }
                            else {
                                // bubble("I received a LoRa packet that has reached the maximum hop count!");
                            }
                            break;

                        //N-hop broadcast forwarding for non-acked nodes
                        case 2 :
                            if ( packet->getHops() > 0 ) {
                            // bubble("I received a LoRa packet to retransmit!");

                            LoRaAppPacket *dataPacket = new LoRaAppPacket("DataFrame");
                            dataPacket->setMsgType(packet->getMsgType());

                            dataPacket->setDataInt(packet->getDataInt());

                            dataPacket->setSource(packet->getSource());
                            dataPacket->setDestination(packet->getDestination());
                            dataPacket->setVia(nodeId);

                            dataPacket->getOptions().setAppACKReq(packet->getOptions().getAppACKReq());
                            dataPacket->getOptions().setAppADRReq(packet->getOptions().getAppADRReq());

                            dataPacket->setHops(packet->getHops() -1 );

                            LoRaPacketBuffer.push_back(*dataPacket);
                            }
                            else {
                            // bubble("I received a LoRa packet that has reached the maximum hop count!");
                            }
                            break;
                        }
                    }
                }
            break;
        case TXCONFIG :
            receivedADRs++;
            //Check if the packet is for the current node
                if (packet->getDestination() == nodeId) {
                    receivedOwnADRs++;
                    // bubble("I received an ACK packet from the app for me!");
                    AppADRReceived = true;

                    // Keep track of first ACKed packet via the sentPackets count
                    if (firstADR == 0) {
                       firstADR = sentCalibrationPackets;
                       firstADRSF = loRaSF;
                    }

                    // (ToDo): Get received transmission settings and apply them
                    // but the network server replies weird too optimistic settings.
                    // LoRaOptions ADROptions = packet->getOptions();
                    // loRaTP = ADROptions.getLoRaTP();
                    // loRaSF = ADROptions.getLoRaSF();

                }
                // (ToDo): By now, do nothing with TXCONFIG packets received.
                // They might be useful for the forwarding phase in order to decide with
                // nodes' packets to forward, which SF to listen to, etc.
                // else {
                    // bubble("I received an ADR packet from the app for another node!");
                   // if (!isACKed(packet->getDestination())) {
                   //     ACKedNodes.push_back(packet->getDestination());
                   // }
                    //
                // }
            break;
        default:
            // bubble("Other type of packet received!");
            break;
    }
}

bool LoRaNodeApp::handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback)
{
    Enter_Method_Silent();

    throw cRuntimeError("Unsupported lifecycle operation '%s'", operation->getClassName());
    return true;
}

void LoRaNodeApp::sendCalibrationPacket()
{
    LoRaAppPacket *calibrationPacket = new LoRaAppPacket("DataFrame");
    bool transmit = false;

    // Own packets are [generated and] sent first, then we may forward others'
    if ((numberOfCalibrationPacketsToSend == 0 || sentCalibrationPackets < numberOfCalibrationPacketsToSend) && (!AppADRReceived || !stopOnADR)) {
        transmit = true;

        calibrationPacket->setMsgType(TXCONFIG);
        calibrationPacket->setDataInt(sentCalibrationPackets);
        calibrationPacket->setSource(nodeId);
        calibrationPacket->setVia(nodeId);
        calibrationPacket->setDestination(-1);
        calibrationPacket->getOptions().setAppADRReq(true);
        calibrationPacket->setHops(0);
    }

    if (increaseSF) {
        if ((packetsInSF) == packetsPerSF) {
            packetsInSF = 1;
            increaseSFIfPossible();
        }
        else {
            packetsInSF++;
        }
     }

    //add LoRa control info
        LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;
        cInfo->setLoRaTP(loRaTP);
        cInfo->setLoRaCF(loRaCF);
        cInfo->setLoRaSF(loRaSF);
        cInfo->setLoRaBW(loRaBW);
        cInfo->setLoRaCR(loRaCR);

    calibrationPacket->setControlInfo(cInfo);

    if (transmit) {
        sfVector.record(loRaSF);
        tpVector.record(loRaTP);
        send(calibrationPacket, "appOut");
        emit(LoRa_AppPacketSent, loRaSF);
    }
}

void LoRaNodeApp::sendDataPacket()
{
    bubble("Sending a data packet!");
    LoRaAppPacket *dataPacket = new LoRaAppPacket("DataFrame");
    bool transmit = false;

    int packetPos = 0;

    // Own packets are [generated and] sent first, then we may forward others'
    if (sentDataPackets < numberOfDataPacketsToSend) {

        transmit = true;

        // char text[32];
        // sprintf(text, "Sending my own packet #%d", sentPackets);
        // bubble(text);

        dataPacket->setMsgType(DATA);
        dataPacket->setDataInt(sentPackets+1);
        dataPacket->setSource(nodeId);
        dataPacket->setVia(nodeId);
        // do dataPacket->setDestination(intuniform(0, numberOfNodes-1));
        // while (dataPacket->getDestination() == nodeId);
        dataPacket->setDestination(-1);

        dataPacket->getOptions().setAppACKReq(requestACKfromApp);

        if ( isNeighbour(dataPacket->getDestination())){
            dataPacket->setHops(0);
        }
        else {
            dataPacket->setHops(numberOfHops);
        }

        if (increaseSF) {
            if ((packetsInSF) == packetsPerSF) {
                packetsInSF = 1;
                increaseSFIfPossible();
            }
            else {
                packetsInSF++;
            }
         }

        sentDataPackets++;
        sentPackets++;
    }
    // Forward other nodes' packets
    else {
        if (LoRaPacketBuffer.size() > 0) {

            transmit = true;

            switch(packetForwarding) {
                // No forwarding
                case 0:
                {
                    // bubble("Packet forwarding disabled!");
                    break;
                }
                // Randomly pick one of the packets
                case 1:
                {
                    packetPos = intuniform(0, LoRaPacketBuffer.size()-1);

                    LoRaAppPacket *randomDataPacket = &LoRaPacketBuffer.at(packetPos);
                    dataPacket = randomDataPacket->dup();
                    LoRaPacketBuffer.erase(LoRaPacketBuffer.begin()+packetPos);
                    break;
                }
                // Randomly pick one of the packets, but discard those that have been ACKed
                case 2:
                {
                    bubble("booo");
                    bool fwdPacket = false;

                    while (LoRaPacketBuffer.size() > 0 && fwdPacket == false){
                        packetPos = intuniform(0, LoRaPacketBuffer.size()-1);

                        LoRaAppPacket *randomDataPacket = &LoRaPacketBuffer.at(packetPos);
                        dataPacket = randomDataPacket->dup();
                        LoRaPacketBuffer.erase(LoRaPacketBuffer.begin()+packetPos);

                        if ( !isACKed(dataPacket->getSource()) )
                        {
                            fwdPacket= true;
                        }
                    }
                    break;
                }
                // FIFO
                default:
                {
                    LoRaAppPacket *frontDataPacket = &LoRaPacketBuffer.front();
                    dataPacket = frontDataPacket->dup();
                    LoRaPacketBuffer.erase(LoRaPacketBuffer.begin());
                }
            }

            sentForwardedPackets++;
            sentPackets++;
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

    //dataPacket->getOptions().setAppADRReq(true);

    if (transmit) {
        sfVector.record(loRaSF);
        tpVector.record(loRaTP);
        send(dataPacket, "appOut");
        emit(LoRa_AppPacketSent, loRaSF);
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

} //end namespace inet

