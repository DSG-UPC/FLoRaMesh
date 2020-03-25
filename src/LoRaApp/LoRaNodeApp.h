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

#ifndef __LORA_OMNET_LORANODEAPP_H_
#define __LORA_OMNET_LORANODEAPP_H_

#include <omnetpp.h>
#include "inet/common/lifecycle/ILifecycle.h"
#include "inet/common/lifecycle/NodeStatus.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/LifecycleOperation.h"

#include "LoRaAppPacket_m.h"
#include "LoRa/LoRaMacControlInfo_m.h"

using namespace omnetpp;

namespace inet {

/**
 * TODO - Generated class
 */
class INET_API LoRaNodeApp : public cSimpleModule, public ILifecycle
{
    protected:
        virtual void initialize(int stage) override;
        void finish() override;
        virtual int numInitStages() const override { return NUM_INIT_STAGES; }
        virtual void handleMessage(cMessage *msg) override;
        virtual bool handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback) override;
        virtual bool isNeighbour(int neighbourId);
        virtual bool isACKed(int nodeId);

        void handleMessageFromLowerLayer(cMessage *msg);
        std::pair<double,double> generateUniformCircleCoordinates(double radius, double gatewayX, double gatewayY);
        void sendJoinRequest();
        void sendDataPacket();
        void sendCalibrationPacket();
        void sendDownMgmtPacket();

        int numberOfDataPacketsToSend;
        int numberOfCalibrationPacketsToSend;
        int numberOfForwardedPacketsToSend;
        int sentPackets;
        int sentDataPackets;
        int sentCalibrationPackets;
        int sentForwardedPackets;
        int receivedPackets;
        int receivedCalibrationPackets;
        int receivedDataPackets;
        int receivedForwardedPackets;
        int receivedACKs;
        int receivedOwnACKs;
        int receivedADRs;
        int receivedOwnADRs;
        int lastSentMeasurement;

        simtime_t calibrationPeriod;
        simtime_t timeToFirstDataPacket;
        simtime_t timeToNextDataPacket;
        simtime_t timeToFirstCalibrationPacket;
        simtime_t timeToNextCalibrationPacket;

        cMessage *configureLoRaParameters;
        cMessage *selfDataPacket;
        cMessage *selfCalibrationPacket;

        //history of sent packets;
        cOutVector sfVector;
        cOutVector tpVector;

        //variables to control ADR
        bool evaluateADRinNode;
        int ADR_ACK_CNT = 0;
        int ADR_ACK_LIMIT = 64; //64;
        int ADR_ACK_DELAY = 32; //32;
        bool sendNextPacketWithADRACKReq = false;
        void increaseSFIfPossible();

        //General network variables
        int numberOfNodes;

        //Routing variables
        int packetForwarding;
        int numberOfHops;

        //Node info
        int nodeId;

        std::vector<int> neighbourNodes;
        std::vector<int> ACKedNodes;
        std::vector<LoRaAppPacket> LoRaPacketBuffer;

        //Application parameters
        bool requestACKfromApp;
        bool stopOnACK;
        bool stopOnCAL;
        bool AppACKReceived;
        bool AppADRReceived;
        int firstACK;
        int firstADR;


        //Spreading factor
        bool increaseSF;
        int firstACKSF;
        int firstADRSF;
        int packetsPerSF;
        int packetsInSF;

    public:
        LoRaNodeApp() {}
        simsignal_t LoRa_AppPacketSent;
        //LoRa physical layer parameters
        double loRaTP;
        units::values::Hz loRaCF;
        int loRaSF;
        units::values::Hz loRaBW;
        int loRaCR;
        bool loRaUseHeader;
};

}

#endif
