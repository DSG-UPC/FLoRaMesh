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
#include "inet/common/FSMA.h"

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
        virtual bool isSMRTNeighbour(int neighbourId);
        virtual bool isKnownNode(int knownNodeId);
        virtual bool isACKed(int nodeId);
        virtual bool isPacketForwarded(cMessage *msg);
        virtual bool isPacketToBeForwarded(cMessage *msg);
        virtual bool isDataPacketForMeUnique(cMessage *msg);

        void handleMessageFromLowerLayer(cMessage *msg);
        void handleSelfDataPacket();
        void handleSelfRoutingPacket();
        void sendRoutingPacket();
        void manageReceivedPacketForMe(cMessage *msg);
        void manageReceivedAckPacketForMe(cMessage *msg);
        void manageReceivedDataPacketForMe(cMessage *msg);
        void manageReceivedPacketToForward(cMessage *msg);
        void manageReceivedAckPacketToForward(cMessage *msg);
        void manageReceivedDataPacketToForward(cMessage *msg);
        void manageReceivedRoutingPacket(cMessage *msg);
        std::pair<double,double> generateUniformCircleCoordinates(double radius, double gatewayX, double gatewayY);
        void sendJoinRequest();
        void sendPacket();
        void sendDownMgmtPacket();
        void generateDataPackets();

        int numberOfDestinationsPerNode;
        int numberOfPacketsPerDestination;

        int numberOfPacketsToForward;

        int sentPackets;
        int sentDataPackets;
        int sentRoutingPackets;
        int sentAckPackets;
        int receivedPackets;
        int receivedPacketsForMe;
        int receivedPacketsFromMe;
        int receivedPacketsToForward;
        int receivedDataPackets;
        int receivedDataPacketsForMe;
        int receivedDataPacketsForMeUnique;
        int receivedDataPacketsFromMe;
        int receivedDataPacketsToForward;
        int receivedDataPacketsToForwardCorrect;
        int receivedDataPacketsToForwardExpired;
        int receivedDataPacketsToForwardUnique;
        int receivedAckPackets;
        int receivedAckPacketsForMe;
        int receivedAckPacketsFromMe;
        int receivedAckPacketsToForward;
        int receivedAckPacketsToForwardCorrect;
        int receivedAckPacketsToForwardExpired;
        int receivedAckPacketsToForwardUnique;
        int receivedRoutingPackets;
        int receivedADRCommands;
        int forwardedPackets;
        int forwardedDataPackets;
        int forwardedAckPackets;
        int lastSentMeasurement;
        simtime_t timeToFirstPacket;
        simtime_t timeToNextPacket;
        simtime_t timeToFirstRoutingPacket;
        simtime_t timeToNextRoutingPacket;

        cMessage *configureLoRaParameters;
        cMessage *selfDataPacket;
        cMessage *selfRoutingPacket;

        //history of sent packets;
        cOutVector txSfVector;
        cOutVector txTpVector;

        // History of received packets
        cOutVector rxRssiVector;
        cOutVector rxSfVector;

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
        double ownDataPriority;
        int maxHops;

        //Node info
        int nodeId;

        std::vector<int> neighbourNodes;
        std::vector<int> knownNodes;
        std::vector<int> ACKedNodes;
        std::vector<LoRaAppPacket> LoRaPacketsToSend;
        std::vector<LoRaAppPacket> LoRaPacketsToForward;
        std::vector<LoRaAppPacket> LoRaPacketsForwarded;
        std::vector<LoRaAppPacket> DataPacketsForMe;


        //Application parameters
        bool requestACKfromApp;
        bool stopOnACK;
        bool AppACKReceived;
        int firstACK;

        //Spreading factor
        bool increaseSF;
        int firstACKSF;
        int packetsPerSF;
        int packetsInSF;

        //LoRa settings
        int minLoRaSF;
        int maxLoRaSF;

        // Routing tables
        class singleMetricRoute {

            public:
                int id;
                int via;
                int metric;
        };
        std::vector<singleMetricRoute> singleMetricRoutingTable;

        /**
         * @name CsmaCaMac state variables
         * Various state information checked and modified according to the state machine.
         */
        //@{
        enum State {
            IDLE,
            TRANSMIT,
            WAIT_DELAY_1,
            LISTENING_1,
            RECEIVING_1,
            WAIT_DELAY_2,
            LISTENING_2,
            RECEIVING_2,
        };

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
        bool loRaCAD;
        double loRaCADatt;

};

}

#endif
