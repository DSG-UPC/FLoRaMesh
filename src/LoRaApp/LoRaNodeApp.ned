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

package loranetwork.LoRaApp;

//
// TODO auto-generated module
//
simple LoRaNodeApp
{
    parameters:
        @signal[LoRa_AppPacketSent](type=long); // optional
        @statistic[LoRa_AppPacketSent](source=LoRa_AppPacketSent; record=count);
        int numberOfPacketsToForward = default(0);
        volatile double timeToFirstDataPacket @unit(s) = default(10s);
        volatile double timeToNextDataPacket @unit(s) = default(10s);
        volatile double timeToFirstRoutingPacket @unit(s) = default(10s);
        volatile double timeToNextRoutingPacket @unit(s) = default(10s);
        double initialLoRaTP @unit(dBm) = default(14dBm);
        double initialLoRaCF @unit(Hz) = default(868MHz);
        int initialLoRaSF = default(12);
        int minLoRaSF = default(7);
        int maxLoRaSF = default(12);
        double initialLoRaBW @unit(Hz) = default(125kHz);
        int initialLoRaCR = default(4);
        bool initialUseHeader = default(true);
        bool initialLoRaCAD = default(false);
        double initialLoRaCADatt @unit(dB) = default(10dB);
        @class(inet::LoRaNodeApp);
        bool evaluateADRinNode = default(false);
        int numberOfNodes;
        int routingMetric = default(0);
        bool routeDiscovery = default(false);
        double ownDataPriority = default(1);
        int packetTTL = default(1);
        double routingPacketPriority = default(0.5);
        bool storeBestRouteOnly = default(false);
        bool getRoutesFromDataPackets = default(true);
        volatile double routeTimeout @unit(s) = default(60s);
        bool requestACKfromApp = default(false);
        bool stopOnACK = default(true);
        bool AppACKReceived = default(false);
        bool increaseSF = default(false);
        int packetsPerSF = default(1);
        int packetsInSF = default(0);
        bool sendPacketsContinuously = default(false);
        bool enforceDutyCycle = default(true);
        double dutyCycle = default(0.01);
        int numberOfDestinationsPerNode = default(1);
        int numberOfPacketsPerDestination = default(1);
        int dataPacketDefaultSize @unit(B) = default(50B);
        int routingPacketMaxSize @unit(B) = default(16B);
    gates:
        output appOut @labels(LoRaAppPacket/down);
        input appIn @labels(LoRaAppPacket/up);
}
