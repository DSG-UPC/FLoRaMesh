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

package loranetwork.LoraNode;

import inet.mobility.static.StationaryMobility;
import inet.networklayer.common.InterfaceTable;
import inet.networklayer.contract.INetworkLayer;
import inet.linklayer.loopback.LoopbackInterface;
import loranetwork.LoRa.LoRaMotoGWNic;
import loranetwork.LoRaApp.LoRaMotoGWApp;
import inet.linklayer.contract.IWiredNic;
import inet.applications.contract.IUDPApp;
import inet.transportlayer.contract.IUDP;
import inet.networklayer.contract.IRoutingTable;

module LoRaMotoGW
{
    parameters:
        @networkNode();
        *.interfaceTableModule = default(absPath(".interfaceTable"));
        *.routingTableModule = default(routingTableType != "" ? absPath(".routingTable") : "");
        string networkLayerType = default("IPv4NetworkLayer");
        string routingTableType = default("IPv4RoutingTable");
        int numUdpApps = default(0);
        bool hasUdp = true;
        string udpType = default(firstAvailableOrEmpty("UDP"));

        @display("bgb=611,560;i=device/antennatower");
    gates:
    submodules:
        mobility: StationaryMobility {
            @display("p=24,88");
        }
        LoRaMotoGWNic: LoRaMotoGWNic {
            @display("p=163.56,367.728");
        }
        LoRaMotoGWApp: LoRaMotoGWApp {
            @display("p=163.56,152.28");
        }
    connections allowunconnected:
        LoRaMotoGWApp.appOut --> LoRaMotoGWNic.upperLayerIn;
        LoRaMotoGWNic.upperLayerOut --> LoRaMotoGWApp.appIn;
}
