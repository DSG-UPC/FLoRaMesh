package loranetwork.simulations;

import inet.applications.udpapp.UDPBasicApp;
import inet.node.inet.Router;
import inet.node.internetcloud.InternetCloud;
import loranetwork.LoRaPhy.LoRaMedium;
import loranetwork.LoraNode.LoRaNode;
import loranetwork.LoraNode.LoRaGW;
import inet.node.inet.StandardHost;
import inet.networklayer.configurator.ipv4.IPv4NetworkConfigurator;
import inet.node.ethernet.Eth1G;

@license(LGPL);
network LoRaMoto
{
    parameters:
        int numberOfNodes = default(1);
        int numberOfGateways = default(1);
        int networkSizeX = default(1400);
        int networkSizeY = default(2500);
        @display("bgb=1400,2500;bgi=background/coquimbo-03;bgl=2");
    submodules:
        loRaNodes[numberOfNodes]: LoRaNode {
            @display("p=83.664,233.856");
        }
        loRaGW[numberOfGateways]: LoRaGW {
            @display("p=83.664,150.192;is=s");
        }
        LoRaMedium: LoRaMedium {
            @display("p=150,100");
        }
        networkServer: StandardHost {
            parameters:
                @display("p=850,-100");
        }
        configurator: IPv4NetworkConfigurator {
            parameters:
                assignDisjunctSubnetAddresses = false;
                @display("p=1200,-100");
        }
        internetCloud: InternetCloud {
            @display("p=300,-100");
        }
        gwRouter[numberOfGateways]: Router {
            @display("p=50,-100");
        }
        nsRouter: Router {
            @display("p=550,-100");
        }
    connections:
        networkServer.ethg++ <--> Eth1G <--> nsRouter.ethg++;
        nsRouter.pppg++ <--> Eth1G <--> internetCloud.pppg++;
        for i=0..numberOfGateways-1 {
            internetCloud.pppg++ <--> Eth1G <--> gwRouter[i].pppg++;
            gwRouter[i].ethg++ <--> Eth1G <--> loRaGW[i].ethg++;
        }
}

