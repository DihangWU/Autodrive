import org.car2x.veins.nodes.Car;
import org.car2x.veins.base.modules.BaseMobility;
import org.car2x.veins.base.modules.BaseWorldUtility;
import org.car2x.veins.base.connectionManager.ConnectionManager;
import org.car2x.veins.modules.world.annotations.AnnotationManager;
import org.car2x.veins.modules.application.ieee80211p.DemoBaseApplLayer;
import org.car2x.veins.modules.phy.PhyLayer80211p;
import org.car2x.veins.modules.mac.ieee80211p.Mac1609_4;
import org.car2x.veins.modules.nic.INic80211p;

simple RangePhy80211p extends PhyLayer80211p {
    parameters:
        @class(RangePhy80211p);
        bool earlyRangeFilter = default(true);
}
module RangeNic80211p like INic80211p {
    parameters:
        string connectionManagerName = default("connectionManager");
    gates:
        input upperLayerIn;
        output upperLayerOut;
        output upperControlOut;
        input upperControlIn;
        input radioIn;
    submodules:
        phy80211p: RangePhy80211p { @display("p=69,218;i=block/process_s"); }
        mac1609_4: Mac1609_4 { @display("p=69,82"); }
    connections:
        radioIn --> phy80211p.radioIn;
        mac1609_4.lowerControlOut --> phy80211p.upperControlIn;
        mac1609_4.lowerLayerOut --> phy80211p.upperLayerIn;
        phy80211p.upperLayerOut --> mac1609_4.lowerLayerIn;
        phy80211p.upperControlOut --> mac1609_4.lowerControlIn;
        mac1609_4.upperControlIn <-- upperControlIn;
        mac1609_4.upperLayerIn <-- upperLayerIn;
        mac1609_4.upperLayerOut --> upperLayerOut;
        mac1609_4.upperControlOut --> upperControlOut;
}

simple ExternalMobility extends BaseMobility { parameters: @class(ExternalMobility); }
simple ConnectivityApp extends DemoBaseApplLayer {
    parameters:
        @class(ConnectivityApp);
        bool isRsu = default(false);
        double interval @unit(s) = default(1s);
        bool sendProbes = default(true);
        bool sendHello = default(true);
        double handshakeTimeout @unit(s) = default(3s);
        string csvDir = default("results");
        double relayJitter @unit(s) = default(20ms);
}
module Obu extends Car {
    parameters:
        double communicationRange @unit(m) = default(-1m);
        applType = "ConnectivityApp";
        nicType = "RangeNic80211p";
        veinsmobilityType = "ExternalMobility";
        @display("i=veins/node/car;is=s;t=OBU");
}
simple CarlaManager {
    parameters:
        @class(CarlaManager);
        int port = default(9998);
        bool smoke = default(false);
        string rsuDefinitions = default("");
        double activityWindow @unit(s) = default(350ms);
        // Detailed packet lines/bubbles for small scenes; persistent coverage
        // and CARLA feedback remain enabled at every scale. -1 means unlimited.
        int packetAnimationLimit = default(20);
        // The GUI processes Stop/Run while waiting for a bridge snapshot, but
        // each idle() call redraws the whole Qtenv scene even in Express mode.
        double guiIdleInterval @unit(s) = default(100ms);
}
network Town04Vanet {
    parameters:
        double playgroundSizeX @unit(m) = default(1200m);
        double playgroundSizeY @unit(m) = default(1200m);
        double playgroundSizeZ @unit(m) = default(100m);
        double vehicleRange @unit(m);
        double rsuRange @unit(m);
        @display("bgb=1200,1200");
    submodules:
        world: BaseWorldUtility {
            parameters:
                @display("p=40,40");
                playgroundSizeX = parent.playgroundSizeX;
                playgroundSizeY = parent.playgroundSizeY;
                playgroundSizeZ = parent.playgroundSizeZ;
        }
        connectionManager: ConnectionManager { parameters: @display("p=120,40"); }
        annotations: AnnotationManager { parameters: @display("p=200,40"); }
        manager: CarlaManager { parameters: @display("p=280,40"); }
    connections allowunconnected:
}
