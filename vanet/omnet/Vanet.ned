import org.car2x.veins.nodes.Car;
import org.car2x.veins.base.modules.BaseMobility;
import org.car2x.veins.base.modules.BaseWorldUtility;
import org.car2x.veins.base.connectionManager.ConnectionManager;
import org.car2x.veins.modules.world.annotations.AnnotationManager;
import org.car2x.veins.modules.application.ieee80211p.DemoBaseApplLayer;

simple ExternalMobility extends BaseMobility { parameters: @class(ExternalMobility); }
simple ConnectivityApp extends DemoBaseApplLayer {
    parameters:
        @class(ConnectivityApp);
        bool isRsu = default(false);
        double interval @unit(s) = default(1s);
}
module Obu extends Car {
    parameters:
        applType = "ConnectivityApp";
        veinsmobilityType = "ExternalMobility";
        @display("i=veins/node/car;is=s;t=OBU");
}
simple CarlaManager {
    parameters:
        @class(CarlaManager);
        int port = default(9998);
        bool smoke = default(false);
}
network Town04Vanet {
    parameters:
        double playgroundSizeX @unit(m) = default(1200m);
        double playgroundSizeY @unit(m) = default(1200m);
        double playgroundSizeZ @unit(m) = default(100m);
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
        rsu: Obu { parameters: @display("i=device/antennatower;t=RSU 924"); }
    connections allowunconnected:
}
