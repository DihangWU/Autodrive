package vanet.radio;
import org.car2x.veins.base.modules.BaseWorldUtility;
import org.car2x.veins.base.modules.BaseMobility;
import org.car2x.veins.base.connectionManager.ConnectionManager;
import org.car2x.veins.nodes.Car;

simple ExternalMobility extends BaseMobility {
    parameters:
        @class(ExternalMobility);
}
simple WirelessApplication like org.car2x.veins.base.modules.IBaseApplLayer {
    parameters:
        @class(WirelessApplication);
        bool notAffectedByHostState = default(true);
        int headerLength @unit(bit) = default(80bit);
    gates:
        input lowerLayerIn;
        output lowerLayerOut;
        input lowerControlIn;
        output lowerControlOut;
}
module ExternalNode extends Car {
    parameters:
        string externalId;
        applType = "vanet.radio.WirelessApplication";
        veinsmobilityType = "vanet.radio.ExternalMobility";
}
simple NetworkGateway {
    parameters:
        @class(NetworkGateway);
        string socketPath;
        string eventFile;
        string stepStatisticsFile;
        string allowedResultsRoot;
        double vehicleAntennaOffsetZ @unit(m) = default(1.5m);
        double v2vRange @unit(m);
        double rsuRange @unit(m);
        string sceneFile;
        double coordinateOffsetX = default(2000);
        double coordinateOffsetY = default(2000);
}
network ExternalNetwork {
    submodules:
        world: BaseWorldUtility {
            parameters:
                playgroundSizeX = 10000m;
                playgroundSizeY = 10000m;
                playgroundSizeZ = 1000m;
                use2D = false;
        }
        connectionManager: ConnectionManager;
        gateway: NetworkGateway;
}
