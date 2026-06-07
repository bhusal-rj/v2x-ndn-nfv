package ned;
import org.car2x.veins.nodes.Car;
import org.car2x.veins.nodes.RSU;
import org.car2x.veins.base.connectionManager.ConnectionManager;
import org.car2x.veins.base.modules.BaseWorldUtility;
import org.car2x.veins.modules.mobility.traci.TraCIScenarioManagerForker;
import org.car2x.veins.modules.obstacle.ObstacleControl;
import org.car2x.veins.modules.world.annotations.AnnotationManager;

network V2X
{
    parameters:
        // These parameters are required by BaseWorldUtility
        double playgroundSizeX @unit(m);
        double playgroundSizeY @unit(m);
        double playgroundSizeZ @unit(m);
        @display("bgb=$playgroundSizeX,$playgroundSizeY");

    submodules:
        // These environment modules are safe to initialize at t=0.
        obstacles: ObstacleControl {
            @display("p=500,100");
        }
        annotations: AnnotationManager {
            @display("p=610,320");
        }
        connectionManager: ConnectionManager {
            @display("p=610,400;i=abstract/multicast");
        }
        world: BaseWorldUtility {
            parameters:
                playgroundSizeX = veins_eval_by_version(veins_omnetpp_buildnum(), "playgroundSizeX", 1525, "parent.playgroundSizeX");
                playgroundSizeY = veins_eval_by_version(veins_omnetpp_buildnum(), "playgroundSizeY", 1525, "parent.playgroundSizeY");
                playgroundSizeZ = veins_eval_by_version(veins_omnetpp_buildnum(), "playgroundSizeZ", 1525, "parent.playgroundSizeZ");
                @display("p=500,280;i=misc/globe");
        }

        car[0]: Vehicle;
        
       timeSynchronizer: TimeSynchronizer {
            @display("i=block/app;p=610,128");
        }
        //  SocketInterface controller
        socketInterface: SocketInterface {
            @display("p=500,200;i=block/app");
        }
       
        
    connections allowunconnected:
        // No connections needed between these top-level modules
}
