package ned;
import org.car2x.veins.modules.mobility.traci.TraCIMobility;

module Vehicle
{
    parameters:
        string applType; //type of the application layer
        string veinsmobilityType = default("org.car2x.veins.modules.mobility.traci.TraCIMobility"); //type of the mobility module
    submodules:
        appl: <applType> like V2XNODE {
            parameters:
                @display("p=60,50");
        }

        veinsmobility: <veinsmobilityType> like org.car2x.veins.base.modules.IMobility {
            parameters:
                @display("p=130,172;i=block/cogwheel");
        }
     
}