package ned;
import org.car2x.veins.base.modules.BaseMobility;

module RSU
{
    parameters:
        string applType; //type of the application layer
    submodules:
        appl: <applType> like V2XNODE {
            parameters:
                @display("p=60,50");
        }
        mobility: BaseMobility {
            parameters:
                @display("p=130,172;i=block/cogwheel");
        }
        
   
}