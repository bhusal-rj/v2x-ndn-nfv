package ned;
simple CentralRSUManager
{
    parameters:
        // Core configuration
        double updateInterval @unit(s) = default(2s); // How often to check density
        int deploymentThreshold = default(3);       // Vehicles in range required for new RSU
        int rsuSpacing=default(300); //spacing for infill rsus
        int minSpacingFromJunctionRsus=default(100); //minimum spacing from junction rsu for inFill rsus
        int rsuCoverage=default(310);   //range of rsu coverage
        int removalThreshold = default(1);           // Vehicles in range threshold for RSU removal
        bool enableDynamicRsu=default(true);
        double hysteresisTime @unit(s) = default(30s); // Time low-density must persist before removal
        string rsuModuleType= default("ned.RSU");
}
