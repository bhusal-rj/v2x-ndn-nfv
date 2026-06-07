package ned;
simple MetricsCollector
{
    parameters:
        double collectionInterval @unit(s) = default(1.0s);
        string nfv_infra = default("metrics/nfv_infrastructure.csv");
        string nfv_service = default("metrics/nfv_service_flows.csv");
        string traffic = default("metrics/traffic_context.csv");
        string socket_sent = default("metrics/socket_throughput_sent.csv");
        string socket_recv = default("metrics/socket_throughput_recv.csv");
        @display("i=block/graph");
    
}
