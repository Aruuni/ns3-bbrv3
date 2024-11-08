#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/traffic-control-module.h"

#include <iostream>
#include <fstream>
#include <cstdio>
#include <iomanip>
#include <unordered_map>
#include <sys/stat.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

#define COUT(log) std::cout << log << std::endl;

using namespace ns3;
struct Change {
    double time;
    std::string type;
    double value;
};


struct Flow {
    std::string src;
    std::string dest;
    int start_time;
    int duration;
    std::string congestion_control;
};

struct ParsedData {
    std::vector<Flow> flows;
    std::vector<Change> changes;
};

ParsedData 
parseData(const std::string &file_path) {
    
    ParsedData parsedData;
    std::ifstream file(file_path);
    json data;
    file >> data;

    for (const auto &flow_data : data["flows"]) {
        Flow flow;
        flow.src = flow_data[0];
        flow.dest = flow_data[1];
        flow.start_time = flow_data[4];
        flow.duration = flow_data[5];
        flow.congestion_control = flow_data[6];
        if (flow_data[7].is_null()) {
            flow.congestion_control[0] = std::toupper(flow.congestion_control[0]);
            flow.congestion_control = "Tcp" + flow.congestion_control;
            parsedData.flows.push_back(flow);
        }
        else{
            if (flow.congestion_control == "tbf"){
                Change change;
                change.time = flow.start_time;
                change.type = "bw";
                change.value = flow_data[7][1];
                parsedData.changes.push_back(change);
            }
            else if (flow.congestion_control == "netem"){
                Change delay, loss;
                loss.time = delay.time = flow.start_time;
                loss.type = "loss";
                delay.type = "delay";
                delay.value = flow_data[7][2];
                if (flow_data[7][6].is_null())
                    loss.value = 0;
                else
                    loss.value = flow_data[7][6];
                parsedData.changes.push_back(delay);
                parsedData.changes.push_back(loss);
            }
        }
    }
    return parsedData;
}
ParsedData traffic_config;


// build ns 3
// ./ns3 clean
// ./ns3 configure --build-profile=optimized 
// ./ns3 run "scratch/CCTestBed.cc --configJSON=/home/mihai/Desktop/emulation_info.json"
// ./ns3 run "scratch/CCTestBed.cc --configJSON=/home/mihai/Desktop/emulation_info.json --path=scratch/test/"
AsciiTraceHelper ascii;
std::unordered_map<std::string, std::vector<std::string>> files;

//simulation paramaters
std::string outpath;
int PORT = 50001;
uint packetSize = 1500;

static void
socketTrace(uint32_t idx, std::string varName, std::string path, auto callback)
{
    files.insert(std::make_pair(varName, std::vector<std::string>()));
    Ptr<OutputStreamWrapper> fstream = ascii.CreateFileStream(outpath + traffic_config.flows[idx].congestion_control + std::string(" ") + std::to_string(idx+1) + "-" + varName +".csv");
    *fstream->GetStream() << "time," << varName << "\n";
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(idx) + 
                                "/$ns3::TcpL4Protocol/SocketList/0/" + path, 
                                MakeBoundCallback(callback,fstream));
    files[varName].push_back(outpath + traffic_config.flows[idx].congestion_control + std::string(" ") + std::to_string(idx+1) + "-" + varName + ".csv");
}

static void
uint32Tracer(Ptr<OutputStreamWrapper> stream, uint32_t, uint32_t newval)
{
    if (newval == 2147483647){
            *stream->GetStream() 
        << Simulator::Now().GetSeconds() 
        << ", " 
        << 0 
        << std::endl;
        return;
    }

    *stream->GetStream() 
        << Simulator::Now().GetSeconds() 
        << ", " 
        << newval 
        << std::endl;
}

// static void
// DataRateTracer(Ptr<OutputStreamWrapper> stream, DataRate, DataRate newval)
// {
//     *stream->GetStream() 
//         << Simulator::Now().GetSeconds() 
//         << ", " << newval.GetBitRate() 
//         << std::endl;
// }

static void
TimeTracer(Ptr<OutputStreamWrapper> stream, Time, Time newval)
{
    *stream->GetStream() 
        << Simulator::Now().GetSeconds() 
        << ", " 
        << newval.GetMilliSeconds() 
        << std::endl;
}

static void
TraceThroughput(Ptr<FlowMonitor> monitor, Ptr<OutputStreamWrapper> stream, uint32_t flowID, uint32_t prevTxBytes, Time prevTime) 
{
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
    FlowMonitor::FlowStats statsNode = stats[flowID];
    *stream->GetStream() 
        << Simulator::Now().GetSeconds() 
        << ", "
        << 8 * (statsNode.txBytes - prevTxBytes) / (1000000 * (Simulator::Now().GetSeconds() - prevTime.GetSeconds()))
        //<< 8 * (statsNode.txBytes - prevTxBytes) / ((Simulator::Now().GetSeconds() - prevTime.GetSeconds()))
        << std::endl;
    Simulator::Schedule(Seconds(0.1), &TraceThroughput, monitor, stream, flowID, statsNode.txBytes, Simulator::Now());
}

std::vector<double> rxBytes;

void ReceivedPacket(uint32_t flowID, Ptr<const Packet> p, const Address& addr)
{
	rxBytes[flowID] += p->GetSize();
}

static void
TraceGoodput(Ptr<OutputStreamWrapper> stream, uint32_t flowID, uint32_t prevRxBytes, Time prevTime)
{
    *stream->GetStream() 
        << Simulator::Now().GetSeconds() 
        << ", "
        << 8 * (rxBytes[flowID] - prevRxBytes) / (1000000 * (Simulator::Now().GetSeconds() - prevTime.GetSeconds()))
        << std::endl;
    Simulator::Schedule(Seconds(1), &TraceGoodput, stream,  flowID, rxBytes[flowID], Simulator::Now());
}

void
QueueSizeTrace(uint32_t nodeID, uint32_t deviceID)
{
    Ptr<OutputStreamWrapper> qtrace = ascii.CreateFileStream(outpath + "queueSize.csv");
    *qtrace->GetStream() << "time,root_pkts" << "\n";
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(nodeID) + 
                                  "/DeviceList/" + std::to_string(deviceID) + 
                                  "/$ns3::PointToPointNetDevice/TxQueue/PacketsInQueue", 
                                  MakeBoundCallback(&uint32Tracer, qtrace));
}

//commented to supress a warning for debug mode, they do work 
static void
delay_change(NetDeviceContainer dev, double delay) 
{
    dev.Get(0)->GetChannel()->GetObject<PointToPointChannel>()->SetAttribute("Delay", StringValue(std::to_string(delay)+"ms"));
}

static void
delay_changer(NetDeviceContainer dev, uint32_t time, double delay) 
{
    Simulator::Schedule(Seconds(time), &delay_change, dev, delay);
}

static void
data_rate_change(NetDeviceContainer dev, uint32_t datarate) 
{
    Config::Set("/NodeList/" + std::to_string(traffic_config.flows.size()) + 
                "/DeviceList/0" + 
                "/$ns3::PointToPointNetDevice/DataRate", StringValue(std::to_string(datarate) + "Mbps") );
    Config::Set("/NodeList/" + std::to_string(traffic_config.flows.size()+1) + 
                "/DeviceList/0" + 
                "/$ns3::PointToPointNetDevice/DataRate", StringValue(std::to_string(datarate) + "Mbps") );
}

static void
data_rate_changer(NetDeviceContainer dev, double time, uint32_t datarate) 
{
    Simulator::Schedule(Seconds(time), &data_rate_change, dev, datarate);
}

static void
error_change(NetDeviceContainer dev, double errorrate) 
{
    Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
    em->SetAttribute("ErrorRate", DoubleValue(errorrate));
    dev.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(em));;
}
static void
error_changer(NetDeviceContainer dev, uint32_t time, double errorrate) 
{
    Simulator::Schedule(Seconds(time), &error_change, dev, errorrate);
}

int 
main(int argc, char* argv[])
{
    //./ns3 run "scratch/CCTestBed.cc --configJSON=/home/mihai/Desktop/emulation_info.json --path=test/"
    
    int seed{123456789};
    int startTime{0};
    double bdpMultiplier{1};
    int bottleneck_bw{10};
    int bottleneck_delay{5};
    //int inter_bottleneck_delay{0};
    
    CommandLine cmd(__FILE__);
    cmd.Usage("CommandLine example program.\n"
              "\n"
              "This little program demonstrates how to use CommandLine.");
    
    std::string json_file;

    cmd.AddValue("configJSON", "json cofig file", json_file);
    cmd.AddValue("path", "output directory", outpath);
    cmd.AddValue("delay", "delay in ms", bottleneck_delay);
    cmd.AddValue("bandwidth", "bandwidth in mbps", bottleneck_bw);
    cmd.AddValue("bdp", "multiple of bdp for queues", bdpMultiplier);
    cmd.AddValue("seed", "append a flow", seed);
    cmd.AddValue("interRTT", "the base flows rtt ", seed);
    
    cmd.Parse (argc, argv);

    traffic_config = parseData(json_file);
    for (const auto& flow : traffic_config.flows) {
        if (flow.duration >= startTime){
            startTime = flow.duration;
        }
    }
     
    outpath = outpath + "/";
    system(("mkdir -p "+ outpath).c_str());
    Time stopTime = Seconds(startTime);
    
    SeedManager::SetSeed(seed);
    // linux default send 4096   16384   4194304
    // linux default recv 4096   131072  6291456
    Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(4194304));
    Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(4194304));
    Config::SetDefault("ns3::TcpSocket::InitialCwnd", UintegerValue(10)); 
    Config::SetDefault("ns3::TcpSocket::InitialSlowStartThreshold", UintegerValue(10)); 
    Config::SetDefault("ns3::TcpSocket::DelAckCount", UintegerValue(1));
    Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(packetSize));
    Config::SetDefault("ns3::TcpSocketState::EnablePacing", BooleanValue(true));
    Config::SetDefault("ns3::TcpL4Protocol::RecoveryType", TypeIdValue(TypeId::LookupByName("ns3::TcpClassicRecovery")));    
    Config::SetDefault("ns3::TcpSocketBase::Sack", BooleanValue(true)); 

    NodeContainer senders, receivers, routers;
    senders.Create(traffic_config.flows.size());
    receivers.Create(traffic_config.flows.size());
    routers.Create(2);
    
    PointToPointHelper botLink, p2pLinkLeft, p2pLinkRight;
    // bottleneck link
    botLink.SetDeviceAttribute("DataRate", StringValue(std::to_string(bottleneck_bw) + "Mbps"));
    botLink.SetChannelAttribute("Delay", StringValue("0ms"));
    botLink.SetQueue("ns3::DropTailQueue", "MaxSize", QueueSizeValue(QueueSize(std::to_string((((bottleneck_bw * 1000 ) * (bottleneck_delay*2) / packetSize) * bdpMultiplier ) / 8 ) + "p")));

    // edge link 
    p2pLinkLeft.SetDeviceAttribute("DataRate", StringValue("10Gbps"));
    p2pLinkLeft.SetQueue("ns3::DropTailQueue", "MaxSize", QueueSizeValue(QueueSize(std::to_string((((bottleneck_bw * 1000) * (bottleneck_delay*2) / packetSize) * bdpMultiplier) / 8 ) + "p")));

    p2pLinkRight.SetDeviceAttribute("DataRate", StringValue("10Gbps"));
    p2pLinkRight.SetChannelAttribute("Delay", StringValue("0ms"));
    p2pLinkRight.SetQueue("ns3::DropTailQueue", "MaxSize", QueueSizeValue(QueueSize(std::to_string((((bottleneck_bw * 1000) * (bottleneck_delay*2) / packetSize) * bdpMultiplier) / 8 ) + "p")));
    //p2pLinkRight.SetQueue("ns3::DropTailQueue", "MaxSize", QueueSizeValue(QueueSize("2p")));

    NetDeviceContainer routerDevices = botLink.Install(routers);
    NetDeviceContainer senderDevices, receiverDevices, leftRouterDevices, rightRouterDevices;
    for(uint32_t i = 0; i < senders.GetN(); i++) {
        p2pLinkLeft.SetChannelAttribute("Delay", StringValue(std::to_string(bottleneck_delay) + "ms"));


		NetDeviceContainer cleft = p2pLinkLeft.Install(routers.Get(0), senders.Get(i));
		leftRouterDevices.Add(cleft.Get(0));
		senderDevices.Add(cleft.Get(1));

		NetDeviceContainer cright = p2pLinkRight.Install(routers.Get(1), receivers.Get(i));
		rightRouterDevices.Add(cright.Get(0));
		receiverDevices.Add(cright.Get(1));
	}
    
    InternetStackHelper internet;
    internet.Install(senders);
    internet.Install(receivers);
    internet.Install(routers);
    // sets the congestion control algo 
    for (uint32_t i = 0; i < senders.GetN(); i++) { 
        Config::Set("/NodeList/" + std::to_string(i) + "/$ns3::TcpL4Protocol/SocketType", TypeIdValue(TypeId::LookupByName("ns3::" + traffic_config.flows[i].congestion_control))); 
    }

    TrafficControlHelper tch;
    tch.SetRootQueueDisc("ns3::FifoQueueDisc");
    tch.Install(senderDevices);
    tch.Install(receiverDevices);
    tch.Install(leftRouterDevices);
    tch.Install(rightRouterDevices);
    
    // node n*2 device 0 has the queue where the bottleneck will occur  
    QueueSizeTrace(senders.GetN()*2,0);

	Ipv4AddressHelper routerIP = Ipv4AddressHelper("10.3.0.0", "255.255.255.0");
	Ipv4AddressHelper senderIP = Ipv4AddressHelper("10.1.0.0", "255.255.255.0");
	Ipv4AddressHelper receiverIP = Ipv4AddressHelper("10.2.0.0", "255.255.255.0");

    Ipv4InterfaceContainer routerIFC, senderIFCs, receiverIFCs, leftRouterIFCs, rightRouterIFCs;
    routerIFC = routerIP.Assign(routerDevices); 

    for(uint32_t i = 0; i < senders.GetN(); i++) {
		NetDeviceContainer senderDevice;
		senderDevice.Add(senderDevices.Get(i));
		senderDevice.Add(leftRouterDevices.Get(i));
		
        Ipv4InterfaceContainer senderIFC = senderIP.Assign(senderDevice);
		senderIFCs.Add(senderIFC.Get(0));
		leftRouterIFCs.Add(senderIFC.Get(1));
        senderIP.NewNetwork();

		NetDeviceContainer receiverDevice;
		receiverDevice.Add(receiverDevices.Get(i));
		receiverDevice.Add(rightRouterDevices.Get(i));
		
        Ipv4InterfaceContainer receiverIFC = receiverIP.Assign(receiverDevice);
		receiverIFCs.Add(receiverIFC.Get(0));
		rightRouterIFCs.Add(receiverIFC.Get(1));
		receiverIP.NewNetwork();
	}

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();
    ApplicationContainer senderApp, receiverApp;
    // variable tracing / installing the apps
    for (uint32_t i = 0; i < senders.GetN(); i++) {
        BulkSendHelper sender("ns3::TcpSocketFactory", InetSocketAddress(receiverIFCs.GetAddress(i), PORT));
        sender.SetAttribute("MaxBytes", UintegerValue(0)); // Unlimited data
        senderApp.Add(sender.Install(senders.Get(i)));
        senderApp.Get(i)->SetStartTime(Seconds(traffic_config.flows[i].start_time)); 
        Simulator::Schedule(Seconds(traffic_config.flows[i].start_time) + MilliSeconds(1), &socketTrace<decltype(&uint32Tracer)>,  senders.Get(i)->GetId(), "bytes", "BytesInFlight",  &uint32Tracer);
        Simulator::Schedule(Seconds(traffic_config.flows[i].start_time) + MilliSeconds(1), &socketTrace<decltype(&uint32Tracer)>,  senders.Get(i)->GetId(), "cwnd", "CongestionWindow", &uint32Tracer);
        Simulator::Schedule(Seconds(traffic_config.flows[i].start_time) + MilliSeconds(1), &socketTrace<decltype(&TimeTracer)>,  senders.Get(i)->GetId(), "rtt", "RTT",  &TimeTracer);
    }
    senderApp.Stop(stopTime);

    // Create receiver applications on each receiver node
    for (uint32_t i = 0; i < receivers.GetN(); i++) {
        PacketSinkHelper sink("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), PORT));
        receiverApp.Add(sink.Install(receivers.Get(i)));
        Config::ConnectWithoutContext("/NodeList/" + std::to_string(senders.GetN() + i) + "/ApplicationList/0/$ns3::PacketSink/Rx", MakeBoundCallback(&ReceivedPacket, i));
        
    }
    receiverApp.Start(Seconds(0.1));
    receiverApp.Stop(stopTime);

    for (const auto& change : traffic_config.changes) {
        if (change.type == "bw")
            data_rate_changer(senderDevices, change.time, (uint32_t)change.value);
        if (change.type == "delay")
            delay_changer(senderDevices, change.time, (double)change.value/2);
        if (change.type == "loss")
            error_changer(senderDevices, change.time, change.value);
    }
     
    
    FlowMonitorHelper flowmonHelperSender;
    for (uint32_t i = 0; i < senders.GetN(); i++) {
        Ptr<FlowMonitor> flowMonitorS = flowmonHelperSender.Install(senders.Get(i));     
        rxBytes.push_back(0);
        Ptr<OutputStreamWrapper> th_stream = ascii.CreateFileStream(outpath + traffic_config.flows[i].congestion_control + std::to_string(i+1) + "-throughput.csv");
        *th_stream->GetStream() << "time,throughput" << "\n";
        Simulator::Schedule(Seconds(0.1) + MilliSeconds(1) + Seconds(traffic_config.flows[i].start_time), &TraceThroughput, flowMonitorS, th_stream, i+1, 0, Seconds(0));
        files["throughtput"].push_back(outpath + traffic_config.flows[i].congestion_control + std::to_string(i+1) + "-throughput.csv");
        
        Ptr<OutputStreamWrapper> gp_stream = ascii.CreateFileStream(outpath + traffic_config.flows[i].congestion_control + std::to_string(i+1) + "-goodput.csv");
        *gp_stream->GetStream() << "time,goodput" << "\n";
        Simulator::Schedule(Seconds(1) + MilliSeconds(1) + Seconds(traffic_config.flows[i].start_time), &TraceGoodput, gp_stream, i, 0, Seconds(0));
        files["goodput"].push_back(outpath + traffic_config.flows[i].congestion_control + std::to_string(i+1) + "-goodput.csv");
    }
    
    Ptr<FlowMonitor> flowMonitor;
    FlowMonitorHelper flowmonHelper;
    flowMonitor = flowmonHelper.InstallAll();



    Simulator::Stop(stopTime + TimeStep(1));
    Simulator::Run();
    exit(0);
}