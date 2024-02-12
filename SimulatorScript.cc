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

#define COUT(log) std::cout << log << std::endl

using namespace ns3;


std::vector<std::string> cca = { "TcpBbr3" }; // 



std::vector<std::vector<std::string>> colors = {
{
    "#00FF0000", // Blue
    "#00FF00FF", // Green
    "#00FFA500", // Orange
    "#00FF0000", // Red
    "#00800080", // Purple
    "#00A52A2A", // Brown
    "#00000000", // Black
    "#00FFFF00", // Yellow
    "#00FFFFFF", // Cyan
    "#00FF00FF", // Magenta
    "#00808080"  // Gray
}, {
    "#00006400", // Dark Green
    "#008B0000", // Dark Red
    "#00483D8B", // Dark Purple
    "#008B4513", // Dark Brown
    "#00A9A9A9", // Dark Gray
    "#00ADD8E6", // Light Blue
    "#00FFFFE0", // Light Yellow
    "#00E0FFFF", // Light Cyan
    "#00FFC0CB", // Light Magenta
    "#00FFA500"  // Light Orange 
    }, {
    "#000000FF", // Dark Green
    "#008B0000", // Dark Red
    "#00483D8B", // Dark Purple
    "#008B4513", // Dark Brown
    "#00A9A9A9", // Dark Gray
    "#00ADD8E6", // Light Blue
    "#00FFFFE0", // Light Yellow
    "#00E0FFFF", // Light Cyan
    "#00FFC0CB", // Light Magenta
    "#00FFA500"  // Light Orange 
    } , {
    "#00FFA500", // Dark Green
    "#008B0000", // Dark Red
    "#00483D8B", // Dark Purple
    "#008B4513", // Dark Brown
    "#00A9A9A9", // Dark Gray
    "#00ADD8E6", // Light Blue
    "#00FFFFE0", // Light Yellow
    "#00E0FFFF", // Light Cyan
    "#00FFC0CB", // Light Magenta
    "#00FFA500"  // Light Orange 
    }
};

// ./ns3 clean
// ./ns3 configure --build-profile=optimized 
// ./ns3 run scratch/SimulatorScript.cc 
AsciiTraceHelper ascii;
std::unordered_map<std::string, std::vector<std::string>> files;

//simulation paramaters

//std::vector<std::string> cca = { "TcpBbr"  };
//, TcpCubic TcpBbr
std::string outpath;
double startTime = 0.1; // in seconds

int PORT = 50001;

uint packetSize = 1460;

///////  LOGGING ////////
bool cleanup = false;
bool plotScriptOut = false;
bool progressLog = true;


static void
socketTrace(uint32_t idx, std::string varName, std::string path, auto callback)
{
    files.insert(std::make_pair(varName, std::vector<std::string>()));
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(idx) + 
                                "/$ns3::TcpL4Protocol/SocketList/0/" + path, 
                                MakeBoundCallback(callback, ascii.CreateFileStream(outpath + cca[idx] + std::to_string(idx) + "-" + varName +".csv")));
    files[varName].push_back(outpath + cca[idx] + std::to_string(idx) + "-" + varName + ".csv");
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

static void
DataRateTracer(Ptr<OutputStreamWrapper> stream, DataRate, DataRate newval)
{
    *stream->GetStream() 
        << Simulator::Now().GetSeconds() 
        << ", " << newval.GetBitRate() 
        << std::endl;
}

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
        //        << 8 * (statsNode.txBytes - prevTxBytes) / (1000000 * (Simulator::Now().GetSeconds() - prevTime.GetSeconds()))
        << 8 * (statsNode.txBytes - prevTxBytes) / ((Simulator::Now().GetSeconds() - prevTime.GetSeconds()))
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
        << 8 * (rxBytes[flowID] - prevRxBytes) / ((Simulator::Now().GetSeconds() - prevTime.GetSeconds()))
        << std::endl;
    Simulator::Schedule(Seconds(0.1), &TraceGoodput, stream,  flowID, rxBytes[flowID], Simulator::Now());
}

// static void 
// TraceGoodPutRatio(Ptr<OutputStreamWrapper> stream, uint32_t flowID, uint32_t prevRxBytes, uint32_t prevTotalRxBytes, Time prevTime)
// {
//     *stream->GetStream() 
//         << Simulator::Now().GetSeconds() 
//         << ", "
//         << 8 * (rxBytes[flowID] - prevRxBytes) / ((Simulator::Now().GetSeconds() - prevTime.GetSeconds()))
//         << std::endl;
//     Simulator::Schedule(Seconds(0.1), &TraceGoodPutRatio, stream,  flowID, rxBytes[flowID], Simulator::Now());
// }

void
QueueSizeTrace(uint32_t nodeID, uint32_t deviceID)
{
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(nodeID) + 
                                  "/DeviceList/" + std::to_string(deviceID) + 
                                  "/$ns3::PointToPointNetDevice/TxQueue/PacketsInQueue", 
                                  MakeBoundCallback(&uint32Tracer, ascii.CreateFileStream(outpath + "queueSize.csv")));
}

void
generatePlot(std::vector<std::vector<std::string>> fileNames, std::string plotTitle, std::string plotYLabel, std::string outPath)
{
    FILE *gnuplotPipe = popen("gnuplot -persist", "w");
    if (gnuplotPipe) {
        fprintf(gnuplotPipe, "set terminal pdf enhanced color dashed lw 1 font 'DejaVuSans,12'\n");
        fprintf(gnuplotPipe, "set style line 81 lt 0\n");
        fprintf(gnuplotPipe, "set style line 81 lt rgb \"#aaaaaa\"\n");
        fprintf(gnuplotPipe, "set grid back linestyle 81\n");
        fprintf(gnuplotPipe, "set border 3 back linestyle 80\n");
        fprintf(gnuplotPipe, "set xtics nomirror\n");
        fprintf(gnuplotPipe, "set ytics nomirror\n");
        fprintf(gnuplotPipe, "set autoscale x\n");
        fprintf(gnuplotPipe, "set autoscale y\n");
        fprintf(gnuplotPipe, "set output \"%s.pdf\"\n", (outPath + plotTitle).c_str());
        fprintf(gnuplotPipe, "set title \"%s\"\n", plotTitle.c_str());
        fprintf(gnuplotPipe, "set xlabel \"Time (sec)\"\n");
        fprintf(gnuplotPipe, "set ylabel \"%s\"\n", plotYLabel.c_str());
        fprintf(gnuplotPipe, "set key right top vertical\n");
        
        std::string plotCommand = "plot ";
        int j = 0;
        for (const auto&  plot : fileNames ){
            for (uint32_t i = 0; i < plot.size(); i++) {
                // in the future repalace 6 with the actual path length
                plotCommand += "\"" + plot[i] + "\" title \"" + plot[i].substr(outpath.length(), plot[i].find('.') -outpath.length()) + "\" with steps lw 0.7 lc '" + colors[j][i] + "'";
                plotCommand += " , ";
                //if (i != plot.size() - 1)
                   // plotCommand += ", ";
            }
            j++;
        }
        fprintf(gnuplotPipe, "%s", plotCommand.c_str());
        fflush(gnuplotPipe);
    } else {
        std::cerr << "Error opening gnuplot pipe." << std::endl;
    }
    pclose(gnuplotPipe);
}

// static void
// ChangeDelay(NetDeviceContainer dev, uint32_t delay) 
// {
//     dev.Get(0)->GetChannel()->GetObject<PointToPointChannel>()->SetAttribute("Delay", StringValue(std::to_string(delay)+"ms"));
// }

// static void
// DelayChanger(NetDeviceContainer dev, uint32_t time, uint32_t delay) 
// {
//     Simulator::Schedule(Seconds(time), &ChangeDelay, dev, delay);
// }

// static void
// ChangeDataRate(NetDeviceContainer dev, uint32_t datarate) 
// {
//     Config::Set("/NodeList/" + std::to_string(cca.size()) + 
//                 "/DeviceList/0" + 
//                 "/$ns3::PointToPointNetDevice/DataRate", StringValue(std::to_string(datarate) + "Mbps") );
//     Config::Set("/NodeList/" + std::to_string(cca.size()+1) + 
//                 "/DeviceList/0" + 
//                 "/$ns3::PointToPointNetDevice/DataRate", StringValue(std::to_string(datarate) + "Mbps") );
// }

// static void
// DataRateChanger(NetDeviceContainer dev, double time, uint32_t datarate) 
// {
//     Simulator::Schedule(Seconds(time), &ChangeDataRate, dev, datarate);
// }

// static void
// ErrorChanger(NetDeviceContainer dev, double errorrate) 
// {
//     Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
//     em->SetAttribute("ErrorRate", DoubleValue(errorrate));
//     dev.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(em));;
// }


void 
progress(Time stop){
    uint8_t barWidth = 50;
    std::cout << "\033[2K";
    std::cout << "\033[A"; 
    std::cout.flush(); 
    std::cout << "Simulation progress: [";
    
    int progressMade = ((double)barWidth / 100) * ((Simulator::Now().GetSeconds() / stop.GetSeconds())*100);

    for (int i = 0; i < barWidth; ++i) {
        if (i == barWidth/2)
            std::cout << std::fixed << std::setprecision(2) << ((Simulator::Now().GetSeconds() / stop.GetSeconds())*100) << "%";
        if (i < progressMade) {
            std::cout << "|";
        } else if (i == progressMade) {
            std::cout << ">";
        } else {
            std::cout << " ";
        }
    }
    std::cout << "] " << std::endl;
    Simulator::Schedule(Seconds(0.1), &progress, stop);
}

int 
main(
    int argc, 
    char* argv[]
    )
{
    //std::vector<std::string> cca = { "TcpBbr" , "TcpCubic"}; // 
    int seed{1};
    double bdpMultiplier{1};
    int startTimeInt{50};
    double flowStartOffset{0}; // in seconds // WATCH OUT FOR THIS BEING LOWER THEN THE SIMUATLION END TIME
    
    
    int bottleneckLinkDataRate{10};
    int bottleneckLinkDelay{5};
    
    int p2pLinkDelay{10}; // this is x2 for left and right
    int p2pLinkOffset{0}; // this is x2 
    int p2pLinkOffsetNFlows{0};

    CommandLine cmd(__FILE__);
    cmd.Usage("CommandLine example program.\n"
              "\n"
              "This little program demonstrates how to use CommandLine.");
    
    std::string flows;
    std::string flowToAppend;

    cmd.AddValue("botLinkDataRate", "datarate  of the bottleneck link in mbps", bottleneckLinkDataRate);
    cmd.AddValue("botLinkDelay", "delay of the bottleneck link in ms", bottleneckLinkDelay);
    cmd.AddValue("p2pLinkDelay", "delay of the other links, from senders to middle routers in ms, this is x2 in the dumbell topology", p2pLinkDelay);
    cmd.AddValue("p2pLinkOffsetDelay", "an int argument", p2pLinkOffset); // finish this not working
    cmd.AddValue("p2pLinkOffsetNFlows", "an int argument", p2pLinkOffsetNFlows); // finish this not working
    cmd.AddValue("flowStartOffset", "an int argument", flowStartOffset);
    cmd.AddValue("stopTime", "Duration of the experiment in seconds", startTimeInt);
    cmd.AddValue("flows", "caa of flows", flows); // parse comma separated  
    cmd.AddValue("path", "path of current experiment", outpath);
    cmd.AddValue("queueBDP", "multipel of bdp for queues", bdpMultiplier);
    cmd.AddValue("appendFlow", "append a flow", flowToAppend);
    cmd.AddValue("seed", "append a flow", seed);


    
    //cmd.AddValue("stopTime", "an int argument", startTimeInt);
    
    //cmd.AddValue("boolArg", "a bool argument", boolArg);
    //cmd.AddValue("strArg", "a string argument", strArg);
    cmd.Parse (argc, argv);
    cca.push_back(flowToAppend);
    outpath = "scratch/" + outpath + "/";
    system(("mkdir -p "+ outpath).c_str());
    system(("mkdir -p "+ outpath + "pdf/").c_str());
    Time stopTime = Seconds(startTimeInt);
    int p2pLinkDataRate = bottleneckLinkDataRate * 10;
    //if (progressLog)
    Simulator::Schedule(Seconds(0), &progress, stopTime);
    SeedManager::SetSeed(seed);
    // linux default send 4096   16384   4194304
    // linux default recv 4096   131072  6291456
    Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(4194304));
    //Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(16384));
    Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(4194304));
    //Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(131072));
    Config::SetDefault("ns3::TcpSocket::InitialCwnd", UintegerValue(10)); 
    Config::SetDefault("ns3::TcpSocket::InitialSlowStartThreshold", UintegerValue(10)); 
    Config::SetDefault("ns3::TcpSocket::DelAckCount", UintegerValue(2));
    //  Config::SetDefault("ns3::TcpSocket::DelAckTimeout", TimeValue(Seconds(0)));
    Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(packetSize));
    //Config::SetDefault("ns3::TcpSocketBase::WindowScaling", BooleanValue(true));
    Config::SetDefault("ns3::TcpSocketState::EnablePacing", BooleanValue(true));
    // Config::SetDefault("ns3::DropTailQueue<Packet>::MaxSize", QueueSizeValue (QueueSizeValue (QueueSize(std::to_string(((bottleneckLinkDataRate * 1000) * bottleneckLinkDelay) / packetSize * bdpMultiplier) + "p"))));
    // std::cout << "bn rate: " << std::to_string(bottleneckLinkDataRate) << "bn delay: " << std::to_string(bottleneckLinkDelay) << "packet size: " << std::to_string(packetSize) << "bdp multiplier: " << std::to_string(bdpMultiplier) << std::endl;
    // std::cout << "Bottleneck link queue ............. >  " << std::to_string((bottleneckLinkDataRate * bottleneckLinkDelay / packetSize) * bdpMultiplier) + "p" << std::endl;
    // Config::SetDefault("ns3::FifoQueueDisc::MaxSize", QueueSizeValue (QueueSize(std::to_string(((bottleneckLinkDataRate * 1000) * bottleneckLinkDelay) / packetSize * bdpMultiplier) + "p")));
    // Config::SetDefault("ns3::DropTailQueue<Packet>::MaxSize", QueueSizeValue (QueueSize ("34p")));
    Config::SetDefault("ns3::TcpL4Protocol::RecoveryType", TypeIdValue(TypeId::LookupByName("ns3::TcpClassicRecovery")));
    // Config::SetDefault("ns3::FifoQueueDisc::MaxSize", QueueSizeValue (QueueSize ("34p")));
    //Config::SetDefault("ns3::TcpL4Protocol::RecoveryType", TypeIdValue(TypeId::LookupByName(recovery)));
    

    //Config::SetDefault("ns3::TcpSocketBase::MinRto", TimeValue(Seconds(10)));
    Config::SetDefault("ns3::TcpSocketBase::Sack", BooleanValue(true)); // will be turned on automatically by the rack

    //EXPERIMENTAL WILL NOT WORK WITH VANILLA NS3 
    //bool exp = false;
    // Config::SetDefault ("ns3::TcpSocketBase::Rack", BooleanValue(exp));
    // Config::SetDefault ("ns3::TcpSocketBase::Fack", BooleanValue(exp));
    // Config::SetDefault ("ns3::TcpSocketBase::Dsack", BooleanValue(exp));


    NodeContainer senders, receivers, routers;
    senders.Create(cca.size());
    receivers.Create(cca.size());
    routers.Create(2);
    
    PointToPointHelper botLink, p2pLinkLeft, p2pLinkRight;

    botLink.SetDeviceAttribute("DataRate", StringValue(std::to_string(bottleneckLinkDataRate) + "Mbps"));
    botLink.SetChannelAttribute("Delay", StringValue(std::to_string(bottleneckLinkDelay) + "ms"));
    botLink.SetQueue("ns3::DropTailQueue", "MaxSize", QueueSizeValue(QueueSize(std::to_string(((bottleneckLinkDataRate * 1000) * bottleneckLinkDelay / packetSize) * bdpMultiplier) + "p")));
    std::cout << "Bottleneck link queue ............. >  " << std::to_string(((bottleneckLinkDataRate * 1000 ) * bottleneckLinkDelay / packetSize) * bdpMultiplier) + "p" << std::endl;

    
    p2pLinkLeft.SetDeviceAttribute("DataRate", StringValue(std::to_string(p2pLinkDataRate) + "Mbps"));

    p2pLinkRight.SetDeviceAttribute("DataRate", StringValue(std::to_string(p2pLinkDataRate) + "Mbps"));
    p2pLinkRight.SetChannelAttribute("Delay", StringValue(std::to_string(p2pLinkDelay) + "ms"));
    p2pLinkRight.SetQueue("ns3::DropTailQueue", "MaxSize",  QueueSizeValue(QueueSize(std::to_string(((p2pLinkDataRate * 1000) * p2pLinkDelay / packetSize) * bdpMultiplier) + "p")));
    //p2pLinkRight.SetQueue("ns3::DropTailQueue", "MaxSize", QueueSizeValue(QueueSize("2p")));

    NetDeviceContainer routerDevices = botLink.Install(routers);

    Ptr<RateErrorModel> em = CreateObject<RateErrorModel>();
    em->SetAttribute("ErrorRate", DoubleValue(0.000001));
    //routerDevices.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(em));
    //packetDropTracer(routerDevices.Get(1)->GetNode()->GetId());    
    //Simulator::Schedule(Seconds(6), &ErrorChanger, routerDevices);


    NetDeviceContainer senderDevices, receiverDevices, leftRouterDevices, rightRouterDevices;
    
    for(uint32_t i = 0; i < senders.GetN(); i++) {
        p2pLinkLeft.SetQueue("ns3::DropTailQueue", "MaxSize",  QueueSizeValue(QueueSize(std::to_string(((p2pLinkDataRate * 1000) * (p2pLinkDelay + (p2pLinkOffset * i)) / packetSize) * bdpMultiplier) + "p")));
        
        //if ( i > (senders.GetN() /2 - 1))
        //    p2pLinkLeft.SetChannelAttribute("Delay", StringValue(std::to_string(p2pLinkDelay + (p2pLinkOffset )) + "ms"));

        p2pLinkLeft.SetChannelAttribute("Delay", StringValue(std::to_string(p2pLinkDelay + (p2pLinkOffset * i)) + "ms"));


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
    for (uint32_t i = 0; i < senders.GetN(); i++) { 
        Config::Set("/NodeList/" + std::to_string(i) + "/$ns3::TcpL4Protocol/SocketType", TypeIdValue(TypeId::LookupByName("ns3::" + cca[i]))); 
    }

    TrafficControlHelper tch;
    //tch.SetRootQueueDisc("ns3::FifoQueueDisc");
    //tch.SetRootQueueDisc("ns3::FifoQueueDisc", "MaxSize", StringValue ("34p"));
    tch.SetRootQueueDisc("ns3::TbfQueueDisc", 
        "Burst", UintegerValue(1600), 
        "Mtu", UintegerValue(packetSize),
        "Rate", DataRateValue(DataRate(std::to_string(bottleneckLinkDataRate) + "Mbps")), 
        "PeakRate", DataRateValue(DataRate(0)) 
    );

    tch.SetQueueLimits("ns3::DynamicQueueLimits", "HoldTime", StringValue("100ms"));
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
    // std::unordered_map<std::string, std::vector<std::string>> files;

    ApplicationContainer senderApp, receiverApp;


    for (uint32_t i = 0; i < senders.GetN(); i++) {
        
        BulkSendHelper sender("ns3::TcpSocketFactory", InetSocketAddress(receiverIFCs.GetAddress(i), PORT));
        sender.SetAttribute("MaxBytes", UintegerValue(0)); // Unlimited data
        senderApp.Add(sender.Install(senders.Get(i)));
        senderApp.Get(i)->SetStartTime(Seconds(startTime +  ( flowStartOffset * i))); 
        // CongestionOps/$ns3::TcpBbr/wildcard
        Simulator::Schedule(Seconds(startTime +  (flowStartOffset * i)) + MilliSeconds(1), &socketTrace<decltype(&uint32Tracer)>,  senders.Get(i)->GetId(), "bif", "BytesInFlight",  &uint32Tracer);
        Simulator::Schedule(Seconds(startTime +  (flowStartOffset * i)) + MilliSeconds(1), &socketTrace<decltype(&uint32Tracer)>,  senders.Get(i)->GetId(), "cwnd", "CongestionWindow", &uint32Tracer);
        Simulator::Schedule(Seconds(startTime +  (flowStartOffset * i)) + MilliSeconds(1), &socketTrace<decltype(&TimeTracer)>,  senders.Get(i)->GetId(), "rtt", "RTT",  &TimeTracer);
        if (cca[i] == "TcpBbr"){
            Simulator::Schedule(Seconds(startTime +  (flowStartOffset * i)) + MilliSeconds(1), &socketTrace<decltype(&uint32Tracer)>,  senders.Get(i)->GetId(), "maxBw", "CongestionOps/$ns3::TcpBbr/maxBw",  &uint32Tracer);

        }
        if (cca[i] == "TcpBbr3"){
            Simulator::Schedule(Seconds(startTime +  (flowStartOffset * i)) + MilliSeconds(1), &socketTrace<decltype(&DataRateTracer)>,  senders.Get(i)->GetId(), "pacing", "PacingRate",  &DataRateTracer);
            Simulator::Schedule(Seconds(startTime +  (flowStartOffset * i)) + MilliSeconds(1), &socketTrace<decltype(&uint32Tracer)>,  senders.Get(i)->GetId(), "wildcard", "CongestionOps/$ns3::TcpBbr3/wildcard",  &uint32Tracer);
            Simulator::Schedule(Seconds(startTime +  (flowStartOffset * i)) + MilliSeconds(1), &socketTrace<decltype(&uint32Tracer)>,  senders.Get(i)->GetId(), "inflightLo", "CongestionOps/$ns3::TcpBbr3/inflightLo",  &uint32Tracer);
            Simulator::Schedule(Seconds(startTime +  (flowStartOffset * i)) + MilliSeconds(1), &socketTrace<decltype(&uint32Tracer)>,  senders.Get(i)->GetId(), "inflightHi", "CongestionOps/$ns3::TcpBbr3/inflightHi",  &uint32Tracer);
            Simulator::Schedule(Seconds(startTime +  (flowStartOffset * i)) + MilliSeconds(1), &socketTrace<decltype(&uint32Tracer)>,  senders.Get(i)->GetId(), "maxBw", "CongestionOps/$ns3::TcpBbr3/maxBw",  &uint32Tracer);
        }
        

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

    std::cout << "Running simulation" << std::endl;
    
    FlowMonitorHelper flowmonHelperSender;
    files.insert(std::make_pair("throughtput", std::vector<std::string>()));
    files.insert(std::make_pair("goodput", std::vector<std::string>()));
    for (uint32_t i = 0; i < senders.GetN(); i++) {
        Ptr<FlowMonitor> flowMonitorS = flowmonHelperSender.Install(senders.Get(i));     
        rxBytes.push_back(0);
        Simulator::Schedule(Seconds(0.1) + MilliSeconds(1) + Seconds(flowStartOffset)*i, &TraceThroughput, flowMonitorS, ascii.CreateFileStream(outpath + cca[i] + std::to_string(i) + "-throughtput.csv"), i+1, 0, Seconds(0));
        files["throughtput"].push_back(outpath + cca[i] + std::to_string(i) + "-throughtput.csv");
        
        Simulator::Schedule(Seconds(0.1) + MilliSeconds(1) + Seconds(flowStartOffset)*i, &TraceGoodput, ascii.CreateFileStream(outpath + cca[i] + std::to_string(i) + "-goodput.csv"), i, 0, Seconds(0));
        files["goodput"].push_back(outpath + cca[i] + std::to_string(i) + "-goodput.csv");
    }
    
    Ptr<FlowMonitor> flowMonitor;
    FlowMonitorHelper flowmonHelper;
    flowMonitor = flowmonHelper.InstallAll();
    
    //DataRateChanger(senderDevices, 12, 20);
    //DelayChanger(senderDevices, 11, 50);
    //DelayChanger(senderDevices, 45, 5);


    Simulator::Stop(stopTime + TimeStep(1));

    Simulator::Run();


    std::string pdfPath = outpath + "pdf/";

    generatePlot({files["cwnd"]}, "Congestion Window", "Cwnd (packets)", pdfPath);
    generatePlot({files["wildcard"]}, "wildcard", "?", pdfPath);
    generatePlot({files["rtt"]}, "Round Trip Time", "RTT (ms)", pdfPath);
    generatePlot({files["pacing"]}, "Pacing", "Pacing (Mbps)", pdfPath);
    generatePlot({files["inflight_hi"], files["inflight_lo"]}, "Inflight Low and High", "bytes", pdfPath);
    generatePlot({files["throughtput"]}, "Throughput", "bps", pdfPath);
    generatePlot({files["goodput"]}, "Goodput", "bps", pdfPath);
    generatePlot({files["maxBw"]}, "Max bandwidth estimate", "bps", pdfPath);

    //, files["inflight_hi"], files["inflight_lo"]
    generatePlot({files["cwnd"], files["bif"]}, "Congestion Window and Bytes In Flight", "Bytes", pdfPath);
    std::vector<std::string>temp;
    temp.push_back(outpath + "queueSize.csv");
    generatePlot({temp}, "Queue Size", "Queue Size (packets)", pdfPath);
    Simulator::Destroy();


    if (cleanup)
        for (const auto& entry : files) {
            const std::vector<std::string>& stringArray = entry.second;
            for (const auto& value : stringArray) {
                remove((value).c_str());
            }
        }
    remove((temp[0]).c_str());
    exit(0);
}

