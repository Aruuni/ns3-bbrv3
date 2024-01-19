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


using namespace ns3;
std::vector<std::string> colors = {
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
};

std::vector<std::string> colors2 = {
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
};
// ./ns3 clean
// ./ns3 configure --build-profile=optimized 
// ./ns3


/*
CHEAT SHEET

Config path for congestion control algorithm tracable values
NodeList/[i]/$ns3::TcpL4Protocol/SocketList/[i]/CongestionOps/$ns3::TcpBbr



*/

//simulation paramaters
std::vector<std::string> cca = { "TcpBbr"  }; // ,"TcpBbr"
//std::vector<std::string> cca = { "TcpBbr"  };
//, TcpCubic TcpBbr
std::vector<std::string> cwndPlotFilesnames = { };
std::vector<std::string> rttPlotFilesnames = { };
std::vector<std::string> throughputPlotFilesnames = { };
std::vector<std::string> rwndPlotFilesnames = { };
std::vector<std::string> goodputPlotFilesnames = { };
std::vector<std::string> bytesInFlightFilesnames = { };
std::vector<std::string> packetDropFilesnames = { };
std::vector<std::string> inflightHiFilenames = { };
std::vector<std::string> pacingPlotFilesnames = { };
std::vector<std::string> wildcardFilenames = { };
std::vector<std::string> rtPropPlotFilesnames = { };

double startTime = 0.1; // in seconds
double startOffset = 0; // in seconds // WATCH OUT FOR THIS BEING LOWER THEN THE SIMUATLION END TIME
int PORT = 50001;
Time stopTime = Seconds(20);
uint packetSize = 1460;


std::vector<double> rxBytes;
AsciiTraceHelper ascii;
uint32_t bdp_multiplier = 1;

int bottleneckLinkDataRate = 10;
int bottleneckLinkDelay = 5;

int p2pLinkDataRate = 100;
int p2pLinkDelay = 10;

int p2pLinkOffset = 0;

///////  LOGGING ////////

bool cleanup = true;
bool plotScriptOut = false;
bool progressLog = false;

static void
uint32Tracer(
    Ptr<OutputStreamWrapper> stream, 
    uint32_t, 
    uint32_t newval
)
{
    *stream->GetStream() 
        << Simulator::Now().GetSeconds() 
        << ", " 
        << newval 
        << std::endl;
}

static void
DataRateTracer(
    Ptr<OutputStreamWrapper> stream, 
    DataRate, 
    DataRate newval
)
{
    *stream->GetStream() 
        << Simulator::Now().GetSeconds() 
        << ", " << newval.GetBitRate() 
        << std::endl;
}

static void
TimeTracer(
    Ptr<OutputStreamWrapper> stream, 
    Time, 
    Time newval
)
{
    *stream->GetStream() 
        << Simulator::Now().GetSeconds() 
        << ", " 
        << newval.GetMilliSeconds() 
        << std::endl;
}

static void
TraceThroughput(
    Ptr<FlowMonitor> monitor, 
    Ptr<OutputStreamWrapper> stream,
    uint32_t flowID, 
    uint32_t prevTxBytes, 
    Time prevTime 
) 
{
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
    FlowMonitor::FlowStats statsNode = stats[flowID];
    *stream->GetStream() 
        << Simulator::Now().GetSeconds() 
        << ", "
        << 8 * (statsNode.txBytes - prevTxBytes) / (1000000 * (Simulator::Now().GetSeconds() - prevTime.GetSeconds()))
        << std::endl;
    Simulator::Schedule(Seconds(0.1), &TraceThroughput, monitor, stream, flowID, statsNode.txBytes, Simulator::Now());
}

void ReceivedPacket(
    uint32_t flowID,
    Ptr<const Packet> p, 
    const Address& addr
)
{
	rxBytes[flowID] += p->GetSize();
}

static void
TraceGoodput(
    Ptr<OutputStreamWrapper> stream,
    uint32_t flowID,
    uint32_t prevRxBytes,
    Time prevTime 
)
{
    *stream->GetStream() 
        << Simulator::Now().GetSeconds() 
        << ", "
        << 8 * (rxBytes[flowID] - prevRxBytes) / (1000000 * (Simulator::Now().GetSeconds() - prevTime.GetSeconds()))
        << std::endl;
    Simulator::Schedule(Seconds(0.1), &TraceGoodput, stream,  flowID, rxBytes[flowID], Simulator::Now());
}

void
TraceCwnd(
    uint32_t nodeID, 
    std::string cca
)
{
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(nodeID) + 
                                  "/$ns3::TcpL4Protocol/SocketList/0/CongestionWindow", 
                                  MakeBoundCallback(&uint32Tracer, ascii.CreateFileStream("zlogs/" + cca + std::to_string(nodeID) + "-cwnd.csv")));
    cwndPlotFilesnames.push_back("zlogs/" + cca + std::to_string(nodeID) + "-cwnd.csv");
}

void
TracePacing(
    uint32_t nodeID, 
    std::string cca
)
{
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(nodeID) + 
                                  "/$ns3::TcpL4Protocol/SocketList/0/PacingRate", 
                                  MakeBoundCallback(&DataRateTracer, ascii.CreateFileStream("zlogs/" + cca + std::to_string(nodeID) + "-pacing.csv")));
    pacingPlotFilesnames.push_back("zlogs/" + cca + std::to_string(nodeID) + "-pacing.csv");
}

void
TraceWildCard(
    uint32_t nodeID, 
    std::string cca
)
{
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(nodeID) + 
                                  "/$ns3::TcpL4Protocol/SocketList/0/CongestionOps/$ns3::TcpBbr/wildcard", 
                                  MakeBoundCallback(&uint32Tracer, ascii.CreateFileStream("zlogs/" + cca + std::to_string(nodeID) + "-wildcard.csv")));
    wildcardFilenames.push_back("zlogs/" + cca + std::to_string(nodeID) + "-wildcard.csv");
}

void
TraceInfligtHi(
    uint32_t nodeID, 
    std::string cca
)
{
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(nodeID) + 
                                  "/$ns3::TcpL4Protocol/SocketList/0/CongestionOps/$ns3::TcpBbr/inflight_hi", 
                                  MakeBoundCallback(&uint32Tracer, ascii.CreateFileStream("zlogs/" + cca + std::to_string(nodeID) + "-inflight_hi.csv")));
    inflightHiFilenames.push_back("zlogs/" + cca + std::to_string(nodeID) + "-inflight_hi.csv");
}

void
TraceRTprop(
    uint32_t nodeID, 
    std::string cca
)
{
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(nodeID) + 
                                  "/$ns3::TcpL4Protocol/SocketList/0/CongestionOps/$ns3::TcpBbr/rt_prop", 
                                  MakeBoundCallback(&TimeTracer, ascii.CreateFileStream("zlogs/" + cca + std::to_string(nodeID) + "-rtprop.csv")));
    rtPropPlotFilesnames.push_back("zlogs/" + cca + std::to_string(nodeID) + "-rtprop.csv");

}

void
TraceRTT(
    uint32_t nodeID, 
    std::string cca
)
{
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(nodeID) + 
                                  "/$ns3::TcpL4Protocol/SocketList/0/RTT", 
                                  MakeBoundCallback(&TimeTracer, ascii.CreateFileStream("zlogs/" + cca + std::to_string(nodeID) + "-rtt.csv")));
    rttPlotFilesnames.push_back("zlogs/" + cca + std::to_string(nodeID) + "-rtt.csv");

}

void
QueueSizeTrace(
    uint32_t nodeID,
    uint32_t deviceID
)
{
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(nodeID) + 
                                  "/DeviceList/" + std::to_string(deviceID) + 
                                  "/$ns3::PointToPointNetDevice/TxQueue/PacketsInQueue", 
                                  MakeBoundCallback(&uint32Tracer, ascii.CreateFileStream("zlogs/queueSize.csv")));
}

void
BytesInFlightTrace(
    uint32_t nodeID,
    std::string cca
)
{
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(nodeID) + 
                                "/$ns3::TcpL4Protocol/SocketList/0/BytesInFlight", 
                                MakeBoundCallback(&uint32Tracer, ascii.CreateFileStream("zlogs/" + cca + std::to_string(nodeID) + "-bif.csv")));
    bytesInFlightFilesnames.push_back("zlogs/" + cca + std::to_string(nodeID) + "-bif.csv");
}

void 
generatePlot( 
    std::vector<std::string> plotFilesnames,
    std::string plotTitle,
    std::string plotYLabel
)
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
        fprintf(gnuplotPipe, "set output \"zout/%s.pdf\"\n", plotTitle.c_str());
        fprintf(gnuplotPipe, "set title \"%s\"\n", plotTitle.c_str());
        fprintf(gnuplotPipe, "set xlabel \"Time (sec)\"\n");
        fprintf(gnuplotPipe, "set ylabel \"%s\"\n", plotYLabel.c_str());
        fprintf(gnuplotPipe, "set key right top vertical\n");

        std::string plotCommand = "plot ";
        for (uint32_t i = 0; i < plotFilesnames.size(); i++) {
            // lines linepoints points steps
            plotCommand += "\"" + plotFilesnames[i] + "\" title \"" + cca[i] +  std::to_string(i) + "\" with steps  lw 0.7 lc '" + colors[i] + "'";
            if (i != plotFilesnames.size() - 1) 
                plotCommand += ", ";
        }  
        fprintf(gnuplotPipe, "%s", plotCommand.c_str());
        fflush(gnuplotPipe);
        //std::cout << "Gnuplot" << plotTitle << " script executed successfully." << std::endl;
    } else {
        std::cerr << "Error opening gnuplot pipe." << std::endl;
    }
    pclose(gnuplotPipe);
}

void
generateCwndInflight(
    std::vector<std::string> cwndFileNames,
    std::vector<std::string> inflightFileNames,
    std::string plotTitle,
    std::string plotYLabel
)
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
        fprintf(gnuplotPipe, "set output \"zout/%s.pdf\"\n", plotTitle.c_str());
        fprintf(gnuplotPipe, "set title \"%s\"\n", plotTitle.c_str());
        fprintf(gnuplotPipe, "set xlabel \"Time (sec)\"\n");
        fprintf(gnuplotPipe, "set ylabel \"%s\"\n", plotYLabel.c_str());
        fprintf(gnuplotPipe, "set key right top vertical\n");
        std::string plotCommand = "plot ";
        for (uint32_t i = 0; i < cwndFileNames.size(); i++) {
            plotCommand += "\"" + cwndFileNames[i] + "\" title \"" + cca[i] +  std::to_string(i) + "cwnd" + "\" with steps lw 0.7 lc '" + colors[i] + "'";
            plotCommand += ", ";
            plotCommand += "\"" + inflightFileNames[i] + "\" title \"" + cca[i] +  std::to_string(i) + "bif" + "\" with steps lw 0.7 lc '" + colors2[i] + "'";
            if (i != cwndFileNames.size() - 1)
                plotCommand += ", ";
        }
        fprintf(gnuplotPipe, "%s", plotCommand.c_str());
        fflush(gnuplotPipe);
        //std::cout << "Gnuplot" << plotTitle << " script executed successfully." << std::endl;
    } else {
        std::cerr << "Error opening gnuplot pipe." << std::endl;
    }
    pclose(gnuplotPipe);
}

static void
ChangeDelay(
    NetDeviceContainer dev,
    uint32_t delay
) 
{
    dev.Get(0)->GetChannel()->GetObject<PointToPointChannel>()->SetAttribute("Delay", StringValue(std::to_string(delay)+"ms"));
}

static void
DelayChanger(
    NetDeviceContainer dev,
    uint32_t time,
    uint32_t delay
) 
{
    Simulator::Schedule(Seconds(time), &ChangeDelay, dev, delay);
}

static void
ChangeDataRate(
    NetDeviceContainer dev,
    uint32_t datarate
) 
{
    Config::Set("/NodeList/" + std::to_string(cca.size()) + 
                "/DeviceList/0" + 
                "/$ns3::PointToPointNetDevice/DataRate", StringValue(std::to_string(datarate) + "Mbps") );
    Config::Set("/NodeList/" + std::to_string(cca.size()+1) + 
                "/DeviceList/0" + 
                "/$ns3::PointToPointNetDevice/DataRate", StringValue(std::to_string(datarate) + "Mbps") );
}

static void
DataRateChanger(
    NetDeviceContainer dev,
    double time,
    uint32_t datarate
) 
{
    Simulator::Schedule(Seconds(time), &ChangeDataRate, dev, datarate);
}


//////////////////////////////////
//          DO LATER       //
//////////////////////////////////
void
packetDrop(
    Ptr<OutputStreamWrapper> stream,
    const Ipv4Header &header, 
    Ptr<const Packet> packet, 
    ns3::Ipv4L3Protocol::DropReason reason, 
    Ptr<Ipv4> ipv4, 
    uint32_t interface
    )
{
    *stream->GetStream() << Simulator::Now().GetSeconds() << ", 0" << std::endl;
}

void
packetDropTracer(
    uint32_t nodeID,
    std::string cca
    )
{
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(nodeID) + 
                                "/$ns3::Ipv4L3Protocol/Drop", 
                                MakeBoundCallback(&packetDrop, ascii.CreateFileStream("zlogs/" + cca + std::to_string(nodeID) + "-pktdrop.csv")));
    packetDropFilesnames.push_back("zlogs/" + cca + std::to_string(nodeID) + "-pktdrop.csv");
}


void 
progress(){
    uint8_t barWidth = 50;
    std::cout << "\033[2K"; // Clear the previous line
    std::cout << "\033[A"; // Move the cursor up one line
    std::cout.flush(); // Flush the output stream
    std::cout << "Simulation progress: [";
    
    int progressMade = ((double)barWidth / 100) * ((Simulator::Now().GetSeconds() / stopTime.GetSeconds())*100);

    for (int i = 0; i < barWidth; ++i) {
        if (i == barWidth/2)
            std::cout << std::fixed << std::setprecision(2) << ((Simulator::Now().GetSeconds() / stopTime.GetSeconds())*100) << "%";
        if (i < progressMade) {
            std::cout << "=";
        } else if (i == progressMade) {
            std::cout << ">";
        } else {
            std::cout << " ";
        }
    }
    std::cout << "] " << std::endl;
    Simulator::Schedule(Seconds(0.1), &progress);
}

int 
main(
    int argc, 
    char* argv[]
    )
{
    if (progressLog)
        Simulator::Schedule(Seconds(0), &progress);

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
    // Config::SetDefault("ns3::DropTailQueue<Packet>::MaxSize", QueueSizeValue (QueueSizeValue (QueueSize(std::to_string(((bottleneckLinkDataRate * 1000) * bottleneckLinkDelay) / packetSize * bdp_multiplier) + "p"))));
    // std::cout << "bn rate: " << std::to_string(bottleneckLinkDataRate) << "bn delay: " << std::to_string(bottleneckLinkDelay) << "packet size: " << std::to_string(packetSize) << "bdp multiplier: " << std::to_string(bdp_multiplier) << std::endl;
    // std::cout << "Bottleneck link queue ............. >  " << std::to_string((bottleneckLinkDataRate * bottleneckLinkDelay / packetSize) * bdp_multiplier) + "p" << std::endl;
    // Config::SetDefault("ns3::FifoQueueDisc::MaxSize", QueueSizeValue (QueueSize(std::to_string(((bottleneckLinkDataRate * 1000) * bottleneckLinkDelay) / packetSize * bdp_multiplier) + "p")));
    // Config::SetDefault("ns3::DropTailQueue<Packet>::MaxSize", QueueSizeValue (QueueSize ("34p")));
    Config::SetDefault("ns3::TcpSocketBase::Sack", BooleanValue(true));

    // Config::SetDefault("ns3::FifoQueueDisc::MaxSize", QueueSizeValue (QueueSize ("34p")));
    //Config::SetDefault("ns3::TcpL4Protocol::RecoveryType", TypeIdValue(TypeId::LookupByName(recovery)));
    NodeContainer senders, receivers, routers;
    senders.Create(cca.size());
    receivers.Create(cca.size());
    routers.Create(2);
    
    PointToPointHelper botLink, p2pLinkLeft, p2pLinkRight;

    botLink.SetDeviceAttribute("DataRate", StringValue(std::to_string(bottleneckLinkDataRate) + "Mbps"));
    botLink.SetChannelAttribute("Delay", StringValue(std::to_string(bottleneckLinkDelay) + "ms"));
    botLink.SetQueue("ns3::DropTailQueue", "MaxSize", QueueSizeValue(QueueSize(std::to_string(((bottleneckLinkDataRate * 1000) * bottleneckLinkDelay / packetSize) * bdp_multiplier) + "p")));
    botLink.SetQueue("ns3::DropTailQueue", "MaxSize", QueueSizeValue(QueueSize("10p")));
    std::cout << "Bottleneck link queue ............. >  " << std::to_string(((bottleneckLinkDataRate * 1000 ) * bottleneckLinkDelay / packetSize) * bdp_multiplier) + "p" << std::endl;

    
    p2pLinkLeft.SetDeviceAttribute("DataRate", StringValue(std::to_string(p2pLinkDataRate) + "Mbps"));
    //p2pLinkLeft.SetQueue("ns3::DropTailQueue", "MaxSize", QueueSizeValue(QueueSize("2p")));


    p2pLinkRight.SetDeviceAttribute("DataRate", StringValue(std::to_string(p2pLinkDataRate) + "Mbps"));
    p2pLinkRight.SetChannelAttribute("Delay", StringValue(std::to_string(p2pLinkDelay) + "ms"));
    p2pLinkRight.SetQueue("ns3::DropTailQueue", "MaxSize",  QueueSizeValue(QueueSize(std::to_string(((p2pLinkDataRate * 1000) * p2pLinkDelay / packetSize) * bdp_multiplier) + "p")));
    //p2pLinkRight.SetQueue("ns3::DropTailQueue", "MaxSize", QueueSizeValue(QueueSize("2p")));




    NetDeviceContainer routerDevices = botLink.Install(routers);
    NetDeviceContainer senderDevices, receiverDevices, leftRouterDevices, rightRouterDevices;
    
    for(uint32_t i = 0; i < senders.GetN(); i++) {
        p2pLinkLeft.SetQueue("ns3::DropTailQueue", "MaxSize",  QueueSizeValue(QueueSize(std::to_string(((p2pLinkDataRate * 1000) * (p2pLinkDelay + (p2pLinkOffset * i)) / packetSize) * bdp_multiplier) + "p")));
        p2pLinkLeft.SetChannelAttribute("Delay", StringValue(std::to_string(p2pLinkDelay + (p2pLinkOffset * i)) + "ms"));


		NetDeviceContainer cleft = p2pLinkLeft.Install(routers.Get(0), senders.Get(i));
		leftRouterDevices.Add(cleft.Get(0));
		senderDevices.Add(cleft.Get(1));
		//cleft.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(em));

		NetDeviceContainer cright = p2pLinkRight.Install(routers.Get(1), receivers.Get(i));
		rightRouterDevices.Add(cright.Get(0));
		receiverDevices.Add(cright.Get(1));
		//cright.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(em));
	}
    
    InternetStackHelper internet;
    internet.Install(senders);
    internet.Install(receivers);
    internet.Install(routers);


    // sets the congestion control algorithm for each node to the corresponding value in the array 
    for (uint32_t i = 0; i < senders.GetN(); i++) { 
        Config::Set("/NodeList/" + std::to_string(i) + "/$ns3::TcpL4Protocol/SocketType", TypeIdValue(TypeId::LookupByName("ns3::" + cca[i]))); 
    }

    TrafficControlHelper tch;
    //tch.SetRootQueueDisc("ns3::FifoQueueDisc");
    //tch.SetRootQueueDisc("ns3::FifoQueueDisc", "MaxSize", StringValue ("34p"));
    //tch.SetQueueLimits("ns3::DynamicQueueLimits", "HoldTime", StringValue("1ms"));
    tch.Install(senderDevices);
    tch.Install(receiverDevices);
    tch.Install(leftRouterDevices);
    tch.Install(rightRouterDevices);
    
    // node 6 device 0 has the queue where the bottleneck will occur  
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
    for (uint32_t i = 0; i < senders.GetN(); i++) {
        BulkSendHelper sender("ns3::TcpSocketFactory", InetSocketAddress(receiverIFCs.GetAddress(i), PORT));
        sender.SetAttribute("MaxBytes", UintegerValue(0)); // Unlimited data
        senderApp.Add(sender.Install(senders.Get(i)));
        senderApp.Get(i)->SetStartTime(Seconds(startTime +  ( startOffset * i))); 
        Simulator::Schedule(Seconds(startTime +  ( startOffset * i)) + MilliSeconds(1), &TraceCwnd,  senders.Get(i)->GetId(), cca[i]);
        Simulator::Schedule(Seconds(startTime +  ( startOffset * i)) + MilliSeconds(1), &TraceRTT,  senders.Get(i)->GetId(), cca[i]);
        Simulator::Schedule(Seconds(startTime +  ( startOffset * i)) + MilliSeconds(1), &BytesInFlightTrace,  senders.Get(i)->GetId(), cca[i]);
        Simulator::Schedule(Seconds(startTime +  ( startOffset * i)) + MilliSeconds(1), &TraceWildCard,  senders.Get(i)->GetId(), cca[i]);
        Simulator::Schedule(Seconds(startTime +  ( startOffset * i)) + MilliSeconds(1), &TracePacing,  senders.Get(i)->GetId(), cca[i]);
        Simulator::Schedule(Seconds(startTime +  ( startOffset * i)) + MilliSeconds(1), &TraceRTprop,  senders.Get(i)->GetId(), cca[i]);
        Simulator::Schedule(Seconds(startTime +  ( startOffset * i)) + MilliSeconds(1), &TraceInfligtHi,  senders.Get(i)->GetId(), cca[i]);
        //Simulator::Schedule(Seconds(startTime +  ( startOffset * i)) + MilliSeconds(1), &packetDropTracer,  senders.Get(i)->GetId(), cca[i]);


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
    for (uint32_t i = 0; i < senders.GetN(); i++) {
        Ptr<FlowMonitor> flowMonitorS = flowmonHelperSender.Install(senders.Get(i));     
        rxBytes.push_back(0);
        Simulator::Schedule(Seconds(0.1) + MilliSeconds(1) + Seconds(startOffset)*i, &TraceThroughput, flowMonitorS, ascii.CreateFileStream("zlogs/" + cca[i] + std::to_string(i) + "-throughtput.csv"), i+1, 0, Seconds(0));
        throughputPlotFilesnames.push_back("zlogs/" + cca[i] + std::to_string(i) + "-throughtput.csv");
        
        Simulator::Schedule(Seconds(0.1) + MilliSeconds(1) + Seconds(startOffset)*i, &TraceGoodput, ascii.CreateFileStream("zlogs/" + cca[i] + std::to_string(i) + "-goodput.csv"), i, 0, Seconds(0));
        goodputPlotFilesnames.push_back("zlogs/" + cca[i] + std::to_string(i) + "-goodput.csv");
    }
    
    Ptr<FlowMonitor> flowMonitor;
    FlowMonitorHelper flowmonHelper;
    flowMonitor = flowmonHelper.InstallAll();
    
    //DataRateChanger(senderDevices, 7, 20);
    //DelayChanger(senderDevices, 11, 50);
    //DelayChanger(senderDevices, 45, 5);


    Simulator::Stop(stopTime + TimeStep(1));
    Simulator::Run();


    generatePlot(cwndPlotFilesnames, "Congestion Window", "Cwnd (packets)");
    generatePlot(wildcardFilenames, "wildcard", "?");
    generatePlot(rttPlotFilesnames, "Round Trip Time", "RTT (ms)");
    generatePlot(throughputPlotFilesnames, "Throughput", "Throughput (Mbps)");
    generatePlot(goodputPlotFilesnames, "Goodput", "Goodput (Mbps)");
    generatePlot(pacingPlotFilesnames, "Pacing", "Pacing (Mbps)");
    generatePlot(inflightHiFilenames, "InflightHi", "bytes");
    //generatePlot(rtPropPlotFilesnames, "RT Propagation", "RT Propagation (ms)");
    generatePlot(bytesInFlightFilesnames, "BytesInFlight", "BytesInFlight (bytes)");
    //generatePlot(packetDropFilesnames , "Packets Dropped", "Packets (segments)");
    generateCwndInflight(cwndPlotFilesnames, bytesInFlightFilesnames, "Congestion Window and Bytes In Flight", "Bytes");
    std::vector<std::string>temp;
    temp.push_back("zlogs/queueSize.csv");
    generatePlot(temp, "Queue Size", "Queue Size (packets)");
    Simulator::Destroy();

    //cleans up the csv files
    if (cleanup)
        for (uint32_t i = 0; i < cca.size(); i++)
        {
            //remove((cwndPlotFilesnames[i]).c_str());
            remove((rttPlotFilesnames[i]).c_str());
            remove((throughputPlotFilesnames[i]).c_str());
            //remove((rwndPlotFilesnames[i]).c_str());
            //remove((pacingPlotFilesnames[i]).c_str());
            remove((goodputPlotFilesnames[i]).c_str());
            remove((bytesInFlightFilesnames[i]).c_str());
            remove((wildcardFilenames[i]).c_str());
            //remove((inflightHiFilenames[i]).c_str());
            
            //remove((packetDropFilesnames[i]).c_str());
        }
    remove((temp[0]).c_str());
    exit(0);
}

