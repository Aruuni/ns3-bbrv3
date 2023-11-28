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

// ./ns3 clean
// ./ns3 configure --build-profile=optimized 
// ./ns3


//simulation paramaters
std::vector<std::string> cca = { "TcpBbr" };
//std::vector<std::string> cca = { "TcpBbr"  };
//, TcpCubic TcpBbr
std::vector<std::string> cwndPlotFilesnames = { };
std::vector<std::string> rttPlotFilesnames = { };
std::vector<std::string> throughputPlotFilesnames = { };
std::vector<std::string> rwndPlotFilesnames = { };
std::vector<std::string> goodputPlotFilesnames = { };
std::vector<std::string> bytesInFlightFilesnames = { };
std::vector<std::string> packetDropFilesnames = { };


double startTime = 0.1; // in seconds
double startOffset = 5; // in seconds // WATCH OUT FOR THIS BEING LOWER THEN THE SIMUATLION END TIME
int PORT = 50001;
Time stopTime = Seconds(20);
uint packetSize = 1460;
std::vector<std::string> colors = { "blue", "green", "orange", "red", "purple", "brown", "black", "yellow", "cyan", "magenta", "gray" };
std::vector<std::string> colors2 = { "green", "orange", "red", "purple", "brown", "black", "yellow", "cyan", "magenta", "gray", "blue" };
double ReadingResolution = 0.1;
AsciiTraceHelper ascii;
uint32_t bdp_multiplier = 5;

int bottleneckLinkDataRate = 10;
int bottleneckLinkDelay = 5;

int p2pLinkDataRate = 100;
int p2pLinkDelay = 10;

int p2pLinkOffset = 0;

///////  LOGGING ////////

bool cleanup = true;
bool plotScriptOut = false;
bool progressLog = false;


//////////////////////////////////
//          THROUGHPUT          //
//////////////////////////////////
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
    Simulator::Schedule(Seconds(ReadingResolution), &TraceThroughput, monitor, stream, flowID, statsNode.txBytes, Simulator::Now());
}

//////////////////////////////////
//           GOODPUT            //
//////////////////////////////////

std::vector<double> rxBytes;

void ReceivedPacket(
    uint32_t flowID,
    Ptr<const Packet> p, 
    const Address& addr
    )
{
	rxBytes[flowID] += p->GetSize();
    //if (flowID == 0)
    //std::cout << "size of rx bytes " << std::to_string(flowID) << " "<< std::to_string(rxBytes[flowID])<< " Received packet of size " << p->GetSize() <<  " at time: " << Now().GetSeconds() << std::endl;
    //if (Now().GetSeconds() > 25 && Now().GetSeconds() < 27 && flowID == 0)
    //std::cout << "size of rx bytes " << std::to_string(flowID) << " "<< std::to_string(rxBytes[flowID])<< " Received packet of size " << p->GetSize() << " from " << addr << " at time: " << Now().GetSeconds() << std::endl;
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
    Simulator::Schedule(Seconds(ReadingResolution), &TraceGoodput, stream,  flowID, rxBytes[flowID], Simulator::Now());
}

//////////////////////////////////
//      CONGESTION WINDOW       //
//////////////////////////////////
static void
CwndTracer(
    Ptr<OutputStreamWrapper> stream, 
    std::string cca,  
    uint32_t, 
    uint32_t newval
    )
{
    *stream->GetStream() << Simulator::Now().GetSeconds() << ", " << newval /* / packetSize */ << std::endl;
    //std::cout << Simulator::Now().GetSeconds() << "  |  " << newval / 1448.0 <<  " | "<< cca <<std::endl;
}
void
TraceCwnd(
    uint32_t nodeID, 
    std::string cca
    )
{
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(nodeID) + 
                                  "/$ns3::TcpL4Protocol/SocketList/0/CongestionWindow", 
                                  MakeBoundCallback(&CwndTracer, ascii.CreateFileStream("zlogs/" + cca + std::to_string(nodeID) + "-cwnd.csv"), cca));
    cwndPlotFilesnames.push_back("zlogs/" + cca + std::to_string(nodeID) + "-cwnd.csv");
}

//////////////////////////////////
//          RTT TRACER          //
//////////////////////////////////
void
RTTTracer(
    Ptr<OutputStreamWrapper> stream,  
    Time oldval, 
    Time newval
    )
{
    //std::cout << Simulator::Now().GetSeconds() << "  |  " << newval.GetMilliSeconds() <<  " | "<< cca << std::endl;
    *stream->GetStream() << Simulator::Now().GetSeconds() << ", " << newval.GetMilliSeconds()  << std::endl;
}
void
TraceRTT(
    uint32_t nodeID, 
    std::string cca
    )
{
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(nodeID) + 
                                  "/$ns3::TcpL4Protocol/SocketList/0/RTT", 
                                  MakeBoundCallback(&RTTTracer, ascii.CreateFileStream("zlogs/" + cca + std::to_string(nodeID) + "-rtt.csv")));
    rttPlotFilesnames.push_back("zlogs/" + cca + std::to_string(nodeID) + "-rtt.csv");

}

//////////////////////////////////
//            IGNORE            //
//////////////////////////////////
void 
RWNDTracer(
    Ptr<OutputStreamWrapper> stream,
    DataRate, 
    DataRate newSize
    ) 
{
    //std::cout << "Queue size: " << newSize << std::endl;
    *stream->GetStream() << Simulator::Now().GetSeconds() << ", " << newSize.GetBitRate() << std::endl;
}

void
RWNDTrace(
    uint32_t nodeID,
    std::string cca
    )
{
    // "/NodeList/[i]/$ns3::TcpL4Protocol/SocketList/[i]/CongestionOps/$ns3::TcpBbr"
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(nodeID) + 
                                   "/$ns3::TcpL4Protocol/SocketList/0/CongestionOps/$ns3::TcpBbr/DataRate", 
                                  // "/$ns3::TcpL4Protocol/SocketList/0/PacingRate", 
                                  MakeBoundCallback(&RWNDTracer, ascii.CreateFileStream("zlogs/" + cca + std::to_string(nodeID) + "-dr.csv")));
    rwndPlotFilesnames.push_back("zlogs/" + cca + std::to_string(nodeID) + "-dr.csv");
}

//////////////////////////////////
//          QUEUE SIZE          //
//////////////////////////////////
void 
QueueSizeTracer(
    Ptr<OutputStreamWrapper> stream,
    uint32_t, 
    uint32_t newSize
    ) 
{
    //std::cout << "Queue size: " << newSize << std::endl;
    *stream->GetStream() << Simulator::Now().GetSeconds() << ", " << newSize << std::endl;
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
                                  MakeBoundCallback(&QueueSizeTracer, ascii.CreateFileStream("zlogs/queueSize.csv")));
}

//////////////////////////////////
//        BYTES IN FLGIHT       //
//////////////////////////////////
void 
BytesInFlightTracer(
    Ptr<OutputStreamWrapper> stream,
    uint32_t, 
    uint32_t newSize
    ) 
{
    //std::cout << "Queue size: " << newSize << std::endl;
    *stream->GetStream() << Simulator::Now().GetSeconds() << ", " << newSize << std::endl;
}
void
BytesInFlightTrace(
    uint32_t nodeID,
    std::string cca
    )
{
    Config::ConnectWithoutContext("/NodeList/" + std::to_string(nodeID) + 
                                "/$ns3::TcpL4Protocol/SocketList/0/BytesInFlight", 
                                MakeBoundCallback(&BytesInFlightTracer, ascii.CreateFileStream("zlogs/" + cca + std::to_string(nodeID) + "-bif.csv")));
    
    bytesInFlightFilesnames.push_back("zlogs/" + cca + std::to_string(nodeID) + "-bif.csv");
}

//////////////////////////////////
//          PACKET DROP         //
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
        plotCommand += "\"" + plotFilesnames[i] + "\" title \"" + cca[i] +  std::to_string(i) + "\" with lines lw 0.7 lc '" + colors[i] + "'";
        if (i != plotFilesnames.size() - 1) 
            plotCommand += ", ";
        }  
        fprintf(gnuplotPipe, "%s", plotCommand.c_str());
        fflush(gnuplotPipe);
        std::cout << "Gnuplot" << plotTitle << " script executed successfully." << std::endl;
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
            plotCommand += "\"" + cwndFileNames[i] + "\" title \"" + cca[i] +  std::to_string(i) + "cwnd" + "\" with lines lw 0.7 lc '" + colors[i] + "'";
            plotCommand += ", ";
            plotCommand += "\"" + inflightFileNames[i] + "\" title \"" + cca[i] +  std::to_string(i) + "bif" + "\" with lines lw 0.7 lc '" + colors2[i] + "'";
            if (i != cwndFileNames.size() - 1)
                plotCommand += ", ";
        }
        fprintf(gnuplotPipe, "%s", plotCommand.c_str());
        fflush(gnuplotPipe);
        std::cout << "Gnuplot" << plotTitle << " script executed successfully." << std::endl;
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

void 
progress(){
    std::cout << "\033[2K"; // Clear the previous line
    std::cout << "\033[A"; // Move the cursor up one line
    std::cout.flush(); // Flush the output stream
    std::cout << "Simulation progress: " << std::fixed << std::setprecision(2) << ((Simulator::Now().GetSeconds() / stopTime.GetSeconds())*100) << "%" << std::endl;
    Simulator::Schedule(Seconds(0.1), &progress);
}

int 
main(
    int argc, 
    char* argv[]
    )
{
    // linux default send 4096   16384   4194304
    // linux default recv 4096   131072  6291456
    Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(4194304/4));
    //Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(16384));
    Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(4194304/4));
    //Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(131072));
    Config::SetDefault("ns3::TcpSocket::InitialCwnd", UintegerValue(10)); 
    Config::SetDefault("ns3::TcpSocket::InitialSlowStartThreshold", UintegerValue(10)); 
    Config::SetDefault("ns3::TcpSocket::DelAckCount", UintegerValue(1));
    //  Config::SetDefault("ns3::TcpSocket::DelAckTimeout", TimeValue(Seconds(0)));
    Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(packetSize));
    //Config::SetDefault("ns3::TcpSocketBase::WindowScaling", BooleanValue(true));
    Config::SetDefault("ns3::TcpSocketState::EnablePacing", BooleanValue(true));
    // Config::SetDefault("ns3::DropTailQueue<Packet>::MaxSize", QueueSizeValue (QueueSizeValue (QueueSize(std::to_string(((bottleneckLinkDataRate * 1000) * bottleneckLinkDelay) / packetSize * bdp_multiplier) + "p"))));
    // std::cout << "bn rate: " << std::to_string(bottleneckLinkDataRate) << "bn delay: " << std::to_string(bottleneckLinkDelay) << "packet size: " << std::to_string(packetSize) << "bdp multiplier: " << std::to_string(bdp_multiplier) << std::endl;
    // std::cout << "Bottleneck link queue ............. >  " << std::to_string((bottleneckLinkDataRate * bottleneckLinkDelay / packetSize) * bdp_multiplier) + "p" << std::endl;
    // Config::SetDefault("ns3::FifoQueueDisc::MaxSize", QueueSizeValue (QueueSize(std::to_string(((bottleneckLinkDataRate * 1000) * bottleneckLinkDelay) / packetSize * bdp_multiplier) + "p")));
    // Config::SetDefault("ns3::DropTailQueue<Packet>::MaxSize", QueueSizeValue (QueueSize ("34p")));
    // Config::SetDefault("ns3::FifoQueueDisc::MaxSize", QueueSizeValue (QueueSize ("34p")));

    
    NodeContainer senders, receivers, routers;
    senders.Create(cca.size());
    receivers.Create(cca.size());
    routers.Create(2);
    
    PointToPointHelper botLink, p2pLinkLeft, p2pLinkRight;

    botLink.SetDeviceAttribute("DataRate", StringValue(std::to_string(bottleneckLinkDataRate) + "Mbps"));
    botLink.SetChannelAttribute("Delay", StringValue(std::to_string(bottleneckLinkDelay) + "ms"));
    botLink.SetQueue("ns3::DropTailQueue", "MaxSize", QueueSizeValue(QueueSize(std::to_string(((bottleneckLinkDataRate * 1000) * bottleneckLinkDelay / packetSize) * bdp_multiplier) + "p")));
    //botLink.SetQueue("ns3::DropTailQueue", "MaxSize", QueueSizeValue(QueueSize("2p")));

    
    p2pLinkLeft.SetDeviceAttribute("DataRate", StringValue(std::to_string(p2pLinkDataRate) + "Mbps"));
    //p2pLinkLeft.SetQueue("ns3::DropTailQueue", "MaxSize", QueueSizeValue(QueueSize("2p")));


    p2pLinkRight.SetDeviceAttribute("DataRate", StringValue(std::to_string(p2pLinkDataRate) + "Mbps"));
    p2pLinkRight.SetChannelAttribute("Delay", StringValue(std::to_string(p2pLinkDelay) + "ms"));
    p2pLinkRight.SetQueue("ns3::DropTailQueue", "MaxSize",  QueueSizeValue(QueueSize(std::to_string(((p2pLinkDataRate * 1000) * p2pLinkDelay / packetSize) * bdp_multiplier) + "p")));
    //p2pLinkRight.SetQueue("ns3::DropTailQueue", "MaxSize", QueueSizeValue(QueueSize("2p")));

    //std::cout << "Bottleneck link queue ............. >  " << std::to_string(((bottleneckLinkDataRate * 1000 ) * bottleneckLinkDelay / packetSize) * bdp_multiplier) + "p" << std::endl;



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
        //Simulator::Schedule(Seconds(startTime +  ( startOffset * i)) + MilliSeconds(1), &RWNDTrace,  senders.Get(i)->GetId(), cca[i]);
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
    if (progressLog)
        Simulator::Schedule(Seconds(0), &progress);

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
    
    //DelayChanger(senderDevices, 11, 50);
    //DelayChanger(senderDevices, 45, 5);


    Simulator::Stop(stopTime + TimeStep(1));
    Simulator::Run();


    generatePlot(cwndPlotFilesnames, "Congestion Window", "Cwnd (packets)");
    generatePlot(rttPlotFilesnames, "Round Trip Time", "RTT (ms)");
    generatePlot(throughputPlotFilesnames, "Throughput", "Throughput (Mbps)");
    generatePlot(goodputPlotFilesnames, "Goodput", "Goodput (Mbps)");
    //generatePlot(rwndPlotFilesnames, "RWND", "RWND (bytes)");
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
            remove((cwndPlotFilesnames[i]).c_str());
            remove((rttPlotFilesnames[i]).c_str());
            remove((throughputPlotFilesnames[i]).c_str());
            //remove((rwndPlotFilesnames[i]).c_str());
            remove((goodputPlotFilesnames[i]).c_str());
            remove((bytesInFlightFilesnames[i]).c_str());
            //remove((packetDropFilesnames[i]).c_str());
        }
    remove((temp[0]).c_str());
    exit(0);
}


