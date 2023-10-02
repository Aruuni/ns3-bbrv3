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

//simulation paramaters
std::vector<std::string> cca = { "TcpBbr"  }; //  , "TcpBbr" }; // };
//, TcpCubic TcpBbr
std::vector<std::string> cwndPlotFilesnames = { };
std::vector<std::string> rttPlotFilesnames = { };
std::vector<std::string> throughputPlotFilesnames = { };
std::vector<std::string> goodputPlotFilesnames = { };
std::vector<std::string> bytesInFlightFilesnames = { };
std::vector<std::string> packetDropFilesnames = { };


double startTime = 0.1; // in seconds
double startOffset = 7; // in seconds
int PORT = 50001;
Time stopTime = Seconds(25);
uint packetSize = 1460;
std::vector<std::string> colors = { "blue", "green", "orange", "red", "purple", "brown", "black", "yellow", "cyan", "magenta", "gray" };
double ReadingResolution = 0.1;
AsciiTraceHelper ascii;
uint32_t bdp_multiplier = 2;



int bottleneckLinkDataRate = 10;
int bottleneckLinkDelay = 5;

int p2pLinkDataRate = 100;
int p2pLinkDelay = 5;


///////  LOGGING ////////

bool cleanup = false;
bool plotScriptOut = false;
bool progressLog = false;



#pragma region Throughput

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

#pragma endregion

#pragma region Goodput

std::vector<double> rxBytes;

void 
ReceivedPacket(
    uint32_t flowID,
    Ptr<const Packet> p, 
    const Address& addr
    )
{
	rxBytes[flowID] += p->GetSize();
    //if (flowID == 0 && Now().GetSeconds() > 35)
    //std::cout << " Received packet of size " << p->GetSize() <<  " at time: " << Now().GetSeconds() << std::endl;
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

#pragma endregion

#pragma region Congestion Window

static void
CwndTracer(
    Ptr<OutputStreamWrapper> stream, 
    std::string cca,  
    uint32_t, 
    uint32_t newval
    )
{
    *stream->GetStream() << Simulator::Now().GetSeconds() << ", " << newval / packetSize << std::endl;
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

#pragma endregion

#pragma region Round Trip Time

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

#pragma endregion

#pragma region Queue Size

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

#pragma endregion

#pragma region Bytes In Flight

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

#pragma endregion

#pragma region Packet Drop


// void
// PhyRxDrop(Ptr<const Packet> p, uint32_t flowID)
// {
//   NS_LOG_INFO("Packet Drop");
//   PhyRxDropCount[flowID]++;
// }

// static void
// TracePacketDrop(
//     Ptr<FlowMonitor> monitor, 
//     Ptr<OutputStreamWrapper> stream,
//     uint32_t flowID
//     ) 
// {
//     Config::ConnectWithoutContext("/NodeList/" + std::to_string(flowID) + "/DeviceList/0/$ns3::PointToPointNetDevice/Phy/PhyRxDrop", MakeCallback(&PhyRxDrop, flowID));
// }

#pragma endregion

#pragma region Plot Script

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
        if (plotScriptOut)
            std::cout << "set terminal pdf enhanced color dashed lw 1 font 'DejaVuSans,12'" << std::endl;
        
        fprintf(gnuplotPipe, "set style line 81 lt 0\n");
        if (plotScriptOut)
            std::cout << "set style line 81 lt 0"  << std::endl;
        fprintf(gnuplotPipe, "set style line 81 lt rgb \"#aaaaaa\"\n");
        if (plotScriptOut)
            std::cout << "set style line 81 lt rgb \"#aaaaaa\""  << std::endl;
        fprintf(gnuplotPipe, "set grid back linestyle 81\n");
        if (plotScriptOut)
            std::cout << "set grid back linestyle 81"  << std::endl;

        fprintf(gnuplotPipe, "set border 3 back linestyle 80\n");
        if (plotScriptOut)
            std::cout << "set border 3 back linestyle 80"  << std::endl;
        fprintf(gnuplotPipe, "set xtics nomirror\n");
        if (plotScriptOut)
            std::cout << "set xtics nomirror"  << std::endl;
        fprintf(gnuplotPipe, "set ytics nomirror\n");
        if (plotScriptOut)
            std::cout << "set ytics nomirror"  << std::endl;

        fprintf(gnuplotPipe, "set autoscale x\n");
        if (plotScriptOut)
            std::cout << "set autoscale x"  << std::endl;
        fprintf(gnuplotPipe, "set autoscale y\n");
        if (plotScriptOut)
            std::cout << "set autoscale y"  << std::endl;
        
        //if (plotTitle == "Congestion Window")
            //fprintf(gnuplotPipe, "set yrange [:100]\n");

        fprintf(gnuplotPipe, "set output \"zout/%s.pdf\"\n", plotTitle.c_str());
        if (plotScriptOut)
            std::cout << "set output \"" << plotTitle << ".pdf\"" << std::endl;
        fprintf(gnuplotPipe, "set title \"%s\"\n", plotTitle.c_str());
        if (plotScriptOut)
            std::cout << "set title \"" << plotTitle << "\"" << std::endl;
        fprintf(gnuplotPipe, "set xlabel \"Time (sec)\"\n");
        if (plotScriptOut)
            std::cout << "set xlabel \"Time (sec)\"" << std::endl;
        fprintf(gnuplotPipe, "set ylabel \"%s\"\n", plotYLabel.c_str());
        if (plotScriptOut)
            std::cout << "set ylabel \"" << plotYLabel << "\"" << std::endl;
        fprintf(gnuplotPipe, "set key right top vertical\n");
            if (plotScriptOut)
        std::cout << "set key right top vertical" << std::endl;
        std::string plotCommand = "plot ";
        for (uint32_t i = 0; i < plotFilesnames.size(); i++) {
        plotCommand += "\"" + plotFilesnames[i] + "\" title \"" + cca[i] +  std::to_string(i) + "\" with lines lw 0.7 lc '" + colors[i] + "'";
        if (i != plotFilesnames.size() - 1) 
            plotCommand += ", ";
        
        }
        
        fprintf(gnuplotPipe, "%s", plotCommand.c_str());
        if (plotScriptOut)
            std::cout << plotCommand << std::endl;
        fflush(gnuplotPipe);
        std::cout << "Gnuplot" << plotTitle << " script executed successfully." << std::endl;
    } else {
        std::cerr << "Error opening gnuplot pipe." << std::endl;
    }
    pclose(gnuplotPipe);
}

#pragma endregion

#pragma region Progress

void 
progress(){
    std::cout << "\033[2K"; // Clear the previous line
    std::cout << "\033[A"; // Move the cursor up one line
    std::cout.flush(); // Flush the output stream
    std::cout << "Simulation progress: " << std::fixed << std::setprecision(2) << ((Simulator::Now().GetSeconds() / stopTime.GetSeconds())*100) << "%" << std::endl;
    Simulator::Schedule(Seconds(0.1), &progress);
}

#pragma endregion

#pragma region RTT change

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

#pragma endregion

#pragma region BW change

// static void
// ChangeDataRate(
//     NetDeviceContainer dev,
//     uint32_t datarate
//     ) 
// {
//     Config::Set("/NodeList/" + std::to_string(cca.size()) + 
//                 "/DeviceList/0" + 
//                 "/$ns3::PointToPointNetDevice/DataRate", StringValue(std::to_string(datarate) + "Mbps") );
//     Config::Set("/NodeList/" + std::to_string(cca.size()+1) + 
//                 "/DeviceList/0" + 
//                 "/$ns3::PointToPointNetDevice/DataRate", StringValue(std::to_string(datarate) + "Mbps") );
// }

// static void
// DataRateChanger(
//     NetDeviceContainer dev,
//     uint32_t time,
//     uint32_t datarate
//     ) 
// {
//     Simulator::Schedule(Seconds(time), &ChangeDataRate, dev, datarate);
// }

#pragma endregion

int 
main(
    int argc, 
    char* argv[]
    )
{

    #pragma region Global Config

    // linux default send 4096   16384   4194304
    // linux default recv 4096   131072  6291456
    Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(4194304/4));
    //Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(16384));
    Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(4194304/4));
    //Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(131072));
    Config::SetDefault("ns3::TcpSocket::InitialCwnd", UintegerValue(10)); 
    Config::SetDefault("ns3::TcpSocket::InitialSlowStartThreshold", UintegerValue(10)); 
    Config::SetDefault("ns3::TcpSocket::DelAckCount", UintegerValue(1));
    Config::SetDefault("ns3::TcpSocket::DelAckTimeout", TimeValue(MilliSeconds(10)));
    Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(packetSize));
    //Config::SetDefault("ns3::TcpSocketBase::WindowScaling", BooleanValue(true));
    Config::SetDefault("ns3::TcpSocketState::EnablePacing", BooleanValue(true));

    #pragma endregion

    #pragma region Node Containters

    NodeContainer senders, receivers, routers;
    senders.Create(cca.size());
    receivers.Create(cca.size());
    routers.Create(2);
    
    #pragma endregion

    #pragma region Link Config

    PointToPointHelper botLink, p2pLink;
    botLink.SetDeviceAttribute("DataRate", StringValue(std::to_string(bottleneckLinkDataRate) + "Mbps"));
    botLink.SetChannelAttribute("Delay", StringValue(std::to_string(bottleneckLinkDelay) + "ms"));
    botLink.SetQueue("ns3::DropTailQueue", "MaxSize", QueueSizeValue(QueueSize(std::to_string(((bottleneckLinkDataRate * 1000) * bottleneckLinkDelay / packetSize) * bdp_multiplier) + "p")));
    
    std::cout << "Bottleneck link datarate .......... >  " << std::to_string(bottleneckLinkDataRate) + "Mbps" << std::endl;
    std::cout << "Bottleneck link delay ............. >  " << std::to_string(bottleneckLinkDelay) + "ms" << std::endl;
    std::cout << "Bottleneck link queue ............. >  " << std::to_string(((bottleneckLinkDataRate * 1000 ) * bottleneckLinkDelay / packetSize) * bdp_multiplier) + "p" << std::endl;
    
    p2pLink.SetDeviceAttribute("DataRate", StringValue(std::to_string(p2pLinkDataRate) + "Mbps"));
    p2pLink.SetChannelAttribute("Delay", StringValue(std::to_string(p2pLinkDelay) + "ms"));
    p2pLink.SetQueue("ns3::DropTailQueue", "MaxSize", QueueSizeValue(QueueSize(std::to_string(((bottleneckLinkDataRate * 1000) * bottleneckLinkDelay / packetSize) * bdp_multiplier) + "p")));
    //p2pLink.SetQueue("ns3::DropTailQueue", "MaxSize", QueueSizeValue(QueueSize(std::to_string(((p2pLinkDataRate * 1000) * p2pLinkDelay / packetSize) * bdp_multiplier) + "p")));
    
    std::cout << "P2P link datarate ................. >  " << std::to_string(p2pLinkDataRate) + "Mbps" << std::endl;
    std::cout << "P2P link delay .................... >  " << std::to_string(p2pLinkDelay) + "ms" << std::endl;
    std::cout << "P2P link queue .................... >  " << std::to_string(((bottleneckLinkDataRate * 1000) * bottleneckLinkDelay / packetSize) * bdp_multiplier) + "p" << std::endl;
    
    #pragma endregion



    NetDeviceContainer routerDevices = botLink.Install(routers);
    NetDeviceContainer senderDevices, receiverDevices, leftRouterDevices, rightRouterDevices;
    
    for(uint32_t i = 0; i < senders.GetN(); i++) {
		NetDeviceContainer cleft = p2pLink.Install(routers.Get(0), senders.Get(i));
		leftRouterDevices.Add(cleft.Get(0));
		senderDevices.Add(cleft.Get(1));
		//cleft.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(em));

		NetDeviceContainer cright = p2pLink.Install(routers.Get(1), receivers.Get(i));
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
    std::cout << "Congestion control algorithm ...... >  " << cca[0] << std::endl;
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

    #pragma region Intrface Congif

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

    #pragma endregion

    ApplicationContainer senderApp, receiverApp;
    for (uint32_t i = 0; i < senders.GetN(); i++) {
        BulkSendHelper sender("ns3::TcpSocketFactory", InetSocketAddress(receiverIFCs.GetAddress(i), PORT));
        sender.SetAttribute("MaxBytes", UintegerValue(0)); // Unlimited data
        senderApp.Add(sender.Install(senders.Get(i)));
        senderApp.Get(i)->SetStartTime(Seconds(startTime +  ( startOffset * i))); 
        Simulator::Schedule(Seconds(startTime +  ( startOffset * i)) + MilliSeconds(1), &TraceCwnd,  senders.Get(i)->GetId(), cca[i]);
        Simulator::Schedule(Seconds(startTime +  ( startOffset * i)) + MilliSeconds(1), &TraceRTT,  senders.Get(i)->GetId(), cca[i]);
        Simulator::Schedule(Seconds(startTime +  ( startOffset * i)) + MilliSeconds(1), &BytesInFlightTrace,  senders.Get(i)->GetId(), cca[i]);
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

        //Simulator::Schedule(Seconds(0.1) + MilliSeconds(1) + Seconds(startOffset)*i, &TracePacketDrop, flowMonitorS, ascii.CreateFileStream("zlogs/" + cca[i] + std::to_string(i) + "-packetdrop.csv"), i+1, 0, Seconds(0));
        //packetDropFilesnames.push_back("zlogs/" + cca[i] + std::to_string(i) + "-packetdrop.csv");
        
        Simulator::Schedule(Seconds(0.1) + MilliSeconds(1) + Seconds(startOffset)*i, &TraceGoodput, ascii.CreateFileStream("zlogs/" + cca[i] + std::to_string(i) + "-goodput.csv"), i, 0, Seconds(0));
        goodputPlotFilesnames.push_back("zlogs/" + cca[i] + std::to_string(i) + "-goodput.csv");
    }
    
    Ptr<FlowMonitor> flowMonitor;
    FlowMonitorHelper flowmonHelper;
    flowMonitor = flowmonHelper.InstallAll();
    
    //DelayChanger(leftRouterDevices, 35, 70);
    //DelayChanger(leftRouterDevices, 55, 60);
    //DelayChanger(leftRouterDevices, 75, 50);
    //DelayChanger(leftRouterDevices, 75, 80);

    //DataRateChanger(leftRouterDevices, 18, 20);
    //DataRateChanger(leftRouterDevices, 36, 10);

    Simulator::Stop(stopTime + TimeStep(1));
    Simulator::Run();

    flowMonitor->SerializeToXmlFile("NameOfFile.xml", true, true);
    // cwndPlotFilesnames.push_back("zlogs/TcpBbr0-cwnd.csv");
    // rttPlotFilesnames.push_back("zlogs/TcpBbr0-rtt.csv");
    // throughputPlotFilesnames.push_back("zlogs/TcpBbr0-throughtput.csv");
    // goodputPlotFilesnames.push_back("zlogs/TcpBbr0-goodput.csv");
    // rwndPlotFilesnames.push_back("zlogs/TcpBbr0-rwnd.csv");
    // bytesInFlightFilesnames.push_back("zlogs/TcpBbr0-bif.csv");

    // cwndPlotFilesnames.push_back("zlogs/TcpBbr1-cwnd.csv");
    // rttPlotFilesnames.push_back("zlogs/TcpBbr1-rtt.csv");
    // throughputPlotFilesnames.push_back("zlogs/TcpBbr1-throughtput.csv");
    // goodputPlotFilesnames.push_back("zlogs/TcpBbr1-goodput.csv");
    // rwndPlotFilesnames.push_back("zlogs/TcpBbr1-rwnd.csv");
    // bytesInFlightFilesnames.push_back("zlogs/TcpBbr1-bif.csv");



    generatePlot(cwndPlotFilesnames, "Congestion Window", "Cwnd (packets)");
    generatePlot(rttPlotFilesnames, "Round Trip Time", "RTT (ms)");
    generatePlot(throughputPlotFilesnames, "Throughput", "Throughput (Mbps)");
    generatePlot(goodputPlotFilesnames, "Goodput", "Goodput (Mbps)");
    generatePlot(bytesInFlightFilesnames, "BytesInFlight", "BytesInFlight (bytes)");
    //generatePlot(packetDropFilesnames , "Packets Dropped", "Packets (segments)");

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
            remove((goodputPlotFilesnames[i]).c_str());
            remove((bytesInFlightFilesnames[i]).c_str());
            //remove((packetDropFilesnames[i]).c_str());
        }
    remove((temp[0]).c_str());
    exit(0);
}