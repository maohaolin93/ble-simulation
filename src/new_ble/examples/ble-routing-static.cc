//#include <ns3/log.h>
#include <unordered_map>
#include "ns3/mac16-address.h"
#include "ns3/ipv4-address.h"
// 添加全局IP到MAC地址映射表，使用自定义哈希函数
std::unordered_map<ns3::Ipv4Address, ns3::Mac16Address, ns3::Ipv4AddressHash> g_ipToMacMap;

#include <ns3/core-module.h>
#include <ns3/ble-module.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/packet.h>
#include <ns3/rng-seed-manager.h>
#include <ns3/spectrum-module.h>
#include <ns3/mobility-module.h>
#include <ns3/energy-module.h>
#include <ns3/spectrum-value.h>
#include <ns3/spectrum-analyzer.h>
#include <iostream>
#include <ns3/isotropic-antenna-model.h>
#include <ns3/trace-helper.h>
#include <ns3/drop-tail-queue.h>
#include <unordered_map>
#include "ns3/network-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/network-module.h"
#include <ns3/okumura-hata-propagation-loss-model.h>
#include "ns3/aodv-module.h"
//#include "ns3/v4ping-helper.h"
#include <ns3/ipv6-routing-table-entry.h>
#include <ns3/ipv6-static-routing-helper.h>
#include <ns3/sixlowpan-module.h>
#include <cmath>

#include <ns3/mobility-helper.h>
#include <fstream>

#include "ns3/applications-module.h"
#include "ns3/udp-echo-client.h"
#include "ns3/udp-echo-server.h"
#include "ns3/onoff-application.h"
#include "ns3/packet-sink.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("BleRoutingAodv");

  /*****************
   * Configuration *
   *****************/ 

  int nbIterations = 1;
  double length = 5; //<! Square room with length as distance
  int pktsize = 20; //!< Size of packtets, in bytes
  int duration = 50; //<! Duration of the simulation in seconds
  int packetSendDuration = 10; 
  //<! Time during which new packets should be quied 
  bool verbose = true; // Enable logging
  bool nakagami = false; // enable nakagami path loss
  bool dynamic = false; // Wether the nodes are moving yes or no
  bool scheduled = true; 
  // Schedule the TX windows instead of random parameters.
  bool broadcastAvoidCollisions = true; 
    // Try to avoid 2 nodes being in advertising mode at the same time
  uint32_t nNodes = 3; // Number of nodes
  uint32_t nbConnInterval = 1000; 
    // [MAX 3200]  nbConnInterval*1,25ms = size of connection interval. 
    // if nbConnInterval = 0, each link will get a random conn interval
  int unicastInterval = 4; //!< Time between two packets from the same node 
  int broadcastInterval = 4*nNodes; 
    //!< Time between two packets from the same node 
    //(for good results, should be larger than nNodes*nbConnInterval(s) 
  int pingInterval = 20; // In seconds
  double internodedistance = 15.0; // De afstand tussen de nodes in meters.
  bool randomNodePlacement = false;

  /*std::vector<double> xx=
  { 10,    // 节点 0
    20,   // 节点 1
    10,    // 节点 2
    0,  // 节点 3
    10,    // 节点 4
    110,  // 节点 5
    120,  // 节点 6
    110,  // 节点 7
    100,   // 节点 8
    110   // 节点 9
  };
  std::vector<double> yy={
    10,   // 节点 0
    10,   // 节点 1
    20,  // 节点 2
    10,   // 节点 3
    0, // 节点 4
    10,   // 节点 5
    10,   // 节点 6
    20,  // 节点 7
    10,   // 节点 8
    0  // 节点 9
};
std::vector<double> zz={
    1,
    1,
    1,
    1,
    1,
    90,
    90,
    90,
    90,
    90
  };*/
std::vector<double> xx=
  { 90,    // 节点 0
    180,   // 节点 1
    90,    // 节点 2
    0,  // 节点 3
    90,    // 节点 4
    90,   // 节点 5
    180,  // 节点 6
    90,   // 节点 7
    0,     // 节点 8
    90   // 节点 9
  };
  std::vector<double> yy={
    90,    // 节点 0
    90,   // 节点 1
    180,    // 节点 2
    90,  // 节点 3
    0,    // 节点 4
    90,   // 节点 5
    90,   // 节点 6
    180,  // 节点 7
    90,    // 节点 8
    0   // 节点 9
  };
   std::vector<double> zz={
    1,
    1,
    1,
    1,
    1,
    90,
    90,
    90,
    90,
    90
  };
  /*std::vector<double> xx=
  { 10,    // 节点 0
    20,   // 节点 1
    10,    // 节点 2
    0,  // 节点 3
    10,    // 节点 4
  };
  std::vector<double> yy={
    10,   // 节点 0
    10,   // 节点 1
    20,  // 节点 2
    10,   // 节点 3
    0, // 节点 4
};
 std::vector<double> zz={
    1,
    1,
    1,
    1,
    1,
    300,
    300,
    300,
    300,
    300
  };*/
  Ptr<OutputStreamWrapper> m_stream = 0; // Stream for waterfallcurve
  Ptr<UniformRandomVariable> randT = CreateObject<UniformRandomVariable> ();

  std::unordered_map<uint32_t,std::tuple<uint32_t,uint32_t,uint32_t,
  uint32_t,uint32_t,uint32_t,uint32_t,uint32_t> > errorMap;
  std::unordered_map<uint32_t,Ptr<BleNetDevice> > deviceMap;
  


  /************************
   * End of configuration *
   ************************/

/// Save that the message has been transmitted
	void
Transmitted (const Ptr<const Packet> packet)
{
	Ptr<Packet> copy = packet->Copy();
	BleMacHeader header;
	copy->RemoveHeader(header);
    uint8_t buffer[2];
    header.GetSrcAddr().CopyTo(buffer);
    uint32_t addr = buffer[1];

	std::get<0>(errorMap[addr-2])++;
  std::cout<<"Tx"<<std::endl;
}

// save that a message has been received
	void
Received (const Ptr<const Packet> packet)
{
  //NS_LOG (LOG_DEBUG, "Packet received  " << packet);
	Ptr<Packet> copy = packet->Copy();
	BleMacHeader header;
	copy->RemoveHeader(header);
	uint8_t buffer[2];
    header.GetDestAddr().CopyTo(buffer);
    uint32_t addr = buffer[1];
    std::get<1>(errorMap[addr-2])++;
    std::cout<<"Rx"<<std::endl;
}

  void
ReceivedError (const Ptr<const Packet> packet)
{
  //NS_LOG (LOG_DEBUG, "Packet received  " << packet);
	Ptr<Packet> copy = packet->Copy();
	BleMacHeader header;
	copy->RemoveHeader(header);
	uint8_t buffer[2];
    header.GetDestAddr().CopyTo(buffer);
    uint32_t addr = buffer[1];
    std::get<3>(errorMap[addr-2])++;
}


// save that a message has been uniquely received
	void
ReceivedUnique (const Ptr<const Packet> packet)
{
	Ptr<Packet> copy = packet->Copy();
	BleMacHeader header;
	copy->RemoveHeader(header);
	uint8_t buffer[2];
    header.GetDestAddr().CopyTo(buffer);
    uint32_t addr = buffer[1];
	std::get<2>(errorMap[addr-2])++;
}

	void
ReceivedBroadcast (const Ptr<const Packet> packet, 
    const Ptr<const BleNetDevice>  netdevice)
{
	Ptr<Packet> copy = packet->Copy();
	BleMacHeader header;
	copy->RemoveHeader(header);
	uint8_t buffer[2];
    netdevice->GetAddress16().CopyTo(buffer);
    uint32_t addr = buffer[1];
	std::get<4>(errorMap[addr-2])++;
  std::cout<<"RxB"<<std::endl;
}

  void
TXWindowSkipped (const Ptr<const BleNetDevice> nd)
{
  //NS_LOG (LOG_DEBUG, "Packet received  " << packet);
	uint8_t buffer[2];
    nd->GetAddress16().CopyTo(buffer);
    uint32_t addr = buffer[1];
    std::get<5>(errorMap[addr-2])++;
}

// 创建星型子网络
void CreateStarSubnetwork(NetDeviceContainer& bleNetDevices,uint32_t centerNodeIndex,std::vector<uint32_t> slaveNodeIndices,
    bool scheduled,uint32_t& nbOffset,uint32_t nbConnInterval)
{
    NS_LOG_FUNCTION(centerNodeIndex << scheduled << nbOffset << nbConnInterval);
    Ptr<BleNetDevice> centerNode = DynamicCast<BleNetDevice>(bleNetDevices.Get(centerNodeIndex));
    NS_ASSERT_MSG(centerNode, "Failed to get center node " << centerNodeIndex);

    Ptr<BleBBManager> centerBBM = centerNode->GetBBManager();
    NS_ASSERT_MSG(centerBBM, "Failed to get BBManager for center node " << centerNodeIndex);

    for (uint32_t slaveIndex : slaveNodeIndices) {
        Ptr<BleNetDevice> slaveNode = DynamicCast<BleNetDevice>(bleNetDevices.Get(slaveIndex));
        NS_ASSERT_MSG(slaveNode, "Failed to get slave node " << slaveIndex);
        Ptr<BleBBManager> slaveBBM = slaveNode->GetBBManager();
        NS_ASSERT_MSG(slaveBBM, "Failed to get BBManager for slave node " << slaveIndex);

        Ptr<BleLink> link = centerBBM->CreateLinkScheduled(
            slaveBBM,
            BleLinkManager::Role::MASTER_ROLE,
            scheduled,
            nbOffset,
            nbConnInterval);
        NS_ASSERT_MSG(link, "Failed to create link from node " << centerNodeIndex << " to node " << slaveIndex);
        NS_LOG_INFO("Created link from node " << centerNodeIndex << " to node " << slaveIndex
                    << " with offset " << nbOffset << ", connInterval " << nbConnInterval * 1250 << "us");
        nbOffset++;
    }
}

// 创建中心节点间的链路
void CreateCenterToCenterLink(NetDeviceContainer& bleNetDevices,uint32_t centerNode1Index,uint32_t centerNode2Index,
    bool scheduled,uint32_t& nbOffset,uint32_t nbConnInterval)
{
    NS_LOG_FUNCTION(centerNode1Index << centerNode2Index << scheduled << nbOffset << nbConnInterval);
    Ptr<BleNetDevice> centerNode1 = DynamicCast<BleNetDevice>(bleNetDevices.Get(centerNode1Index));
    NS_ASSERT_MSG(centerNode1, "Failed to get center node " << centerNode1Index);

    Ptr<BleBBManager> centerBBM1 = centerNode1->GetBBManager();
    NS_ASSERT_MSG(centerBBM1, "Failed to get BBManager for center node " << centerNode1Index);

    Ptr<BleNetDevice> centerNode2 = DynamicCast<BleNetDevice>(bleNetDevices.Get(centerNode2Index));
    NS_ASSERT_MSG(centerNode2, "Failed to get center node " << centerNode2Index);

    Ptr<BleBBManager> centerBBM2 = centerNode2->GetBBManager();
    NS_ASSERT_MSG(centerBBM2, "Failed to get BBManager for center node " << centerNode2Index);

    Ptr<BleLink> link = centerBBM1->CreateLinkScheduled(
        centerBBM2,
        BleLinkManager::Role::MASTER_ROLE,
        scheduled,
        nbOffset,
        nbConnInterval);
    NS_ASSERT_MSG(link, "Failed to create link from node " << centerNode1Index << " to node " << centerNode2Index);
    NS_LOG_INFO("Created link from node " << centerNode1Index << " to node " << centerNode2Index
                << " with offset " << nbOffset << ", connInterval " << nbConnInterval * 1250 << "us");
    nbOffset++;
}

void CreateCenterToCenterLink_broadcast(NetDeviceContainer& bleNetDevices,uint32_t centerNode1Index,uint32_t centerNode2Index,
    bool scheduled,uint32_t& nbOffset,uint32_t nbConnInterval)
{
    NS_LOG_FUNCTION(centerNode1Index << centerNode2Index << scheduled << nbOffset << nbConnInterval);
    Ptr<BleNetDevice> centerNode1 = DynamicCast<BleNetDevice>(bleNetDevices.Get(centerNode1Index));
    NS_ASSERT_MSG(centerNode1, "Failed to get center node " << centerNode1Index);

    Ptr<BleBBManager> centerBBM1 = centerNode1->GetBBManager();
    NS_ASSERT_MSG(centerBBM1, "Failed to get BBManager for center node " << centerNode1Index);

    Ptr<BleNetDevice> centerNode2 = DynamicCast<BleNetDevice>(bleNetDevices.Get(centerNode2Index));
    NS_ASSERT_MSG(centerNode2, "Failed to get center node " << centerNode2Index);

    Ptr<BleBBManager> centerBBM2 = centerNode2->GetBBManager();
    NS_ASSERT_MSG(centerBBM2, "Failed to get BBManager for center node " << centerNode2Index);

    //uint32_t nbOffset = c.GetN()*(c.GetN()+1)/2;
    std::list<Ptr<BleBBManager>> otherBBManagers;
    otherBBManagers.push_back(centerBBM2);
    
    Ptr<BleLink> link = centerBBM1->CreateLinkScheduledMultipleNodes(
          otherBBManagers, 
          scheduled, nbOffset, nbConnInterval, broadcastAvoidCollisions);
    
    NS_ASSERT_MSG(link, "Failed to create link from node " << centerNode1Index << " to node " << centerNode2Index);
    NS_LOG_INFO("Created link from node " << centerNode1Index << " to node " << centerNode2Index
                << " with offset " << nbOffset << ", connInterval " << nbConnInterval * 1250 << "us");
    nbOffset++;
}

// 创建多个星型子网络拓扑
void CreateStarToStarTopology(NetDeviceContainer& bleNetDevices, bool scheduled, uint32_t nbConnInterval)
{
    NS_LOG_INFO("Creating star-to-star topology with " << bleNetDevices.GetN() << " nodes");

    // 定义星型子网络：每个子网络的中心节点和从节点
    std::vector<std::pair<uint32_t, std::vector<uint32_t>>> subnetworks = {
        {0,{1, 2}},
        //{1,{0}},
        //{2,{0}}
        //{0, {1, 2, 3, 4}},      // 中心节点 0 连接从节点 1~4
        //{5, {6, 7, 8, 9}},      // 中心节点 5 连接从节点 6~9
        //{10, {11, 12, 13, 14}},      // 中心节点 10 连接从节点 11~13
        //{15, {16, 17, 18, 19}},
        //{20, {21, 22, 23, 24}},
    };

    // 中心节点间的连接
    std::vector<std::pair<uint32_t, uint32_t>> centerLinks = {
        //{0, 5},   // 节点 0 和节点 5
        //{5, 10},   // 节点 5 和节点 10
        //{10, 15},
        //{15, 20},
        //{20, 0}
    };

    uint32_t nbOffset = 0;

    // 创建星型子网络
    for (const auto& subnet : subnetworks) {
        uint32_t centerIndex = subnet.first;
        const std::vector<uint32_t>& slaveIndices = subnet.second;
        NS_ASSERT_MSG(centerIndex < bleNetDevices.GetN(), "Center node index " << centerIndex << " out of range");
        for (uint32_t slaveIndex : slaveIndices) {
        NS_ASSERT_MSG(slaveIndex < bleNetDevices.GetN(), "Slave node index " << slaveIndex << " out of range");
        }
        CreateStarSubnetwork(bleNetDevices, centerIndex, slaveIndices, scheduled, nbOffset, nbConnInterval);
    }

    // 创建中心节点间的链路
    for (const auto& link : centerLinks) {
        uint32_t center1 = link.first;
        uint32_t center2 = link.second;
        NS_ASSERT_MSG(center1 < bleNetDevices.GetN() && center2 < bleNetDevices.GetN(),
                    "Center node indices " << center1 << ", " << center2 << " out of range");
        CreateCenterToCenterLink(bleNetDevices, center1, center2, scheduled, nbOffset, nbConnInterval);
         // 创建广播链路
        //CreateCenterToCenterLink_broadcast(bleNetDevices, center1, center2, scheduled, nbOffset, nbConnInterval);
    }

   
    /*BleHelper helper;
    helper.CreateBroadcastLink(bleNetDevices, scheduled, nbConnInterval, broadcastAvoidCollisions);
    NS_LOG_INFO("Created broadcast link for all nodes, connInterval=" << nbConnInterval * 1250
                << "us, avoidCollisions=" << (broadcastAvoidCollisions ? "true" : "false"));*/
}

// 设置IP到MAC地址的映射
void SetupIpToMacMapping(NetDeviceContainer& bleNetDevices, Ipv4InterfaceContainer& interfaces)
{
    NS_LOG_INFO("Setting up IP to MAC address mapping");
    
    for (uint32_t i = 0; i < bleNetDevices.GetN(); i++) {
        Ptr<BleNetDevice> device = DynamicCast<BleNetDevice>(bleNetDevices.Get(i));
        ns3::Ipv4Address ipAddr = interfaces.GetAddress(i);
        ns3::Mac16Address macAddr = device->GetAddress16();
        
        g_ipToMacMap[ipAddr] = macAddr;
        NS_LOG_INFO("Mapped IP " << ipAddr << " to MAC " << macAddr);
    }
    
    // 同时更新到 IPv4Interface 可访问的全局映射表
    extern std::unordered_map<ns3::Ipv4Address, ns3::Mac16Address, ns3::Ipv4AddressHash>* GetGlobalIpToMacMap();
    auto* globalMap = GetGlobalIpToMacMap();
    globalMap->clear();
    for (const auto& pair : g_ipToMacMap) {
        (*globalMap)[pair.first] = pair.second;
    }
}

// 获取目标MAC地址的辅助函数
Mac16Address GetDestinationMac(Ipv4Address destIp)
{
    auto it = g_ipToMacMap.find(destIp);
    if (it != g_ipToMacMap.end()) {
        return it->second;
    }
    NS_LOG_WARN("No MAC found for IP " << destIp << ", using broadcast");
    return ns3::Mac16Address("FF:FF");
}

int main (int argc, char** argv)
{
  bool pcap = false;
  bool printRoutes = true;

  CommandLine cmd;
  cmd.AddValue ("verbose", "Tell application to log if true", verbose);
  cmd.AddValue ("pcap", "Write PCAP traces.", pcap);


  cmd.Parse (argc,argv);
  //LogComponentEnableAll (LOG_PREFIX_TIME);
  //LogComponentEnableAll (LOG_PREFIX_FUNC);
  //LogComponentEnableAll (LOG_PREFIX_LEVEL);
  //LogComponentEnableAll (LOG_PREFIX_NODE);
 
  // 保存原始 clog 缓冲区以便在程序结束时恢复
    std::streambuf* originalClogBuf = std::clog.rdbuf();
    std::ofstream logFile;

    // 创建日志文件并重定向 clog
    if (verbose)
    {
        logFile.open("ble-log.log", std::ios::out | std::ios::trunc);
        if (!logFile.is_open())
        {
            std::cerr << "错误：无法打开日志文件 ble-log.log" << std::endl;
            return 1;
        }
        // 重定向 clog 到日志文件
        std::clog.rdbuf(logFile.rdbuf());
        // 启用所有日志级别
        //LogComponentEnableAll(LOG_LEVEL_ALL);
        // 可选：启用特定组件的日志以减少输出
        LogComponentEnable("BleNetDevice", LOG_LEVEL_ALL);
        LogComponentEnable("BlePhy", LOG_LEVEL_ALL);
        //LogComponentEnable("BleBBManager", LOG_LEVEL_ALL);
        LogComponentEnable("Ipv4Interface", LOG_LEVEL_ALL);
        LogComponentEnable("BleLinkController", LOG_LEVEL_ALL);
        LogComponentEnable("BleLinkManager", LOG_LEVEL_ALL);
        //LogComponentEnable("BleApplication", LOG_LEVEL_ALL);
        //LogComponentEnable("BleLink", LOG_LEVEL_ALL);
        //LogComponentEnable("BleHelper", LOG_LEVEL_ALL);
        LogComponentEnableAll (LOG_PREFIX_TIME);
        LogComponentEnableAll (LOG_PREFIX_FUNC);
        LogComponentEnableAll (LOG_PREFIX_LEVEL);
        LogComponentEnableAll (LOG_PREFIX_NODE);
    }
  
  // Enable logging
  BleHelper helper;
  if (verbose)
  {
    //helper.EnableLogComponents();
  }

  Packet::EnablePrinting ();
  Packet::EnableChecking ();

  NS_LOG_INFO ("BLE Routing example file");

  // Enable debug output
  NS_LOG_INFO ("Enable debug output");
  AsciiTraceHelper ascii;
  //helper.EnableAsciiAll (ascii.CreateFileStream ("example-ble.tr"));
  m_stream = ascii.CreateFileStream ("example-routing.csv");
  *m_stream->GetStream() << "#Scenario " << (int)nNodes 
    <<  " nodes on a square field with side " << length << " meter" 
    << " TX window scheduling enabled: " << scheduled 
    << ", connection interval = " << nbConnInterval*1.25 
    << " millisec, (0 = random) " << std::endl;
  // print Iteration, ID, transmitted, received, received unique, 
  // received at closest gateway, x coords, y coords, 
  // get average amount of retransmissions, get average time of transmissions, 
  // number of missed messages, amount of received messages.
  *m_stream->GetStream() << "Iteration, ID, transmitted, received, "
    "received unique, received error, broadcast received, TX Windows Skipped, "
    "x coords, y coords " <<std::endl;
  for (uint8_t iterationI=0;iterationI<nbIterations;iterationI++){
		std::cout << "Iteration: " << (int)iterationI << std::endl;
 
    randT->SetAttribute("Max", DoubleValue (600));
   
    NS_LOG (LOG_INFO, "Ble setup starts now");

    NodeContainer bleDeviceNodes;
    bleDeviceNodes.Create(nNodes);


    // Create mobility
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> nodePositionList = 
      CreateObject<ListPositionAllocator> (); 
    int i=0;
    for (uint32_t nodePositionsAssigned = 0; nodePositionsAssigned < nNodes; nodePositionsAssigned++)
    {
      double x,y,z;
      if (randomNodePlacement)
      {
        x = randT->GetInteger(0,length);
        y = randT->GetInteger(0,length);
      }
      else {
        //x = double (nodePositionsAssigned)*internodedistance;
        //y = 0.0;
        //z = 1.0;
        x=xx[i];
        y=yy[i];
        z=zz[i];
        i++;
      }
      NS_LOG (LOG_INFO, "x = " << x << " y = " << y);
      nodePositionList->Add (Vector (x,y,z));
    }
    mobility.SetPositionAllocator (nodePositionList);
   
    if (dynamic)
      mobility.SetMobilityModel ("ns3::RandomWalk2DMobilityModel");
    else
      mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install(bleDeviceNodes);
    
    /*Ptr<MobilityModel> node0Mobility = bleDeviceNodes.Get(0)->GetObject<MobilityModel>();
    Ptr<MobilityModel> node5Mobility = bleDeviceNodes.Get(5)->GetObject<MobilityModel>();
    node0Mobility->SetPosition(Vector(0, 0, 0));   // 节点 0 在 (0, 0)
    node5Mobility->SetPosition(Vector(100, 0, 0)); // 节点 5 在 (100, 0)
    // 节点 1~4 围绕节点 0
    bleDeviceNodes.Get(1)->GetObject<MobilityModel>()->SetPosition(Vector(10, 0, 0));
    bleDeviceNodes.Get(2)->GetObject<MobilityModel>()->SetPosition(Vector(0, 10, 0));
    bleDeviceNodes.Get(3)->GetObject<MobilityModel>()->SetPosition(Vector(-10, 0, 0));
    bleDeviceNodes.Get(4)->GetObject<MobilityModel>()->SetPosition(Vector(0, -10, 0));
    // 节点 6~9 围绕节点 5
    bleDeviceNodes.Get(6)->GetObject<MobilityModel>()->SetPosition(Vector(110, 0, 0));
    bleDeviceNodes.Get(7)->GetObject<MobilityModel>()->SetPosition(Vector(100, 10, 0));
    bleDeviceNodes.Get(8)->GetObject<MobilityModel>()->SetPosition(Vector(90, 0, 0));
    bleDeviceNodes.Get(9)->GetObject<MobilityModel>()->SetPosition(Vector(100, -10, 0));*/

    // Create the nodes
    NetDeviceContainer bleNetDevices;
    bleNetDevices = helper.Install (bleDeviceNodes);

    // Install Internet stack
    AodvHelper aodv;
    InternetStackHelper stack;
    aodv.Set("HelloInterval", TimeValue(Seconds(36)));
    aodv.Set("ActiveRouteTimeout", TimeValue(Seconds(480)));
    aodv.Set("BlackListTimeout", TimeValue(Seconds(488)));
    aodv.Set("NodeTraversalTime", TimeValue(Seconds(40)));
    aodv.Set("NextHopWait", TimeValue(Seconds(40)));
    stack.SetRoutingHelper (aodv);
    stack.Install (bleDeviceNodes);
   
    Ipv4AddressHelper address;
    address.SetBase ("10.0.0.0", "255.0.0.0");

    Ipv4InterfaceContainer interfaces;
    interfaces = address.Assign (bleNetDevices);

    // 设置IP到MAC地址的映射
    SetupIpToMacMapping(bleNetDevices, interfaces);

    if (printRoutes)
    {
      Ptr<OutputStreamWrapper> routingStream = 
        Create<OutputStreamWrapper> ("aodv5.routes", std::ios::out);
      aodv.PrintRoutingTableAllAt (Seconds (5), routingStream);
      Ptr<OutputStreamWrapper> routingStream2 = 
        Create<OutputStreamWrapper> ("aodv15.routes", std::ios::out);
      aodv.PrintRoutingTableAllAt (Seconds (15), routingStream2);
      Ptr<OutputStreamWrapper> routingStream3 = 
        Create<OutputStreamWrapper> ("aodv16.routes", std::ios::out);
      aodv.PrintRoutingTableAllAt (Seconds (16), routingStream3);
      Ptr<OutputStreamWrapper> routingStream4 = 
        Create<OutputStreamWrapper> ("aodv17.routes", std::ios::out);
      aodv.PrintRoutingTableAllAt (Seconds (17), routingStream4);
      Ptr<OutputStreamWrapper> routingStream5 = 
        Create<OutputStreamWrapper> ("aodv18.routes", std::ios::out);
      aodv.PrintRoutingTableAllAt (Seconds (18), routingStream5);
      Ptr<OutputStreamWrapper> routingStream6 = 
        Create<OutputStreamWrapper> ("aodv19.routes", std::ios::out);
      aodv.PrintRoutingTableAllAt (Seconds (19), routingStream6);
      Ptr<OutputStreamWrapper> routingStream7 = 
        Create<OutputStreamWrapper> ("aodv20.routes", std::ios::out);
      aodv.PrintRoutingTableAllAt (Seconds (20), routingStream7);
      Ptr<OutputStreamWrapper> routingStream8 = 
        Create<OutputStreamWrapper> ("aodv21.routes", std::ios::out);
      aodv.PrintRoutingTableAllAt (Seconds (21), routingStream8);
      Ptr<OutputStreamWrapper> routingStream9 = 
        Create<OutputStreamWrapper> ("aodv22.routes", std::ios::out);
      aodv.PrintRoutingTableAllAt (Seconds (22), routingStream9);
      Ptr<OutputStreamWrapper> routingStream10 = 
        Create<OutputStreamWrapper> ("aodv23.routes", std::ios::out);
      aodv.PrintRoutingTableAllAt (Seconds (23), routingStream10);
      Ptr<OutputStreamWrapper> routingStream11 = 
        Create<OutputStreamWrapper> ("aodv25.routes", std::ios::out);
      aodv.PrintRoutingTableAllAt (Seconds (25), routingStream11);
      Ptr<OutputStreamWrapper> routingStream12 = 
        Create<OutputStreamWrapper> ("aodv28.routes", std::ios::out);
      aodv.PrintRoutingTableAllAt (Seconds (28), routingStream12);
      Ptr<OutputStreamWrapper> routingStream13 = 
        Create<OutputStreamWrapper> ("aodv29.routes", std::ios::out);
      aodv.PrintRoutingTableAllAt (Seconds (29), routingStream13);
      Ptr<OutputStreamWrapper> routingStream14 = 
        Create<OutputStreamWrapper> ("aodv30.routes", std::ios::out);
      aodv.PrintRoutingTableAllAt (Seconds (30), routingStream14);
      Ptr<OutputStreamWrapper> routingStream15 = 
        Create<OutputStreamWrapper> ("aodv31.routes", std::ios::out);
      aodv.PrintRoutingTableAllAt (Seconds (31), routingStream15);
    }

    // // Create links between the nodes
    // helper.CreateAllLinks (bleNetDevices, scheduled, nbConnInterval);
    // helper.CreateBroadcastLink (bleNetDevices, scheduled, 
    //     nbConnInterval, broadcastAvoidCollisions);
    NS_LOG_INFO ("Creating custom star2star topo");

    CreateStarToStarTopology(bleNetDevices, scheduled, nbConnInterval);
    /*uint32_t nbOffset = 0;

    Ptr<BleNetDevice> centerNode0 = DynamicCast<BleNetDevice>(bleNetDevices.Get(0));
    Ptr<BleBBManager> centerNode0_BBM = centerNode0->GetBBManager();
    for(uint32_t i = 1; i <= 4; ++i){
      Ptr<BleNetDevice> starNode = DynamicCast<BleNetDevice>(bleNetDevices.Get(i));
      Ptr<BleBBManager> starNode_BBM = starNode->GetBBManager();
      centerNode0_BBM->CreateLinkScheduled(
        starNode_BBM,
        BleLinkManager::Role::MASTER_ROLE,
        scheduled,
        nbOffset,
        nbConnInterval
      );
      nbOffset++;
    }

    Ptr<BleNetDevice> centerNode5 = DynamicCast<BleNetDevice>(bleNetDevices.Get(0));
    Ptr<BleBBManager> centerNode5_BBM = centerNode0->GetBBManager();
    for(uint32_t i = 6; i <= 9; ++i){
      Ptr<BleNetDevice> starNode = DynamicCast<BleNetDevice>(bleNetDevices.Get(i));
      Ptr<BleBBManager> starNode_BBM = starNode->GetBBManager();
      centerNode5_BBM->CreateLinkScheduled(
        starNode_BBM,
        BleLinkManager::Role::MASTER_ROLE,
        scheduled,
        nbOffset,
        nbConnInterval
      );
      nbOffset++;
    }

    centerNode0_BBM->CreateLinkScheduled(
      centerNode5_BBM,
      BleLinkManager::Role::MASTER_ROLE,
      scheduled,
      nbOffset,
      nbConnInterval
    );
    nbOffset++;*/

    helper.CreateBroadcastLink (bleNetDevices, scheduled, nbConnInterval, broadcastAvoidCollisions);
   
    NS_LOG (LOG_INFO, " Generate data ");
    //ApplicationContainer apps1 = helper.GenerateBroadcastTraffic 
    // (randT, bleDeviceNodes, pktsize, 0, packetSendDuration, broadcastInterval);
    //ApplicationContainer apps2 = helper.GenerateTraffic 
    // (randT, bleDeviceNodes, pktsize, 0, packetSendDuration, unicastInterval);

    ApplicationContainer app;

  /*for (uint32_t src = 0; src < nNodes; src++)        // 源节点
  {
    for (uint32_t dst = 0; dst < nNodes; dst++)    // 目标节点
    {
        if (src == dst) continue; // 跳过自己

        V4PingHelper ping(interfaces.GetAddress(dst)); // 目标地址
        ping.SetAttribute("Verbose", BooleanValue(true));
        ping.SetAttribute("Interval", TimeValue(Seconds(pingInterval)));

        ApplicationContainer p = ping.Install(bleDeviceNodes.Get(src)); // 安装在源节点
        p.Start(Seconds(10+50*src)); // 所有 Ping 从第 10 秒开始
        p.Stop(Seconds(duration - 10)); // 在结束前 10 秒停止
        app.Add(p);
    }
  }*/

  /*uint32_t src = 1;
  for (uint32_t dst = 0; dst < nNodes; dst++)        // 源节点
  {
        if (src == dst) continue; // 跳过自己

        V4PingHelper ping(interfaces.GetAddress(dst)); // 目标地址
        ping.SetAttribute("Verbose", BooleanValue(true));
        ping.SetAttribute("Interval", TimeValue(Seconds(pingInterval)));

        ApplicationContainer p = ping.Install(bleDeviceNodes.Get(src)); // 安装在源节点
        p.Start(Seconds(10)); // 所有 Ping 从第 10 秒开始
        p.Stop(Seconds(20)); // 在结束前 10 秒停止
        app.Add(p);
  }*/

  /*ping流量
  uint32_t src = 1;
  uint32_t dst = 2;
  V4PingHelper ping(interfaces.GetAddress(dst)); // 目标地址
  ping.SetAttribute("Verbose", BooleanValue(true));
  ping.SetAttribute("Interval", TimeValue(Seconds(pingInterval)));

  ApplicationContainer p = ping.Install(bleDeviceNodes.Get(src)); // 安装在源节点
  p.Start(Seconds(10)); // 所有 Ping 从第 10 秒开始
  p.Stop(Seconds(31)); // 在结束前 10 秒停止
  app.Add(p);
  */
  // 改为UDP流量设置：
uint32_t src = 1;
uint32_t dst = 2;

// 创建UDP客户端和服务端
uint16_t port = 9;  // Discard端口
UdpEchoClientHelper client(interfaces.GetAddress(dst), port);
client.SetAttribute("MaxPackets", UintegerValue(5));  // 发送5个包
client.SetAttribute("Interval", TimeValue(Seconds(30.0))); // 每20秒发送一次
client.SetAttribute("PacketSize", UintegerValue(pktsize)); // 使用配置的包大小

UdpEchoServerHelper server(port);

// 安装服务端和客户端应用
ApplicationContainer serverApps = server.Install(bleDeviceNodes.Get(dst));
serverApps.Start(Seconds(9));  // 服务端提前启动
serverApps.Stop(Seconds(40));

ApplicationContainer clientApps = client.Install(bleDeviceNodes.Get(src));
clientApps.Start(Seconds(25)); // 客户端从第10秒开始
clientApps.Stop(Seconds(40));  // 到第31秒结束

app.Add(serverApps);
app.Add(clientApps);
    
  

  if(pcap){
    helper.EnablePcap("ble-star2star", bleNetDevices, true);
  }
     // Hookup functions to measure performance
    for (uint32_t i=0; i< bleNetDevices.GetN(); i++)
    {
      uint8_t buffer[2];
      Mac16Address::ConvertFrom(bleNetDevices.Get(i)->GetAddress()).CopyTo(buffer);
      uint32_t addr = buffer[1];  
      deviceMap[addr ]=DynamicCast<BleNetDevice>(bleNetDevices.Get(i));
      uint32_t x  = bleNetDevices.Get(i)->GetNode()
        ->GetObject<MobilityModel>()->GetPosition ().x;
      uint32_t y  = bleNetDevices.Get(i)->GetNode()
        ->GetObject<MobilityModel>()->GetPosition ().y;
      errorMap[addr-2] = std::make_tuple (0,0,0,0,0,0,x,y);
      DynamicCast<BleNetDevice>(bleNetDevices.Get(i))
        ->TraceConnectWithoutContext ("MacTx",MakeCallback(&Transmitted));
      DynamicCast<BleNetDevice>(bleNetDevices.Get(i))
        ->TraceConnectWithoutContext ("MacRx",MakeCallback(&ReceivedUnique));
      DynamicCast<BleNetDevice>(bleNetDevices.Get(i))
        ->TraceConnectWithoutContext ("MacRxBroadcast",
            MakeCallback(&ReceivedBroadcast));
      DynamicCast<BleNetDevice>(bleNetDevices.Get(i))
        ->TraceConnectWithoutContext ("MacPromiscRx",MakeCallback(&Received));
      DynamicCast<BleNetDevice>(bleNetDevices.Get(i))
        ->TraceConnectWithoutContext ("MacRxError",MakeCallback(&ReceivedError));
      DynamicCast<BleNetDevice>(bleNetDevices.Get(i))
        ->TraceConnectWithoutContext ("TXWindowSkipped",
            MakeCallback(&TXWindowSkipped));
    }

    NS_LOG (LOG_INFO, "Simulator will run now");
    
    Simulator::Stop(Seconds (duration));
    Simulator::Run ();

    for (uint32_t i=0; i< bleNetDevices.GetN(); i++)
    {
            uint8_t buffer[2];
            
            Mac16Address::ConvertFrom(
                bleNetDevices.Get(i)->GetAddress()).CopyTo(buffer);
            uint32_t addr = buffer[1];  
            Ptr<BleNetDevice> netdevice =
              DynamicCast<BleNetDevice>(bleNetDevices.Get(i));
            NS_LOG (LOG_DEBUG, "nd = " << netdevice << " addr = " << addr);
			std::tuple<uint32_t,uint32_t,uint32_t,uint32_t,
              uint32_t,uint32_t,uint32_t,uint32_t> tuple = errorMap[addr-2];
			// print iteration, ID, transmitted, 
            // received, received unique, x coords, y coords.
			*m_stream->GetStream() << (int)iterationI << "," 
              << netdevice->GetAddress16() << "," << std::get<0>(tuple)
              << "," << std::get<1>(tuple) << "," <<   std::get<2>(tuple) 
              <<  "," << std::get<3>(tuple) << "," << std::get<4>(tuple) 
              << "," << std::get<5>(tuple)  << "," << std::get<6>(tuple) 
              << "," << std::get<7>(tuple) << std::endl;
		
    }
    errorMap.clear();
    Simulator::Destroy ();

  }
  
  // 恢复原始 clog 缓冲区并关闭日志文件
    if (verbose)
    {
        std::clog.rdbuf(originalClogBuf);
        logFile.close();
    }

  NS_LOG_INFO ("Done.");
  return 0;
}