/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 KU Leuven
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Stijn Geysen <stijn.geysen@student.kuleuven.be> 
 *          Based on the lora ns-3 module written by Brecht Reynders.
 *          This module can be found here:
 *https://github.com/networkedsystems/lora-ns3/blob/master/model/lora-mac-header.h
 
 */



// Include a header file from your module to test.
//#include <ns3/log.h>
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
#include "ns3/dsdv-module.h"
//#include "ns3/v4ping-helper.h"
#include <ns3/ipv6-routing-table-entry.h>
#include <ns3/ipv6-static-routing-helper.h>
#include <ns3/sixlowpan-module.h>
#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("BleRoutingDsdv");

  /*****************
   * Configuration *
   *****************/ 

  int nbIterations = 1;
  double length = 1000; //<! Square room with length as distance
  int pktsize = 20; //!< Size of packtets, in bytes
  int duration = 130; //<! Duration of the simulation in seconds
  int packetSendDuration = 10; //<! Time during which new packets should be quied 
  bool verbose = true; // Enable logging
  bool nakagami = false; // enable nakagami path loss
  bool dynamic = false; // Wether the nodes are moving yes or no
  bool scheduled = true; // Schedule the TX windows instead of random parameters.
  //不避免广播冲突，与 AODV 版本不同
  bool broadcastAvoidCollisions = true; // Try to avoid 2 nodes being in advertising mode at the same time
  uint32_t nNodes = 10; // Number of nodes
  //uint32_t nbConnInterval = 3200; // [MAX 3200]  nbConnInterval*1,25ms = size of connection interval. if nbConnInterval = 0, each link will get a random conn interval
  uint32_t nbConnInterval = 1000; // [MAX 3200]  nbConnInterval*1,25ms = size of connection interval. if nbConnInterval = 0, each link will get a random conn interval
  int unicastInterval = 4; //!< Time between two packets from the same node 
  int broadcastInterval = 4*nNodes; //!< Time between two packets from the same node (for good results, should be larger than nNodes*nbConnInterval(s) 
  int pingInterval = 10; // In seconds
  double internodedistance = 15.0; // De afstand tussen de nodes in meters.

  Ptr<OutputStreamWrapper> m_stream = 0; // Stream for waterfallcurve
  Ptr<UniformRandomVariable> randT = CreateObject<UniformRandomVariable> ();

  std::unordered_map<uint32_t,std::tuple<uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t> > errorMap;
  //Errormap: transmitted, received, received unique, received original, xlocation, ylocation
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

    //NS_LOG (LOG_DEBUG, "Packet transmitted  " << packet << " src addr = " << addr);
	std::get<0>(errorMap[addr-2])++;
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
    //NS_LOG (LOG_DEBUG, "Packet unique received  " << packet << " dest addr = " << addr);
	std::get<2>(errorMap[addr-2])++;
}

	void
ReceivedBroadcast (const Ptr<const Packet> packet, const Ptr<const BleNetDevice>  netdevice)
{
	Ptr<Packet> copy = packet->Copy();
	BleMacHeader header;
	copy->RemoveHeader(header);
	uint8_t buffer[2];
    netdevice->GetAddress16().CopyTo(buffer);
    uint32_t addr = buffer[1];
    //NS_LOG (LOG_DEBUG, "Packet unique received  " << packet << " dest addr = " << addr);
	std::get<4>(errorMap[addr-2])++;
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


int main (int argc, char** argv)
{
  bool pcap = false;
  bool printRoutes = true;

  CommandLine cmd;
  cmd.AddValue ("verbose", "Tell application to log if true", verbose);
  cmd.AddValue ("pcap", "Write PCAP traces.", pcap);


  cmd.Parse (argc,argv);
  //减少日志输出，依赖 verbose 控制日志
 // LogComponentEnableAll (LOG_PREFIX_TIME);
 // LogComponentEnableAll (LOG_PREFIX_FUNC);
 // LogComponentEnableAll (LOG_PREFIX_LEVEL);
 // LogComponentEnableAll (LOG_PREFIX_NODE);
 
  // Enable logging
  BleHelper helper;
  if (verbose)
  {
    helper.EnableLogComponents();
  }
  
 // LogComponentEnable ("Ipv4AddressHelper", LOG_LEVEL_ALL);
 // LogComponentEnable ("AodvRoutingProtocol", LOG_LEVEL_ALL);
 // LogComponentEnable ("AodvNeighbors", LOG_LEVEL_ALL);
 // LogComponentEnable ("AodvRequestQueue", LOG_LEVEL_ALL);
 // LogComponentEnable ("AodvRoutingTable", LOG_LEVEL_ALL);
 // LogComponentEnable ("GlobalRoutingHelper", LOG_LEVEL_ALL);
 // LogComponentEnable ("TraceHelper", LOG_LEVEL_ALL);
    


  Packet::EnablePrinting ();
  Packet::EnableChecking ();

  NS_LOG_INFO ("BLE Routing example file");

  // Enable debug output
  NS_LOG_INFO ("Enable debug output");
  AsciiTraceHelper ascii;
  //helper.EnableAsciiAll (ascii.CreateFileStream ("example-ble.tr"));
  m_stream = ascii.CreateFileStream ("example-routing.csv");
  *m_stream->GetStream() << "#Scenario " << (int)nNodes <<  " nodes on a square field with side " << length << " meter" << " TX window scheduling enabled: " << scheduled << ", connection interval = " << nbConnInterval*1.25 << " millisec, (0 = random) " << std::endl;
  // print Iteration, ID, transmitted, received, received unique, received at closest gateway, x coords, y coords, get average amount of retransmissions, get average time of transmissions, number of missed messages, amount of received messages.
  //迭代编号、节点 ID、发送、接收、唯一接收、错误接收、广播接收、跳过传输窗口、坐标
  *m_stream->GetStream() << "Iteration, ID, transmitted, received, received unique, received error, broadcast received, TX Windows Skipped, x coords, y coords " <<std::endl;
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
    for (uint32_t nodePositionsAssigned = 0; nodePositionsAssigned < nNodes; nodePositionsAssigned++)
    {
      double x,y;
      x = double (nodePositionsAssigned)*internodedistance;
      y = 0.0;
      //x = randT->GetInteger(0,length);
      //y = randT->GetInteger(0,length);
      NS_LOG (LOG_INFO, "x = " << x << " y = " << y);
      nodePositionList->Add (Vector (x,y,1.0));
    }

    //设置移动模型并安装
    mobility.SetPositionAllocator (nodePositionList);
    if (dynamic)
      mobility.SetMobilityModel ("ns3::RandomWalk2DMobilityModel");
    else
      mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install(bleDeviceNodes);
    
    // Create the nodes
    NetDeviceContainer bleNetDevices;
    bleNetDevices = helper.Install (bleDeviceNodes);

    // Set addresses
   // NS_LOG (LOG_INFO, "Set addresses");
   // for (uint32_t nodeI = 0; nodeI < nNodes; nodeI++)
   // {
   //   //std::string s = std::to_string (std::hex(nodeI+2));

   //   std::stringstream stream;
   //   stream << std::hex << nodeI+2;
   //   std::string s( stream.str());
   //   while (s.size() < 4)
   //     s.insert(0,1,'0');
   //   s.insert(2,1,':');
   //   char const * buffer = s.c_str();
   //   DynamicCast<BleNetDevice>(bleNetDevices.Get(nodeI))->SetAddress (Mac16Address (buffer));
   //   NS_LOG (LOG_INFO, "address = " << DynamicCast<BleNetDevice>(bleNetDevices.Get(nodeI))->GetAddress ());
   // }

    // Install Internet stack
   // AodvHelper aodv;
   // InternetStackHelper stack;
   // aodv.Set("HelloInterval", TimeValue(Seconds(6)));
   // aodv.Set("ActiveRouteTimeout", TimeValue(Seconds(20)));
   // aodv.Set("BlackListTimeout", TimeValue(Seconds(18)));
   // aodv.Set("NodeTraversalTime", TimeValue(Seconds(3)));
   // aodv.Set("NextHopWait", TimeValue(Seconds(4)));
    
    //配置 DSDV 路由协议
    DsdvHelper dsdv;
    dsdv.Set ("SettlingTime", TimeValue (Seconds (6)));
    dsdv.Set ("PeriodicUpdateInterval", TimeValue (Seconds (15)));

    InternetStackHelper stack;
    stack.SetRoutingHelper (dsdv);
    stack.Install (bleDeviceNodes);

 //   SixLowPanHelper sixlowpan;
 //   sixlowpan.SetDeviceAttribute ("ForceEtherType", BooleanValue (true));

 //   NetDeviceContainer six1 = sixlowpan.Install (bleNetDevices);
 //   Ipv6AddressHelper ipv6;
 //   ipv6.SetBase (Ipv6Address ("2001:1::"), Ipv6Prefix (64));
 //   Ipv6InterfaceContainer interfaces = ipv6.Assign (six1);
 //   interfaces.SetForwarding (1, true);
 //   interfaces.SetDefaultRouteInAllNodes (0);
    
    Ipv4AddressHelper address;
    address.SetBase ("10.0.0.0", "255.0.0.0");

    Ipv4InterfaceContainer interfaces;
    interfaces = address.Assign (bleNetDevices);

    //打印 DSDV 路由表
    if (printRoutes)
    {
      Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("dsdv.routes", std::ios::out);
      dsdv.PrintRoutingTableAllAt (Seconds (duration - 10), routingStream);
    }

    // Create links between the nodes
    helper.CreateAllLinks (bleNetDevices, scheduled, nbConnInterval);
    //创建广播链接，不避免冲突
    helper.CreateBroadcastLink (bleNetDevices, scheduled, nbConnInterval, broadcastAvoidCollisions);
   
    NS_LOG (LOG_INFO, " Generate data ");
    //ApplicationContainer apps1 = helper.GenerateBroadcastTraffic (randT, bleDeviceNodes, pktsize, 0, packetSendDuration, broadcastInterval);
    //ApplicationContainer apps2 = helper.GenerateTraffic (randT, bleDeviceNodes, pktsize, 0, packetSendDuration, unicastInterval);

    //配置节点 0 向节点 9（10.0.0.10）发送 ping 请求
    V4PingHelper ping (interfaces.GetAddress (nNodes-1));

   // Ping6Helper ping; // (interfaces.GetAddress (nNodes-1));
   // ping.SetLocal (interfaces.GetAddress (0,1));
    ping.SetAttribute ("Verbose", BooleanValue (true));
    ping.SetAttribute ("Interval", TimeValue (Seconds (pingInterval)));

    ApplicationContainer p = ping.Install (bleDeviceNodes.Get (0));
    p.Start (Seconds (10));
    p.Stop (Seconds (duration) - Seconds (50));

    // Move a node to see effect on routing
   // Ptr<Node> node = bleDeviceNodes.Get (nNodes/2);
   // Ptr<MobilityModel> mob = node->GetObject<MobilityModel> ();
   // Simulator::Schedule (Seconds (duration/3), &MobilityModel::SetPosition, mob, Vector (1e5, 1e5, 1e5));
    
     // Hookup functions to measure performance

    for (uint32_t i=0; i< bleNetDevices.GetN(); i++)
    {
      uint8_t buffer[2];
      Mac16Address::ConvertFrom(bleNetDevices.Get(i)->GetAddress()).CopyTo(buffer);
      uint32_t addr = buffer[1];  
      deviceMap[addr ]=DynamicCast<BleNetDevice>(bleNetDevices.Get(i));
      uint32_t x  = bleNetDevices.Get(i)->GetNode()->GetObject<MobilityModel>()->GetPosition ().x;
      uint32_t y  = bleNetDevices.Get(i)->GetNode()->GetObject<MobilityModel>()->GetPosition ().y;
      errorMap[addr-2] = std::make_tuple (0,0,0,0,0,0,x,y);
      DynamicCast<BleNetDevice>(bleNetDevices.Get(i))->TraceConnectWithoutContext ("MacTx",MakeCallback(&Transmitted));
      DynamicCast<BleNetDevice>(bleNetDevices.Get(i))->TraceConnectWithoutContext ("MacRx",MakeCallback(&ReceivedUnique));
      DynamicCast<BleNetDevice>(bleNetDevices.Get(i))->TraceConnectWithoutContext ("MacRxBroadcast",MakeCallback(&ReceivedBroadcast));
      DynamicCast<BleNetDevice>(bleNetDevices.Get(i))->TraceConnectWithoutContext ("MacPromiscRx",MakeCallback(&Received));
      DynamicCast<BleNetDevice>(bleNetDevices.Get(i))->TraceConnectWithoutContext ("MacRxError",MakeCallback(&ReceivedError));
      DynamicCast<BleNetDevice>(bleNetDevices.Get(i))->TraceConnectWithoutContext ("TXWindowSkipped",MakeCallback(&TXWindowSkipped));
    }

    NS_LOG (LOG_INFO, "Simulator will run now");
    
    Simulator::Stop(Seconds (duration));
    Simulator::Run ();

    for (uint32_t i=0; i< bleNetDevices.GetN(); i++)
    {
            uint8_t buffer[2];
            Mac16Address::ConvertFrom(bleNetDevices.Get(i)->GetAddress()).CopyTo(buffer);
            uint32_t addr = buffer[1];  
            Ptr<BleNetDevice> netdevice =DynamicCast<BleNetDevice>(bleNetDevices.Get(i));
            NS_LOG (LOG_DEBUG, "nd = " << netdevice << " addr = " << addr);
			std::tuple<uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t> tuple = errorMap[i];
			// print iteration, ID, transmitted, received, received unique, x coords, y coords.
			*m_stream->GetStream() << (int)iterationI << "," << netdevice->GetAddress16() << "," << std::get<0>(tuple)<< "," << std::get<1>(tuple) << "," <<   std::get<2>(tuple) <<  "," << std::get<3>(tuple) << "," << std::get<4>(tuple) << "," << std::get<5>(tuple)  << "," << std::get<6>(tuple) << "," << std::get<7>(tuple) << std::endl;
		
    }
    errorMap.clear();
    Simulator::Destroy ();

  }
  
  NS_LOG_INFO ("Done.");
  return 0;
}


