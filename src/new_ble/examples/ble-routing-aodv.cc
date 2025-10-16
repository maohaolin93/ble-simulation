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
#include "ns3/aodv-module.h"
//#include "ns3/v4ping-helper.h"
#include <ns3/ipv6-routing-table-entry.h>
#include <ns3/ipv6-static-routing-helper.h>
#include <ns3/sixlowpan-module.h>
#include <cmath>
#include<vector>

#include <ns3/mobility-helper.h>
#include <fstream>
using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("BleRoutingAodv");

  /*****************
   * Configuration *
   *****************/ 

  int nbIterations = 1;
  double length = 5; //<! Square room with length as distance
  int pktsize = 20; //!< Size of packtets, in bytes
  int duration = 40; //<! Duration of the simulation in seconds
  int packetSendDuration = 10; //数据包发送的持续时间为 10 秒，之后停止生成新数据包。
  //<! Time during which new packets should be quied 
  bool verbose = false; // Enable logging
  bool nakagami = false; // enable nakagami path loss禁用 Nakagami 路径损耗模型
  bool dynamic = false; // Wether the nodes are moving yes or no节点不移动，使用固定位置模型
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
  int pingInterval = 20; // In seconds节点 0 向其他节点发送 ping 请求的间隔为 10 秒
  double internodedistance = 15.0; // De afstand tussen de nodes in meters.节点之间的固定距离为 15 米
  bool randomNodePlacement = false;//默认使用线性排列（沿 x 轴以 15 米间隔放置），而不是随机放置

  Ptr<OutputStreamWrapper> m_stream = 0; // Stream for waterfallcurve
  Ptr<UniformRandomVariable> randT = CreateObject<UniformRandomVariable> ();

  std::unordered_map<uint32_t,std::tuple<uint32_t,uint32_t,uint32_t,
  uint32_t,uint32_t,uint32_t,uint32_t,uint32_t> > errorMap;
  std::unordered_map<uint32_t,Ptr<BleNetDevice> > deviceMap;

  //vector<double> speed;
  //vector<double> angle;

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
  bool pcap = false; //默认禁用 PCAP 跟踪（用于捕获网络数据包）
  bool printRoutes = true; //默认启用路由表打印，输出 AODV 路由信息

  CommandLine cmd;
  cmd.AddValue ("verbose", "Tell application to log if true", verbose);
  cmd.AddValue ("pcap", "Write PCAP traces.", pcap);

  cmd.Parse (argc,argv);
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
        LogComponentEnable("BleLinkController", LOG_LEVEL_ALL);
        LogComponentEnable("BleLinkManager", LOG_LEVEL_ALL);
        LogComponentEnable("BleApplication", LOG_LEVEL_ALL);
        //LogComponentEnable("BleLink", LOG_LEVEL_ALL);
        //LogComponentEnable("BleHelper", LOG_LEVEL_ALL);
        LogComponentEnableAll (LOG_PREFIX_TIME);
        LogComponentEnableAll (LOG_PREFIX_FUNC);
        LogComponentEnableAll (LOG_PREFIX_LEVEL);
        LogComponentEnableAll (LOG_PREFIX_NODE);
    }
  
  /*LogComponentEnableAll (LOG_PREFIX_TIME);
  LogComponentEnableAll (LOG_PREFIX_FUNC);
  LogComponentEnableAll (LOG_PREFIX_LEVEL);
  LogComponentEnableAll (LOG_PREFIX_NODE);*/
 
  // Enable logging
  BleHelper helper;
  if (verbose)
  {
    helper.EnableLogComponents();
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
  //迭代编号、节点 ID、发送计数、接收计数、唯一接收计数、错误接收计数、广播接收计数、跳过传输窗口计数、节点坐标
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
    for (uint32_t nodePositionsAssigned = 0; 
        nodePositionsAssigned < nNodes; nodePositionsAssigned++)
      {
        double x,y;
        if (randomNodePlacement)
        {
          x = randT->GetInteger(0,length);
          y = randT->GetInteger(0,length);
        }
        else {
          x = double (nodePositionsAssigned)*internodedistance;
          y = 0.0;
        }
        NS_LOG (LOG_INFO, "x = " << x << " y = " << y);
        nodePositionList->Add (Vector (x,y,1.0));
      }

    //设置节点的移动模型并安装到节点上
    mobility.SetPositionAllocator (nodePositionList);
    /*if (dynamic)
      //mobility.SetMobilityModel ("ns3::RandomWalk2DMobilityModel");
      mobility.SetMobilityModel("ns3::RandomWalk2DMobilityModel",
                             "Mode", StringValue ("Time"),
                             "Bounds", RectangleValue(Rectangle(0, length, 0, length)),
                             "Speed", StringValue("ns3::UniformRandomVariable[Min=0.5|Max=2.0]"), // 速度范围 0.5-2 m/s
                             "Time", StringValue("ns3::ConstantRandomVariable[Constant=1.0]")); // 每秒更新方向
    else
      mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install(bleDeviceNodes);*/
    
    if (dynamic)
    {
        mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
        mobility.Install(bleDeviceNodes);
        // 为每个节点设置速度
        for (uint32_t i = 0; i < bleDeviceNodes.GetN(); i++)
        {
            Ptr<ConstantVelocityMobilityModel> velModel =
            bleDeviceNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>();
            double speed = 5;//speed[i];
            double angle = 0;//angle[i];
            Vector vel(speed * cos(angle), speed * sin(angle), 0.0);
            velModel->SetVelocity(vel);
        }
        // 位置已由 ListPositionAllocator 设置，无需再设
    }
    else
    {
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(bleDeviceNodes);
    }
    
    // Create the nodes 为节点安装 BLE 网络设备
    NetDeviceContainer bleNetDevices;
    bleNetDevices = helper.Install (bleDeviceNodes);

    // Install Internet stack 安装互联网协议栈并配置 AODV 路由协议
    AodvHelper aodv;
    InternetStackHelper stack;
    aodv.Set("HelloInterval", TimeValue(Seconds(36)));//控制路由发现的 HELLO 消息间隔
    aodv.Set("ActiveRouteTimeout", TimeValue(Seconds(480)));//路由条目有效时间
    aodv.Set("BlackListTimeout", TimeValue(Seconds(488)));//黑名单超时时间
    aodv.Set("NodeTraversalTime", TimeValue(Seconds(40)));//节点遍历时间估计
    aodv.Set("NextHopWait", TimeValue(Seconds(40)));//等待下一跳的时间
    stack.SetRoutingHelper (aodv);
    stack.Install (bleDeviceNodes);
   
   //为网络设备分配 IPv4 地址。
    Ipv4AddressHelper address;
    address.SetBase ("10.0.0.0", "255.0.0.0");

    Ipv4InterfaceContainer interfaces;
    interfaces = address.Assign (bleNetDevices);

    //在指定时间点打印 AODV 路由表
    if (printRoutes)
    {
      Ptr<OutputStreamWrapper> routingStream = 
        Create<OutputStreamWrapper> ("aodv5.routes", std::ios::out);
      aodv.PrintRoutingTableAllAt (Seconds (5), routingStream);
      Ptr<OutputStreamWrapper> routingStream2 = 
        Create<OutputStreamWrapper> ("aodv120.routes", std::ios::out);
      aodv.PrintRoutingTableAllAt (Seconds (120), routingStream2);
      Ptr<OutputStreamWrapper> routingStream3 = 
        Create<OutputStreamWrapper> ("aodv470.routes", std::ios::out);
      aodv.PrintRoutingTableAllAt (Seconds (470), routingStream3);
    }

    // Create links between the nodes
    //创建节点之间的单播和广播链接
    helper.CreateAllLinks (bleNetDevices, scheduled, nbConnInterval);
    helper.CreateBroadcastLink (bleNetDevices, scheduled, nbConnInterval, broadcastAvoidCollisions);
   
    NS_LOG (LOG_INFO, " Generate data ");
    //ApplicationContainer apps1 = helper.GenerateBroadcastTraffic 
    // (randT, bleDeviceNodes, pktsize, 0, packetSendDuration, broadcastInterval);
    //ApplicationContainer apps2 = helper.GenerateTraffic 
    // (randT, bleDeviceNodes, pktsize, 0, packetSendDuration, unicastInterval);

    //为节点 0 配置 ping 应用，定期向其他节点发送 ICMP ping 请求
    //模拟节点 0 通过 AODV 路由向其他节点发送 ping 请求
    ApplicationContainer app;
    /*for (uint32_t i=1; i < nNodes; i++)
    {
      V4PingHelper ping (interfaces.GetAddress (i));

      //设置ping属性
      ping.SetAttribute ("Verbose", BooleanValue (true));
      ping.SetAttribute ("Interval", TimeValue (Seconds (pingInterval)));

      ApplicationContainer p = ping.Install (bleDeviceNodes.Get (0));
      p.Start (Seconds (10));
      p.Stop (Seconds (duration) - Seconds (10));
      app.Add(p);//将每个 ping 应用添加到 app 容器
    }*/
   uint32_t src = 1;
  uint32_t dst = 2;
  V4PingHelper ping(interfaces.GetAddress(dst)); // 目标地址
  ping.SetAttribute("Verbose", BooleanValue(true));
  ping.SetAttribute("Interval", TimeValue(Seconds(pingInterval)));

  ApplicationContainer p = ping.Install(bleDeviceNodes.Get(src)); // 安装在源节点
  p.Start(Seconds(10)); // 所有 Ping 从第 10 秒开始
  p.Stop(Seconds(31)); // 在结束前 10 秒停止
  app.Add(p);

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
              uint32_t,uint32_t,uint32_t,uint32_t> tuple = errorMap[i];
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


