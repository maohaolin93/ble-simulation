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
#include <ns3/okumura-hata-propagation-loss-model.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("BleBroadcastUnicastExample");

  /*****************
   * Configuration *
   *****************/

  int nbIterations = 1; //模拟运行的迭代次数
  double length = 30; //<! Square room with length as distance 场景是一个边长为 30 米的正方形区域
  int pktsize = 20; //!< Size of packtets, in bytes 
  int duration = 110; //<! Duration of the simulation in seconds 模拟总时长为 110 秒
  int packetSendDuration = 100; 
  //<! Time during which new packets should be quied 
  bool verbose = true; // Enable logging 启用详细日志输出
  bool nakagami = false; // enable nakagami path loss 禁用 Nakagami 路径损耗模型
  bool dynamic = false; // Wether the nodes are moving yes or no 节点不移动，使用固定位置模型
  bool scheduled = true; // Schedule the TX windows instead of random parameters. 节点不移动，使用固定位置模型
  bool broadcastAvoidCollisions = true; 
  // Try to avoid 2 nodes being in advertising mode at the same time
  //尝试避免多个节点同时处于广播模式
  uint32_t nNodes = 5; // Number of nodes
  uint32_t nbConnInterval = 3200; 
  // [MAX 3200]  nbConnInterval*1,25ms = size of connection interval. 
  // if nbConnInterval = 0, each link will get a random conn interval
  //连接间隔为 3200 * 1.25ms = 4 秒。如果设为 0，则每个链接的连接间隔随机
  int unicastInterval = 4; //!< Time between two packets from the same node 同一节点单播数据包之间的时间间隔
  int broadcastInterval = 4*nNodes; 
  //!< Time between two packets from the same node 
  //(for good results, should be larger than nNodes*nbConnInterval(s) 

  Ptr<OutputStreamWrapper> m_stream = 0; // Stream for waterfallcurve用于输出模拟结果的流对象，初始化为空
  //均匀分布随机数生成器，用于生成随机位置或时间
  Ptr<UniformRandomVariable> randT = CreateObject<UniformRandomVariable> ();

  //8 个统计指标的元组（传输、接收、唯一接收、错误接收、广播接收、跳过传输窗口、x 坐标、y 坐标）
  std::unordered_map<uint32_t,std::tuple<uint32_t,uint32_t,uint32_t,
  uint32_t,uint32_t,uint32_t,uint32_t,uint32_t> > errorMap;
  std::unordered_map<uint32_t,Ptr<BleNetDevice> > deviceMap;

  /************************
   * End of configuration *
   ************************/

/// Save that the message has been transmitted
//记录数据包的发送事件
	void
Transmitted (const Ptr<const Packet> packet)
{
	Ptr<Packet> copy = packet->Copy();
	BleMacHeader header;
	copy->RemoveHeader(header);
    uint8_t buffer[2];
    header.GetSrcAddr().CopyTo(buffer);
    uint32_t addr = buffer[1];

	std::get<0>(errorMap[addr-1])++;
}

// save that a message has been received
//记录数据包的接收事件（包括可能重复接收的包）
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
    std::get<1>(errorMap[addr-1])++;
}

//记录接收到错误数据包的事件
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
    std::get<3>(errorMap[addr-1])++;
}


// save that a message has been uniquely received
//记录唯一接收的数据包（无重复）
	void
ReceivedUnique (const Ptr<const Packet> packet)
{
	Ptr<Packet> copy = packet->Copy();
	BleMacHeader header;
	copy->RemoveHeader(header);
	uint8_t buffer[2];
    header.GetDestAddr().CopyTo(buffer);
    uint32_t addr = buffer[1];
	std::get<2>(errorMap[addr-1])++;
}

//记录广播数据包的接收事件
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
	std::get<4>(errorMap[addr-1])++;
}

//记录跳过的传输窗口事件
  void
TXWindowSkipped (const Ptr<const BleNetDevice> nd)
{
  //NS_LOG (LOG_DEBUG, "Packet received  " << packet);
	uint8_t buffer[2];
    nd->GetAddress16().CopyTo(buffer);
    uint32_t addr = buffer[1];
    std::get<5>(errorMap[addr-1])++;
}


int main (int argc, char** argv)
{
  bool verbose = false;

  CommandLine cmd;
  cmd.AddValue ("verbose", "Tell application to log if true", verbose);

  cmd.Parse (argc,argv);
 
  // Enable logging
  BleHelper helper;
  if (verbose)
    helper.EnableLogComponents();


  Packet::EnablePrinting ();
  Packet::EnableChecking ();

  NS_LOG_INFO ("BLE Unicast - Broadcast example file");

  // Enable debug output
  NS_LOG_INFO ("Enable debug output");
 // AsciiTraceHelper ascii;
 // helper.EnableAsciiAll (ascii.CreateFileStream ("example-ble.tr"));
  AsciiTraceHelper ascii;
  m_stream = ascii.CreateFileStream ("example-unicast-broadcast.csv");
  *m_stream->GetStream() << "#Scenario " << (int)nNodes 
    <<  " nodes on a square field with side " << length << " meter" 
    << " TX window scheduling enabled: " << scheduled 
    << ", connection interval = " 
    << nbConnInterval*1.25 << " millisec, (0 = random) " << std::endl;
  //写入场景描述，包括节点数量、场景大小、是否启用调度、连接间隔等
  // print Iteration, ID, transmitted, received, received unique, 
  // received at closest gateway, x coords, y coords, 
  // get average amount of retransmissions, get average time of transmissions, 
  // number of missed messages, amount of received messages.
  //迭代编号、节点 ID、发送计数、接收计数、唯一接收计数、错误接收计数、广播接收计数、跳过传输窗口计数、节点坐标
  *m_stream->GetStream() << "Iteration, ID, transmitted, received, "
    "received unique, received error, broadcast received, "
    "TX Windows Skipped x coords, y coords " <<std::endl;
  for (uint8_t iterationI=0;iterationI<nbIterations;iterationI++){
		std::cout << "Iteration: " << (int)iterationI << std::endl;
 

    randT->SetAttribute("Max", DoubleValue (600));
   
    NS_LOG (LOG_INFO, "Ble setup starts now");

    //创建 BLE 设备节点
    NodeContainer bleDeviceNodes;
    bleDeviceNodes.Create(nNodes);


    // Create mobility 为节点分配随机位置
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> nodePositionList = 
      CreateObject<ListPositionAllocator> ();
    for (uint32_t nodePositionsAssigned = 0; 
        nodePositionsAssigned < nNodes; nodePositionsAssigned++)
    {
      double x,y;
      x = randT->GetInteger(0,length);
      y = randT->GetInteger(0,length);
      NS_LOG (LOG_INFO, "x = " << x << " y = " << y);
      nodePositionList->Add (Vector (x,y,1.0));
    }

    //设置节点的移动模型并安装到节点上
    mobility.SetPositionAllocator (nodePositionList);
    if (dynamic)
      mobility.SetMobilityModel ("ns3::RandomWalk2DMobilityModel");
    else
      mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install(bleDeviceNodes);
    
    // Create the nodes 为节点安装 BLE 网络设备
    NetDeviceContainer bleNetDevices;
    bleNetDevices = helper.Install (bleDeviceNodes);

    // Set addresses 为每个 BLE 设备设置 16 位 MAC 地址
    NS_LOG (LOG_INFO, "Set addresses");
    for (uint32_t nodeI = 0; nodeI < nNodes; nodeI++)
    {
      //std::string s = std::to_string (std::hex(nodeI+1));

      std::stringstream stream;
      stream << std::hex << nodeI+1;
      std::string s( stream.str());
      while (s.size() < 4)
        s.insert(0,1,'0');
      s.insert(2,1,':');
      char const * buffer = s.c_str();
      DynamicCast<BleNetDevice>(bleNetDevices.Get(nodeI))
        ->SetAddress (Mac16Address (buffer));
      NS_LOG (LOG_INFO, "address = " 
          << DynamicCast<BleNetDevice>(bleNetDevices.Get(nodeI))->GetAddress ());
    }

    // Create links between the nodes
    //创建所有节点之间的单播链接
    helper.CreateAllLinks (bleNetDevices, scheduled, nbConnInterval);
    //helper.CreateBroadcastLink (bleNetDevices, scheduled, nbConnInterval);
    //创建广播链接，支持避免冲突 (broadcastAvoidCollisions)
    helper.CreateBroadcastLink (
        bleNetDevices, scheduled, nbConnInterval, broadcastAvoidCollisions);
   

    //生成广播和单播流量
    NS_LOG (LOG_INFO, " Generate data ");
    ApplicationContainer apps1 = helper.GenerateBroadcastTraffic (
        randT, bleDeviceNodes, pktsize, 0, packetSendDuration, broadcastInterval);
    ApplicationContainer apps2 = helper.GenerateTraffic (
        randT, bleDeviceNodes, pktsize, 0, packetSendDuration, unicastInterval);
    
     // Hookup functions to measure performance

    //初始化设备映射和错误统计，并连接跟踪回调
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
      errorMap[addr-1] = std::make_tuple (0,0,0,0,0,0,x,y);
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
            Mac16Address::ConvertFrom(bleNetDevices.Get(i)
                ->GetAddress()).CopyTo(buffer);
            uint32_t addr = buffer[1];  
            Ptr<BleNetDevice> netdevice =
              DynamicCast<BleNetDevice>(bleNetDevices.Get(i));
            NS_LOG (LOG_DEBUG, "nd = " << netdevice << " addr = " << addr);
			std::tuple<uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,
              uint32_t,uint32_t> tuple = errorMap[i];
			// print iteration, ID, transmitted, received, received unique, 
            // x coords, y coords.
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
  
  NS_LOG_INFO ("Done.");
  return 0;
}


