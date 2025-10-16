/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 KULeuven 
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
 *  Based on lora-application from the lora ns-3 module,
 *      designed by Brecht Reynders.
 *          This module can be found here:
 *https://github.com/networkedsystems/lora-ns3/blob/master/model/lora-mac-header.h
 */

// Implementation for ns3 Application base class.
// George F. Riley, Georgia Tech, Fall 2006

/*数据包生成：定期（基于 m_interPacketTime）生成固定大小（m_dataSize）的空数据包，模拟传感器行为。
数据包发送：通过 PacketSocket 直接向网络设备（BleNetDevice）发送数据包，无需路由。
数据包接收：处理接收到的数据包，提取目标地址。
网络设备关联：绑定到指定的 BleNetDevice，支持点对点或广播通信。
应用生命周期：支持启动（StartApplication）、停止（StopApplication）和初始化/清理（DoInitialize/DoDispose）*/

#include "ble-application.h"
#include "ble-net-device.h"
#include "ble-mac-header.h"
#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"
#include <ns3/mac16-address.h>
#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"
#include "ns3/packet.h"
#include "ns3/socket-factory.h"
#include "ns3/socket.h"
#include "ns3/packet-socket.h"
#include "ns3/packet-socket-address.h"

namespace ns3 {


	NS_LOG_COMPONENT_DEFINE ("BleApplication");

	NS_OBJECT_ENSURE_REGISTERED (BleApplication);

	// Application Methods

	TypeId 
		BleApplication::GetTypeId (void)
		{
			static TypeId tid = TypeId ("ns3::BleApplication")
				.SetParent<Application> ()
				.SetGroupName("Ble")
				.AddAttribute ("InterPacketTime", "Time between each burst of data",
						TimeValue (Seconds (6.0)),
						MakeTimeAccessor (&BleApplication::m_interPacketTime),
						MakeTimeChecker ())
	            .AddAttribute ("TimeOffset", "Time before first packet is send",
						TimeValue (MicroSeconds (0)),
						MakeTimeAccessor (&BleApplication::m_timeOffset),
						MakeTimeChecker ())
				.AddAttribute ("DataSize", "Data size of each packet",
						UintegerValue (50),
						MakeUintegerAccessor (&BleApplication::m_dataSize),
						MakeUintegerChecker<uint8_t> ())
				.AddAttribute ("Port", "Port number to be used",
						UintegerValue (0),
						MakeUintegerAccessor (&BleApplication::m_port),
						MakeUintegerChecker<uint8_t> ())
				.AddAttribute ("Destination", "Destination address",
                        Mac16AddressValue (Mac16Address ("00:00")),
                        MakeMac16AddressAccessor (&BleApplication::m_destination),
                        MakeMac16AddressChecker ())
				;
			return tid;
		}

	// \brief Application Constructor
	BleApplication::BleApplication()
	{
		NS_LOG_FUNCTION (this);
		m_socket = 0;
        m_destination = Mac16Address("00:00");
	}

	// \brief BleApplication Destructor
	BleApplication::~BleApplication()
	{
		NS_LOG_FUNCTION (this);
	}

	void
		BleApplication::DoDispose (void)
		{
			NS_LOG_FUNCTION (this);
			m_socket = 0;
		}

	void
		BleApplication::DoInitialize (void)
		{
			Application::DoInitialize ();
		}

	void BleApplication::StartApplication ()
	{ 
		NS_LOG_FUNCTION (this);
		// if there is no socket, generate one
		if (m_socket == 0)
		{
			m_socket = CreateObject<PacketSocket> ();
			StaticCast<PacketSocket>(m_socket)->SetNode (m_node);
			// We bind it directly to the netdevice
            m_socket->BindToNetDevice (m_device);
			PacketSocketAddress addr;
			addr.SetSingleDevice(false);
            addr.SetProtocol (0);
			addr.SetPhysicalAddress (m_destination);
            m_socket->Bind (addr);
            m_socket->Listen ();
			m_socket->Connect (addr);
			m_socket->SetRecvCallback (MakeCallback (&BleApplication::HandleBle, this));
			m_socket->SetAllowBroadcast (true);
		}
		
		// Start sensing as a sensor
        NS_LOG_INFO (" Time Offset for this node = " << m_timeOffset);
		m_SenseEvent = Simulator::Schedule (m_timeOffset, &BleApplication::Sense, this);
	}

    void BleApplication::SetNetDevice (Ptr<NetDevice> device)
    {
      NS_LOG_FUNCTION (this << device);
      m_device = device;
    }

    void BleApplication::HandleBle (Ptr<Socket> socket)
    {
      NS_LOG_FUNCTION (this << socket);
      Ptr<Packet> pkt;
      while (pkt = socket->Recv ())
      {
        Ptr<Packet> pack = pkt->Copy();
        BleMacHeader mac;
        pack->PeekHeader (mac);
        PacketSocketAddress destination;
        destination.SetProtocol (0);
        destination.SetSingleDevice (false);
        destination.SetPhysicalAddress (mac.GetDestAddr ());
      }
    }

	void BleApplication::StopApplication ()
	{ 
		NS_LOG_FUNCTION (this);
		// Stop sensing
		m_SenseEvent.Cancel();
	}

	void BleApplication::Sense (void)
	{
		NS_LOG_FUNCTION (this);
		// Create a sensor reading of some size ...
		Ptr<Packet> packet = Create<Packet> (m_dataSize);
		// ... and send it.
		m_socket->Send (packet,0);

		// Schedule a new event
		m_SenseEvent = Simulator::Schedule(
            m_interPacketTime,&BleApplication::Sense,this);
	}

} // namespace ns3


