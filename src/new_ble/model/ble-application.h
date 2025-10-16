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
 *
 */
//模拟了一个传感器应用，定期生成并发送空数据包到指定的目标地址，接收数据包并处理 MAC 头部信息

#ifndef BLE_APPLICATION_H
#define BLE_APPLICATION_H

#include "ns3/event-id.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/node.h"
#include "ns3/callback.h"
#include "ns3/application.h"
#include <ns3/mac16-address.h>

namespace ns3 {

	class Node;
	class Socket;

	/**
	 * \ingroup ble
	 * \brief Ble default application.
	 *
	 * This application generates empty packets every X seconds.  
	 * This application mimics a sensor. 
	 */
	class BleApplication : public Application
	{
		public:
			/**
			 * \brief Get the type ID.
			 * \return the object TypeId
			 */
			static TypeId GetTypeId (void);
			BleApplication ();
			virtual ~BleApplication ();

            void HandleBle (Ptr<Socket> socket);//处理接收到的数据包
            void SetNetDevice (Ptr<NetDevice> device);//设置关联的网络设备

		private:
			/**
			 * This function is the function to schedule a sensing event. 
			 * With every sensing event, we generate the packet.
			 */
			void Sense (void);//生成并发送数据包，模拟传感器行为

            Mac16Address m_destination;
            Ptr<NetDevice> m_device;

			EventId m_SenseEvent;     
              //!< The event that will fire at m_stopTime to end the application
			Ptr<Socket> m_socket;//用于发送和接收数据包的 PacketSocket		
              //!< The socket for this application. 
              // It sends directly to the netdevice (no routing!)
			uint8_t m_dataSize;		//!< The size of an empty message
			uint8_t m_port;			
              //!< The port to use (this is equal to the port MAC field in Ble)
			Time m_interPacketTime;	//!< The time between two packets 数据包发送间隔
            Time m_timeOffset; //!< Time before first packet is send 首次发送的延迟
		protected:
			virtual void DoDispose (void);
			virtual void DoInitialize (void);
			/**
			 * \brief Application specific startup code
			 *
			 * The StartApplication method is called at the start time specified by Start
			 * This method should be overridden by all or most application
			 * subclasses.
			 */
			void StartApplication (void);

			/**
			 * \brief Application specific shutdown code
			 *
			 * The StopApplication method is called at the stop time specified by Stop
			 * This method should be overridden by all or most application
			 * subclasses.
			 */
			void StopApplication (void);

	};

} // namespace ns3

#endif /* BLE_APPLICATION_H */
