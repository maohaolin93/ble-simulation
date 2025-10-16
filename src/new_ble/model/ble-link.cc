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
 */


#include "ble-link.h"
#include "ns3/log.h"
#include "ble-bb-manager.h"
#include <ns3/ble-net-device.h>

#include <iostream>

namespace ns3 {

  NS_LOG_COMPONENT_DEFINE ("BleLink");
  
  NS_OBJECT_ENSURE_REGISTERED (BleLink);

  TypeId
    BleLink::GetTypeId (void)
    {
      static TypeId tid = TypeId ("ns3::BleLink")
        .SetParent<Object> ()
        .AddConstructor<BleLink> ()
        // Add attributes and tracesources
        ;
      return tid;
    }

  BleLink::BleLink ()
  {
    NS_LOG_FUNCTION (this);
    currentLinkType = UNCONNECTED;
    m_channel = 0;
    m_master = 0;
  }

  BleLink::~BleLink ()
  {
    NS_LOG_FUNCTION (this);
    m_channel = 0;
    m_master = 0;
  }

  BleLink::LinkType
  BleLink::GetLinkType()
  {
    NS_LOG_FUNCTION (this);
    return currentLinkType;
  }

  void
    BleLink::SetLinkType(BleLink::LinkType linkType)
    {
      NS_LOG_FUNCTION (this);
      currentLinkType = linkType;
    }

  void
    BleLink::SetChannel (Ptr<SpectrumChannel> c)
    {
      NS_LOG_FUNCTION (this);
      m_channel = c;
      // For all devices in this link (this is a hack for now 
      // //if time allows / in the future: 
      //    change this way of setting channel)
      //设置链路的频谱信道，并将该信道应用到所有连接设备（主设备和从设备）的物理层GetPhy()
      for (auto v : this->GetLinkedDevices()) // Devices are BBM
      {
        v->GetPhy()->SetChannel(c);
      }
    }
         

  Ptr<SpectrumChannel>
    BleLink::GetChannel()
  {
    NS_LOG_FUNCTION (this);
    return m_channel;
  }

  void
    BleLink::AddSlave(Ptr<BleBBManager> bleBBManager)
    {
      NS_ASSERT (bleBBManager != 0);
      m_slaves.push_back(bleBBManager);
    }

  void
    BleLink::SetMaster(Ptr<BleBBManager> bleBBManager)
    {
      NS_ASSERT (bleBBManager != 0);
      // If there already is a master, make that master a slave
      if (m_master != 0)
      {
        this->AddSlave(m_master);
      }
      m_master = bleBBManager;
    }

  Ptr<BleBBManager>
    BleLink::GetMaster()
    {
      NS_LOG_FUNCTION (this);
      return m_master;
    }

  std::list<Ptr<BleBBManager>>
    BleLink::GetLinkedDevices ()
    {
      std::list<Ptr<BleBBManager>> all_devices = m_slaves;
      all_devices.push_back(m_master);
      
   //   for (auto v : all_devices)
   //     std::cout << v << "\n";
      
      return all_devices;
    }

  Ptr<BleBBManager>
    BleLink::GetLinkedDevice (Mac16Address addr)
    {
      NS_LOG_FUNCTION (this);
      for (Ptr<BleBBManager> v : GetLinkedDevices () )
      {
        Ptr<BleNetDevice> nd = v->GetNetDevice();//获取对应设备
        Mac16Address ndAddress = nd->GetAddress16();
        if (ndAddress == addr)  
        {
          return v;
        }
      }
      return 0;
    }
}

