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

/*管理 BLE 网络设备、物理层和基带管理器之间的关联。
处理数据包的发送（StartTransmission、StartPacketTransmission）和接收准备（PrepareForReception）。
检查接收到的数据包是否需要确认，并处理确认机制（CheckReceivedAckPacket）。
管理 BLE 的信道选择，基于信道索引获取频谱信道（GetChannelBasedOnChannelIndex）。
提供回调和跟踪机制，用于通知上层数据包传输和错误事件*/

#include "ble-link-controller.h"
#include "ns3/ble-net-device.h"
#include "ns3/ble-bb-manager.h"
#include "ns3/ble-link-manager.h"
#include "ns3/ble-phy.h"
#include "ns3/log.h"
#include "ns3/ble-mac-header.h"

namespace ns3 {

  NS_LOG_COMPONENT_DEFINE ("BleLinkController");
  
  NS_OBJECT_ENSURE_REGISTERED (BleLinkController);

  TypeId
    BleLinkController::GetTypeId (void)
    {
      static TypeId tid = TypeId ("ns3::BleLinkController")
        .SetParent<Object> ()
        .AddConstructor<BleLinkController> ()
        // Add attributes and tracesources
        .AddTraceSource ("MacTx",
            "Trace source indicating a packet has arrived "
            "for transmission by this device",
            MakeTraceSourceAccessor (&BleLinkController::m_macTxTrace),
            "ns3::Packet::TracedCallback")
        ;
      return tid;
    }

  BleLinkController::BleLinkController ()
  {
    NS_LOG_FUNCTION (this);
    m_currentPkt = 0;
    retransmissionCount = 0;
  }

  BleLinkController::~BleLinkController ()
  {
    NS_LOG_FUNCTION (this);
  }

  void
    BleLinkController::DoDispose (void)
    {
      NS_LOG_FUNCTION (this);
      m_netDevice = 0;
      m_currentPkt = 0;
      retransmissionCount = 0;

    }

  /***********************
   * GETTERS AND SETTERS *
   ***********************/

  void
    BleLinkController::SetNetDevice(Ptr<BleNetDevice> netDevice)
    {
      m_netDevice = netDevice;
    }

  Ptr<BleNetDevice>
    BleLinkController::GetNetDevice()
    {
      return m_netDevice;
    }


  void
    BleLinkController::SetPhy(Ptr<BlePhy> phy)
    {
      this->GetNetDevice()->SetPhy(phy);
    }

  Ptr<BlePhy>
    BleLinkController::GetPhy()
    {
      return this->GetNetDevice()->GetPhy();
    }


  void
    BleLinkController::SetBBManager(Ptr<BleBBManager> bbManager)
    {
      this->GetNetDevice()->SetBBManager(bbManager); 
    }

  Ptr<BleBBManager>
    BleLinkController::GetBBManager()
    {
      return this->GetNetDevice()->GetBBManager();
    }

  Ptr<Packet>
    BleLinkController::GetCurrentPacket()
    {
      return this->m_currentPkt;
    }

  void
    BleLinkController::SetCurrentPacket(Ptr<Packet> packet)
    {
      NS_LOG_FUNCTION (this);
      this->m_currentPkt = packet;
    }

  uint8_t
    BleLinkController::GetRetransmissionCount ()
    {
      return this->retransmissionCount;
    }

  void
    BleLinkController::SetRetransmissionCount (uint8_t count)
    {
      this->retransmissionCount = count;
    }

  Time
    BleLinkController::GetStartTimePacket()
    {
      return startTimePacket;
    }
    
  void
    BleLinkController::SetStartTimePacket(Time time)
    {
      startTimePacket = Time(time);
    }

  void
    BleLinkController::SetCheckedAckCallback (Callback<void, 
        Ptr<Packet> > callback)
    {
      NS_LOG_FUNCTION (this);
      m_ackChecked = callback;
    }

  void
    BleLinkController::SetCheckedAckErrorCallback (Callback<void, 
        Ptr<Packet> > callback)
    {
      NS_LOG_FUNCTION (this);
      m_ackCheckedError = callback;
    }


  void 
    BleLinkController::SetAllChannels (
        std::vector<Ptr<SpectrumChannel>> allChannels)
    {
      NS_LOG_FUNCTION (this);
      m_allChannels = allChannels;
    }

  Ptr<SpectrumChannel>
    BleLinkController::GetChannelBasedOnChannelIndex (uint8_t channelIndex)
    {
      if (m_allChannels.size() == 0)
      {
        return 0;
      }
      NS_ASSERT (channelIndex < m_allChannels.size());
      return m_allChannels.at(channelIndex); 

    }



  /******************************
   * END OF GETTERS AND SETTERS *
   ******************************/

  /******************************
   * CALLBACK related functions *
   ******************************/

  void
    BleLinkController::SetGenericPhyTxStartCallback (GenericPhyTxStartCallback c)
    {
      NS_LOG_FUNCTION (this);
      m_phyMacTxStartCallback = c;
    }
  
  void
    BleLinkController::StartTransmissionNoArgs()
    {
      NS_LOG_FUNCTION(this);
      NS_ASSERT (this->GetCurrentPacket() != 0);

      if (StartTransmission (this->GetCurrentPacket()->Copy(), false))
      {
        retransmissionCount++;
        m_macTxTrace (this->GetCurrentPacket());
        // Get lastSend time
        lastSend = Simulator::Now();
      }
      else
      {
        NS_LOG_FUNCTION(" Start transmission failed " );
      }
    }

  bool
    BleLinkController::StartTransmission (Ptr<Packet> packet, bool ackPacket)
    {
      NS_LOG_FUNCTION (this);
      // Set parameters of phy device
      
      if (this->GetPhy()->GetState() != BlePhy::State::IDLE)
      {
        NS_LOG_WARN ("Phy is busy at the moment, "
            "not possible to change to TX state, current PHY State = " 
            << this->GetPhy()->GetState());
        return false;
      }
      return this->GetPhy()->PrepareTX (packet);
    }

  void
    BleLinkController::PrepareForReception (Ptr<BleLinkManager> lm)
    {
      NS_LOG_FUNCTION (this);
      if (! this->GetBBManager()->GetActiveLinkManager() == 0)
      {
        if (this->GetPhy()->GetState() == BlePhy::State::IDLE)
          this->GetPhy()->PrepareRX();
        else
          NS_LOG_WARN ("Phy is still busy, "
              "not possible to RX, current PHY state = " 
              << this->GetPhy()->GetState());
      }
      else
      {
        this->GetPhy()->ChangeState(BlePhy::State::IDLE);
      }
    }

  void
    BleLinkController::StartPacketTransmission (Ptr<BleLinkManager> lm)
    {
      NS_LOG_FUNCTION(this);
      NS_ASSERT (lm->GetCurrentPacket() != 0);
      
      if (StartTransmission (lm->GetCurrentPacket()->Copy(), false)) 
      {
        m_macTxTrace (lm->GetCurrentPacket());
      }
      else
      {
        NS_LOG_FUNCTION(" Start transmission failed " );
      }
    }

  // Checks if a received packet is arrived correctly or 
  // needs an acknowledgement and also sends this acknowledgement
  void
    BleLinkController::CheckReceivedAckPacket (Ptr<Packet> packet, 
        bool receptionError) 
    {
      NS_LOG_FUNCTION (this);
      if (this->GetBBManager()->GetActiveLinkManager() != 0)
      {
      if (receptionError) // Error during packet reception,
      {
        // Ber was too high
        BleMacHeader bmh;
        packet->PeekHeader(bmh); 
        
        // Ignore broadcast for error callback
        //非广播（FF:FF）或状态为 SCANNER，触发错误回调（m_ackCheckedError）
        if (bmh.GetDestAddr() != Mac16Address("FF:FF") || this->GetBBManager()->GetActiveLinkManager()->GetState() == BleLinkManager::State::SCANNER)
        {
          NS_LOG_INFO (this->GetBBManager()->GetActiveLinkManager()->GetState());
          NS_LOG_ERROR("Reception ERROR: packet for " << bmh.GetDestAddr() 
              << " arrived with a too high BER. Here addr: " 
              << this->GetNetDevice()->GetAddress16() 
              << " My active channel index: " << 
              (int) this->GetBBManager()->GetActiveLinkManager()
              ->GetCurrentChannelIndex());
          m_ackCheckedError (packet);
        }
      }
      else
      {
        BleMacHeader bmh;
        Ptr<BleLinkManager> lm = this->GetBBManager()->GetActiveLinkManager();
        NS_ASSERT (lm != 0);
        packet->PeekHeader(bmh);
        NS_LOG_DEBUG ("Header received. LLID = " <<
            int(bmh.GetLLID()) << " MD = " << bmh.GetMD() <<
            " SN = " << bmh.GetSN() << " NESN = " <<
            bmh.GetNESN() ); //<< " length = " << int(bmh.GetLength()));
        //若为本设备地址或广播地址，处理包
        if (bmh.GetDestAddr() == this->GetNetDevice()->GetAddress16() ||
            bmh.GetDestAddr() == Mac16Address("FF:FF") )
        {
          if (lm->GetState() == BleLinkManager::State::SCANNER )
          {
            NS_LOG_INFO ("Received an ADVERTISING packet, length = " 
                << int(bmh.GetLength()));
            m_ackChecked (packet);
          }
          else
          {
            //检查是否为保持活动包
            bool keepAlive = (bmh.GetLLID() == 0b01) 
              && (bmh.GetLength() == 0);

            // Acknowledgements and flow control:
            lm->SetNESN(bmh.GetNESN()); 
            lm->SetSN(bmh.GetSN());
            bool receivedNew = lm->ManageSequenceNumberRX();
            if (receivedNew)
            {
              lm->SetPeerHasMoreData(bmh.GetMD());
              if (keepAlive)
              {
                NS_LOG_INFO ("Received a Keep Alive packet");
              }
              else // Received data, callback upper layers
              {
                //NS_ASSERT (bmh.GetLength() > 0);
                NS_LOG_INFO ("Received a data packet, length = " 
                    << int(bmh.GetLength()));
                m_ackChecked (packet);
              }
            }
            else
            {
              NS_LOG_INFO ("Received an old packet, will be ignored");
            }
 
            // Set phy back to IDLE
            this->GetPhy()->ChangeState(BlePhy::State::IDLE);
            
            // Check if I need to send another packet
            //若双方 MD=0 且为保持活动包，关闭事件；否则调度 SendNextPacket（T_IFS=150us）
            if (! (lm->GetMyLastMD() == 0 && bmh.GetMD() == 0 && keepAlive))
            {
              /*if (! (lm->GetState() == BleLinkManager::State::SCANNER))
              {
                Simulator::Schedule(
                    MicroSeconds(T_IFS),&BleLinkManager::SendNextPacket, lm);
              }*/
              //mhl修改
              if(lm->GetState() == BleLinkManager::State::MASTER || lm->GetState() == BleLinkManager::State::SLAVE){
                Simulator::Schedule(
                    MicroSeconds(T_IFS),&BleLinkManager::SendNextPacket, lm);
              }
              else{

              }
            }
            else
            {
              NS_LOG_INFO ("This connection event is closed");
              lm->SetPeerHasMoreData(false);
              this->GetBBManager()->SetActiveLinkManager(0);
            }
          }
        }
        else // packet was not for me
        {
          NS_LOG_INFO ("Received a packet that was not for me");
        }
      }
      }
    }
}

