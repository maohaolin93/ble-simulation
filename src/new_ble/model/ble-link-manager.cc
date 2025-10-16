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

/*状态机管理：实现 BLE 设备状态（待机、广播、扫描、主设备、从设备等）的转换。
链路设置：支持单设备（点对点）和多设备（广播）链路配置。
数据传输：管理数据包的发送和接收，包括序列号（SN/NESN）和更多数据（MD）标志。
传输窗口：控制 BLE 连接的传输窗口，包括开始、结束和准备下一个窗口。
信道选择：实现 BLE 的跳频机制，选择合适的信道进行通信。
队列管理：使用 DropTailQueue 缓冲数据包，支持重传和保活机制*/

#include "ble-link-manager.h"
#include "ns3/log.h"
#include <ns3/ble-bb-manager.h>
#include <ns3/ble-net-device.h>
#include <ns3/ble-link-controller.h>
#include <ns3/ble-mac-header.h>
#include <ns3/mac16-address.h>
#include <ns3/queue.h>
#include <ns3/drop-tail-queue.h>
#include <ns3/queue-item.h>
#include <ns3/multi-model-spectrum-channel.h>

namespace ns3 {

  NS_LOG_COMPONENT_DEFINE ("BleLinkManager");
  
  NS_OBJECT_ENSURE_REGISTERED (BleLinkManager);

  TypeId
    BleLinkManager::GetTypeId (void)
    {
      static TypeId tid = TypeId ("ns3::BleLinkManager")
        .SetParent<Object> ()
        .AddConstructor<BleLinkManager> ()
        // Add attributes and tracesources
        ;
      return tid;
    }

  BleLinkManager::BleLinkManager ()
  {
    NS_LOG_FUNCTION (this);
    currentState = STANDBY;
    expectedRole = STANDBY_ROLE;
    m_firstTransmitWindowDone = false;
    m_nextExpectedSequenceNumber = false;
    m_sequenceNumber = false;
    m_peerHasMoreData = false;
    m_onePacketSend = false;
    m_lastUnmappedChannelIndex = 0;

    m_broadcastCollisionAvoidance = true;
    //广播休眠计数器，最大值
    m_advSleepCounter = 0;
    m_advSleepMax = 10;

    //默认跳频增量
    SetHopIncrement (1);
    SetKeepAliveActive (true);
    // The numbers that will be defined now are just generic
    //  You can change these numbers.
    /*连接间隔：500ms（MicroSeconds(500000)）。
      监督超时：200ms（MilliSeconds(200)）。
      从设备延迟：0。
      连接事件计数器：0。
      传输窗口大小：5ms。
      传输窗口偏移：2.5ms*/
    SetConnInterval (MicroSeconds(500000));
    SetConnSupervisionTimeout (MilliSeconds (200));
    SetConnSlaveLatency (0);
    SetConnEventCounter (0);
    SetTransmitWindowSize (MilliSeconds (5));
    SetTransmitWindowOffset (MicroSeconds (2500));

    Ptr<DropTailQueue<QueueItem>> buffer = Create<DropTailQueue<QueueItem>> ();
    buffer->SetMaxSize ( QueueSize(QUEUE_SIZE_PACKETS));
    this->m_queue = buffer;
  }

  void
    BleLinkManager::DoDispose () {
      NS_LOG_FUNCTION (this);
      m_queue = 0;
    }

  BleLinkManager::~BleLinkManager ()
  {
    NS_LOG_FUNCTION (this);
  }

  void
    BleLinkManager::SetState(BleLinkManager::State state)
    {
      NS_LOG_FUNCTION (this<<state);
      // Check if state is a valid state
      // Set state
      this->currentState = state;
    }

  BleLinkManager::State
    BleLinkManager::GetState()
    {
      NS_LOG_FUNCTION (this<<this->currentState);
      return this->currentState;
    }

  //主从连接
  bool
    BleLinkManager::IsConnected()
    {
      NS_LOG_FUNCTION (this);
     
      if (currentState == SLAVE || currentState == MASTER)
      {
        return true;
      }
      else
      {
        return false;
      }
    }

  /*
STANDBY：
  SLAVE_ROLE → ADVERTISER（开始广播）。
  MASTER_ROLE → SCANNER（开始扫描）。
  其他 → STANDBY。

ADVERTISER：
  SLAVE_ROLE → SLAVE（接收到 CONNECT_REQ）。
  MASTER_ROLE → STANDBY（不符合逻辑）。
  其他 → ADVERTISER（继续广播）。

SCANNER：
  SLAVE_ROLE → STANDBY（不符合逻辑）。
  MASTER_ROLE → INITIATOR（发现广播，发起连接）。
  其他 → SCANNER（继续扫描）。

INITIATOR：
  SLAVE_ROLE → STANDBY（不符合逻辑）。
  MASTER_ROLE → MASTER（连接建立）。
  其他 → INITIATOR（继续尝试连接）。

MASTER：
  SLAVE_ROLE → STANDBY（连接断开）。
  MASTER_ROLE → MASTER（维持主设备）。
  其他 → MASTER。

默认：回退到 STANDBY*/
  void
    BleLinkManager::NextState()
    {
      NS_LOG_FUNCTION (this);
      BleLinkManager::State nextState = STANDBY;
      switch (currentState)
      {
        case STANDBY : // Next state can be STANDBY, ADVERTISER or SCANNER
                      switch (expectedRole)
                      {
                        case SLAVE_ROLE:
                          nextState = ADVERTISER;
                          break;
                        case MASTER_ROLE:
                          nextState = SCANNER;
                          break;
                        default:
                          nextState = STANDBY;
                          break;
                      }
                      break;
        case ADVERTISER : // Next state can be STANDBY, ADVERTISER or SLAVE
                      switch (expectedRole)
                      {
                        case SLAVE_ROLE:
                          nextState = SLAVE;
                          break;
                        case MASTER_ROLE:
                          nextState = STANDBY;
                          break;
                        default:
                          nextState = ADVERTISER;
                          break;
                      }
                      break;

        case SCANNER : // Next state can be STANDBY, INITIATOR or SCANNER
                      switch (expectedRole)
                      {
                        case SLAVE_ROLE:
                          nextState = STANDBY;
                          break;
                        case MASTER_ROLE:
                          nextState = INITIATOR;
                          break;
                        default:
                          nextState = SCANNER;
                          break;
                      }
                      break;

        case INITIATOR : // Next state can be STANDBY, INITIATOR or MASTER
                      switch (expectedRole)
                      {
                        case SLAVE_ROLE:
                          nextState = STANDBY;
                          break;
                        case MASTER_ROLE:
                          nextState = MASTER;
                          break;
                        default:
                          nextState = INITIATOR;
                          break;
                      }
                      break;

        case MASTER : // Next state can be STANDBY or MASTER 
                      switch (expectedRole)
                      {
                        case SLAVE_ROLE:
                          nextState = STANDBY;
                          break;
                        case MASTER_ROLE:
                          nextState = MASTER;
                          break;
                        default:
                          nextState = MASTER;
                          break;
                      }
                      break;
        default:
                      nextState = STANDBY;
                      break;
      }
      SetState(nextState);
    }

  // The BB Manager represents a device in a link
  //设置与单个对端设备（otherLinkManager）的点对点链路
  void
    BleLinkManager::SetupLink(BleLinkManager::Role myRole, 
        Ptr<BleLinkManager> otherLinkManager, bool scheduled, 
        uint32_t nbTxWindowOffset, uint32_t nbConnectionInterval)
    {
      NS_LOG_FUNCTION (this << myRole);
      this->expectedRole = myRole;
     
      Ptr<BleLink> link = CreateObject<BleLink> ();
      this->SetAssociatedLink(link);
      otherLinkManager->SetAssociatedLink(link);
      this->m_nextExpectedSequenceNumber = false;
      this->m_sequenceNumber = false;
      otherLinkManager->m_nextExpectedSequenceNumber = false;
      otherLinkManager->m_sequenceNumber = false;
      this->m_lastUnmappedChannelIndex = 0;
      otherLinkManager->m_lastUnmappedChannelIndex = 0;
      // If SLAVE: start advertising in order to find master
      //
      // to start: assume that links are created instantly 
      // and receiver is found immediately.
      if (this->expectedRole == SLAVE_ROLE)
      {
        link->AddSlave(this->GetBBManager());
        link->SetMaster(otherLinkManager->GetBBManager());
        link->SetLinkType(BleLink::LinkType::POINT_TO_POINT);
        otherLinkManager->expectedRole = MASTER_ROLE;
      }
      else if (this->expectedRole == MASTER_ROLE)
      {
        link->AddSlave(otherLinkManager->GetBBManager());
        link->SetMaster(this->GetBBManager());
        link->SetLinkType(BleLink::LinkType::POINT_TO_POINT);
        otherLinkManager->expectedRole = SLAVE_ROLE;
      }
      else // STANDBY and CONNECTIONLESS can be different,
        // but lets start with connected links 
        // For CONNECTIONLESS: use other SetupLink function
      {
        link->SetMaster(this->GetBBManager());
        link->SetLinkType(BleLink::LinkType::UNCONNECTED);
      }
      link->SetChannel (
          this->GetBBManager()->GetLinkController()
          ->GetChannelBasedOnChannelIndex (0));
     
      int connInterval = nbConnectionInterval; //3200
      int txWindowSize = 5000; //4*1250; // in Microseconds
      int txWindowOffset = nbTxWindowOffset*(txWindowSize/1250+1); //偏移量以时间槽为单位，用于在连接间隔内定位连接事件
       
      if (scheduled)
      {
        this->SetConnInterval (MicroSeconds(connInterval*1250));//设置连接间隔，BLE 标准要求 7.5ms~4s，超出范围时记录警告
        otherLinkManager->SetConnInterval (MicroSeconds(connInterval*1250));
        this->SetTransmitWindowOffset (MicroSeconds (txWindowOffset*1250));//决定连接事件在连接间隔中的起始时间，设置传输窗口偏移，需小于连接间隔
        this->SetTransmitWindowSize (MicroSeconds (txWindowSize));
        otherLinkManager->SetTransmitWindowOffset (
            MicroSeconds (txWindowOffset*1250));
        otherLinkManager->SetTransmitWindowSize (MicroSeconds (txWindowSize));
      }
      else
      // Random connection parameters
      {
        Ptr<UniformRandomVariable> randT = CreateObject<UniformRandomVariable> ();
        randT->SetAttribute("Max", DoubleValue (3200));
        if (connInterval == 0)
          connInterval = randT->GetInteger(6, 3200);
        txWindowOffset = randT->GetInteger(0, connInterval);
        this->SetConnInterval (MicroSeconds(connInterval*1250));
        otherLinkManager->SetConnInterval (MicroSeconds(connInterval*1250));
        this->SetTransmitWindowOffset (MicroSeconds (txWindowOffset*1250));
        this->SetTransmitWindowSize (MilliSeconds (5));
        otherLinkManager->SetTransmitWindowOffset (
            MicroSeconds (txWindowOffset*1250));
        otherLinkManager->SetTransmitWindowSize (MilliSeconds (5));
      }

      NS_LOG_INFO ("For link " << link << " connInterval = " 
          << connInterval*1250 << "us, txWindowOffset = " 
          << txWindowOffset*1250 << "us, WindowSize = 5 ms ");


      Simulator::ScheduleNow(
          &BleLinkManager::PrepareNextTransmitWindow,
          this);
      Simulator::ScheduleNow(
          &BleLinkManager::PrepareNextTransmitWindow,
          otherLinkManager);
      
    }


  // Same as above, but now for multiple peers, I will be the slave
  //  keepAlive needs to be disabled on other peers 
  //  (this will function as advertisement.)
  //设置与多个对端设备的广播链路（无连接模式）
  void
    BleLinkManager::SetupLink(
        std::vector<Ptr<BleLinkManager>> otherLinkManagers, 
        bool scheduled, uint32_t nbTxWindowOffset, 
        uint32_t nbConnectionInterval, bool collAvoid)
    {
      NS_LOG_FUNCTION (this );
      this->expectedRole = BleLinkManager::Role::CONNECTIONLESS_ROLE;
      this->SetState(BleLinkManager::State::SCANNER);
      
      int connInterval = nbConnectionInterval; //3200
      int txWindowSize = 5000; // in Microseconds
      int txWindowOffset = nbTxWindowOffset*(txWindowSize/1250+1); 
     
      if (! scheduled)
      {
        Ptr<UniformRandomVariable> randT = 
          CreateObject<UniformRandomVariable> ();
        randT->SetAttribute("Max", DoubleValue (3200));
        if (connInterval == 0)
          connInterval = randT->GetInteger(6, 3200);
        txWindowOffset = randT->GetInteger(0, connInterval);
      }

      this->SetConnInterval (MicroSeconds(connInterval*1250));
      this->SetTransmitWindowOffset (MicroSeconds (txWindowOffset*1250));
      this->SetTransmitWindowSize (MicroSeconds (txWindowSize));
      this->SetKeepAliveActive (false);
  
      Ptr<BleLink> link = CreateObject<BleLink> ();
      link->SetMaster(this->GetBBManager());
      link->SetLinkType(BleLink::LinkType::BROADCAST);
      link->SetChannel (
          this->GetBBManager()->GetLinkController()
          ->GetChannelBasedOnChannelIndex (0));
      this->SetAssociatedLink(link);
      this->m_nextExpectedSequenceNumber = false;
      this->m_sequenceNumber = false;
      this->m_lastUnmappedChannelIndex = 0;
      uint16_t counter = 1;
      uint16_t max_counter = otherLinkManagers.size() + 1; 
 
      this->SetAdvSleepCounter (0);
      this->SetMaxAdvSleep (max_counter);
      this->SetAdvCollisionAvoidance (collAvoid);

      for (auto lm : otherLinkManagers)
      {
        lm->SetAssociatedLink (link);
        lm->m_nextExpectedSequenceNumber = false;
        lm->m_sequenceNumber = false;
        lm->m_lastUnmappedChannelIndex = 0;
        link->AddSlave(lm->GetBBManager());
        lm->expectedRole = BleLinkManager::Role::CONNECTIONLESS_ROLE;
        lm->SetState(BleLinkManager::State::SCANNER);

        lm->SetConnInterval (MicroSeconds(connInterval*1250));
        lm->SetTransmitWindowOffset (MicroSeconds (txWindowOffset*1250));
        lm->SetTransmitWindowSize (MicroSeconds (txWindowSize));
        lm->SetKeepAliveActive (false);

        lm->SetAdvSleepCounter (counter);
        lm->SetMaxAdvSleep (max_counter);
        lm->SetAdvCollisionAvoidance (collAvoid);
        counter++;

        Simulator::ScheduleNow(
          &BleLinkManager::PrepareNextTransmitWindow,
          lm);
      }
 
      NS_LOG_INFO ("For link " << link << " connInterval = " 
          << connInterval*1250 << "us, txWindowOffset = " 
          << txWindowOffset*1250 << "us, WindowSize = 5 ms ");

      Simulator::ScheduleNow(
          &BleLinkManager::PrepareNextTransmitWindow,
          this);
    }
    
  /*********************
   * GETTERS & SETTERS *
   *********************/
  //广播冲突避免
  void
    BleLinkManager::SetAdvCollisionAvoidance (bool collAvoid)
    {
      NS_LOG_FUNCTION (this);
      m_broadcastCollisionAvoidance = collAvoid;
    }

  void
    BleLinkManager::SetAdvSleepCounter (uint16_t cntr)
    {
      NS_LOG_FUNCTION (this);
      m_advSleepCounter = cntr;
    }

  void
    BleLinkManager::SetMaxAdvSleep (uint16_t max_counter)
    {
      NS_LOG_FUNCTION (this);
      m_advSleepMax = max_counter;
    }

  uint8_t 
    BleLinkManager::GetCurrentChannelIndex()
    {
      NS_LOG_FUNCTION (this);
      return m_dataChannelIndex;
    }

  Ptr<BleLink>
    BleLinkManager::GetAssociatedLink()
    {
      return m_associatedLink;
    }

  void
    BleLinkManager::SetAssociatedLink(Ptr<BleLink> link)
    {
      NS_LOG_FUNCTION (this);
      m_associatedLink = link;
    }

  //设置连接间隔，BLE 标准要求 7.5ms~4s，超出范围时记录警告
  void
    BleLinkManager::SetConnInterval (Time connInterval)
    {
      NS_LOG_FUNCTION (this);
      if (connInterval >= MicroSeconds (7500) 
          && connInterval <= Seconds (4))
         // && connInterval%MilliSeconds(1.25) < NanoSeconds(PRECISION))
      {
        m_connInterval = connInterval;
      }
      else
      {
        NS_LOG_WARN ("Connection interval will now be set to a value that"
            "is outside the range that the BLE standard accepts");
        m_connInterval = connInterval;
      }
    }

  //设置从设备延迟，需满足 BLE 标准（小于监督超时/连接间隔 - 1 且不超过 500）
  void
    BleLinkManager::SetConnSlaveLatency (uint16_t connSlaveLatency)
    {
      NS_LOG_FUNCTION (this);
      if (! (connSlaveLatency <= 
            (GetConnSupervisionTimeout()/GetConnInterval() - 1) ||
            connSlaveLatency <= 500))
      {
        NS_LOG_WARN (" Connection Slave Latency will be set "
            "to an invalid value" );
      }
      m_connSlaveLatency = connSlaveLatency;
    }

  //设置监督超时，BLE 标准要求 100ms~32s
  void
    BleLinkManager::SetConnSupervisionTimeout (Time connSupervisionTimeout)
    {
      NS_LOG_FUNCTION (this);
      if (! ((connSupervisionTimeout >= MilliSeconds (100)) 
            && (connSupervisionTimeout <= Seconds(32))
            //&& (connSupervisionTimeout%MilliSeconds(10) < NanoSeconds(PRECISION)) 
            ) )
      {
        NS_LOG_WARN (" Connection Supervision "
            "timout will be set to an invalid value" );
      }
      m_connSupervisionTimeout = connSupervisionTimeout;
    }

  //设置连接事件计数器，无有效性检查 
  void
    BleLinkManager::SetConnEventCounter (uint16_t connEventCounter)
    {
      NS_LOG_FUNCTION (this);
      m_connEventCounter = connEventCounter;
    }

  //设置传输窗口偏移，需小于连接间隔
  void
    BleLinkManager::SetTransmitWindowOffset (Time transmitWindowOffset)
    {
      NS_LOG_FUNCTION (this);
      if (! ((transmitWindowOffset <= this->GetConnInterval())
            //&& (transmitWindowOffset%MilliSeconds(1.25) < NanoSeconds(PRECISION)) 
            ) )
      {
        NS_LOG_WARN ("Transmit Window Offset will be set to an invalid value " );
      }
      m_transmitWindowOffset = transmitWindowOffset;
    }

  //设置传输窗口大小，需满足小于连接间隔减 1.25ms 且不超过 10ms
  void
    BleLinkManager::SetTransmitWindowSize (Time transmitWindowSize)
    {
      NS_LOG_FUNCTION (this);
      if (! ((transmitWindowSize <= this->GetConnInterval() - MicroSeconds  (1250))
            && (transmitWindowSize <= MilliSeconds (10))
            //&& (transmitWindowSize%MilliSeconds(1.25) < NanoSeconds(PRECISION)) 
            ) )
      {
        NS_LOG_WARN ("Transmit Window Size will be set to an invalid value " );
      }
      m_transmitWindowSize = transmitWindowSize;
    }

  Time 
    BleLinkManager::GetConnInterval (void)
    {                
      return m_connInterval;//连接间隔
    }                
                     
                     
  uint16_t               
    BleLinkManager::GetConnSlaveLatency (void)
    {                
      return m_connSlaveLatency; //设备延迟
    }                
                     
                     
  Time               
    BleLinkManager::GetConnSupervisionTimeout (void)
    {                
      return m_connSupervisionTimeout;//监督超时
    }                
                     
                     
  uint16_t               
    BleLinkManager::GetConnEventCounter (void)
    {                
      return m_connEventCounter;//事件计数器
    }                
                     
                     
  Time               
    BleLinkManager::GetTransmitWindowOffset (void)
    {                
      return m_transmitWindowOffset;//传输窗口偏移
    }                
                     
                     
  Time               
    BleLinkManager::GetTransmitWindowSize (void)
    {
      return m_transmitWindowSize;
    }

  Time
    BleLinkManager::GetLastTimeConnectionEstablished (void)
    {
      return m_lastTimeConnectionEstablished;
    }

  //设置最后连接建立时间，并重置 m_firstTransmitWindowDone，表示尚未完成首次传输窗口
  void
    BleLinkManager::SetLastTimeConnectionEstablished (Time lasttime)
    {
      NS_LOG_FUNCTION (this);
      m_lastTimeConnectionEstablished = lasttime;
      m_firstTransmitWindowDone = false;
    }

  //设置上一次传输窗口时间，并标记首次传输窗口已完成
  Time
    BleLinkManager::GetLastTransmitWindowTime (void)
    {
      return m_lastTransmitWindowTime;
    }

  void
    BleLinkManager::SetLastTransmitWindowTime (Time lasttime)
    {
      NS_LOG_FUNCTION (this);
      m_lastTransmitWindowTime = lasttime;
      m_firstTransmitWindowDone = true;
    }

  void 
    BleLinkManager::SetPeerHasMoreData (bool md)
    {
      NS_LOG_FUNCTION (this << md);
      m_peerHasMoreData = md;
    }

  bool
    BleLinkManager::GetPeerHasMoreData (void)
    {
      return m_peerHasMoreData;
    }

  //mhl修改
  void BleLinkManager::SetNotifyPeerHasMoreDataCallback(Callback<void, bool> cb) {
    NS_LOG_FUNCTION(this);
    m_notifyPeerHasMoreData = cb;
  }

  void BleLinkManager::ChangePeerHasMoreData(bool flag){
    NS_LOG_FUNCTION(this);
    if(!m_notifyPeerHasMoreData.IsNull())
      m_notifyPeerHasMoreData(flag);
  }

  void BleLinkManager::SetNotifyPeerChangeStateCallback(Callback<void, State> cb) {
      NS_LOG_FUNCTION(this);
      m_notifyPeerChangeState = cb;
  }

  Ptr<DropTailQueue<QueueItem>> 
    BleLinkManager::GetQueue (void)
    {
      NS_LOG_FUNCTION (this);
      NS_ASSERT(m_queue != 0);
      return m_queue;
    }

  Ptr<BleBBManager>
    BleLinkManager::GetBBManager (void)
    {
      return m_bbManager;
    }

  void
    BleLinkManager::SetBBManager (Ptr<BleBBManager> bbm)
    {
      NS_LOG_FUNCTION (this);
      m_bbManager = bbm;
    }

  Ptr<Packet>
    BleLinkManager::GetCurrentPacket (void)
    {
      return m_currentPacket;
    }

  void
    BleLinkManager::SetCurrentPacket (Ptr<Packet> packet)
    {
      NS_LOG_FUNCTION (this);
      m_currentPacket = packet;
    }

  void
    BleLinkManager::SetKeepAliveActive (bool keepAliveActive)
    {
      m_keepAliveActive = keepAliveActive;
    }

  bool
    BleLinkManager::NeedToSendAtLeastOne()
    {

      if (expectedRole == MASTER_ROLE)
      {
        return ! m_onePacketSend;
      }
      else if (expectedRole == SLAVE_ROLE)
      {
        return (! m_onePacketSend) && m_keepAliveActive;
      }
      else
      {
        return false;
      }
    }

  void
    BleLinkManager::SetNESN(bool nesn)
    {
      NS_LOG_FUNCTION (this);
      m_nextExpectedSequenceNumber = nesn;//预期序列号
    }

  void
    BleLinkManager::SetSN (bool sn)
    {
      m_sequenceNumber = sn;//序列号
    }

  void
    BleLinkManager::SetHopIncrement (uint8_t hopIncrement)
    {
      NS_LOG_FUNCTION (this);
      m_hopIncrement = hopIncrement;
    }



   Time
    BleLinkManager::GetNextTransmitWindowTime()
    {
      NS_LOG_FUNCTION (this);

      Time nextTXWindow = Seconds(0);
      if (! m_firstTransmitWindowDone) // first window
      {
        nextTXWindow = MicroSeconds (1250) + GetTransmitWindowOffset(); 
      }
      else
      {
        nextTXWindow = GetConnInterval ();
      }
      NS_LOG_INFO (" Next TX Window in " << nextTXWindow.GetSeconds() 
          << " my link = " << this->GetAssociatedLink());
      return nextTXWindow;
    }

   void
     BleLinkManager::SetMyLastMD (bool md)
     {
       m_lastMD = md;
     }

   bool
     BleLinkManager::GetMyLastMD ()
     {
       return m_lastMD;
     }

   bool
     BleLinkManager::IsInsideLastTransmitWindow (Time thisTime)
     {
       return (thisTime >= GetLastTransmitWindowTime() 
           && thisTime <= GetLastTransmitWindowTime () + GetTransmitWindowSize ());
     }

   // Prepares the simulator for the next transmitwindow
   // that is comming
   void
     BleLinkManager::PrepareNextTransmitWindow ()
     {
       NS_LOG_FUNCTION (this);
       m_nextWindow = Simulator::Schedule(
           GetNextTransmitWindowTime(),
           &BleLinkManager::StartTransmitWindow,
           this);
     }

  //管理发送序列号（SN），判断是否发送新数据
  bool 
     BleLinkManager::ManageSequenceNumberTX(void)
     {
       NS_LOG_FUNCTION (this << m_sequenceNumber << m_nextExpectedSequenceNumber);
       if (m_sequenceNumber != m_nextExpectedSequenceNumber)
       {
         NS_LOG_INFO ("previous ack, sending new data");
         m_sequenceNumber = ! m_sequenceNumber;
         return true;
       }
       else
       {
         NS_LOG_INFO ("previous nack, sending old data");
         return false;
       }
     }

  //管理接收序列号（NESN），判断是否接收到新数据
  bool 
     BleLinkManager::ManageSequenceNumberRX(void)
     {
       NS_LOG_FUNCTION (this << m_sequenceNumber << m_nextExpectedSequenceNumber);
       if (m_sequenceNumber != m_nextExpectedSequenceNumber)
       {
         NS_LOG_INFO ("Received old data, ignoring");
         return false;
       }
       else
       {
         NS_LOG_INFO ("Receiving new data");
         m_nextExpectedSequenceNumber = ! m_nextExpectedSequenceNumber;
         return true;
       }
     }

   void
     BleLinkManager::SendNextPacket()
     {
       NS_LOG_FUNCTION (this);
       Time currentTime = Simulator::Now();
       NS_ASSERT (this->GetState() != SCANNER ); // A scanner cannot send data. 确保非扫描状态（SCANNER 不发送数据）
       
       bool readyForNewData;
       if (this->GetState() == ADVERTISER )
         readyForNewData = true; // Advertising will not get ack，广播包无需确认，强制 readyForNewData = true
       else
         readyForNewData = ManageSequenceNumberTX();//调用 ManageSequenceNumberTX 检查是否发送新包（基于 NESN 确认）
       
       if (IsInsideLastTransmitWindow (currentTime))
       {
            //重传情况：如果有当前包（m_currentPacket != 0）且非新数据且非广播状态，记录重传日志（不构建新包，直接重用当前包）
           if (this->GetCurrentPacket () != 0 && (! readyForNewData) 
               && (! (this->GetState() == ADVERTISER)))
           {
             // Transmission of a packet failed during the last
             // TX slot, resend this packet first.
             NS_LOG_INFO(" Retransmitting previous packet ");
           }

           //新包情况
           else // No current packet
           {
             NS_ASSERT(m_queue != 0);
             //队列非空
             if (! (m_queue->IsEmpty()))
             {
               BleMacHeader bmh1;
               Ptr<QueueItem> item = m_queue->Dequeue ();
               NS_ASSERT (item);
               NS_LOG_DEBUG ("New packet set as current packet. "
                   "This new packet is not a dummy / Keep Alive Packet. "
                   "Packets left in the queue: "
                   << m_queue->GetCurrentSize());
               Ptr<Packet> packet = item->GetPacket();
               packet->RemoveHeader(bmh1);

               if (this->GetState() == ADVERTISER)
               {
                 // If advertising, dest address needs to be broadcast address
                 NS_ASSERT (bmh1.GetDestAddr() == Mac16Address("ff:ff"));
               }
               /*LLID=0b10（数据 PDU）。
                NESN/SN：更新序列号。
                MD：根据队列状态（空为 0，非空为 1）。
                长度：硬编码为 1（可能为调试）*/
               bmh1.SetLLID(0b10);
               bmh1.SetNESN(m_nextExpectedSequenceNumber);
               bmh1.SetSN(m_sequenceNumber);
               // More data to send
               bmh1.SetMD(! m_queue->IsEmpty ());
               //mhl修改
               if(m_queue->IsEmpty ()==false){
                 NS_LOG_INFO ("MD = 1 , m_queue_size = "<<m_queue->GetCurrentSize());
               } 

               // 通知对端（设备0）更新 m_peerHasMoreData
                /*if (m_queue->IsEmpty() == false  && !m_notifyPeerHasMoreData.IsNull()) {
                    NS_LOG_INFO("Notifying peer of MD=1");
                    m_notifyPeerHasMoreData(true);
                } */
               /*else if (m_queue->IsEmpty() && !m_notifyPeerHasMoreData.IsNull()) {
                    NS_LOG_INFO("Notifying peer of MD=0");
                    m_notifyPeerHasMoreData(false);
                }*/

               this->SetMyLastMD(! m_queue->IsEmpty ());
               //bmh1.SetLength(item->GetPacket ()->GetSize());
               bmh1.SetLength(1);
               packet->AddHeader(bmh1);
               this->SetCurrentPacket (packet);
               m_onePacketSend =true;
             }
             //队列为空
             else
             {
               // Queue was empty, send dummy packet on broadcast
               //若非 ADVERTISER，或需发送至少一个包（NeedToSendAtLeastOne），或对端有数据（GetPeerHasMoreData）
               if (( ! ( this->GetState() == ADVERTISER) ) || 
                   NeedToSendAtLeastOne() || 
                   GetPeerHasMoreData()) 
               // Peer has more data, 
               //   so I should answer to him as long as he has data
               // OR:
               // I need to send at least one packet in a TX window
               //  because:
               //   - I am a master
               //   - Keep alive is set to active
               //   - If I don't answer, connection can be considered lost
               //   - I need to ack received packet
               /*发送保持活动包：
                LLID=0b01，长度=0，MD=0。
                设置广播地址（FF:FF）*/
               {
                 BleMacHeader bmh2;
                 Ptr<Packet> dummyPacket = Create<Packet> ();
                 bmh2.SetLength(0);
                 bmh2.SetLLID(0b01);
                 bmh2.SetMD(0);
                 this->SetMyLastMD(! m_queue->IsEmpty ());
                 bmh2.SetNESN(m_nextExpectedSequenceNumber);
                 bmh2.SetSN(m_sequenceNumber);
                 bmh2.SetSrcAddr(
                     this->GetBBManager()->GetNetDevice()->GetAddress16());
                 bmh2.SetDestAddr(Mac16Address("FF:FF"));
                 dummyPacket->AddHeader(bmh2);
                 SetCurrentPacket (dummyPacket);
                 m_onePacketSend = true;
               }
               else
               {
               }
             }
           }

           if (! this->GetCurrentPacket() == 0)
           {
             BleMacHeader bmh3;
             this->GetCurrentPacket()->PeekHeader(bmh3);
             NS_LOG_INFO ("Src Addr for current packet: " 
                 << bmh3.GetSrcAddr() << " Dest address for current packet: " 
                 << bmh3.GetDestAddr()); 
              //mhl修改
             if(bmh3.GetDestAddr()!=Mac16Address("FF:FF")){
                NS_LOG_INFO("Notifying peer of MD=1");
                    m_notifyPeerHasMoreData(true);
             }
             Simulator::ScheduleNow(
                     &BleLinkController::StartPacketTransmission, 
                     this->GetBBManager()->GetLinkController(),
                     this);
           }
           if (this->GetState() == ADVERTISER)
           {
             this->SetState(SCANNER);//ADVERTISER → SCANNER：广播发送后切换到监听状态
           }  
       }
       else
       {
       //  NS_LOG_INFO("No Time left inside this window");
          this->GetBBManager()->GetPhy()->ChangeState(BlePhy::State::IDLE);
          
          this->GetBBManager()->SetActiveLinkManager(0);
       }
     }

   void
     BleLinkManager::HandleTXDone ()
     {
       NS_LOG_FUNCTION (this);
       this->GetBBManager()->GetPhy()->ChangeState(BlePhy::State::IDLE);
       this->SetCurrentPacket(0);
       Time currentTime = Simulator::Now();
        NS_LOG_FUNCTION ("RX-MHL");
       /*if (IsInsideLastTransmitWindow (currentTime) && GetPeerHasMoreData()  )
       {
          NS_LOG_INFO ("RX-MHL-new");
         Simulator::Schedule(MicroSeconds(T_IFS),
             &BleLinkController::PrepareForReception,
             this->GetBBManager()->GetLinkController(),
             this);
       }
       else
       {
          this->GetBBManager()->SetActiveLinkManager(0);
       }*/
        // 延迟检查，确保对端数据包到达
       // 延迟调度辅助函数，而不是lambda
      if(this->GetState()==MASTER)
      {
        NS_LOG_INFO("RX-MHL-new");
        Simulator::Schedule(NanoSeconds(150310),//MicroSeconds(150.3),
          &BleLinkManager::DelayedPrepareForReception,
          this);  // 使用成员函数指针 &BleLinkManager::DelayedPrepareForReception
      } 
      else if(this->GetState()==SLAVE && !m_notifyPeerChangeState.IsNull())
      {
        NS_LOG_INFO("SetState STANDBY");
        this->SetState(STANDBY);
        m_notifyPeerChangeState(STANDBY);
      }
    }

  void BleLinkManager::DelayedPrepareForReception() {
    NS_LOG_FUNCTION(this<<expectedRole<<m_lastMD<<GetPeerHasMoreData());
    Time currentTime = Simulator::Now();
    if (IsInsideLastTransmitWindow(currentTime) && GetPeerHasMoreData() ) //这里判断的是0的对端，但是0是主节点，对端有好几个，所以这里一直是0
    {
      Simulator::ScheduleNow(&BleLinkController::PrepareForReception,
            this->GetBBManager()->GetLinkController(),
            this);
    } 
    else 
    {
        this->GetBBManager()->SetActiveLinkManager(0);
    }
  }

  //模拟BLE的周期性传输窗口（connection event），主设备发起发送，从设备等待接收。
  //支持广播模式（CONNECTIONLESS_ROLE）的广告发送与扫描切换。
  //管理窗口时间参数（如 m_transmitWindowSize、m_connInterval）和状态（如 m_onePacketSend、m_firstTransmitWindowDone）
   void
     BleLinkManager::StartTransmitWindow ()
     {
       // Schedule EndTransmitWindow
       // If Master:
       // Go in TX mode
       // Check if there is a packet in queue
       // If there is a packet:
       // transmit
       // else
       // send dummy packet, so Slave know he can send
       // If slave:
       // Go in RX mode
       // wait for packet from master to arrive

       NS_LOG_FUNCTION (this);
       if ( this->GetBBManager()->GetActiveLinkManager() == 0)
       {
         this->GetBBManager()->SetActiveLinkManager(this);

         NS_LOG_INFO (this << " Start of a TransmitWindow, my Role = " 
             << expectedRole << " my state = " << GetState() 
             << " my link = " << this->GetAssociatedLink() << " this BBM = " 
             << this->GetBBManager());

         SetLastTransmitWindowTime(Simulator::Now());
         m_endOfCurrentWindow = Simulator::Schedule (
             GetTransmitWindowSize(),
             &BleLinkManager::EndTransmitWindow,
             this);

         m_firstTransmitWindowDone = true;//标记首次窗口完成
         m_onePacketSend = false;//重置窗口内发送标志
         SetMyLastMD(true);

         PrepareNextTransmitWindow ();
         ManageChannelSelection();

         if(this->GetCurrentPacket()!=0){
          NS_LOG_INFO(" 具有数据包 "<<this->GetCurrentPacket());
         }

         if (expectedRole == MASTER_ROLE)
         {
           this->SetState(MASTER);
           SendNextPacket();
           
         }
         
         else if (expectedRole == SLAVE_ROLE)
         {
           this->SetState(SLAVE);
           Simulator::ScheduleNow(
               &BleLinkController::PrepareForReception,
               this->GetBBManager()->GetLinkController(),
               this);
               
         }
         else if (expectedRole == CONNECTIONLESS_ROLE )
         {
           
           if (this->GetState () == SCANNER)
           {
             if ((! m_queue->IsEmpty()) && ((m_advSleepCounter == 0) 
                   || (m_broadcastCollisionAvoidance == false)))
             {
               // Data in Queue to advertise 
               // ==> move to advertiser state, make sure to exit afterwards
               this->SetState (ADVERTISER);
               SendNextPacket ();
             }
             else 
             {
             Simulator::ScheduleNow(
               &BleLinkController::PrepareForReception,
               this->GetBBManager()->GetLinkController(),
               this);
             }
             if (m_advSleepCounter == m_advSleepMax)
             {
               m_advSleepCounter = 0;
             }
             else
             {
               m_advSleepCounter++;
             }
           }
           else
           {
             NS_LOG_ERROR ("The Link Manager is "
                 "in an impossible Connectionless State!");
             NS_ASSERT (false);
           }
         }
         else
         {
           NS_LOG_WARN ("The Link Manager is neither in Master or Slave role!");
         }
       }
       else
       {
         NS_LOG_INFO (this << " The BB manager " << this->GetBBManager() 
             << " is busy, this tx window will be skipped,  my link = " 
             << this->GetAssociatedLink() << " Active LM = " 
             << this->GetBBManager()->GetActiveLinkManager() 
             << " current PHY state = " << this->GetBBManager()->GetPhyState());
         NS_ASSERT (this->GetBBManager()->GetActiveLinkManager() != this);
         NS_ASSERT (this->GetBBManager()->GetActiveLinkManager()->GetBBManager() 
             == this->GetBBManager());

         // This can be false only if the active link manager 
         // is transmitting outside tx window
         NS_ASSERT (
             this->GetBBManager()->GetActiveLinkManager()
             ->IsInsideLastTransmitWindow(Simulator::Now()));

         // Callback management 
         this->GetBBManager()->GetNetDevice()->NotifyTXWindowSkipped();

         SetLastTransmitWindowTime(Simulator::Now());
         PrepareNextTransmitWindow ();
         ManageChannelSelection();
       }
     }

    //结束当前传输窗口，清理状态
   void
     BleLinkManager::EndTransmitWindow ()
     {
       NS_LOG_FUNCTION (this);

       NS_LOG_INFO (this << " End of a TransmitWindow, my Role = " 
           << expectedRole << " my link = " 
           << this->GetAssociatedLink() 
           << " this BBM = " << this->GetBBManager());
       
       NS_LOG_INFO ("End of a TransmitWindow");
       // set phy in standby mode after current TX / RX event is done,
       // deactive activeLinkManager in BBM
       // schedule next tx window
       if (this->GetBBManager()->GetActiveLinkManager() == 0)
       {
         this->GetBBManager()->GetPhy()->ChangeState(BlePhy::State::IDLE);
       }
       else if (this->expectedRole == CONNECTIONLESS_ROLE)
       {
         this->GetBBManager()->GetPhy()->ChangeState(BlePhy::State::IDLE);
         this->GetBBManager()->SetActiveLinkManager(0);
         this->SetState (SCANNER);
       }
       else if (this->GetBBManager()->GetActiveLinkManager() == this)
       {
         if (this->GetBBManager()->GetPhy()->GetState () == BlePhy::State::RX)
         {
           // BB manager is prepared to receive packet, 
           // but no packet will be comming 
           this->GetBBManager()->GetPhy()->ChangeState(BlePhy::State::IDLE);
            this->GetBBManager()->SetActiveLinkManager(0);
 
           NS_LOG_INFO (" TXWindow closed, I was still in receive mode");
         }
         else if (this->GetBBManager()->GetPhyState() == BlePhy::State::IDLE)
         {
            this->GetBBManager()->SetActiveLinkManager(0);
         }
         else
         {
           NS_LOG_WARN (" End of transmitwindow, but still busy, PHY state = " 
               << this->GetBBManager()->GetPhyState());
         }
       }

     }

   bool
     BleLinkManager::IsUsedChannel (uint8_t channelIndex)
     {
       for (auto v : this->m_usedChannels) 
       {
         if (v == channelIndex)
           return true;
       }
       return false;
     }
 
   void
     BleLinkManager::SetUsedChannels (std::vector<uint8_t> usedChannels)
     {
       NS_LOG_FUNCTION (this);
       m_usedChannels = usedChannels;
     }


   void
     BleLinkManager::ManageChannelSelection ()
     {
       NS_LOG_FUNCTION (this);
       m_unmappedChannelIndex = (m_lastUnmappedChannelIndex + m_hopIncrement) % 37;
       if (IsUsedChannel (m_unmappedChannelIndex)) 
         // Is unmappedChannelIndex = used channel
       {
         // Select channel index
         m_dataChannelIndex = m_unmappedChannelIndex;
       }
       else
       {
         NS_ASSERT (m_usedChannels.size() != 0);
         uint8_t remappingIndex = m_unmappedChannelIndex % m_usedChannels.size();
         // Find corresponding channelIndex (m_dataChannelIndex)
         m_dataChannelIndex = m_usedChannels.at(remappingIndex);
       }
       m_lastUnmappedChannelIndex = m_unmappedChannelIndex;
      
       // Make sure PHY listens / sends on this channel
       this->GetBBManager()->GetPhy()->SetChannel(
           this->GetBBManager()->GetLinkController()->
           GetChannelBasedOnChannelIndex (m_dataChannelIndex));
       this->GetBBManager()->GetPhy()->SetChannelIndex(m_dataChannelIndex);
       NS_LOG_INFO (this << " Current Channel Index is : " 
           << int(m_dataChannelIndex) );
     }
}

