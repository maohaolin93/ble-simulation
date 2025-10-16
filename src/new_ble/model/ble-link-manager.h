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

//BLE 链路的建立、状态转换、数据传输、信道选择
/*管理 BLE 设备的状态（如待机、广播、扫描、主设备、从设备等）。
建立和管理 BLE 链路（点对点或广播）。
处理数据包的发送、接收、队列管理和重传机制。
实现 BLE 的信道选择算法（频率跳跃）。
支持连接参数的配置（如连接间隔、传输窗口偏移等）*/

#ifndef BLE_LINK_MANAGER_H
#define BLE_LINK_MANAGER_H

// Includes
#include <ns3/callback.h>
#include <ns3/traced-callback.h>
#include <ns3/object.h>
#include <ns3/pointer.h>
#include <ns3/ptr.h>
#include <ns3/ble-link.h>
#include <ns3/nstime.h>
#include <ns3/constants.h>
#include <ns3/packet.h>
#include <ns3/simulator.h>
#include <ns3/multi-model-spectrum-channel.h>

namespace ns3 {

  // Classes
  template <typename Item> class DropTailQueue;
  class BleBBManager;
  class BleLinkController;
  class BleNetDevice;
  class QueueItem;
/** 
 * \ingroup ble
 * \brief Implementation for the Link Manager of the BLE protocol
 * This implements the following features:
 *
 */
  class BleLinkManager : public Object
  {
    public:

      // State variable 定义设备状态
      //待机状态、广播状态、扫描状态（监听广播包）、发起连接状态、从设备状态、主设备状态
      enum State
      {
        STANDBY, ADVERTISER, SCANNER, INITIATOR, SLAVE, MASTER
      };

      //定义设备角色
      enum Role
      {
        SLAVE_ROLE, MASTER_ROLE, STANDBY_ROLE, CONNECTIONLESS_ROLE 
      };

      //连接，无连接
      enum StateType
      {
        CONNECTIONLESS, CONNECTED
      };

      BleLinkManager ();
      ~BleLinkManager ();

      void DoDispose ();

      static TypeId GetTypeId (void);

      void SetState(State state);
      State GetState();

      bool IsConnected();

      void NextState();//根据当前状态和预期角色转换到下一状态

      // Setup a Link to a specific receiver with the role as 
      // expected role at the end of the setup process.

      // nbConnectionInterval == 0 means random connectioninterval
      //用于点对点链路设置（一个主设备和一个从设备）
      void SetupLink(Role myRole, 
          Ptr<BleLinkManager> otherLinkManager, bool scheduled, 
          uint32_t nbTxWindowOffset, uint32_t nbConnectionInterval);

      //用于广播链路设置（一个设备与多个对等设备）
      void SetupLink(
          std::vector<Ptr<BleLinkManager>> otherLinkManagers, 
          bool scheduled, uint32_t nbTxWindowOffset, 
          uint32_t nbConnectionInterval, bool collAvoid);

      Ptr<BleLink> GetAssociatedLink();
      void SetAssociatedLink(Ptr<BleLink> link);

      Ptr<BleBBManager> GetBBManager();
      void SetBBManager (Ptr<BleBBManager> bbm);

      // Getters and setters for scheduling parameters
      /*connInterval：连接间隔。
        connSlaveLatency：从设备延迟。
        connSupervisionTimeout：连接监督超时。
        connEventCounter：连接事件计数器。
        transmitWindowOffset：传输窗口偏移。
        transmitWindowSize：传输窗口大小*/
      void SetConnInterval (Time connInterval);
      void SetConnSlaveLatency (uint16_t connSlaveLatency);
      void SetConnSupervisionTimeout (Time connSupervisionTimeout);
      void SetConnEventCounter (uint16_t connEventCounter);
      void SetTransmitWindowOffset (Time transmitWindowOffset);
      void SetTransmitWindowSize (Time transmitWindowSize);
      Time GetConnInterval (void);
      uint16_t GetConnSlaveLatency (void);
      Time GetConnSupervisionTimeout (void);
      uint16_t GetConnEventCounter (void);
      Time GetTransmitWindowOffset (void);
      Time GetTransmitWindowSize (void);

      /*
       * Returns the last Time that the connection was established
       * this equals the end of the last CONNECT_REQ PDU that was received.
       */
      //连接建立时间
      Time GetLastTimeConnectionEstablished (void);
      void SetLastTimeConnectionEstablished (Time lasttime);

      /*
       * Returns the start of the previous transmit window.
       */
      //传输窗口时间
      Time GetLastTransmitWindowTime (void);
      void SetLastTransmitWindowTime (Time lasttime);



      // Returns the next time that the device can send
      // a packet over the associated link
      // Also, this time is set to be the beginning of
      // the last transmit window and will be used for the
      // calculation of the next transmit window.
      //下一次传输窗口时间
      Time GetNextTransmitWindowTime (void);

      /*
       * Returns true if 'thisTime' is inside the transmitWindow
       * that was last calculated.
       */
      //判断当前时间是否在传输窗口内
      bool IsInsideLastTransmitWindow (Time thisTime);

      /*
       * Put packet in the queue / buffer, so it can be transmitted
       */
      //管理数据包队列
      Ptr<DropTailQueue<QueueItem>> GetQueue (void);

      //当前发送的数据包
      void SetCurrentPacket (Ptr<Packet> packet);
      Ptr<Packet> GetCurrentPacket (void);

      void StartTransmitWindow (void);
      void EndTransmitWindow (void);
      void PrepareNextTransmitWindow (void);

      void HandleTXDone (void);//处理传输完成
      void SendNextPacket (void);//发送下一数据包

      //mhl修改
      void DelayedPrepareForReception();  // 新增辅助函数

      //管理对端设备是否还有更多数据
      void SetPeerHasMoreData (bool md);
      bool GetPeerHasMoreData (void);

      /*
       * If true: SLAVE will answer so the connection is not lost
       * if there is no data to send.
       */
      //启用/禁用保活机制（从设备即使无数据也发送响应以维持连接）
      void SetKeepAliveActive (bool keepAliveActive);

      /*
       * Returns true if the device needs to send at least
       * one packet in this TX window, even though it has
       * no data in the queue
       */
      //检查是否需要在当前传输窗口发送至少一个数据包（即使队列为空）
      bool NeedToSendAtLeastOne();

     // void SetPreviousNeedsAck(bool needsAck);
     //管理本设备发送的最后一个 MD 位
      void SetMyLastMD(bool md); // The last md bit that I have send to peer.
      bool GetMyLastMD ();


      //管理发送的序列号（SN 和 NESN）
      // Returns true if TX new data
      bool ManageSequenceNumberTX (void);
      // Returns true if RX new data
      //管理接收的序列号
      bool ManageSequenceNumberRX (void);

      //设置序列号（SN）和下一预期序列号（NESN）
      void SetSN (bool sn);
      void SetNESN (bool nesn);

      // Hop increment is used for channel selection algorithm, 
      // should be a random number
      //设置跳频增量（随机数，用于信道选择算法）
      void SetHopIncrement (uint8_t hopIncrement);

      //实现信道选择算法（BLE 的跳频机制）
      void ManageChannelSelection ();

      //检查指定信道是否在使用
      bool IsUsedChannel (uint8_t channelIndex);
      void SetUsedChannels (std::vector<uint8_t> usedChannels);

      uint8_t GetCurrentChannelIndex ();

      //管理广播休眠计数器
      void SetAdvSleepCounter (uint16_t cntr);
      //控制广播事件的间隔
      void SetMaxAdvSleep (uint16_t max_counter);
      //启用/禁用广播冲突避免机制
      void SetAdvCollisionAvoidance (bool collAvoid);

      //mhl修改
      void SetNotifyPeerHasMoreDataCallback(Callback<void, bool> cb);
      void ChangePeerHasMoreData(bool flag);
      void SetNotifyPeerChangeStateCallback(Callback<void, State> cb);

    private:

      Callback<void, bool> m_notifyPeerHasMoreData; // 通知对端更新 MD
      Callback<void, State> m_notifyPeerChangeState; //通知对端更新状态
      // This is false as long as no transmit window has past
      // sinds last connection establishment. This value is
      // set to false by the SetLastTimeConnectionEstablished()
      // function, set to true by SetLastTransmitWindowTime ()
      bool m_firstTransmitWindowDone;//标记是否已完成首次传输窗口

      EventId m_nextWindow;
      EventId m_endOfCurrentWindow;

      State currentState;//当前状态
      Role expectedRole;//期望角色
      Ptr<BleLink> m_associatedLink;//关联的链路对象 
      //<! there is a link manager for each link, 
      // this is the link associated to this link manager

      Time m_lastTimeConnectionEstablished;//记录连接时间
      Time m_lastTransmitWindowTime;//传输窗口时间

      // Scheduling parameters 
      Time m_connInterval;
      uint16_t m_connSlaveLatency;
      // Max time between two received 
      // Data packet PDUs before connection is considered lost
      Time m_connSupervisionTimeout; 
      uint16_t m_connEventCounter;
      Time m_transmitWindowOffset;
      Time m_transmitWindowSize;

      // Packet buffer
      Ptr<DropTailQueue<QueueItem>> m_queue;

      Ptr<BleBBManager> m_bbManager;
      Ptr<Packet> m_currentPacket;//当前数据包
      bool m_currentIsDummy;//是否为占位包

      bool m_nextExpectedSequenceNumber;
      bool m_sequenceNumber;
      bool m_peerHasMoreData;
      bool m_onePacketSend; //标记当前窗口是否发送了数据包
      //m_onePacketSend is true if there was one packet send inside this TX window
      bool m_keepAliveActive;//保活机制状态

      bool m_lastMD;

      uint16_t m_advSleepCounter; 
      // Keeps track of the number of advertising events in which 
      // this link manager cannot send a packet. 
      // Only valid if broadcastCollisionAvoidance is enabled.

      uint16_t m_advSleepMax;
      bool m_broadcastCollisionAvoidance;

      //信道设置
      uint8_t m_lastUnmappedChannelIndex;
      uint8_t m_unmappedChannelIndex;
      uint8_t m_hopIncrement;
      uint8_t m_dataChannelIndex;
      std::vector<uint8_t> m_usedChannels;
  };
}
#endif /* BLE_LINK_MANAGER_H */

