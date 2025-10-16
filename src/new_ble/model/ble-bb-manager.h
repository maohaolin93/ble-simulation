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
/*协调 BLE 设备中的链路管理器（BleLinkManager）、网络设备（BleNetDevice）
和物理层（BlePhy），并管理多个链路（点对点或广播），以及将数据包分发到正确的链路管理器队列*/

#ifndef BLE_BB_MANAGER_H
#define BLE_BB_MANAGER_H

// Includes
#include <ns3/callback.h>
#include <ns3/traced-callback.h>
#include <ns3/object.h>
#include <ns3/ble-phy.h>
#include <ns3/ble-link-manager.h>

#include <ns3/generic-phy.h>

#include <ns3/ptr.h>
#include <ns3/pointer.h>
#include <ns3/packet.h>
#include <ns3/simulator.h>

#include <ns3/constants.h>

namespace ns3 {

  // Classes

  template <typename Item> class DropTailQueue;
  class QueueItem;
  class BleLinkController;
  class BleLink;
  class BleNetDevice;

/** 
 * \ingroup ble
 * \brief Implementation for the broadbandmanager of the BLE protocol
 * This implements the following features:
 *
 */
  class BleBBManager : public Object
  {
    public:

      BleBBManager ();
      BleBBManager (Ptr<BleNetDevice> bleNetDevice);
      ~BleBBManager ();
      void DoDispose (void);

      static TypeId GetTypeId (void);

      BlePhy::State GetPhyState ();//获取物理层状态

      Ptr<BlePhy> GetPhy ();//物理层对象
      Ptr<BleLinkController> GetLinkController ();//链路控制器
      Ptr<BleNetDevice> GetNetDevice ();
      void SetNetDevice (Ptr<BleNetDevice> netDevice);
      void SetPhy (Ptr<BlePhy> phy);
      Ptr<DropTailQueue<QueueItem>> GetQueue (void);

      Ptr<Packet> GetCurrentPacket();
      void SetCurrentPacket(Ptr<Packet> packet);

      // Add a link to the list of links that can be associated
      // to this device
      void AddLinkManager(Ptr<BleLinkManager> linkManager);//添加链路管理器到列表
      //创建单设备链路
      Ptr<BleLink> CreateLink(Ptr<BleBBManager> otherBBManager, 
          BleLinkManager::Role myRole);
      //创建与单个对端设备的点对点链路，支持调度参数
      Ptr<BleLink> CreateLinkScheduled(Ptr<BleBBManager> otherBBManager, 
          BleLinkManager::Role myRole, bool scheduled, 
          uint32_t nbTxWindowOffset, uint32_t nbConnectionInterval);
      
      // Create a link with multiple nodes. this device will be the master
      //多设备广播链路
      Ptr<BleLink> CreateLinkScheduledMultipleNodes(
          std::list<Ptr<BleBBManager>> otherBBManagers, bool scheduled, 
          uint32_t nbTxWindowOffset, uint32_t nbConnectionInterval, 
          bool collAvoid);

      // Check if a specific link exists
      //检查指定链路是否关联到某个链路管理器
      bool LinkExists (Ptr<BleLink> link);
      //检查指定链路管理器是否在 m_linkManagers 中
      bool LinkManagerExists (Ptr<BleLinkManager> linkManager);

      // Check if a link to a device with a specific address exists
      bool LinkExists (Mac16Address address);
      // Get link to a specific address.
      Ptr<BleLink> GetLink (Mac16Address address);
      Ptr<BleLinkManager> GetLinkManager (Mac16Address address);

      uint32_t CountLinks ();//返回链路数量

      void TryAgain();//尝试重新处理

      /*
       * Checks if there is a new item in the netdevice queue qnd
       * forwards this item to the right LinkManager's queue.
       */
      //从网络设备队列中获取数据包并转发到合适的链路管理器队列
      void HandlePacket ();

      /*
       * The link manager that has control over the phy device at the moment
       * this is necessary so we could reply using the right parameters / 
       * window etc.
       */
      //设置和获取当前活跃的链路管理器
      void SetActiveLinkManager(Ptr<BleLinkManager> lm);
      Ptr<BleLinkManager> GetActiveLinkManager();

    private:
      Ptr<BleNetDevice> m_netDevice;
      std::list<Ptr<BleLinkManager>> m_linkManagers; //存储所有链路管理器

      // The LinkManager that has control over the device
      // at this moment
      Ptr<BleLinkManager> m_activeLinkManager;//当前控制物理层的链路管理器
 };

}

#endif /* BLE_BB_MANAGER_H */

