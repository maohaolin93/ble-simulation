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
 *   Adaptation 
 *   from the LoRa ns-3 module lora-net-device,
 *   written by Brecht Reynders.
 *   This module can be found here:
 *   github.com/networkedsystems/lora-ns3/blob/master/model/lora-mac-header.h
 */

#ifndef BLE_NET_DEVICE_H
#define BLE_NET_DEVICE_H

#include <cstring>
#include <ns3/node.h>
#include <ns3/address.h>
#include <ns3/net-device.h>
#include <ns3/callback.h>
#include <ns3/packet.h>
#include <ns3/traced-callback.h>
#include <ns3/nstime.h>
#include <ns3/ptr.h>
#include <ns3/mac16-address.h>
#include <ns3/generic-phy.h>
#include <ns3/random-variable-stream.h>
#include <ns3/event-id.h>
#include <ns3/backoff.h>

#include <ns3/constants.h>


namespace ns3 {

template <typename Item> class DropTailQueue;
class BlePhy;
class SpectrumChannel;
class Channel;
class QueueItem;
class BleChannel;
class SpectrumErrorModel;
class BleBBManager;
class BleLinkManager;
class BleLinkController;
class BleMacHeader;

//负责BLE协议的MAC层功能，并与物理层（BlePhy）、链路控制器（BleLinkController）和其他组件（如 BleBBManager 和 BleLinkManager）协作
/**
 * \ingroup ble
 * \brief The MAC implementation of the BLE protocol
 *
  */
class BleNetDevice : public NetDevice
{
public:

	/**
		* See also in higher classes
		*/
  static TypeId GetTypeId (void);

  BleNetDevice ();
  virtual ~BleNetDevice ();


  /**
   * set the queue which is going to be used by this device
   *
   * \param queue the wanted queue structure
   */
  //设置设备使用的发送队列
  virtual void SetQueue (Ptr<DropTailQueue<QueueItem>> queue);


  /**
   * Notify the MAC that the PHY has finished a previously started transmission
   *
	 * \param the transmitted packet
   */
  //通知 MAC 层传输结束
  void NotifyTransmissionEnd (Ptr<const Packet>);

  /**
   * Notify the MAC that the PHY has started a reception
   */
  //通知 MAC 层开始接收数据
  void NotifyReceptionStart ();


  /**
   * Notify the MAC that the PHY finished a reception with an error
   */
  //通知接收错误
  void NotifyReceptionEndError (Ptr<Packet> packet);

  /**
   * Notify the MAC that the PHY finished a reception successfully
   *
   * \param p the received packet
	 * \param rssi the received signal strength of the received packet
   */
  //通知接收成功
  void NotifyReceptionEndOk (Ptr<Packet> p);

  //通知传输窗口被跳过
  void NotifyTXWindowSkipped ();

  /**
   * This class doesn't talk directly with the underlying channel (a
   * dedicated PHY class is expected to do it), however the NetDevice
   * specification features a GetChannel() method. This method here
   * is therefore provide to allow BleNetDevice::GetChannel() to have
   * something meaningful to return.
   *
   * \param c the underlying channel
   */
  //设置关联的信道
  void SetChannel (Ptr<Channel> c);


  /**
   * set the callback used to instruct the lower layer to start a TX
   *
   * \param c The generic start of phy message
   */

  //设置物理层传输开始的回调
  void SetGenericPhyTxStartCallback (GenericPhyTxStartCallback c);

  /**
   * Set the Phy object which is attached to this device.
   * This object is needed so that we can set/get attributes and
   * connect to trace sources of the PHY from the net device.
   *
   * \param phy the Phy object attached to the device.  Note that the
   * API between the PHY and the above (this NetDevice which also
   * implements the MAC) is implemented entirely by
   * callbacks, so we do not require that the PHY inherits by any
   * specific class.
   */
  //设置关联的物理层对象
  void SetPhy (Ptr<BlePhy> phy);

  /**
		* Get the physical layer corresponding to this MAC layer
		* 
   * \return a reference to the PHY object embedded in this NetDevice.
   */
  //获取物理层对象
  Ptr<BlePhy> GetPhy () const;

  //mhl修改
  //void SetNotifyPeerHasMoreDataCallback(Callback<void, bool> cb);
  //Callback<void, bool> m_notifyPeerHasMoreData; // 通知对端更新 MD

  // inherited from NetDevice
  virtual void DoDispose (void);
  virtual void SetIfIndex (const uint32_t index);
  virtual uint32_t GetIfIndex (void) const;
  virtual Ptr<Channel> GetChannel (void) const;
  virtual bool SetMtu (const uint16_t mtu);
  virtual uint16_t GetMtu (void) const;
  virtual void SetAddress (Address address);
  virtual Address GetAddress (void) const;
  virtual bool IsLinkUp (void) const;
  virtual void AddLinkChangeCallback (Callback<void> callback);
  /**
   * Returns true if this interface supports a broadcast address, 
   * false otherwise: true in case of BLE
   */
  virtual bool IsBroadcast (void) const;
  
  virtual Address GetBroadcast (void) const;
  virtual bool IsMulticast (void) const;//BLE 是否支持组播或桥接。
  virtual bool IsPointToPoint (void) const;//BLE 是否支持点对点但非严格点对点设备,返回false
  virtual bool IsBridge (void) const;
  virtual bool Send (Ptr<Packet> packet, uint16_t protocolNumber);
  virtual bool Send (Ptr<Packet> packet, const Address& dest);
  virtual bool Send (Ptr<Packet> packet, const Address& dest, 
      uint16_t protocolNumber);
  virtual bool SendFrom (Ptr<Packet> packet, 
      const Address& source, const Address& dest,
                         uint16_t protocolNumber);
  virtual Ptr<Node> GetNode (void) const;
  virtual void SetNode (Ptr<Node> node);
  virtual bool NeedsArp (void) const;//BLE 是否使用 ARP
  virtual void SetReceiveCallback (NetDevice::ReceiveCallback cb);
  virtual Address GetMulticast (Ipv4Address addr) const;
  virtual Address GetMulticast (Ipv6Address addr) const;
  virtual void SetPromiscReceiveCallback (PromiscReceiveCallback cb);
  virtual bool SupportsSendFrom (void) const;
  
  Mac16Address GetAddress16 (void) const;

 
  Ptr<DropTailQueue<QueueItem>> GetQueue (void);
  Ptr<BleBBManager> GetBBManager();
  void SetBBManager(Ptr<BleBBManager> bbManager);

  Ptr<BleLinkManager> GetLinkManager();
  void SetLinkManager(Ptr<BleLinkManager> linkManager);

  Ptr<BleLinkController> GetLinkController();
  void SetLinkController(Ptr<BleLinkController> linkController);

protected:

  Ptr<DropTailQueue<QueueItem>> m_queue; //!< queue for packets to send 用于存储待发送的数据包，采用丢尾队列（DropTailQueue）策略
  Ptr<Node>    m_node; //!< node of this netdevice 表示该设备所属的 ns-3 节点
  Mac16Address m_address; //!< address of this device 表示设备的 MAC 地址，BLE 使用 16 位地址
  Ipv4Address m_ip_address; //!< address of this device 设备的 IPv4 地址

  uint32_t m_ifIndex; //!< indexnumber of the interface 接口索引
  uint32_t m_mtu; //!< maximal amount of bytes to transmit
  bool m_linkUp; //!< tells if the link is up 表示链路是否可用
  
  Ptr<BlePhy> m_phy; //!< physical layer of this device 关联的物理层对象

  //traceback functions
  /*m_macTxTrace：数据包发送。
    m_macTxDropTrace：数据包丢弃。
    m_macPromiscRxTrace：混杂模式接收。
    m_macRxTrace：正常接收。
    m_macRxBroadcastTrace：广播接收。
    m_macRxErrorTrace：接收错误。
    m_macTXWindowSkipped：传输窗口跳过。
  */
  TracedCallback<Ptr<const Packet> > m_macTxTrace;
  TracedCallback<Ptr<const Packet> > m_macTxDropTrace;
  TracedCallback<Ptr<const Packet> > m_macPromiscRxTrace;
  TracedCallback<Ptr<const Packet> > m_macRxTrace;
  TracedCallback<Ptr<const Packet>, 
    Ptr<const BleNetDevice> > m_macRxBroadcastTrace;
  TracedCallback<Ptr<const Packet> > m_macRxErrorTrace;
  TracedCallback<Ptr<const BleNetDevice> > m_macTXWindowSkipped;
  
	/**
   * List of callbacks to fire if the link changes state (up or down).
   */
  /*m_linkChangeCallbacks：链路状态变化。
    m_rxCallback, m_promiscRxCallback：接收和混杂接收回调*/
  TracedCallback<> m_linkChangeCallbacks;
  NetDevice::ReceiveCallback m_rxCallback;
  NetDevice::PromiscReceiveCallback m_promiscRxCallback;
	
// Associated objects
  Ptr<BleBBManager> m_bbManager; 
  //<! the BroadBand manager associated to this device.
  Ptr<BleLinkController> m_linkController; 
  //<! the link controller associated to this device.
  Ptr<BleLinkManager> m_linkManager; 
  //<! the link manager associated to this device.
	

};


} // namespace ns3

#endif /* BLE_NET_DEVICE_H */
