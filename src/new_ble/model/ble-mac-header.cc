/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
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
 * Author: kwong yin <kwong-sang.yin@boeing.com>
 *				Brecht Reynders
 *				Stijn Geysen
 *   Simplification from the LoRa ns-3 module lora-mac-header,
 *   written by Brecht Reynders.
 *   This module can be found here:
 *   github.com/networkedsystems/lora-ns3/blob/master/model/lora-mac-header.h
 */

#include "ble-mac-header.h"
#include <ns3/address-utils.h>
#include <ns3/log.h>

#include <list>
#include <tuple>
namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (BleMacHeader);
NS_LOG_COMPONENT_DEFINE ("BleMacHeader");


BleMacHeader::BleMacHeader ()
{
	NS_LOG_FUNCTION (this);
    SetNESN(0);
    SetSN(0);
    SetMD(0);
    SetLength(0);
    SetLLID(0);
    SetSrcAddr("00:00");
    SetDestAddr("00:00");
}

BleMacHeader::~BleMacHeader ()
{
	NS_LOG_FUNCTION (this);
}

/*
 * Getters And Setters
 */
Mac16Address
BleMacHeader::GetSrcAddr (void) const
{
	NS_LOG_FUNCTION (this << m_src_addr);
    return(m_src_addr);
}

Mac16Address
BleMacHeader::GetDestAddr (void) const
{
	NS_LOG_FUNCTION (this << m_dest_addr);
    return(m_dest_addr);
}

uint16_t
BleMacHeader::GetProtocol (void) const
{
  NS_LOG_FUNCTION (this << m_protocol);
  return (m_protocol);
}

void
BleMacHeader::SetLLID (uint8_t llid)
{
  m_llid = llid;
}

void
BleMacHeader::SetLength (uint8_t length)
{
  NS_LOG_FUNCTION (this);
  m_length = length;
}

uint8_t
BleMacHeader::GetLLID (void) const
{
  NS_LOG_FUNCTION (this);
  return m_llid;
}

uint8_t
BleMacHeader::GetLength (void) const
{
  NS_LOG_FUNCTION (this);
  return m_length;
}

void
BleMacHeader::SetNESN (bool nesn)
{
  NS_LOG_FUNCTION (this);
  m_nesn = nesn;
}

bool
BleMacHeader::GetNESN (void) const
{
  return m_nesn;
}

void
BleMacHeader::SetSN (bool sn)
{
  NS_LOG_FUNCTION (this);
  m_sn = sn;
}

bool
BleMacHeader::GetSN (void) const
{
  return m_sn;
}

void
BleMacHeader::SetMD (bool md)
{
  NS_LOG_FUNCTION (this);
  m_md = md;
}

bool
BleMacHeader::GetMD (void) const
{
  return m_md;
}

void
BleMacHeader::SetSrcAddr ( Mac16Address addr)
{
	NS_LOG_FUNCTION (this << addr);
  m_src_addr = addr;
}

void
BleMacHeader::SetDestAddr ( Mac16Address addr)
{
	NS_LOG_FUNCTION (this << addr);
  m_dest_addr = addr;
}

void
BleMacHeader::SetProtocol (uint16_t protocol)
{
  NS_LOG_FUNCTION (this << protocol);
  m_protocol = protocol;
}

std::string
BleMacHeader::GetName (void) const
{
	NS_LOG_FUNCTION (this);
  return "Ble MAC Header";
}

TypeId
BleMacHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::BleMacHeader")
    .SetParent<Header> ()
    .AddConstructor<BleMacHeader> ();
  return tid;
}


TypeId
BleMacHeader::GetInstanceTypeId (void) const
{
	NS_LOG_FUNCTION (this);
  return GetTypeId ();
}

void
BleMacHeader::Print (std::ostream &os) const
{
	NS_LOG_FUNCTION (this);

  os << "Protocol = " << m_protocol 
    << ", Source Addr = " << m_src_addr
    << ", Dest Addr = " << m_dest_addr;
}

uint32_t
BleMacHeader::GetSerializedSize (void) const
{
	NS_LOG_FUNCTION (this);

  return 6+2; 
}


void
BleMacHeader::Serialize (Buffer::Iterator start) const
{
	NS_LOG_FUNCTION (this);
  Buffer::Iterator i = start;
  WriteTo (i, m_src_addr);
  WriteTo (i, m_dest_addr);
  i.WriteU16 (GetProtocol());
  i.WriteU8 (GetLLID());
  i.WriteU8 (
      (this->GetLength()&0x5 << 3) |
      (this->GetNESN()&0x1 << 2 ) |  
      (this->GetSN()&0x1 << 1 ) |
      (this->GetMD()&0x1 ) );
}


uint32_t
BleMacHeader::Deserialize (Buffer::Iterator start)
{
	NS_LOG_FUNCTION (this);

  Buffer::Iterator i = start;
	
  ReadFrom (i, m_src_addr);
  ReadFrom (i, m_dest_addr);
  SetProtocol (i.ReadU16 ());
  SetLLID (i.ReadU8 ());
  uint8_t temp = i.ReadU8();
  SetLength (bool((temp >> 3) & 0x5));
  SetNESN (bool((temp >> 2) & 0x1));
  SetSN (bool((temp >> 1) & 0x1));
  SetMD (bool((temp) & 0x1));
  return i.GetDistanceFrom (start);
}

} //namespace ns3

