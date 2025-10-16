/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2012 The Boeing Company
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
 * Author: Gary Pei <guangyu.pei@boeing.com>
 * Author: Stijn Geysen <stijn.geysen@student.kuleuven.be>
 *          Based on the lora ns-3 module written by Brecht Reynders.
 *          This module can be found here:
 *https://github.com/networkedsystems/lora-ns3/blob/master/model/lora-mac-header.h
 */
#include "ble-spectrum-signal-parameters.h"
#include <ns3/log.h>
#include <ns3/packet.h>
#include "ns3/antenna-model.h"
#include "ns3/spectrum-phy.h"
#include "ns3/spectrum-value.h"


namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("BleSpectrumSignalParameters");

BleSpectrumSignalParameters::BleSpectrumSignalParameters (void)
{
  NS_LOG_FUNCTION (this);
}

BleSpectrumSignalParameters::BleSpectrumSignalParameters (
    const BleSpectrumSignalParameters& p)
  : SpectrumSignalParameters (p)
{
  NS_LOG_FUNCTION (this << &p);
  packet = p.packet->Copy ();
  m_channel = p.m_channel;
}

BleSpectrumSignalParameters::~BleSpectrumSignalParameters (void)
{
  NS_LOG_FUNCTION (this);
  packet = 0;
	if (txAntenna !=0)
		txAntenna = 0;
	if (txPhy != 0)
		txPhy = 0;
	if (psd !=0)
		psd = 0;
}

Ptr<SpectrumSignalParameters>
BleSpectrumSignalParameters::Copy (void)
{
  NS_LOG_FUNCTION (this);
  return Create<BleSpectrumSignalParameters> (*this);
}

uint8_t
BleSpectrumSignalParameters::GetChannel ()
{
  return m_channel;
}

void
BleSpectrumSignalParameters::SetChannel (uint8_t channel)
{
  m_channel = channel;
}

void
BleSpectrumSignalParameters::SetBer (double ber)
{
  m_ber = ber;
}

double
BleSpectrumSignalParameters::GetBer (void)
{
  return m_ber;
}

void 
BleSpectrumSignalParameters::SetEvent (EventId event)
{
  m_event = event;
}

EventId
BleSpectrumSignalParameters::GetEvent (void)
{
  return m_event;
}
} // namespace ns3
