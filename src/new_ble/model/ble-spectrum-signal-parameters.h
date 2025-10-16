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

#ifndef BLE_SPECTRUM_SIGNAL_PARAMETERS_H
#define BLE_SPECTRUM_SIGNAL_PARAMETERS_H


#include <ns3/spectrum-signal-parameters.h>
#include <ns3/packet.h>
#include <ns3/event-id.h>
namespace ns3 {


/**
 * \ingroup BLE 
 *
 * Signal parameters for BLE.
 */
struct BleSpectrumSignalParameters : public SpectrumSignalParameters
{

  // inherited from SpectrumSignalParameters
  virtual Ptr<SpectrumSignalParameters> Copy (void);
  /**
   * default constructor
   */
  BleSpectrumSignalParameters (void);
  ~BleSpectrumSignalParameters (void);
  /**
   * copy constructor
   */
  BleSpectrumSignalParameters (const BleSpectrumSignalParameters& p);
  /**
   * The packet burst being transmitted with this signal
   */
  Ptr<Packet> packet;
  uint8_t m_channel;
  void SetChannel (uint8_t channel);
  uint8_t GetChannel (void);
  void SetBer (double ber);
  double GetBer (void);
  double m_ber;
  EventId m_event;
  EventId GetEvent (void);
  void SetEvent (EventId event);  

};

}  // namespace ns3


#endif /* BLE_SPECTRUM_SIGNAL_PARAMETERS_H */
