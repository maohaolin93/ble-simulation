/* -*- Mode:Ci++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 CTTC
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
 * Author: Nicola Baldo <nbaldo@cttc.es>
 * Author: Stijn Geysen <stijn.geysen@student.kuleuven.be>
 *          Based on the lora ns-3 module written by Brecht Reynders.
 *          This module can be found here:
 *https://github.com/networkedsystems/lora-ns3/blob/master/model/lora-mac-header.h
 */

#include "ble-phy.h"
#include "ble-spectrum-signal-parameters.h"
#include <ns3/ble-net-device.h>
#include <ns3/ble-bb-manager.h>
#include <ns3/object.h>
#include <ns3/spectrum-phy.h>
#include <ns3/net-device.h>
#include <ns3/mobility-model.h>
#include <ns3/spectrum-value.h>
#include <ns3/spectrum-channel.h>
#include <ns3/log.h>
#include <ns3/spectrum-model.h>
#include <ns3/antenna-model.h>
#include <ns3/simulator.h>
#include <ns3/event-id.h>
#include <ns3/random-variable-stream.h>
#include <ns3/double.h>


namespace ns3 {

	NS_LOG_COMPONENT_DEFINE ("BlePhy");

	NS_OBJECT_ENSURE_REGISTERED (BlePhy);


	TypeId
		BlePhy::GetTypeId (void)
		{
			static TypeId tid = TypeId ("ns3::BlePhy")
				.SetParent<Object> ()
				.AddConstructor<BlePhy> ()
				;
			return tid;
		}

	BlePhy::BlePhy ()
	{
		NS_LOG_FUNCTION (this);
        m_currentState = IDLE;
		m_k = 1.38e-23;
		m_temperature = 273;
		m_bandWidth = BANDWIDTH; // 100;
		m_antenna = 0;
		m_bitrate = 1000000 * 4; 
                // times 4 to allow for larger packets 
                // (necessary for ipv4 routing protocols)  
		m_mobility = 0;
		m_channelIndex = 20;
		m_receiver = false;
		m_channel = 0;
		m_netDevice = 0;
		m_random=CreateObject<UniformRandomVariable> ();
		m_channelSelector=CreateObject<UniformRandomVariable> ();
		m_random->SetAttribute ("Min",DoubleValue(0.0));
		m_random->SetAttribute ("Max",DoubleValue(1.0));
		m_equivalentNoiseTemperature = 293;
		m_power = 0.010; 
                // BLE specifications: min output power: 0.01 mW, max 10 mW
		m_errorModel =Create<BleErrorModel> (); 
		InitTxPowerSpectralDensity (m_channelIndex,m_power); //0.001);
		m_receivingPower = Create<SpectrumValue> (m_txPsd->GetSpectrumModel ());
	}

	BlePhy::~BlePhy ()
	{
		NS_LOG_FUNCTION (this);
		m_netDevice = 0;
		m_mobility = 0;
		m_channel = 0;
		m_antenna = 0;
		m_txPsd = 0;
	}

	void
		BlePhy::SetDevice (Ptr<NetDevice> d)
		{
			NS_LOG_FUNCTION (this);
			m_netDevice = d;
		}

	Ptr<NetDevice>
		BlePhy::GetDevice () const
		{
			NS_LOG_FUNCTION (this);
			return m_netDevice;
		}

    Ptr<BleBBManager>
      BlePhy::GetBBManager ()
      {
        NS_LOG_FUNCTION (this);
        Ptr<BleNetDevice> nd = DynamicCast<BleNetDevice> (this->GetDevice());
        return nd->GetBBManager ();
      }
	void
		BlePhy::SetMobility (Ptr<MobilityModel> m)
		{
			NS_LOG_FUNCTION (this);
			m_mobility = m;
		}

	Ptr<MobilityModel>
		BlePhy::GetMobility () const
		{
			return m_mobility;
		}

	void
		BlePhy::SetChannel (Ptr<SpectrumChannel> c)
		{
			NS_LOG_FUNCTION (this);
			c->AddRx(this);
			m_channel = c;
		}

    Ptr<SpectrumChannel>
		BlePhy::GetChannel ()
        {
          return m_channel;
        }

//功率分配（0.7737, 0.0787等）模拟GFSK调制的频谱形状，中心集中，边带衰减
	void
		BlePhy::InitTxPowerSpectralDensity (uint8_t channeloffset, double power)
		{
			NS_LOG_FUNCTION (this);
			Bands bands;
			//定义46个频带（NB_BANDS=40 + 6边带），覆盖BLE的2.4GHz频段（2402-2480MHz）
			for (int i= 0; i < NB_BANDS+6;i++){ //0 to 40
				BandInfo bi;
                bi.fc = 2402e6+(i-3)*BANDWIDTH;//每个频带宽2MHz（BANDWIDTH=2e6），中心频率计算为 2402e6 + (i-3)*BANDWIDTH（i=0到45）
				bi.fl = bi.fc-BANDWIDTH/2;//频带0-2和43-45为边带（处理泄漏），频道0-39对应i=3到42
				bi.fh = bi.fc+BANDWIDTH/2;
				bands.push_back (bi);
			}
			Ptr<SpectrumModel> sm = Create<SpectrumModel> (bands);
			m_txPsd = Create <SpectrumValue> (sm);
			double txPowerDensity = power;
//			channeloffset = 0;
            NS_ASSERT(channeloffset >= 0);
            NS_ASSERT(channeloffset <= 40);
			(*m_txPsd)[channeloffset + 3] = txPowerDensity*0.7737; 
			(*m_txPsd)[channeloffset + 2] = txPowerDensity*0.0787;
			(*m_txPsd)[channeloffset + 1] = txPowerDensity*0.0140;
			(*m_txPsd)[channeloffset + 0] = txPowerDensity*0.0059;
			(*m_txPsd)[channeloffset + 4] = txPowerDensity*0.0787;
			(*m_txPsd)[channeloffset + 5] = txPowerDensity*0.0140;
			(*m_txPsd)[channeloffset + 6] = txPowerDensity*0.0059;
		}

	void
		BlePhy::SetTxPowerSpectralDensity (uint8_t channeloffset, double power)
		{
			NS_LOG_FUNCTION(this << channeloffset << power);
			double txPowerDensity = power/m_bandWidth;
			//reset m_txPsd
			for (uint32_t i = 0; i<NB_BANDS+6; i++) //40
			{
				(*m_txPsd)[i]=0;
			}
            NS_ASSERT(channeloffset >= 0);
            NS_ASSERT(channeloffset <= 40);
			(*m_txPsd)[channeloffset + 3] = txPowerDensity*0.7737; 
			(*m_txPsd)[channeloffset + 2] = txPowerDensity*0.0787;
			(*m_txPsd)[channeloffset + 1] = txPowerDensity*0.0140;
			(*m_txPsd)[channeloffset + 0] = txPowerDensity*0.0059;
			(*m_txPsd)[channeloffset + 4] = txPowerDensity*0.0787;
			(*m_txPsd)[channeloffset + 5] = txPowerDensity*0.0140;
			(*m_txPsd)[channeloffset + 6] = txPowerDensity*0.0059;
		}

	Ptr<const SpectrumModel>
		BlePhy::GetRxSpectrumModel () const
		{
			NS_LOG_FUNCTION (this);
			return m_txPsd->GetSpectrumModel ();
		}

    void
      BlePhy::SetRxSpectrumModel (Ptr<const SpectrumModel> model)
      {
        m_txPsd = 0;
        m_txPsd = Create <SpectrumValue> (model);
        m_receivingPower = Create<SpectrumValue> (m_txPsd->GetSpectrumModel ());
      }

	void
		BlePhy::SetRxAntenna (Ptr<AntennaModel> a)
		{
			NS_LOG_FUNCTION (this);
			m_antenna = a;
		}

	Ptr<AntennaModel>
		BlePhy::GetRxAntenna () const
		{
			NS_LOG_FUNCTION (this);
			return m_antenna;
		}

    void
		BlePhy::SetTransmissionEndCallback (Callback<void, 
            Ptr<const Packet> > callback)
		{
			NS_LOG_FUNCTION (this);
			m_transmissionEnd = callback;
		}

	void
		BlePhy::SetReceptionStartCallback (Callback<void> callback)
		{
			NS_LOG_FUNCTION (this);
			m_ReceptionStart = callback;
		}

	void 
		BlePhy::SetReceptionErrorCallback (Callback<void> callback)
		{
			NS_LOG_FUNCTION (this);
			m_ReceptionError = callback;
		}

	void
		BlePhy::SetReceptionEndCallback (Callback<void,Ptr<Packet>, bool > callback)
		{
			NS_LOG_FUNCTION (this);
			m_ReceptionEnd = callback;
		}

	//启动BLE数据包的实际传输，通过频谱信道广播信号
	bool
		BlePhy::StartTx (Ptr<Packet> packet)
		{
			NS_LOG_FUNCTION (this->GetState());
			//NS_LOG_FUNCTION(this->GetBBManager()->GetActiveLinkManager()->GetState());
			if(this->GetState() == BlePhy::State::TX)
			{
              this->ChangeState(BlePhy::State::TX_BUSY);
				Ptr<BleSpectrumSignalParameters> txParams = 
                  Create<BleSpectrumSignalParameters> ();
				////计算传输时长：duration = Seconds((packet->GetSize()-1)*8 / m_bitrate)。
				//包大小（字节）减1（可能排除头或CRC），转换为比特（*8），除以比特率（m_bitrate=4Mbps）
				txParams->duration = Seconds((packet->GetSize()-1)*8/m_bitrate);
				txParams->packet = packet;
				txParams->txPhy = GetObject<SpectrumPhy> ();
                SetTxPowerSpectralDensity(m_channelIndex,m_power);
				txParams->psd = m_txPsd;
				txParams->txAntenna = m_antenna;
				txParams->SetChannel(m_channelIndex);
                NS_ASSERT(m_channel != 0);
				m_channel->StartTx (txParams);
				Simulator::Schedule(txParams->duration,
                    &BlePhy::EndTx,this,packet->Copy());
                NS_LOG_INFO ("EndTx event scheduled in: " << txParams->duration);
				return true;
			}
			return false;  
		}

	void 
	BlePhy::EndTx (Ptr<Packet> packet)
	{
      NS_LOG_FUNCTION (this->GetState());
	  //NS_LOG_FUNCTION(this->GetBBManager()->GetActiveLinkManager()->GetState());
      this->ChangeState(BlePhy::State::IDLE);

      NS_ASSERT (this->GetBBManager()->GetActiveLinkManager() != 0);
      this->GetBBManager()->GetActiveLinkManager()->HandleTXDone();
      NS_LOG_INFO ("Packet transmission done");
      m_transmissionEnd (packet);
	}

	//处理接收到的无线信号开始，更新接收功率、处理同信道干扰（Co-channel rejection, CCR）、初始化BER并调度接收结束。
	void
		BlePhy::StartRx (Ptr<SpectrumSignalParameters> params)
		{
          NS_LOG_FUNCTION (this->GetState());
			if (this->GetState() == BlePhy::State::RX_BUSY) //m_receiver)
			{
                NS_LOG_INFO ("Receiving starts now");
				//update BER
				UpdateBer();
				NS_LOG_FUNCTION (this);
				// do something with params
				
				Ptr<BleSpectrumSignalParameters> sfParams = 
                  DynamicCast<BleSpectrumSignalParameters> (params);
				// add power to received power
				Simulator::Schedule(params->duration,
                    &BlePhy::EndNoise,this,params->psd);
				*m_receivingPower += *params->psd;
				//m_ReceptionStart();
				if (sfParams != 0){
					uint8_t channel = sfParams->GetChannel();
					//Choose highest SNR if multiple
					if (m_params.size()<1){
						m_params.push_back(sfParams);
						sfParams->SetBer(0);
						//generate ending event
						sfParams->SetEvent(Simulator::Schedule(
                              sfParams->duration,&BlePhy::EndRx,this,sfParams));
					}
					else {
						//check if there is a collision
						sfParams->SetBer(0);
						sfParams->SetEvent(Simulator::Schedule(
                              sfParams->duration,&BlePhy::EndRx,this,sfParams));  
						for (auto &it : m_params)
						{
							if (it->GetChannel() == channel){
                                // Co-channel rejection:
                                // 6 dB ==> 4.0
                                // 11 dB ==> 12.6
                                double ccrejection = 12.6;

								//if there is a collision, 
                                //  No problem if 6dB power difference, 
                                //  but other is corrupted;
								if (Integral(*it->psd)*ccrejection 
                                    < Integral(*sfParams->psd)){
									it->SetBer(10);
								}
								else{
									//if 6dB lower power, there is no detection
									if (Integral(*it->psd) 
                                        > ccrejection*Integral(*sfParams->psd)){
										sfParams->SetBer(10);
									}
									else
									{
										// in all the other cases, 
                                        // the signal gets not detected, 
                                        // but the other packet is corrupted.
										sfParams->SetBer(10);
										it->SetBer(10);
									}
								}
							}
						}
						m_params.push_back(sfParams);
					}
				}
			}
            else
            {
              NS_LOG_INFO ("Cannot start "
                  "receiving without being in receive state");
            }
		}

	void
		BlePhy::EndNoise (Ptr<SpectrumValue> sv)
		{
			NS_LOG_FUNCTION(this);
			UpdateBer();
			*m_receivingPower -= *sv;
		}

	void 
		BlePhy::EndRx (Ptr<BleSpectrumSignalParameters> params)
		{
			NS_LOG_FUNCTION(this->GetState());
            NS_LOG_INFO ("Receiving stops now");
			//update BER
			UpdateBer();
			int temp = 0;
			for (auto &it : m_params)
			{
				if (it==params)
				{
					m_params.erase(m_params.begin()+temp);
				}
				temp++;
			}
			//decide packet error or not
			//if(m_random->GetValue()>=per)
			if(params->GetBer()<1)
			{
				//no packet error
				m_ReceptionEnd(params->packet, false);
			}
			else
			{
				//packet error
				m_ReceptionEnd(params->packet, true);
			}
            this->ChangeState(BlePhy::State::IDLE);
		}

   BlePhy::State
     BlePhy::GetState ()
     {
       return m_currentState;
     }

   void
     BlePhy::ChangeState (BlePhy::State state)
     {
       NS_LOG_FUNCTION (this);
       switch (m_currentState) {
          case IDLE : 
              NS_ASSERT(state != TX_BUSY);
              NS_ASSERT(state != RX_BUSY);
              m_currentState = state;
              break;
          case TX :
              NS_ASSERT (state != RX);
              NS_ASSERT (state != RX_BUSY);
              m_currentState = state;
              break;
          case RX :
              NS_ASSERT (state != TX);
              NS_ASSERT (state != TX_BUSY);
              m_currentState = state;
              break;
          case TX_BUSY :
              NS_ASSERT (state != RX);
              NS_ASSERT (state != RX_BUSY);
              NS_ASSERT (state != TX);
              m_currentState = state;
              break;
          case RX_BUSY :
              NS_ASSERT (state != TX);
              NS_ASSERT (state != TX_BUSY);
              NS_ASSERT (state != RX);
              m_currentState = state;
              break;   
          default :
              NS_LOG_ERROR (" Current PHY state is not part of the FSM "); 
              m_currentState = state;
              break;
       }
     }

   void
     BlePhy::SetBandwidth (uint32_t bandwidth)
     {
       NS_LOG_FUNCTION (this);
       m_bandWidth = bandwidth;
     }

   void
     BlePhy::SetChannelIndex (uint8_t channelIndex)
     {
        m_channelIndex = channelIndex;
     }

   bool
    BlePhy::PrepareTX (Ptr<Packet> packet)
    {
			NS_LOG_FUNCTION(this->GetState());
			//NS_LOG_FUNCTION(this->GetBBManager()->GetActiveLinkManager()->GetState());
      // Can only be the case if coming from IDLE or TX
      if (m_currentState == IDLE || m_currentState == TX)
      {
        this->ChangeState(TX);
        SetReceiverMode (false);
        // Schedule TX on event
        Simulator::Schedule(MicroSeconds(TX_PREP_TIME), 
            &BlePhy::StartTx, this, packet);
        // Manage battery?
        return true;
      }
      else
      {
        return false;
      }

    }


  bool
    BlePhy::PrepareRX ()
    {
			NS_LOG_FUNCTION(this->GetState());
			//NS_LOG_FUNCTION(this->GetBBManager()->GetActiveLinkManager()->GetState());
      // Can only be the case if current state is RX or IDLE
      if (m_currentState == RX || m_currentState == IDLE)
      {
        this->ChangeState(RX);
        Simulator::Schedule(MicroSeconds(RX_PREP_TIME), 
            &BlePhy::ChangeState, this, BlePhy::State::RX_BUSY);
        SetReceiverMode (true);
        // Reset look for preamble timer
        return true;
      }
      else
      {
        return false;
      }
    }


  bool
    BlePhy::SetIdle ()
    {
			NS_LOG_FUNCTION(this);
      // Delete possible scheduled events

      m_currentState = IDLE;
      SetReceiverMode (false);
      return true;
    }

  void 
		BlePhy::UpdateBer ()
		{
			double timeNow = Simulator::Now().GetSeconds(); 
			for (auto &i : m_params) {
              uint8_t paramsChannelIndex = i->GetChannel();
              if (m_channelIndex == paramsChannelIndex )
              {
				int m_bitErrors = 0;
				//calculate SNR
				Ptr<SpectrumValue> noise = m_receivingPower->Copy();
				*noise -= *i->psd;
				uint32_t channel = i->GetChannel();
				double snr = 
                  (*i->psd)[channel+3]/((*noise)[channel+3]+m_k*m_temperature);
				//getBER
				long double berEs = m_errorModel->GetBER (snr);
				int bits = (timeNow - m_lastCheck)*m_bitrate / 4; 
                // Divided by 4 to compensate for higher bitrate 
                //  (see other remark at bitrate instantiation)
				for ( int it = 0; it < bits; it++)
				{
					if(m_random->GetValue()<berEs)
					{
						//calculate numbers of biterrors	
						m_bitErrors += 1;
					}
				}
				i->SetBer(m_bitErrors+i->GetBer());
              }
			}
			m_lastCheck = timeNow;
		}

		void
			BlePhy::SetReceiverMode (bool receiver)
			{
				m_receiver = receiver;
			}

		} // namespace
