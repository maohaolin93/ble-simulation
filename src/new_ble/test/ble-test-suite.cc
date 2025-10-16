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
 *          The last testcase is based on the the battery example 
 *          from the lora ns-3 module written by Brecht Reynders.
 *          This module can be found here:
 *https://github.com/networkedsystems/lora-ns3/blob/master/model/lora-mac-header.h
 */


// Include a header file from your module to test.
#include <ns3/log.h>
#include <ns3/core-module.h>
#include <ns3/ble-module.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/packet.h>
#include <ns3/rng-seed-manager.h>
#include <ns3/spectrum-module.h>
#include <ns3/mobility-module.h>
#include <ns3/energy-module.h>
#include <ns3/spectrum-value.h>
#include <ns3/spectrum-analyzer.h>
#include <iostream>
#include <ns3/isotropic-antenna-model.h>
#include <ns3/trace-helper.h>
#include <ns3/drop-tail-queue.h>
#include <unordered_map>
#include "ns3/network-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"


#include <ns3/okumura-hata-propagation-loss-model.h>

// An essential include is test.h
#include "ns3/test.h"

// Do not put your test classes in namespace ns3.  You may find it useful
// to use the using directive to access the ns3 namespace directly
using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("ble-easy-test");

// This is an example TestCase.
class BleTestCase1 : public TestCase
{
public:
  BleTestCase1 ();
  virtual ~BleTestCase1 ();

private:
  virtual void DoRun (void);
};

// Add some help text to this case to describe what it is intended to test
BleTestCase1::BleTestCase1 ()
  : TestCase ("Ble test case (does nothing)")
{
}

// This destructor does nothing but we include it as a reminder that
// the test case should clean up after itself
BleTestCase1::~BleTestCase1 ()
{
}

//
// This method is the pure virtual method from class TestCase that every
// TestCase must implement
//
void
BleTestCase1::DoRun (void)
{
  // A wide variety of test macros are available in src/core/test.h
  NS_TEST_ASSERT_MSG_EQ (true, true, "true doesn't equal true for some reason");
  // Use this one for floating point comparisons
  NS_TEST_ASSERT_MSG_EQ_TOL (0.01, 0.01, 0.001, 
      "Numbers are not equal within tolerance");
  
  // Failing test case deiliberately:
  //NS_TEST_ASSERT_MSG_EQ (true, false, 
  //"true doesn't equal false for some reason");
}

// This my first real test case: case2.
class BleTestCase2 : public TestCase
{
public:
  BleTestCase2 ();
  virtual ~BleTestCase2 ();

private:
  virtual void DoRun (void);

  // Parameters go here
};

// Add some help text to this case to describe what it is intended to test
BleTestCase2::BleTestCase2()
  : TestCase ("Ble test case creates two Ble nodes")
{
  //set initial parameter values
  //NS_LOG_INFO ("This message rests inside the test case 2 constructor.");

}

// This destructor does nothing but we include it as a reminder that
// the test case should clean up after itself
BleTestCase2::~BleTestCase2 ()
{
}

//
// This method is the pure virtual method from class TestCase that every
// TestCase must implement
//
void
BleTestCase2::DoRun (void)
{
  /*****************
   * Configuration *
   *****************/
  uint32_t nNodes = 2;


  // Obvious, example asserts:
  // A wide variety of test macros are available in src/core/test.h
  NS_TEST_ASSERT_MSG_EQ (true, true, "true doesn't equal true for some reason");
  // Use this one for floating point comparisons
  NS_TEST_ASSERT_MSG_EQ_TOL (
      0.01, 0.01, 0.001, "Numbers are not equal within tolerance");

  // Enable logging
  BleHelper helper;
  //helper.EnableLogComponents();

  //LogComponentEnable ("Channel", LOG_LEVEL_INFO);

  NS_LOG (LOG_INFO, "Ble Test Case 2 (creation of 2 nodes) setup starts now");
  // Test setup:
  // Two BLE nodes will be created.
  // Next an artificial link will be created,
  //  The nodes will be added to the link and the link state will be checked.

  NodeContainer bleDeviceNodes;
  bleDeviceNodes.Create(nNodes);

  // For later tests, devices needs to be attached to the same channel
  // 
  // A propagation and delaymodel can be assigned to the channel
  
  Ptr<SingleModelSpectrumChannel> channel = 
    CreateObject<SingleModelSpectrumChannel> ();
  Ptr<LogDistancePropagationLossModel> propModel = 
    CreateObject<LogDistancePropagationLossModel> ();
  Ptr<ConstantSpeedPropagationDelayModel> delayModel = 
    CreateObject<ConstantSpeedPropagationDelayModel> ();
  channel->AddPropagationLossModel (propModel);
  channel->SetPropagationDelayModel (delayModel);
  helper.SetChannel (channel);

  NetDeviceContainer bleNetDevices;
  bleNetDevices = helper.Install (bleDeviceNodes);

  DynamicCast<BleNetDevice>(bleNetDevices.Get(0))->SetAddress (
      Mac16Address ("00:01"));
  DynamicCast<BleNetDevice>(bleNetDevices.Get(1))->SetAddress (
      Mac16Address ("00:02"));

  NS_LOG (LOG_INFO, "Setup done, creation of links. Empty link first");
  
  // Simulator code goes here:
  NS_TEST_ASSERT_MSG_EQ (DynamicCast<BleNetDevice>(bleNetDevices.Get(0))
      ->GetBBManager()->LinkExists(
        DynamicCast<BleNetDevice>(bleNetDevices.Get(1))->GetAddress16()), 
      false, "Dev0 has a link to the address of dev1, but that is not possible.");
  
  NS_LOG (LOG_INFO, "First fase done, now point to point link");
  // Second link: create a link between the 2 nodes using the CreateLink function
  Ptr<BleLink> link2 = DynamicCast<BleNetDevice>(
      bleNetDevices.Get(0))->GetBBManager()->CreateLink(
      DynamicCast<BleNetDevice>(bleNetDevices.Get(1))->GetBBManager(), 
      BleLinkManager::Role::MASTER_ROLE);
  
  NS_TEST_ASSERT_MSG_EQ (true, true, "true doesn't equal true for some reason");

  NS_TEST_ASSERT_MSG_EQ (DynamicCast<BleNetDevice>(bleNetDevices.Get(0))
      ->GetBBManager()->LinkExists(link2), true, 
      "Addition of link 2 on dev0 failed");
  NS_TEST_ASSERT_MSG_EQ (DynamicCast<BleNetDevice>(
        bleNetDevices.Get(1))->GetBBManager()->LinkExists(link2), 
      true, "Addition of link 2 on dev1 failed");
  NS_TEST_ASSERT_MSG_EQ (link2->GetLinkType(), 
      BleLink::LinkType::POINT_TO_POINT, "Link 2 is not a POINT_TO_POINT link");
  
  NS_TEST_ASSERT_MSG_EQ (
      DynamicCast<BleNetDevice>(bleNetDevices.Get(0))->GetBBManager()->LinkExists(
        DynamicCast<BleNetDevice>(bleNetDevices.Get(1))->GetAddress16()), 
      true, "Dev0 has no link to the address of dev1");

  // At this point, creating a link at startup works, let's try to create a link 
  // during simulation.
  Simulator::Schedule(MilliSeconds(10), &BleBBManager::CreateLink, 
      DynamicCast<BleNetDevice>(bleNetDevices.Get(0))->GetBBManager(), 
      DynamicCast<BleNetDevice>(bleNetDevices.Get(1))->GetBBManager(), 
      BleLinkManager::Role::SLAVE_ROLE);
  // Result: 
  //  dev0 should be involved in 2 links now
  //  dev1 should be involved in 2 links now


  // Go step further: send a packet.
  Ptr<Packet> p0 = Create<Packet> (50); // 50 bytes of dummy data
  uint16_t protocolNb = 1;
  Simulator::Schedule(MilliSeconds(30), &BleNetDevice::SendFrom, 
      DynamicCast<BleNetDevice>(bleNetDevices.Get(0)), p0, 
      DynamicCast<BleNetDevice>(bleNetDevices.Get(0))->GetAddress(),
      DynamicCast<BleNetDevice>(bleNetDevices.Get(1))->GetAddress(), protocolNb);

  NS_LOG (LOG_INFO, "Simulator will run now");
  
  Simulator::Stop(MilliSeconds(40));
  Simulator::Run ();

  NS_TEST_ASSERT_MSG_EQ (
      DynamicCast<BleNetDevice>(bleNetDevices.Get(0))->GetBBManager()->CountLinks(), 
      2, "Dev 0 is not involved in 2 links");
  NS_TEST_ASSERT_MSG_EQ (DynamicCast<BleNetDevice>(
        bleNetDevices.Get(1))->GetBBManager()->CountLinks(), 
      2, "Dev 1 is not involved in 2 links");
  
  Simulator::Destroy ();
}

class BleTestCase3 : public TestCase
{
public:
  BleTestCase3 ();
  virtual ~BleTestCase3 ();

private:
  virtual void DoRun (void);

  // Parameters go here
};

// Add some help text to this case to describe what it is intended to test
BleTestCase3::BleTestCase3()
  : TestCase (
      "Ble test case creates two Ble nodes and "
      "sends multiple packets between the nodes")
{
  //set initial parameter values
  //NS_LOG_INFO ("This message rests inside the test case 2 constructor.");

}

// This destructor does nothing but we include it as a reminder that
// the test case should clean up after itself
BleTestCase3::~BleTestCase3 ()
{
}

//
// This method is the pure virtual method from class TestCase that every
// TestCase must implement
//
void
BleTestCase3::DoRun (void)
{
  /*****************
   * Configuration *
   *****************/
  uint32_t nNodes = 2;


  // Obvious, example asserts:
  // A wide variety of test macros are available in src/core/test.h
  NS_TEST_ASSERT_MSG_EQ (true, true, "true doesn't equal true for some reason");
  // Use this one for floating point comparisons
  NS_TEST_ASSERT_MSG_EQ_TOL (0.01, 0.01, 
      0.001, "Numbers are not equal within tolerance");

  // Enable logging
  BleHelper helper;
  //helper.EnableLogComponents();

  //LogComponentEnable ("Channel", LOG_LEVEL_INFO);

  NS_LOG (LOG_INFO, "Ble Test Case 3 (creation of 2 nodes and "
      "send multiple packets) setup starts now");
  // Test setup:
  // Two BLE nodes will be created.
  // Next an artificial link will be created,
  //  The nodes will be added to the link and the link state will be checked.

  NodeContainer bleDeviceNodes;
  bleDeviceNodes.Create(nNodes);

  // For later tests, devices needs to be attached to the same channel
  // 
  // A propagation and delaymodel can be assigned to the channel
  
  Ptr<SingleModelSpectrumChannel> channel = 
    CreateObject<SingleModelSpectrumChannel> ();
  Ptr<LogDistancePropagationLossModel> propModel = 
    CreateObject<LogDistancePropagationLossModel> ();
  Ptr<ConstantSpeedPropagationDelayModel> delayModel = 
    CreateObject<ConstantSpeedPropagationDelayModel> ();
  channel->AddPropagationLossModel (propModel);
  channel->SetPropagationDelayModel (delayModel);
  helper.SetChannel (channel);

  NetDeviceContainer bleNetDevices;
  bleNetDevices = helper.Install (bleDeviceNodes);

  DynamicCast<BleNetDevice>(bleNetDevices.Get(0))
    ->SetAddress (Mac16Address ("00:01"));
  DynamicCast<BleNetDevice>(bleNetDevices.Get(1))
    ->SetAddress (Mac16Address ("00:02"));

  NS_LOG (LOG_INFO, "Setup done, creation of links. Empty link first");
  
  // Simulator code goes here:
  NS_TEST_ASSERT_MSG_EQ (DynamicCast<BleNetDevice>(bleNetDevices.Get(0))
      ->GetBBManager()->LinkExists(DynamicCast<BleNetDevice>(
          bleNetDevices.Get(1))->GetAddress16()), false, 
      "Dev0 has a link to the address of dev1, but that is not possible.");
  
  NS_LOG (LOG_INFO, "First fase done, now point to point link");
  // Second link: create a link between the 2 nodes using the CreateLink function
  Ptr<BleLink> link2 = DynamicCast<BleNetDevice>(
      bleNetDevices.Get(0))->GetBBManager()->CreateLink(
      DynamicCast<BleNetDevice>(bleNetDevices.Get(1))->GetBBManager(), 
      BleLinkManager::Role::MASTER_ROLE);
  
  NS_TEST_ASSERT_MSG_EQ (true, true, "true doesn't equal true for some reason");

  NS_TEST_ASSERT_MSG_EQ (DynamicCast<BleNetDevice>(bleNetDevices.Get(0))
      ->GetBBManager()->LinkExists(link2), true, 
      "Addition of link 2 on dev0 failed");
  NS_TEST_ASSERT_MSG_EQ (DynamicCast<BleNetDevice>(bleNetDevices.Get(1))
      ->GetBBManager()->LinkExists(link2), true, 
      "Addition of link 2 on dev1 failed");
  NS_TEST_ASSERT_MSG_EQ (link2->GetLinkType(), BleLink::LinkType::POINT_TO_POINT, 
      "Link 2 is not a POINT_TO_POINT link");
  
  NS_TEST_ASSERT_MSG_EQ (DynamicCast<BleNetDevice>(bleNetDevices.Get(0))
      ->GetBBManager()->LinkExists(
        DynamicCast<BleNetDevice>(bleNetDevices.Get(1))->GetAddress16()), 
      true, "Dev0 has no link to the address of dev1");

  // Go step further: send a packet.
  Ptr<Packet> p0 = Create<Packet> (50); // 50 bytes of dummy data
  uint16_t protocolNb = 1;
  Simulator::Schedule(MilliSeconds(10), &BleNetDevice::SendFrom, 
      DynamicCast<BleNetDevice>(bleNetDevices.Get(0)), p0, 
      DynamicCast<BleNetDevice>(bleNetDevices.Get(0))->GetAddress(),
      DynamicCast<BleNetDevice>(bleNetDevices.Get(1))->GetAddress(), protocolNb);
  
  // Step 2: 2 packets at the same time from the same node.
  Ptr<Packet> p1 = Create<Packet> (4);
  Ptr<Packet> p2 = Create<Packet> (4);
  Simulator::Schedule(MilliSeconds(30), &BleNetDevice::SendFrom, 
      DynamicCast<BleNetDevice>(bleNetDevices.Get(0)), p1, 
      DynamicCast<BleNetDevice>(bleNetDevices.Get(0))->GetAddress(),
      DynamicCast<BleNetDevice>(bleNetDevices.Get(1))->GetAddress(), protocolNb);
  Simulator::Schedule(MilliSeconds(30), &BleNetDevice::SendFrom, 
      DynamicCast<BleNetDevice>(bleNetDevices.Get(0)), p2, 
      DynamicCast<BleNetDevice>(bleNetDevices.Get(0))->GetAddress(),
      DynamicCast<BleNetDevice>(bleNetDevices.Get(1))->GetAddress(), protocolNb);

  // Step 3: 2 packets from different nodes.
  Ptr<Packet> p3 = Create<Packet> (4);
  Ptr<Packet> p4 = Create<Packet> (4);
  Simulator::Schedule(MilliSeconds(40), &BleNetDevice::SendFrom, 
      DynamicCast<BleNetDevice>(bleNetDevices.Get(0)), p3, 
      DynamicCast<BleNetDevice>(bleNetDevices.Get(0))->GetAddress(),
      DynamicCast<BleNetDevice>(bleNetDevices.Get(1))->GetAddress(), protocolNb);
  Simulator::Schedule(MilliSeconds(40), &BleNetDevice::SendFrom, 
      DynamicCast<BleNetDevice>(bleNetDevices.Get(1)), p4, 
      DynamicCast<BleNetDevice>(bleNetDevices.Get(1))->GetAddress(),
      DynamicCast<BleNetDevice>(bleNetDevices.Get(0))->GetAddress(), protocolNb);


  NS_LOG (LOG_INFO, "Simulator will run now");
  
  Simulator::Stop(MilliSeconds(50));
  Simulator::Run ();

  NS_TEST_ASSERT_MSG_EQ (DynamicCast<BleNetDevice>(bleNetDevices.Get(0))
      ->GetBBManager()->CountLinks(), 1, "Dev 0 is not involved in 1 links");
  NS_TEST_ASSERT_MSG_EQ (DynamicCast<BleNetDevice>(bleNetDevices.Get(1))
      ->GetBBManager()->CountLinks(), 1, "Dev 1 is not involved in 1 links");
  
  Simulator::Destroy ();
}

// Test case that includes mobility/propagation loss etc.
class BleTestCase4 : public TestCase
{
public:
  BleTestCase4 ();
  virtual ~BleTestCase4 ();

  void Transmitted (const Ptr<const Packet> packet);
  void Received (const Ptr<const Packet> packet);
  void ReceivedError (const Ptr<const Packet> packet);
  void ReceivedUnique (const Ptr<const Packet> packet);
  void TXWindowSkipped (const Ptr<const BleNetDevice> nd);
private:
  virtual void DoRun (void);
 /*****************
   * Configuration *
   *****************/

  // Based on Lora battery example

  int nbIterations = 1;
  double length = 30; //<! Square room with length as distance
  int pktsize = 20; //!< Size of packtets, in bytes
  int duration = 50; //<! Duration of the simulation in seconds
  int interval = 6; //!< Time between two packets
  int packetSendDuration = 37; 
  //<! Time during which new packets should be quied 
  bool verbose = false; // Enable logging
  bool nakagami = false; // enable nakagami path loss
  bool dynamic = false; // Wether the nodes are moving yes or no
  bool scheduled = true; // Schedule the TX windows instead of random parameters.
  uint32_t nNodes = 25; // Number of nodes
  uint32_t nbConnInterval = 3200; 
  // [MAX 3200]  nbConnInterval*1,25ms = size of connection interval. 
  // if nbConnInterval = 0, each link will get a random conn interval
  Ptr<OutputStreamWrapper> m_stream = 0; // Stream for waterfallcurve
  Ptr<UniformRandomVariable> randT = CreateObject<UniformRandomVariable> ();

  std::unordered_map<uint32_t,std::tuple<uint32_t,uint32_t,uint32_t,uint32_t,
    uint32_t,uint32_t,uint32_t> > errorMap;
  //Errormap: transmitted, received, received unique, received original, 
  //xlocation, ylocation
  std::unordered_map<uint32_t,Ptr<BleNetDevice> > deviceMap;

  /************************
   * End of configuration *
   ************************/



  // Parameters go here
};

// Add some help text to this case to describe what it is intended to test
BleTestCase4::BleTestCase4()
  : TestCase ("Ble test case that includes mobility and propagation")
{
  //set initial parameter values
  LogComponentDisableAll(LOG_LEVEL_ALL);
}

// This destructor does nothing but we include it as a reminder that
// the test case should clean up after itself
BleTestCase4::~BleTestCase4 ()
{
}

/// Save that the message has been transmitted
	void
BleTestCase4::Transmitted (const Ptr<const Packet> packet)
{
	Ptr<Packet> copy = packet->Copy();
	BleMacHeader header;
	copy->RemoveHeader(header);
    uint8_t buffer[2];
    header.GetSrcAddr().CopyTo(buffer);
    uint32_t addr = buffer[1];

	std::get<0>(errorMap[addr-1])++;
}

// save that a message has been received
	void
BleTestCase4::Received (const Ptr<const Packet> packet)
{
  //NS_LOG (LOG_DEBUG, "Packet received  " << packet);
	Ptr<Packet> copy = packet->Copy();
	BleMacHeader header;
	copy->RemoveHeader(header);
	uint8_t buffer[2];
    header.GetDestAddr().CopyTo(buffer);
    uint32_t addr = buffer[1];
    std::get<1>(errorMap[addr-1])++;
}

  void
BleTestCase4::ReceivedError (const Ptr<const Packet> packet)
{
  //NS_LOG (LOG_DEBUG, "Packet received  " << packet);
	Ptr<Packet> copy = packet->Copy();
	BleMacHeader header;
	copy->RemoveHeader(header);
	uint8_t buffer[2];
    header.GetDestAddr().CopyTo(buffer);
    uint32_t addr = buffer[1];
    std::get<3>(errorMap[addr-1])++;
}


// save that a message has been uniquely received
	void
BleTestCase4::ReceivedUnique (const Ptr<const Packet> packet)
{
	Ptr<Packet> copy = packet->Copy();
	BleMacHeader header;
	copy->RemoveHeader(header);
	uint8_t buffer[2];
    header.GetDestAddr().CopyTo(buffer);
    uint32_t addr = buffer[1];
	std::get<2>(errorMap[addr-1])++;
}

  void
BleTestCase4::TXWindowSkipped (const Ptr<const BleNetDevice> nd)
{
  //NS_LOG (LOG_DEBUG, "Packet received  " << packet);
	uint8_t buffer[2];
    nd->GetAddress16().CopyTo(buffer);
    uint32_t addr = buffer[1];
    std::get<4>(errorMap[addr-1])++;
}



//
// This method is the pure virtual method from class TestCase that every
// TestCase must implement
//
void
BleTestCase4::DoRun (void)
{
  AsciiTraceHelper ascii;
  m_stream = ascii.CreateFileStream ("data.csv");
  *m_stream->GetStream() << "#Scenario " << (int)nNodes 
    <<  " nodes on a square field with side " << length << " meter" 
    << " TX window scheduling enabled: " << scheduled 
    << ", connection interval = " << nbConnInterval*1.25 
    << " millisec, (0 = random) " << std::endl;
  // print Iteration, ID, transmitted, received, received unique, 
  // received at closest gateway, x coords, y coords, 
  // get average amount of retransmissions, get average time of transmissions, 
  // number of missed messages, amount of received messages.
  *m_stream->GetStream() << "Iteration, ID, transmitted, received, "
    "received unique, received error, TX Windows Skipped x coords, y coords " 
    <<std::endl;
  for (uint8_t iterationI=0;iterationI<nbIterations;iterationI++){
		std::cout << "Iteration: " << (int)iterationI << std::endl;
 
    randT->SetAttribute("Max", DoubleValue (600));
    // Enable logging
    BleHelper helper;
    if (verbose)
      helper.EnableLogComponents();

    //LogComponentEnable ("Channel", LOG_LEVEL_INFO);

    NS_LOG (LOG_INFO, "Ble Test Case 4 setup starts now");

    NodeContainer bleDeviceNodes;
    bleDeviceNodes.Create(nNodes);


    // Create mobility
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> nodePositionList = 
      CreateObject<ListPositionAllocator> ();
    for (uint32_t nodePositionsAssigned = 0; 
        nodePositionsAssigned < nNodes; nodePositionsAssigned++)
    {
      double x,y;
      x = randT->GetInteger(0,length);
      y = randT->GetInteger(0,length);
      NS_LOG (LOG_INFO, "x = " << x << " y = " << y);
      nodePositionList->Add (Vector (x,y,1.0));
    }
    mobility.SetPositionAllocator (nodePositionList);
    if (dynamic)
      mobility.SetMobilityModel ("ns3::RandomWalk2DMobilityModel");
    else
      mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install(bleDeviceNodes);

    // Create the nodes
    NetDeviceContainer bleNetDevices;
    bleNetDevices = helper.Install (bleDeviceNodes);

    // Set addresses
    NS_LOG (LOG_INFO, "Set addresses");
    for (uint32_t nodeI = 0; nodeI < nNodes; nodeI++)
    {
      std::stringstream stream;
      stream << std::hex << nodeI+1;
      std::string s( stream.str());
      while (s.size() < 4)
        s.insert(0,1,'0');
      s.insert(2,1,':');
      char const * buffer = s.c_str();
      DynamicCast<BleNetDevice>(
          bleNetDevices.Get(nodeI))->SetAddress (Mac16Address (buffer));
      NS_LOG (LOG_INFO, "address = " << DynamicCast<BleNetDevice>(
            bleNetDevices.Get(nodeI))->GetAddress ());
    }

    // Create links between the nodes
   helper.CreateAllLinks (bleNetDevices, scheduled, nbConnInterval);

    NS_LOG (LOG_INFO, " Generate data ");
    ApplicationContainer apps = helper.GenerateTraffic (
        randT, bleDeviceNodes, pktsize, 0, packetSendDuration, interval);

    // Hookup functions to measure performance

    for (uint32_t i=0; i< bleNetDevices.GetN(); i++)
    {
      uint8_t buffer[2];
      Mac16Address::ConvertFrom(
          bleNetDevices.Get(i)->GetAddress()).CopyTo(buffer);
      uint32_t addr = buffer[1];  
      deviceMap[addr ]=DynamicCast<BleNetDevice>(bleNetDevices.Get(i));
      uint32_t x  = bleNetDevices.Get(i)->GetNode()
        ->GetObject<MobilityModel>()->GetPosition ().x;
      uint32_t y  = bleNetDevices.Get(i)->GetNode()
        ->GetObject<MobilityModel>()->GetPosition ().y;
      errorMap[addr-1] = std::make_tuple (0,0,0,0,0,x,y);
      DynamicCast<BleNetDevice>(bleNetDevices.Get(i))
        ->TraceConnectWithoutContext ("MacTx",
            MakeCallback(&BleTestCase4::Transmitted, this));
      DynamicCast<BleNetDevice>(bleNetDevices.Get(i))
        ->TraceConnectWithoutContext ("MacRx",
            MakeCallback(&BleTestCase4::ReceivedUnique, this));
      DynamicCast<BleNetDevice>(bleNetDevices.Get(i))
        ->TraceConnectWithoutContext ("MacPromiscRx",
            MakeCallback(&BleTestCase4::Received, this));
      DynamicCast<BleNetDevice>(bleNetDevices.Get(i))
        ->TraceConnectWithoutContext ("MacRxError",
            MakeCallback(&BleTestCase4::ReceivedError, this));
      DynamicCast<BleNetDevice>(bleNetDevices.Get(i))
        ->TraceConnectWithoutContext ("TXWindowSkipped",
            MakeCallback(&BleTestCase4::TXWindowSkipped, this));
    }

    NS_LOG (LOG_INFO, "Simulator will run now");
    
    Simulator::Stop(Seconds (duration));
    Simulator::Run ();

    for (uint32_t i=0; i< bleNetDevices.GetN(); i++)
    {
            uint8_t buffer[2];
            Mac16Address::ConvertFrom(
                bleNetDevices.Get(i)->GetAddress()).CopyTo(buffer);
            uint32_t addr = buffer[1];  
            Ptr<BleNetDevice> netdevice =
              DynamicCast<BleNetDevice>(bleNetDevices.Get(i));
            NS_LOG (LOG_DEBUG, "nd = " << netdevice << " addr = " << addr);
			std::tuple<uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,
              uint32_t,uint32_t> tuple = errorMap[i];
			// print iteration, ID, transmitted, received, received unique, 
            // x coords, y coords.
			*m_stream->GetStream() << (int)iterationI << "," 
              << netdevice->GetAddress16() << "," << std::get<0>(tuple)
              << "," << std::get<1>(tuple) << "," <<   std::get<2>(tuple) 
              <<  "," << std::get<3>(tuple) << "," << std::get<4>(tuple) 
              << "," << std::get<5>(tuple) << "," << std::get<6>(tuple) 
              << std::endl;
		
    }
    errorMap.clear();
    Simulator::Destroy ();
  }
}



// The TestSuite class names the TestSuite, identifies what type of TestSuite,
// and enables the TestCases to be run.  Typically, only the constructor for
// this class must be defined
//
class BleTestSuite : public TestSuite
{
public:
  BleTestSuite ();
};

BleTestSuite::BleTestSuite ()
  : TestSuite ("ble1", UNIT)
{
  // TestDuration for TestCase can be QUICK, EXTENSIVE or TAKES_FOREVER
  //AddTestCase (new BleTestCase1, TestCase::QUICK);
  AddTestCase (new BleTestCase2, TestCase::QUICK);
  AddTestCase (new BleTestCase3, TestCase::QUICK);
  AddTestCase (new BleTestCase4, TestCase::QUICK);
}

// Do not forget to allocate an instance of this TestSuite
static BleTestSuite bleTestSuite;

