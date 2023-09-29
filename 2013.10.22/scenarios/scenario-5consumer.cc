/* -*- Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil -*- */
/*
 * Copyright (c) 2012 University of California, Los Angeles
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
 * Author:
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/ndnSIM-module.h"
#include "ns3/position-allocator.h"

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

#include "ndn-v2v-net-device-face.h"
//#include "ns3/ndn-net-device-face.h"
#include "vehicular-2d-mobility-model.h"

#include "v2v-tracer.h"

using namespace ns3;
using namespace boost;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("Experiment");

Ptr<ndn::NetDeviceFace>
V2vNetDeviceFaceCallback (Ptr<Node> node, Ptr<ndn::L3Protocol> ndn, Ptr<NetDevice> device)
{
  NS_LOG_DEBUG ("Creating ndn::V2vNetDeviceFace on node " << node->GetId ());

  //abaixo, implementacao COM timers
  Ptr<ndn::NetDeviceFace> face = CreateObject<ndn::V2vNetDeviceFace> (node, device);
  //abaixo, implementacao SEM timers (???)
  //Ptr<ndn::NetDeviceFace> face = CreateObject<ndn::NetDeviceFace> (node, device);
  ndn->AddFace (face);
   NS_LOG_LOGIC ("Node " << node->GetId () << ": added NetDeviceFace as face #" << *face);

  return face;
}

int
main (int argc, char *argv[])
{
  // disable fragmentation
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue ("OfdmRate24Mbps"));

  // vanet hacks to CcnxL3Protocol
  Config::SetDefault ("ns3::ndn::V2vNetDeviceFace::MaxDelay", StringValue ("2ms"));
  Config::SetDefault ("ns3::ndn::V2vNetDeviceFace::MaxDelayLowPriority", StringValue ("5ms"));
  Config::SetDefault ("ns3::ndn::V2vNetDeviceFace::MaxDistance", StringValue ("150"));

  // !!! very important parameter !!!
  // Should keep PIT entry to prevent duplicate interests from re-propagating
  Config::SetDefault ("ns3::ndn::Pit::PitEntryPruningTimout", StringValue ("1s"));

  CommandLine cmd;

  uint32_t run = 2;
  cmd.AddValue ("run", "Run", run);

  string batches = "1s 1";
  cmd.AddValue ("batches", "Consumer interest batches", batches);

  double distance = 50;
  cmd.AddValue ("distance", "Distance between cars (default 10 meters)", distance);

//  double fixedDistance = -1;
//  cmd.AddValue ("fixedDistance", "Length of the highway. Number of cars will be set as (fixedDistance / distance + 1). If not set, there are 1000 cars", fixedDistance);

  cmd.Parse (argc,argv);

  uint32_t numberOfCars = 100;

  Config::SetGlobal ("RngRun", IntegerValue (run));

// Configure Link Layer /////////////////////////////////////////////////////////////////////////

  WifiHelper wifi = WifiHelper::Default ();
  wifi.SetStandard (WIFI_PHY_STANDARD_80211a);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue ("OfdmRate24Mbps"));

  YansWifiChannelHelper wifiChannel;// = YansWifiChannelHelper::Default ();
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::ThreeLogDistancePropagationLossModel");
  wifiChannel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");

  YansWifiPhyHelper wifiPhyHelper = YansWifiPhyHelper::Default ();
  wifiPhyHelper.SetChannel (wifiChannel.Create ());
  wifiPhyHelper.Set("TxPowerStart", DoubleValue(5));
  wifiPhyHelper.Set("TxPowerEnd", DoubleValue(5));

  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  wifiMac.SetType("ns3::AdhocWifiMac");


// Create nodes ///////////////////////////////////////////////////////////////////////////////

  NodeContainer relayNodes, consumerNodeA, producerNodeA,
  	  	  	  	  	  	  	consumerNodeB, producerNodeB,
  	  	  	  	  	  	  	consumerNodeC, producerNodeC,
  	  	  	  	  	  	  	consumerNodeD, producerNodeD,
  	  	  	  	  	  	  	consumerNodeE, producerNodeE;
  relayNodes.Create (100);
  consumerNodeA.Create(1);
  producerNodeA.Create(1);
  consumerNodeB.Create(1);
  producerNodeB.Create(1);
  consumerNodeC.Create(1);
  producerNodeC.Create(1);
  consumerNodeD.Create(1);
  producerNodeD.Create(1);
  consumerNodeE.Create(1);
  producerNodeE.Create(1);


// Configure Mobility //////////////////////////////////////////////////////////////////////////

  MobilityHelper mobility;

  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
          	  	  	  	  	  	 "GridWidth", UintegerValue (10),
                                 "MinX", DoubleValue (50.0),
                                 "MinY", DoubleValue (50.0),
                                 "DeltaX", DoubleValue (100.0),
                                 "DeltaY", DoubleValue (100.0),
                                 "LayoutType", StringValue ("RowFirst"));

  mobility.SetMobilityModel ("ns3::Vehicular2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (0.0, 1000.0, 0.0, 1000.0)),
                             "Distance", DoubleValue (100.0),
                             "UniqueDirection", DoubleValue (0.0),
                             "Direction", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=6.28]"),
                             "Mode", EnumValue (ns3::Vehicular2dMobilityModel::MODE_DISTANCE),
                             "Speed", StringValue("ns3::UniformRandomVariable[Min=10.0|Max=10.0]"));

  mobility.Install(relayNodes);

//-- Producer Nodes ---------------------------------------------------------------------------
  Ptr<ListPositionAllocator>  positionAllocPA = CreateObject<ListPositionAllocator>();
  positionAllocPA->Add(Vector(1000.0, 100.0, 0.0));
  mobility.SetPositionAllocator(positionAllocPA);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install(producerNodeA);

  Ptr<ListPositionAllocator>  positionAllocPB = CreateObject<ListPositionAllocator>();
  positionAllocPB->Add(Vector(1000.0, 300.0, 0.0));
  mobility.SetPositionAllocator(positionAllocPB);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install(producerNodeB);

  Ptr<ListPositionAllocator>  positionAllocPC = CreateObject<ListPositionAllocator>();
  positionAllocPC->Add(Vector(1000.0, 500.0, 0.0));
  mobility.SetPositionAllocator(positionAllocPC);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install(producerNodeC);

  Ptr<ListPositionAllocator>  positionAllocPD = CreateObject<ListPositionAllocator>();
  positionAllocPD->Add(Vector(1000.0, 700.0, 0.0));
  mobility.SetPositionAllocator(positionAllocPD);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install(producerNodeD);

  Ptr<ListPositionAllocator>  positionAllocPE = CreateObject<ListPositionAllocator>();
  positionAllocPE->Add(Vector(1000.0, 900.0, 0.0));
  mobility.SetPositionAllocator(positionAllocPE);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install(producerNodeE);

//-- Consumer Nodes ---------------------------------------------------------------------------

  Ptr<ListPositionAllocator>  positionAllocCA = CreateObject<ListPositionAllocator>();
  positionAllocCA->Add(Vector(0.0, 100.0, 0.0));
  mobility.SetPositionAllocator(positionAllocCA);
  mobility.SetMobilityModel ("ns3::Vehicular2dMobilityModel",
						 "Distance", DoubleValue (1000.0),
						 "UniqueDirection", DoubleValue (1.0),
						 "Direction", StringValue("ns3::UniformRandomVariable[Min=0.00|Max=0.00]"),
						 "Mode", EnumValue (ns3::RandomWalk2dMobilityModel::MODE_DISTANCE),
						 "Speed", StringValue("ns3::UniformRandomVariable[Min=10.0|Max=10.0]"));
  mobility.Install(consumerNodeA);

  Ptr<ListPositionAllocator>  positionAllocCB = CreateObject<ListPositionAllocator>();
  positionAllocCB->Add(Vector(0.0, 300.0, 0.0));
  mobility.SetPositionAllocator(positionAllocCB);
  mobility.SetMobilityModel ("ns3::Vehicular2dMobilityModel",
						 "Distance", DoubleValue (1000.0),
						 "UniqueDirection", DoubleValue (1.0),
						 "Direction", StringValue("ns3::UniformRandomVariable[Min=0.00|Max=0.00]"),
						 "Mode", EnumValue (ns3::RandomWalk2dMobilityModel::MODE_DISTANCE),
						 "Speed", StringValue("ns3::UniformRandomVariable[Min=10.0|Max=10.0]"));
  mobility.Install(consumerNodeB);

  Ptr<ListPositionAllocator>  positionAllocCC = CreateObject<ListPositionAllocator>();
  positionAllocCC->Add(Vector(0.0, 500.0, 0.0));
  mobility.SetPositionAllocator(positionAllocCC);
  mobility.SetMobilityModel ("ns3::Vehicular2dMobilityModel",
						 "Distance", DoubleValue (1000.0),
						 "UniqueDirection", DoubleValue (1.0),
						 "Direction", StringValue("ns3::UniformRandomVariable[Min=0.00|Max=0.00]"),
						 "Mode", EnumValue (ns3::RandomWalk2dMobilityModel::MODE_DISTANCE),
						 "Speed", StringValue("ns3::UniformRandomVariable[Min=10.0|Max=10.0]"));
  mobility.Install(consumerNodeC);

  Ptr<ListPositionAllocator>  positionAllocCD = CreateObject<ListPositionAllocator>();
  positionAllocCD->Add(Vector(0.0, 700.0, 0.0));
  mobility.SetPositionAllocator(positionAllocCD);
  mobility.SetMobilityModel ("ns3::Vehicular2dMobilityModel",
						 "Distance", DoubleValue (1000.0),
						 "UniqueDirection", DoubleValue (1.0),
						 "Direction", StringValue("ns3::UniformRandomVariable[Min=0.00|Max=0.00]"),
						 "Mode", EnumValue (ns3::RandomWalk2dMobilityModel::MODE_DISTANCE),
						 "Speed", StringValue("ns3::UniformRandomVariable[Min=10.0|Max=10.0]"));
  mobility.Install(consumerNodeD);

  Ptr<ListPositionAllocator>  positionAllocCE = CreateObject<ListPositionAllocator>();
  positionAllocCE->Add(Vector(0.0, 900.0, 0.0));
  mobility.SetPositionAllocator(positionAllocCE);
  mobility.SetMobilityModel ("ns3::Vehicular2dMobilityModel",
						 "Distance", DoubleValue (100.0),
						 "UniqueDirection", DoubleValue (1.0),
						 "Direction", StringValue("ns3::UniformRandomVariable[Min=0.00|Max=0.00]"),
						 "Mode", EnumValue (ns3::RandomWalk2dMobilityModel::MODE_DISTANCE),
						 "Speed", StringValue("ns3::UniformRandomVariable[Min=10.0|Max=10.0]"));
  mobility.Install(consumerNodeE);

//// 4. Install WiFi on Nodes ////////////////////////////////////////////////////////////////////

  NetDeviceContainer wifiNetDevices;
  wifiNetDevices.Add(wifi.Install (wifiPhyHelper, wifiMac, relayNodes));
  wifiNetDevices.Add(wifi.Install (wifiPhyHelper, wifiMac, consumerNodeA));
  wifiNetDevices.Add(wifi.Install (wifiPhyHelper, wifiMac, consumerNodeB));
  wifiNetDevices.Add(wifi.Install (wifiPhyHelper, wifiMac, consumerNodeC));
  wifiNetDevices.Add(wifi.Install (wifiPhyHelper, wifiMac, consumerNodeD));
  wifiNetDevices.Add(wifi.Install (wifiPhyHelper, wifiMac, consumerNodeE));
  wifiNetDevices.Add(wifi.Install (wifiPhyHelper, wifiMac, producerNodeA));
  wifiNetDevices.Add(wifi.Install (wifiPhyHelper, wifiMac, producerNodeB));
  wifiNetDevices.Add(wifi.Install (wifiPhyHelper, wifiMac, producerNodeC));
  wifiNetDevices.Add(wifi.Install (wifiPhyHelper, wifiMac, producerNodeD));
  wifiNetDevices.Add(wifi.Install (wifiPhyHelper, wifiMac, producerNodeE));

// 5. Install NDN stack /////////////////////////////////////////////////////////////////////////

  NS_LOG_INFO ("Installing NDN stack");
  ndn::StackHelper ndnHelper;
  ndnHelper.AddNetDeviceFaceCreateCallback (WifiNetDevice::GetTypeId (), MakeCallback (V2vNetDeviceFaceCallback));
  //ndnHelper.SetForwardingStrategy ("ns3::ndn::fw::Flooding");
  ndnHelper.SetForwardingStrategy ("ns3::ndn::fw::V2v");
  ndnHelper.SetContentStore ("ns3::ndn::cs::Lru",
                             "MaxSize", "10000");
  ndnHelper.SetDefaultRoutes(true);
  ndnHelper.Install(relayNodes);
  ndnHelper.Install(consumerNodeA);
  ndnHelper.Install(consumerNodeB);
  ndnHelper.Install(consumerNodeC);
  ndnHelper.Install(consumerNodeD);
  ndnHelper.Install(consumerNodeE);
  ndnHelper.Install(producerNodeA);
  ndnHelper.Install(producerNodeB);
  ndnHelper.Install(producerNodeC);
  ndnHelper.Install(producerNodeD);
  ndnHelper.Install(producerNodeE);


// 6. Set up applications ///////////////////////////////////////////////////////////////////////

  NS_LOG_INFO ("Installing Applications");

//- Consumer Aplications ------------------------------------------------------------------------
  ndn::AppHelper consumerHelperA ("ns3::ndn::ConsumerBatches");
  consumerHelperA.SetAttribute ("Batches", StringValue (batches));
  consumerHelperA.SetPrefix ("/aplicacaoGPS/1000/100/");
  consumerHelperA.Install (consumerNodeA.Get (0));

  ndn::AppHelper consumerHelperB ("ns3::ndn::ConsumerBatches");
  consumerHelperB.SetAttribute ("Batches", StringValue (batches));
  consumerHelperB.SetPrefix ("/aplicacaoGPS/1000/300/");
  consumerHelperB.Install (consumerNodeB.Get (0));

  ndn::AppHelper consumerHelperC ("ns3::ndn::ConsumerBatches");
  consumerHelperC.SetAttribute ("Batches", StringValue (batches));
  consumerHelperC.SetPrefix ("/aplicacaoGPS/1000/500/");
  consumerHelperC.Install (consumerNodeC.Get (0));

  ndn::AppHelper consumerHelperD ("ns3::ndn::ConsumerBatches");
  consumerHelperD.SetAttribute ("Batches", StringValue (batches));
  consumerHelperD.SetPrefix ("/aplicacaoGPS/1000/700/");
  consumerHelperD.Install (consumerNodeD.Get (0));

  ndn::AppHelper consumerHelperE ("ns3::ndn::ConsumerBatches");
  consumerHelperE.SetAttribute ("Batches", StringValue (batches));
  consumerHelperE.SetPrefix ("/aplicacaoGPS/1000/900/");
  consumerHelperE.Install (consumerNodeE.Get (0));

//- Producer Aplications ------------------------------------------------------------------------
  ndn::AppHelper producerHelperA ("ns3::ndn::Producer");
  producerHelperA.SetPrefix ("/aplicacaoGPS/1000/100/");
  producerHelperA.SetAttribute ("PayloadSize", StringValue("300"));
  producerHelperA.Install (producerNodeA.Get (0));

  ndn::AppHelper producerHelperB ("ns3::ndn::Producer");
  producerHelperB.SetPrefix ("/aplicacaoGPS/1000/300/");
  producerHelperB.SetAttribute ("PayloadSize", StringValue("300"));
  producerHelperB.Install (producerNodeB.Get (0));

  ndn::AppHelper producerHelperC ("ns3::ndn::Producer");
  producerHelperC.SetPrefix ("/aplicacaoGPS/1000/500/");
  producerHelperC.SetAttribute ("PayloadSize", StringValue("300"));
  producerHelperC.Install (producerNodeC.Get (0));

  ndn::AppHelper producerHelperD ("ns3::ndn::Producer");
  producerHelperD.SetPrefix ("/aplicacaoGPS/1000/700/");
  producerHelperD.SetAttribute ("PayloadSize", StringValue("300"));
  producerHelperD.Install (producerNodeD.Get (0));

  ndn::AppHelper producerHelperE ("ns3::ndn::Producer");
  producerHelperE.SetPrefix ("/aplicacaoGPS/1000/900/");
  producerHelperE.SetAttribute ("PayloadSize", StringValue("300"));
  producerHelperE.Install (producerNodeE.Get (0));


// 7. Configure Result Traces //////////////////////////////////////////////////////////////////
  boost::tuple< boost::shared_ptr<std::ostream>, std::list<boost::shared_ptr<ndn::V2vTracer> > >
    tracing = ndn::V2vTracer::InstallAll ("results/car-pusher.txt");

  Simulator::Stop (Seconds (150.0));

  NS_LOG_INFO ("Starting");

  Simulator::Run ();

  NS_LOG_INFO ("Done");

  Simulator::Destroy ();

  return 0;
}
