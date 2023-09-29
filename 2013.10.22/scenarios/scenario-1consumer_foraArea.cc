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

  Ptr<ndn::NetDeviceFace> face = CreateObject<ndn::V2vNetDeviceFace> (node, device);
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

  Config::SetDefault ("ns3::ndn::ForwardingStrategy::CacheUnsolicitedData", StringValue ("true")); // not necessary, but for clarity

  // vanet hacks to CcnxL3Protocol
  Config::SetDefault ("ns3::ndn::V2vNetDeviceFace::MaxDelay", StringValue ("2ms"));
  Config::SetDefault ("ns3::ndn::V2vNetDeviceFace::MaxDelayLowPriority", StringValue ("5ms"));
  Config::SetDefault ("ns3::ndn::V2vNetDeviceFace::MaxDistance", StringValue ("150"));

  // !!! very important parameter !!!
  // Should keep PIT entry to prevent duplicate interests from re-propagating
  Config::SetDefault ("ns3::ndn::Pit::PitEntryPruningTimout", StringValue ("1s"));

  CommandLine cmd;

  uint32_t run = 9;
  cmd.AddValue ("run", "Run", run);

  string batches = "1s 1";
  cmd.AddValue ("batches", "Consumer interest batches", batches);

//  double distance = 50;
//  cmd.AddValue ("distance", "Distance between cars (default 10 meters)", distance);

//  double fixedDistance = -1;
//  cmd.AddValue ("fixedDistance", "Length of the highway. Number of cars will be set as (fixedDistance / distance + 1). If not set, there are 1000 cars", fixedDistance);

  cmd.Parse (argc,argv);

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

  NodeContainer relayNodes, consumerNodes, producerNodes;
  relayNodes.Create (100);
  consumerNodes.Create(1);
  producerNodes.Create(1);


// Configure Mobility //////////////////////////////////////////////////////////////////////////

  MobilityHelper mobility;

  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
          	  	  	  	  	  	 "GridWidth", UintegerValue (10),
                                 "MinX", DoubleValue (2000.0), //onde comeca o grid
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (100.0), //espaco entre objetos
                                 "DeltaY", DoubleValue (100.0),
                                 "LayoutType", StringValue ("RowFirst"));

  mobility.SetMobilityModel ("ns3::Vehicular2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (0.0, 10000.0, 0.0, 10000.0)),
                             "Distance", DoubleValue (100.0),
                             "UniqueDirection", DoubleValue (0.0),
                             "Direction", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=6.28]"),
                             "Mode", EnumValue (ns3::Vehicular2dMobilityModel::MODE_DISTANCE),
                             "Speed", StringValue("ns3::UniformRandomVariable[Min=10.0|Max=10.0]"));

  mobility.Install(relayNodes);


  Ptr<ListPositionAllocator>  positionAlloc = CreateObject<ListPositionAllocator>();
  positionAlloc->Add(Vector(3000.0, 500.0, 0.0));
  mobility.SetPositionAllocator(positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install(producerNodes);


  Ptr<ListPositionAllocator>  positionAllocB = CreateObject<ListPositionAllocator>();
  positionAllocB->Add(Vector(2000.0, 500.0, 0.0));
  mobility.SetPositionAllocator(positionAllocB);
  mobility.SetMobilityModel ("ns3::Vehicular2dMobilityModel",
						 "Distance", DoubleValue (100.0),
						 "UniqueDirection", DoubleValue (1.0),
						 "Direction", StringValue("ns3::UniformRandomVariable[Min=3.1415|Max=3.1415]"),
						 "Mode", EnumValue (ns3::RandomWalk2dMobilityModel::MODE_DISTANCE),
						 "Speed", StringValue("ns3::UniformRandomVariable[Min=50.0|Max=50.0]"));
  mobility.Install(consumerNodes);


//// 4. Install WiFi on Nodes ////////////////////////////////////////////////////////////////////

  NetDeviceContainer wifiNetDevices;
  wifiNetDevices.Add(wifi.Install (wifiPhyHelper, wifiMac, relayNodes));
  wifiNetDevices.Add(wifi.Install (wifiPhyHelper, wifiMac, consumerNodes));
  wifiNetDevices.Add(wifi.Install (wifiPhyHelper, wifiMac, producerNodes));


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
  ndnHelper.Install(consumerNodes);
  ndnHelper.Install(producerNodes);


// 6. Set up applications ///////////////////////////////////////////////////////////////////////

  NS_LOG_INFO ("Installing Applications");
  //ndn::AppHelper consumerHelper ("ns3::ndn::ConsumerCbr");
  //consumerHelper.SetAttribute ("Frequency", StringValue ("1"));
  ndn::AppHelper consumerHelper ("ns3::ndn::ConsumerBatches");
  consumerHelper.SetAttribute ("Batches", StringValue (batches));
  consumerHelper.SetPrefix ("/aplicacaoGPS/3000/500/");
  consumerHelper.Install (consumerNodes.Get (0));

  ndn::AppHelper producerHelper ("ns3::ndn::Producer");
  producerHelper.SetPrefix ("/aplicacaoGPS/3000/500/");
  producerHelper.SetAttribute ("PayloadSize", StringValue("300"));
  producerHelper.Install (producerNodes.Get (0));

// 7. Configure Result Traces //////////////////////////////////////////////////////////////////
//  boost::tuple< boost::shared_ptr<std::ostream>, std::list<boost::shared_ptr<ndn::V2vTracer> > >
//    tracing = ndn::V2vTracer::InstallAll ("results/car-pusher.txt");

  Simulator::Stop (Seconds (150.0));

  NS_LOG_INFO ("Starting");

  Simulator::Run ();

  NS_LOG_INFO ("Done");

  Simulator::Destroy ();

  return 0;
}
