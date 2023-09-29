
// ndn-grid.cc
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ndnSIM-module.h"

using namespace ns3;

int
main (int argc, char *argv[])
{

  // disable fragmentation
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue ("OfdmRate24Mbps"));

  // !!! very important parameter !!!
  // Should keep PIT entry to prevent duplicate interests from re-propagating
  Config::SetDefault ("ns3::ndn::Pit::PitEntryPruningTimout", StringValue ("1s"));

  // Read optional command-line parameters (e.g., enable visualizer with ./waf --run=<> --visualize
  CommandLine cmd;
  cmd.Parse (argc, argv);

  // Creating Nodes
  NodeContainer nodes, consumerNodes, producerNodes;
  nodes.Create (100);
  consumerNodes.Create(1);
  producerNodes.Create(1);

  // Creating Link Layer
  WifiHelper wifi = WifiHelper::Default ();
  wifi.SetStandard (WIFI_PHY_STANDARD_80211a);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue ("OfdmRate24Mbps"));

  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::ThreeLogDistancePropagationLossModel");
  wifiChannel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");

  YansWifiPhyHelper wifiPhyHelper = YansWifiPhyHelper::Default ();
  wifiPhyHelper.SetChannel (wifiChannel.Create ());
  wifiPhyHelper.Set("TxPowerStart", DoubleValue(5));
  wifiPhyHelper.Set("TxPowerEnd", DoubleValue(5));

  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  wifiMac.SetType("ns3::AdhocWifiMac");

  // 2. Install Wifi
  NetDeviceContainer wifiNetDevicesA = wifi.Install (wifiPhyHelper, wifiMac, nodes);
  NetDeviceContainer wifiNetDevicesB = wifi.Install (wifiPhyHelper, wifiMac, producerNodes);
  NetDeviceContainer wifiNetDevicesC = wifi.Install (wifiPhyHelper, wifiMac, consumerNodes);

  // Install Mobility
  MobilityHelper mobility, mobilityProducer, mobilityConsumer;

  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
          	  	  	  	  	  	 "GridWidth", UintegerValue (10),
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (100.0),
                                 "DeltaY", DoubleValue (100.0),
                                 "LayoutType", StringValue ("RowFirst"));

  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (0.0, 1000.0, 0.0, 1000.0)),
                             "Distance", DoubleValue (100.0),
                             "Direction", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=6.28]"),
                             "Mode", EnumValue (ns3::RandomWalk2dMobilityModel::MODE_DISTANCE),
                             "Speed", StringValue("ns3::UniformRandomVariable[Min=10.0|Max=10.0]"));

  mobility.Install(nodes);

  Ptr<ListPositionAllocator>  positionAlloc = CreateObject<ListPositionAllocator>();
  positionAlloc->Add(Vector(1000.0, 500.0, 0.0));
  mobility.SetPositionAllocator(positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install(producerNodes);


  Ptr<ListPositionAllocator>  positionAllocB = CreateObject<ListPositionAllocator>();
  positionAllocB->Add(Vector(0.0, 500.0, 0.0));
  mobility.SetPositionAllocator(positionAllocB);
  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
						 "Distance", DoubleValue (1.0),
						 "Direction", StringValue("ns3::UniformRandomVariable[Min=0.00|Max=0.00]"),
						 "Mode", EnumValue (ns3::RandomWalk2dMobilityModel::MODE_DISTANCE),
						 "Speed", StringValue("ns3::UniformRandomVariable[Min=10.0|Max=10.0]"));
  mobility.Install(consumerNodes);



  // Install NDN stack on all nodes
  ndn::StackHelper ndnHelper;
  ndnHelper.SetForwardingStrategy ("ns3::ndn::fw::Flooding");
  ndnHelper.SetDefaultRoutes(true);
  ndnHelper.InstallAll ();

  // Installing global routing interface on all nodes
  //ndn::GlobalRoutingHelper ndnGlobalRoutingHelper;
  //ndnGlobalRoutingHelper.Install (nodes);


  // Install NDN applications
  std::string prefix = "/prefix";

  ndn::AppHelper consumerHelper ("ns3::ndn::ConsumerCbr");
  consumerHelper.SetPrefix (prefix);
  consumerHelper.SetAttribute ("Frequency", StringValue ("1")); // 1 interests a second
  consumerHelper.Install (consumerNodes);

  ndn::AppHelper producerHelper ("ns3::ndn::Producer");
  producerHelper.SetPrefix (prefix);
  producerHelper.SetAttribute ("PayloadSize", StringValue("1024"));
  producerHelper.Install (producerNodes);

  // Add /prefix origins to ndn::GlobalRouter
  //ndnGlobalRoutingHelper.AddOrigins (prefix, producerNodes);

  // Calculate and install FIBs
  //ndn::GlobalRoutingHelper::CalculateRoutes ();

  Simulator::Stop (Seconds (10.0));

  Simulator::Run ();

  Simulator::Destroy ();

  return 0;
}
