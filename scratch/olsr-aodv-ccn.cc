#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/ndnSIM-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("Adriano");

Ptr<ndn::NetDeviceFace>
V2vNetDeviceFaceCallback (Ptr<Node> node, Ptr<ndn::L3Protocol> ndn, Ptr<NetDevice> device)
{
  NS_LOG_DEBUG ("Creating ndn::V2vNetDeviceFace on node " << node->GetId ());

  Ptr<ndn::NetDeviceFace> face = CreateObject<ndn::NetDeviceFace> (node, device);
  ndn->AddFace (face);
   NS_LOG_LOGIC ("Node " << node->GetId () << ": added NetDeviceFace as face #" << *face);

  return face;
}


int 
main (int argc, char *argv[])
{
  bool verbose = true;
  uint32_t nCsma = 3;
  uint32_t nWifi = 15;

  CommandLine cmd;
  cmd.AddValue ("nCsma", "Number of \"extra\" CSMA nodes/devices", nCsma);
  cmd.AddValue ("nWifi", "Number of wifi STA devices", nWifi);
  cmd.AddValue ("verbose", "Tell echo applications to log if true", verbose);

  cmd.Parse (argc,argv);

  if (verbose)
    {
      LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
      LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);
    }

  NodeContainer wifiStaNodes;
  wifiStaNodes.Create (nWifi);

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  phy.SetChannel (channel.Create ());

  WifiHelper wifi = WifiHelper::Default ();
  //wifi.SetRemoteStationManager ("ns3::AarfWifiManager");
  wifi.SetStandard (WIFI_PHY_STANDARD_80211a);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue ("OfdmRate24Mbps"));

  NqosWifiMacHelper mac = NqosWifiMacHelper::Default ();
  mac.SetType("ns3::AdhocWifiMac");

  //Ssid ssid = Ssid ("ns-3-ssid");
  //mac.SetType ("ns3::StaWifiMac",
  //            "Ssid", SsidValue (ssid),
  //             "ActiveProbing", BooleanValue (false));

  NetDeviceContainer staDevices;
  staDevices = wifi.Install (phy, mac, wifiStaNodes);

//  mac.SetType ("ns3::ApWifiMac",
//               "Ssid", SsidValue (ssid));

  MobilityHelper mobility;

  //double distance = 10;

  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (5.0),
                                 "DeltaY", DoubleValue (10.0),
                                 "GridWidth", UintegerValue (3),
                                 "LayoutType", StringValue ("RowFirst"));

  //mobility.SetPositionAllocator ("ns3::HighwayPositionAllocator",
  //                               "Start", VectorValue(Vector(0.0, 0.0, 0.0)),
  //                              "Direction", DoubleValue(0.0),
  //                               "Length", DoubleValue(1000.0),
  //                               "MinGap", DoubleValue(distance),
  //                               "MaxGap", DoubleValue(distance));

  //mobility.SetMobilityModel("ns3::CustomConstantVelocityMobilityModel",
  //                          "ConstantVelocity", VectorValue(Vector(26.8224, 0, 0)));

  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (-150, 150, -1500, 1500)),
  	          "Speed", StringValue("ns3::UniformRandomVariable[Min=8.0|Max=15.0]"));

  mobility.Install (wifiStaNodes);

  // 3. Install NDN stack
  NS_LOG_INFO ("Installing NDN stack");
  ndn::StackHelper ndnHelper;
  ndnHelper.AddNetDeviceFaceCreateCallback (WifiNetDevice::GetTypeId (), MakeCallback (V2vNetDeviceFaceCallback));
  ndnHelper.SetForwardingStrategy ("ns3::ndn::fw::Flooding");
  ndnHelper.SetContentStore ("ns3::ndn::cs::Lru",
                             "MaxSize", "10000");
  ndnHelper.SetDefaultRoutes(true);
  ndnHelper.Install(wifiStaNodes);

  // 4. Set up applications
  NS_LOG_INFO ("Installing Applications");

  ndn::AppHelper consumerHelper ("ns3::ndn::ConsumerBatches");
  consumerHelper.SetPrefix ("/aplicacaoGPS/-2600/");
  consumerHelper.SetAttribute ("Batches", StringValue ("2s 1 5s 1"));

  ndn::AppHelper producerHelper ("ns3::ndn::Producer");
  producerHelper.SetPrefix ("/aplicacaoGPS/");
  producerHelper.SetAttribute ("PayloadSize", StringValue("300"));

  // install producer at one end
  producerHelper.Install (wifiStaNodes.Get (0));
  // install consumer at the other end
  consumerHelper.Install (wifiStaNodes.Get (50));

  Simulator::Stop (Seconds (30.0));

//  pointToPoint.EnablePcapAll ("third");
//  phy.EnablePcap ("third", apDevices.Get (0));
//  csma.EnablePcap ("third", csmaDevices.Get (0), true);

  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}
