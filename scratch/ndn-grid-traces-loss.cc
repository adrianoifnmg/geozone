#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ndnSIM-module.h"
#include <ns3/ndnSIM/utils/tracers/ndn-l3-aggregate-tracer.h>
#include <ns3/ndnSIM/utils/tracers/ndn-l3-rate-tracer.h>
#include <ns3/ndnSIM/utils/tracers/l2-rate-tracer.h>

using namespace ns3;

int
main (int argc, char *argv[])
{
  CommandLine cmd;
  cmd.Parse (argc, argv);

  AnnotatedTopologyReader topologyReader ("", 25);
  topologyReader.SetFileName ("src/ndnSIM/examples/topologies/topo-grid-3x3-loss.txt");
  topologyReader.Read ();

  // Install NDN stack on all nodes
  ndn::StackHelper ndnHelper;
  ndnHelper.SetForwardingStrategy ("ns3::ndn::fw::BestRoute");
  ndnHelper.InstallAll ();

  // Installing global routing interface on all nodes
  ndn::GlobalRoutingHelper ndnGlobalRoutingHelper;
  ndnGlobalRoutingHelper.InstallAll ();

  // Getting containers for the consumer/producer
  //Ptr<Node> producer = Names::Find<Node> ("Node8");
  NodeContainer producerNodes;
  producerNodes.Add (Names::Find<Node> ("Node2"));
  producerNodes.Add (Names::Find<Node> ("Node8"));

  NodeContainer consumerNodes;
  consumerNodes.Add (Names::Find<Node> ("Node0"));
  consumerNodes.Add (Names::Find<Node> ("Node6"));

  // Install NDN applications
  std::string prefix = "/prefix";

  ndn::AppHelper consumerHelper ("ns3::ndn::ConsumerCbr");
  consumerHelper.SetPrefix (prefix);
  consumerHelper.SetAttribute ("Frequency", StringValue ("100")); // 100 interests a second
  consumerHelper.Install (consumerNodes);

  ndn::AppHelper producerHelper ("ns3::ndn::Producer");
  producerHelper.SetPrefix (prefix);
  producerHelper.SetAttribute ("PayloadSize", StringValue("1024"));
  producerHelper.Install (producerNodes);

  // Add /prefix origins to ndn::GlobalRouter
  ndnGlobalRoutingHelper.AddOrigins (prefix, producerNodes);

  // Calculate and install FIBs
  ndn::GlobalRoutingHelper::CalculateRoutes ();

  Simulator::Stop (Seconds (10.0));

boost::tuple< boost::shared_ptr<std::ostream>, std::list<Ptr<ndn::L3AggregateTracer> > >
  aggTracers = ndn::L3AggregateTracer::InstallAll ("aggregate-trace.txt", Seconds (0.5));

boost::tuple< boost::shared_ptr<std::ostream>, std::list<Ptr<ndn::L3RateTracer> > >
  rateTracers = ndn::L3RateTracer::InstallAll ("rate-trace.txt", Seconds (0.5));

boost::tuple< boost::shared_ptr<std::ostream>, std::list<Ptr<L2RateTracer> > >
    l2tracers = L2RateTracer::InstallAll ("drop-trace.txt", Seconds (0.5));

  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
