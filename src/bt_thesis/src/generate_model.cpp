#include "bt_nodes.hpp"

#include <behaviortree_cpp/xml_parsing.h>

#include <fstream>
#include <iostream>

int main(int argc, char ** argv)
{
  BT::BehaviorTreeFactory factory;

  // Existing nodes
  factory.registerNodeType<MoveToJoints>("MoveToJoints");
  factory.registerNodeType<EStopClear>("EStopClear");
  
  factory.registerNodeType<JointStatesOk>("JointStatesOk");

  // A.4 orchestration nodes
  factory.registerNodeType<AgentHealthy>("AgentHealthy");
  factory.registerNodeType<WaitForPerception>("WaitForPerception");
  factory.registerNodeType<RequestSkill>("RequestSkill");
  factory.registerNodeType<PartDetected>("PartDetected");
  factory.registerNodeType<TrackPointServo>("TrackPointServo");  

  const std::string xml =
    BT::writeTreeNodesModelXML(factory);

  const std::string out =
    (argc > 1) ? argv[1] : "bt_thesis_models.xml";

  std::ofstream(out) << xml;

  std::cout << "wrote " << out << std::endl;

  return 0;
}
