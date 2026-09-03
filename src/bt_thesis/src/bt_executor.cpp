#include "bt_nodes.hpp"
#include "ros_status_logger.hpp"

#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/loggers/bt_sqlite_logger.h>
#include <behaviortree_cpp/loggers/bt_minitrace_logger.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("bt_executor");

  node->declare_parameter<std::string>("tree_file", "");
  node->declare_parameter<int>("groot2_port", 1667);
  node->declare_parameter<double>("tick_rate", 20.0);

  const auto groot_port = node->get_parameter("groot2_port").as_int();
  const auto tick_rate  = node->get_parameter("tick_rate").as_double();

  std::string tree_file = node->get_parameter("tree_file").as_string();

  if (tree_file.empty()) {
    tree_file =
      ament_index_cpp::get_package_share_directory("bt_thesis")
      + "/trees/orchestration.xml";
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Loading tree: %s",
    tree_file.c_str());

  auto ctx = std::make_shared<RosContext>();
  auto point_sub = node->create_subscription<geometry_msgs::msg::PointStamped>(
    "/part_point", 10,
    [ctx](geometry_msgs::msg::PointStamped::SharedPtr m) {
      ctx->part_point = m->point;
      ctx->part_frame = m->header.frame_id;
      ctx->part_stamp = ctx->node->now();   // executor clock, as with heartbeats
    });  

  ctx->node = node;

  // ---------------------------------------------------------------------------
  // Robot joint command publisher
  // ---------------------------------------------------------------------------
  ctx->cmd_pub =
    node->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_command", 10);

  // ---------------------------------------------------------------------------
  // Skill request publisher
  // ---------------------------------------------------------------------------
  ctx->skill_pub =
    node->create_publisher<std_msgs::msg::String>(
      "/skill_request", 10);

  // ---------------------------------------------------------------------------
  // Joint states
  // ---------------------------------------------------------------------------
  auto js_sub =
    node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::SensorDataQoS(),
      [ctx](sensor_msgs::msg::JointState::SharedPtr m) {
        for (size_t i = 0;
             i < m->name.size() && i < m->position.size();
             ++i)
        {
          ctx->joint_pos[m->name[i]] = m->position[i];
        }
      });

  // ---------------------------------------------------------------------------
  // E-stop
  // ---------------------------------------------------------------------------
  auto estop_sub =
    node->create_subscription<std_msgs::msg::Bool>(
      "/estop",
      10,
      [ctx](std_msgs::msg::Bool::SharedPtr m) {
        ctx->estop = m->data;
      });

  // ---------------------------------------------------------------------------
  // Part detected
  // ---------------------------------------------------------------------------
  auto part_sub =
    node->create_subscription<std_msgs::msg::Bool>(
      "/part_detected",
      10,
      [ctx](std_msgs::msg::Bool::SharedPtr m) {
        ctx->part_detected = m->data;
      });

  // ---------------------------------------------------------------------------
  // Skill status
  // ---------------------------------------------------------------------------
  auto skill_sub =
    node->create_subscription<std_msgs::msg::String>(
      "/skill_status",
      10,
      [ctx](std_msgs::msg::String::SharedPtr m) {
        ctx->skill_status = m->data;
      });

  // ---------------------------------------------------------------------------
  // Perception agent heartbeat
  //
  // Receipt time is stamped using the executor's own clock so heartbeat
  // age remains consistent when use_sim_time is enabled.
  // ---------------------------------------------------------------------------
  auto hb_perception =
    node->create_subscription<std_msgs::msg::String>(
      "/agent/perception/heartbeat",
      10,
      [ctx](std_msgs::msg::String::SharedPtr) {
        ctx->agent_last_seen["perception"] = ctx->node->now();
      });

  // ---------------------------------------------------------------------------
  // Skill agent heartbeat
  // ---------------------------------------------------------------------------
  auto hb_skill =
    node->create_subscription<std_msgs::msg::String>(
      "/agent/skill/heartbeat",
      10,
      [ctx](std_msgs::msg::String::SharedPtr) {
        ctx->agent_last_seen["skill"] = ctx->node->now();
      });

  // ---------------------------------------------------------------------------
  // BehaviorTree factory
  // ---------------------------------------------------------------------------
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

  // ---------------------------------------------------------------------------
  // Blackboard
  // ---------------------------------------------------------------------------
  auto blackboard = BT::Blackboard::create();
  blackboard->set("ros_ctx", ctx);

  // ---------------------------------------------------------------------------
  // Load behavior tree
  // ---------------------------------------------------------------------------
  auto tree =
    factory.createTreeFromFile(tree_file, blackboard);

  // ---------------------------------------------------------------------------
  // Logging / Groot2
  // ---------------------------------------------------------------------------
  BT::Groot2Publisher groot_pub(tree, groot_port);

  BT::SqliteLogger sqlite(
    tree,
    "/tmp/bt_trace.db3");

  BT::MinitraceLogger mini(
    tree,
    "/tmp/bt_trace.json");

  BT::StdCoutLogger cout_log(tree);

  RosStatusLogger ros_log(tree, node);

  RCLCPP_INFO(
    node->get_logger(),
    "Groot2 monitor on ports %ld/%ld",
    groot_port,
    groot_port + 1);

  // ---------------------------------------------------------------------------
  // Main execution loop
  // ---------------------------------------------------------------------------
  rclcpp::Rate rate(tick_rate);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    tree.tickOnce();

    rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
