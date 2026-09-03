#pragma once
#include <behaviortree_cpp/loggers/abstract_logger.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <map>
#include <string>

// IDLE=0  RUNNING=1  SUCCESS=2  FAILURE=3  -> /bt/status/<node_name>
class RosStatusLogger : public BT::StatusChangeLogger
{
public:
  RosStatusLogger(const BT::Tree & tree, rclcpp::Node::SharedPtr node)
  : BT::StatusChangeLogger(tree.rootNode()), node_(node) {}

  void callback(BT::Duration, const BT::TreeNode & bt_node,
                BT::NodeStatus, BT::NodeStatus status) override
  {
    const std::string topic = "/bt/status/" + sanitize(bt_node.name());
    auto it = pubs_.find(topic);
    if (it == pubs_.end()) {
      it = pubs_.emplace(
        topic, node_->create_publisher<std_msgs::msg::Int32>(topic, 10)).first;
    }
    std_msgs::msg::Int32 msg;
    switch (status) {
      case BT::NodeStatus::IDLE:    msg.data = 0; break;
      case BT::NodeStatus::RUNNING: msg.data = 1; break;
      case BT::NodeStatus::SUCCESS: msg.data = 2; break;
      default:                      msg.data = 3; break;
    }
    it->second->publish(msg);
  }

  void flush() override {}

private:
  static std::string sanitize(std::string s)
  {
    for (auto & c : s) {
      if (!std::isalnum(static_cast<unsigned char>(c))) { c = '_'; }
    }
    return s;
  }

  rclcpp::Node::SharedPtr node_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> pubs_;
};
