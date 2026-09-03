#pragma once
#include <std_msgs/msg/string.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>
#include <string>
#include <vector>

// Shared ROS state, handed to every node through the blackboard.
struct RosContext
{
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr skill_pub;

  std::map<std::string, double> joint_pos;              // /joint_states
  bool estop{false};                                    // /estop
  bool part_detected{false};                            // /part_detected
  std::string skill_status{"idle"};                     // /skill_status
  std::map<std::string, rclcpp::Time> agent_last_seen;  // heartbeats

  geometry_msgs::msg::Point part_point;   // latest /part_point
  std::string part_frame{"panda_link0"};
  rclcpp::Time part_stamp;                // for staleness checks
};

// "a;b;c" -> {"a","b","c"}
inline std::vector<std::string> splitList(const std::string & in)
{
  std::vector<std::string> out;
  std::stringstream ss(in);
  std::string item;
  while (std::getline(ss, item, ';')) {
    item.erase(0, item.find_first_not_of(" \t"));
    item.erase(item.find_last_not_of(" \t") + 1);
    if (!item.empty()) { out.push_back(item); }
  }
  return out;
}

// ---------------------------------------------------------------------------
// MoveToJoints — publish a JointState command and stay RUNNING until every
// named joint is within tolerance of its target, or the timeout expires.
// Asynchronous on purpose: RUNNING is what makes the tree worth watching.
// ---------------------------------------------------------------------------
class MoveToJoints : public BT::StatefulActionNode
{
public:
  MoveToJoints(const std::string & name, const BT::NodeConfig & cfg)
  : BT::StatefulActionNode(name, cfg) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("joints",  "semicolon-separated joint names"),
      BT::InputPort<std::string>("targets", "semicolon-separated targets [rad]"),
      BT::InputPort<double>("tolerance", 0.02, "per-joint tolerance [rad]"),
      BT::InputPort<double>("timeout",   8.0,  "give up after [s]")
    };
  }

  BT::NodeStatus onStart() override
  {
    ctx_ = config().blackboard->get<std::shared_ptr<RosContext>>("ros_ctx");
    names_   = splitList(getInput<std::string>("joints").value());
    auto tgt = splitList(getInput<std::string>("targets").value());
    tol_     = getInput<double>("tolerance").value();
    timeout_ = getInput<double>("timeout").value();

    if (names_.size() != tgt.size() || names_.empty()) {
      RCLCPP_ERROR(ctx_->node->get_logger(),
                   "[%s] joints/targets length mismatch", name().c_str());
      return BT::NodeStatus::FAILURE;
    }
    targets_.clear();
    for (const auto & t : tgt) { targets_.push_back(std::stod(t)); }

    start_ = ctx_->node->now();
    publish(targets_);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    publish(targets_);                       // hold the command
    if (reached()) { return BT::NodeStatus::SUCCESS; }
    if ((ctx_->node->now() - start_).seconds() > timeout_) {
      RCLCPP_WARN(ctx_->node->get_logger(), "[%s] timed out", name().c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  // Halted mid-motion (e.g. a higher-priority branch took over):
  // freeze the arm where it currently is rather than letting it drift.
  void onHalted() override
  {
    std::vector<double> here;
    for (const auto & n : names_) {
      auto it = ctx_->joint_pos.find(n);
      here.push_back(it == ctx_->joint_pos.end() ? 0.0 : it->second);
    }
    publish(here);
  }

private:
  bool reached() const
  {
    for (size_t i = 0; i < names_.size(); ++i) {
      auto it = ctx_->joint_pos.find(names_[i]);
      if (it == ctx_->joint_pos.end()) { return false; }
      if (std::abs(it->second - targets_[i]) > tol_) { return false; }
    }
    return true;
  }

  void publish(const std::vector<double> & positions)
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = ctx_->node->now();
    msg.name = names_;
    msg.position = positions;
    ctx_->cmd_pub->publish(msg);
  }

  std::shared_ptr<RosContext> ctx_;
  std::vector<std::string> names_;
  std::vector<double> targets_;
  rclcpp::Time start_;
  double tol_{0.02}, timeout_{8.0};
};

// ---------------------------------------------------------------------------
// Conditions you can flip live from the terminal during the demo.
// ---------------------------------------------------------------------------
class EStopClear : public BT::ConditionNode
{
public:
  EStopClear(const std::string & n, const BT::NodeConfig & c)
  : BT::ConditionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override
  {
    auto ctx = config().blackboard->get<std::shared_ptr<RosContext>>("ros_ctx");
    return ctx->estop ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }
};


class JointStatesOk : public BT::ConditionNode
{
public:
  JointStatesOk(const std::string & n, const BT::NodeConfig & c)
  : BT::ConditionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override
  {
    auto ctx = config().blackboard->get<std::shared_ptr<RosContext>>("ros_ctx");
    return ctx->joint_pos.empty() ? BT::NodeStatus::FAILURE
                                  : BT::NodeStatus::SUCCESS;
  }
};

// ---------------------------------------------------------------------------
// AgentHealthy — liveness check on an agent's heartbeat.
//
// Deliberately fails when the agent has NEVER been seen, not only when it goes
// stale: "not yet available" and "died" are both unsafe states for the
// orchestrator, and both must route to the same fallback.
// ---------------------------------------------------------------------------
class AgentHealthy : public BT::ConditionNode
{
public:
  AgentHealthy(const std::string & n, const BT::NodeConfig & c)
  : BT::ConditionNode(n, c) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("agent", "agent name, e.g. perception"),
      BT::InputPort<double>("timeout", 2.0, "heartbeat staleness limit [s]")
    };
  }

  BT::NodeStatus tick() override
  {
    auto ctx = config().blackboard->get<std::shared_ptr<RosContext>>("ros_ctx");
    const auto name = getInput<std::string>("agent").value();
    const auto limit = getInput<double>("timeout").value();

    auto it = ctx->agent_last_seen.find(name);
    if (it == ctx->agent_last_seen.end()) {
      return BT::NodeStatus::FAILURE;              // never seen
    }
    const double age = (ctx->node->now() - it->second).seconds();
    return (age > limit) ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }
};

// ---------------------------------------------------------------------------
// WaitForPerception — block until the perception agent reports a part.
// ---------------------------------------------------------------------------
class WaitForPerception : public BT::StatefulActionNode
{
public:
  WaitForPerception(const std::string & n, const BT::NodeConfig & c)
  : BT::StatefulActionNode(n, c) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("timeout", 10.0, "give up after [s]") };
  }

  BT::NodeStatus onStart() override
  {
    ctx_ = config().blackboard->get<std::shared_ptr<RosContext>>("ros_ctx");
    timeout_ = getInput<double>("timeout").value();
    start_ = ctx_->node->now();
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    if (ctx_->part_detected) { return BT::NodeStatus::SUCCESS; }
    if ((ctx_->node->now() - start_).seconds() > timeout_) {
      RCLCPP_WARN(ctx_->node->get_logger(), "[%s] perception timeout",
                  name().c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  std::shared_ptr<RosContext> ctx_;
  rclcpp::Time start_;
  double timeout_{10.0};
};

// ---------------------------------------------------------------------------
// RequestSkill — delegate to the skill agent and await its verdict.
//
// Note onHalted(): when a higher-priority branch preempts this node, the agent
// is explicitly cancelled. An orchestrator that abandons a request without
// cancelling it leaves the agent running work nobody is waiting for — the most
// common orchestration bug, and worth a sentence in the thesis.
// ---------------------------------------------------------------------------
class RequestSkill : public BT::StatefulActionNode
{
public:
  RequestSkill(const std::string & n, const BT::NodeConfig & c)
  : BT::StatefulActionNode(n, c) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("skill", "skill name to request"),
      BT::InputPort<double>("timeout", 8.0, "give up after [s]")
    };
  }

  BT::NodeStatus onStart() override
  {
    ctx_ = config().blackboard->get<std::shared_ptr<RosContext>>("ros_ctx");
    skill_ = getInput<std::string>("skill").value();
    timeout_ = getInput<double>("timeout").value();

    ctx_->skill_status = "idle";        // clear any verdict from a previous run
    std_msgs::msg::String msg;
    msg.data = skill_;
    ctx_->skill_pub->publish(msg);
    start_ = ctx_->node->now();
    RCLCPP_INFO(ctx_->node->get_logger(), "[%s] requested '%s'",
                name().c_str(), skill_.c_str());
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (ctx_->skill_status == "success") { return BT::NodeStatus::SUCCESS; }
    if (ctx_->skill_status == "failure") {
      RCLCPP_ERROR(ctx_->node->get_logger(), "[%s] agent reported failure",
                   name().c_str());
      return BT::NodeStatus::FAILURE;
    }
    if ((ctx_->node->now() - start_).seconds() > timeout_) {
      cancel();
      RCLCPP_ERROR(ctx_->node->get_logger(), "[%s] agent timed out",
                   name().c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override { cancel(); }

private:
  void cancel()
  {
    std_msgs::msg::String msg;
    msg.data = "cancel";
    ctx_->skill_pub->publish(msg);
  }

  std::shared_ptr<RosContext> ctx_;
  std::string skill_;
  rclcpp::Time start_;
  double timeout_{8.0};
};

// ---------------------------------------------------------------------------
// PartDetected — is there a valid target right now?
//
// Also fails when the last position is stale, so that an agent which is alive
// but has stopped producing output is treated as "no valid target" rather than
// letting the arm chase a position from ten seconds ago.
// ---------------------------------------------------------------------------
class PartDetected : public BT::ConditionNode
{
public:
  PartDetected(const std::string & n, const BT::NodeConfig & c)
  : BT::ConditionNode(n, c) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("max_age", 1.0, "reject positions older than [s]") };
  }

  BT::NodeStatus tick() override
  {
    auto ctx = config().blackboard->get<std::shared_ptr<RosContext>>("ros_ctx");
    if (!ctx->part_detected) { return BT::NodeStatus::FAILURE; }
    if (ctx->part_stamp.nanoseconds() == 0) { return BT::NodeStatus::FAILURE; }

    const double age = (ctx->node->now() - ctx->part_stamp).seconds();
    const double max_age = getInput<double>("max_age").value();
    return (age > max_age) ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }
};

// ---------------------------------------------------------------------------
// TrackPointServo — turn the arm's base joint to face the target and hold a
// canned approach posture for the remaining joints.
//
// Deliberately not inverse kinematics. The bearing to the target is derived
// from perception; the rest of the posture is scripted. This is the honest
// minimum that demonstrates "the arm goes where perception says", and it is
// the fallback if the MoveIt integration on Day 4 does not land.
//
// Because it recomputes the bearing every tick, dragging the object in the
// simulator makes the arm follow it.
// ---------------------------------------------------------------------------
class TrackPointServo : public BT::StatefulActionNode
{
public:
  TrackPointServo(const std::string & n, const BT::NodeConfig & c)
  : BT::StatefulActionNode(n, c) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("posture_joints",
        "panda_joint2;panda_joint4;panda_joint6",
        "joints held at a fixed approach posture"),
      BT::InputPort<std::string>("posture_targets", "-0.3;-1.9;1.6",
        "posture values [rad]"),
      BT::InputPort<double>("tolerance", 0.05, "bearing tolerance [rad]"),
      BT::InputPort<double>("settle_time", 1.5,
        "hold within tolerance for [s] before SUCCESS"),
      BT::InputPort<double>("timeout", 20.0, "give up after [s]")
    };
  }

  BT::NodeStatus onStart() override
  {
    ctx_ = config().blackboard->get<std::shared_ptr<RosContext>>("ros_ctx");
    posture_names_ = splitList(getInput<std::string>("posture_joints").value());
    auto pv = splitList(getInput<std::string>("posture_targets").value());
    posture_.clear();
    for (const auto & v : pv) { posture_.push_back(std::stod(v)); }

    tol_     = getInput<double>("tolerance").value();
    settle_  = getInput<double>("settle_time").value();
    timeout_ = getInput<double>("timeout").value();

    start_ = ctx_->node->now();
    in_tol_since_ = rclcpp::Time(0, 0, ctx_->node->get_clock()->get_clock_type());
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    // Bearing to the target in the robot base frame.
    const double bearing = std::atan2(ctx_->part_point.y, ctx_->part_point.x);

    std::vector<std::string> names{"panda_joint1"};
    std::vector<double> pos{bearing};
    for (size_t i = 0; i < posture_names_.size(); ++i) {
      names.push_back(posture_names_[i]);
      pos.push_back(posture_[i]);
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = ctx_->node->now();
    msg.name = names;
    msg.position = pos;
    ctx_->cmd_pub->publish(msg);

    // Settled = base joint within tolerance of the commanded bearing,
    // continuously, for settle_time. Prevents SUCCESS while still swinging.
    auto it = ctx_->joint_pos.find("panda_joint1");
    const bool close = (it != ctx_->joint_pos.end()) &&
                       (std::abs(it->second - bearing) < tol_);

    const auto now = ctx_->node->now();
    if (close) {
      if (in_tol_since_.nanoseconds() == 0) { in_tol_since_ = now; }
      if ((now - in_tol_since_).seconds() >= settle_) {
        return BT::NodeStatus::SUCCESS;
      }
    } else {
      in_tol_since_ = rclcpp::Time(0, 0, ctx_->node->get_clock()->get_clock_type());
    }

    if ((now - start_).seconds() > timeout_) {
      RCLCPP_WARN(ctx_->node->get_logger(), "[%s] timed out", name().c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    // Freeze where we are, consistent with MoveToJoints.
    std::vector<std::string> names;
    std::vector<double> pos;
    for (const auto & kv : ctx_->joint_pos) {
      names.push_back(kv.first);
      pos.push_back(kv.second);
    }
    if (names.empty()) { return; }
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = ctx_->node->now();
    msg.name = names;
    msg.position = pos;
    ctx_->cmd_pub->publish(msg);
  }

private:
  std::shared_ptr<RosContext> ctx_;
  std::vector<std::string> posture_names_;
  std::vector<double> posture_;
  rclcpp::Time start_, in_tol_since_;
  double tol_{0.05}, settle_{1.5}, timeout_{20.0};
};
