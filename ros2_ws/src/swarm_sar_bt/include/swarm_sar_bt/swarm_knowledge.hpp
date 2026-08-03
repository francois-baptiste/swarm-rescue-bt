// Shared, per-robot view of the swarm. One instance is created by
// mission_bt_node and stored on the BT blackboard under "swarm_knowledge";
// every custom BT leaf reads/writes through it instead of managing its own
// ROS interfaces. This is the only place messages are exchanged with the
// rest of the swarm - and it never arbitrates anything, it only records
// what has been heard and lets each node decide for itself what to do
// with that information. Header-only so it links cleanly into both the
// swarm_sar_bt_nodes plugin library and the mission_bt_node executable.
#ifndef SWARM_SAR_BT__SWARM_KNOWLEDGE_HPP_
#define SWARM_SAR_BT__SWARM_KNOWLEDGE_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "swarm_sar_bt/msg/victim_array.hpp"
#include "swarm_sar_bt/msg/victim_claim.hpp"
#include "swarm_sar_bt/msg/victim_rescue.hpp"

namespace swarm_sar_bt
{

struct KnownClaim
{
  int robot_id;
  rclcpp::Time stamp;
  geometry_msgs::msg::PoseStamped victim_pose;
};

class SwarmKnowledge
{
public:
  explicit SwarmKnowledge(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    claims_sub_ = node_->create_subscription<swarm_sar_bt::msg::VictimClaim>(
      "/swarm/claims", rclcpp::QoS(20).reliable(),
      [this](swarm_sar_bt::msg::VictimClaim::ConstSharedPtr msg) {onClaim(msg);});

    rescues_sub_ = node_->create_subscription<swarm_sar_bt::msg::VictimRescue>(
      "/swarm/rescues", rclcpp::QoS(20).reliable(),
      [this](swarm_sar_bt::msg::VictimRescue::ConstSharedPtr msg) {onRescue(msg);});

    victims_sub_ = node_->create_subscription<swarm_sar_bt::msg::VictimArray>(
      "/victims_ground_truth", rclcpp::SensorDataQoS(),
      [this](swarm_sar_bt::msg::VictimArray::ConstSharedPtr msg) {onVictims(msg);});

    claim_pub_ = node_->create_publisher<swarm_sar_bt::msg::VictimClaim>(
      "/swarm/claims", rclcpp::QoS(20).reliable());
    rescue_pub_ = node_->create_publisher<swarm_sar_bt::msg::VictimRescue>(
      "/swarm/rescues", rclcpp::QoS(20).reliable());
  }

  // Everything a robot currently perceives (ground-truth stand-in for a
  // real perception stack / Gazebo actor detector).
  std::vector<swarm_sar_bt::msg::Victim> latestVictims() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return victims_;
  }

  // True if some OTHER robot's claim on this victim is still fresh. A
  // claim older than claim_timeout_s with no matching rescue is treated
  // as abandoned (the claimant likely failed) and no longer blocks anyone.
  bool isActivelyClaimedByOther(int victim_id, int self_robot_id, double claim_timeout_s) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = known_claims_.find(victim_id);
    if (it == known_claims_.end()) {
      return false;
    }
    if (it->second.robot_id == self_robot_id) {
      return false;
    }
    const auto age = (node_->now() - it->second.stamp).seconds();
    return age < claim_timeout_s;
  }

  void publishClaim(int robot_id, int victim_id, const geometry_msgs::msg::PoseStamped & pose)
  {
    swarm_sar_bt::msg::VictimClaim msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = pose.header.frame_id;
    msg.robot_id = robot_id;
    msg.victim_id = victim_id;
    msg.victim_pose = pose;
    claim_pub_->publish(msg);

    // A robot always trusts its own claim immediately rather than waiting
    // for its own message to round-trip back through the subscription.
    std::lock_guard<std::mutex> lock(mutex_);
    known_claims_[victim_id] = KnownClaim{robot_id, node_->now(), pose};
  }

  void publishRescue(int robot_id, int victim_id)
  {
    swarm_sar_bt::msg::VictimRescue msg;
    msg.header.stamp = node_->now();
    msg.robot_id = robot_id;
    msg.victim_id = victim_id;
    rescue_pub_->publish(msg);

    std::lock_guard<std::mutex> lock(mutex_);
    known_claims_.erase(victim_id);
  }

private:
  void onClaim(swarm_sar_bt::msg::VictimClaim::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = known_claims_.find(msg->victim_id);
    // Keep the freshest claim only; on a same-instant tie the lower robot_id
    // wins so every robot resolves the race identically without talking to
    // an arbiter.
    if (it == known_claims_.end() ||
      rclcpp::Time(msg->header.stamp) >= it->second.stamp ||
      msg->robot_id < it->second.robot_id)
    {
      known_claims_[msg->victim_id] =
        KnownClaim{msg->robot_id, rclcpp::Time(msg->header.stamp), msg->victim_pose};
    }
  }

  void onRescue(swarm_sar_bt::msg::VictimRescue::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    known_claims_.erase(msg->victim_id);
  }

  void onVictims(swarm_sar_bt::msg::VictimArray::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    victims_ = msg->victims;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<swarm_sar_bt::msg::VictimClaim>::SharedPtr claims_sub_;
  rclcpp::Subscription<swarm_sar_bt::msg::VictimRescue>::SharedPtr rescues_sub_;
  rclcpp::Subscription<swarm_sar_bt::msg::VictimArray>::SharedPtr victims_sub_;
  rclcpp::Publisher<swarm_sar_bt::msg::VictimClaim>::SharedPtr claim_pub_;
  rclcpp::Publisher<swarm_sar_bt::msg::VictimRescue>::SharedPtr rescue_pub_;

  mutable std::mutex mutex_;
  std::unordered_map<int, KnownClaim> known_claims_;
  std::vector<swarm_sar_bt::msg::Victim> victims_;
};

}  // namespace swarm_sar_bt

#endif  // SWARM_SAR_BT__SWARM_KNOWLEDGE_HPP_
