// Publishes the scenario's victim list on /victims_ground_truth. In a real
// deployment this would be replaced by each robot's own perception stack
// (thermal/camera victim detector) or, in Gazebo, by actors + a sensor
// plugin; this node exists only so the simulated scenario has *some*
// source of victims for DetectVictimCondition to sense. It also listens to
// /swarm/rescues purely to keep the ground-truth "rescued" flag in sync,
// which is convenient for `ros2 topic echo` / rviz markers when watching a
// run - it plays no part in the swarm's own decision-making.
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "swarm_sar_bt/msg/victim_array.hpp"
#include "swarm_sar_bt/msg/victim_rescue.hpp"

using namespace std::chrono_literals;

class VictimGroundTruthNode : public rclcpp::Node
{
public:
  VictimGroundTruthNode()
  : rclcpp::Node("victim_ground_truth_node")
  {
    const auto ids = declare_parameter("victim_ids", std::vector<int64_t>{});
    const auto xs = declare_parameter("victim_x", std::vector<double>{});
    const auto ys = declare_parameter("victim_y", std::vector<double>{});
    const double publish_rate_hz = declare_parameter("publish_rate_hz", 2.0);

    if (ids.size() != xs.size() || ids.size() != ys.size()) {
      RCLCPP_FATAL(
        get_logger(),
        "victim_ids (%zu), victim_x (%zu) and victim_y (%zu) must have the same length",
        ids.size(), xs.size(), ys.size());
      throw std::runtime_error("mismatched victim_ids/victim_x/victim_y lengths");
    }

    for (size_t i = 0; i < ids.size(); ++i) {
      swarm_sar_bt::msg::Victim v;
      v.id = static_cast<int32_t>(ids[i]);
      v.pose.x = xs[i];
      v.pose.y = ys[i];
      v.rescued = false;
      v.rescued_by = -1;
      victims_.push_back(v);
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu victims for the scenario", victims_.size());

    pub_ = create_publisher<swarm_sar_bt::msg::VictimArray>(
      "/victims_ground_truth", rclcpp::SensorDataQoS());
    rescue_sub_ = create_subscription<swarm_sar_bt::msg::VictimRescue>(
      "/swarm/rescues", rclcpp::QoS(20).reliable(),
      std::bind(&VictimGroundTruthNode::onRescue, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz),
      std::bind(&VictimGroundTruthNode::publish, this));
  }

private:
  void onRescue(swarm_sar_bt::msg::VictimRescue::ConstSharedPtr msg)
  {
    for (auto & v : victims_) {
      if (v.id == msg->victim_id && !v.rescued) {
        v.rescued = true;
        v.rescued_by = msg->robot_id;
        RCLCPP_INFO(get_logger(), "victim %d rescued by robot %d", v.id, msg->robot_id);
      }
    }
  }

  void publish()
  {
    swarm_sar_bt::msg::VictimArray msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.victims = victims_;
    pub_->publish(msg);
  }

  std::vector<swarm_sar_bt::msg::Victim> victims_;
  rclcpp::Publisher<swarm_sar_bt::msg::VictimArray>::SharedPtr pub_;
  rclcpp::Subscription<swarm_sar_bt::msg::VictimRescue>::SharedPtr rescue_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VictimGroundTruthNode>());
  rclcpp::shutdown();
  return 0;
}
