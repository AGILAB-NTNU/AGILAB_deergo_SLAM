// SPDX-License-Identifier: Apache-2.0
#include "deergo_nav2_plugins/queue_layers.hpp"

#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "message_filters/subscriber.h"
#include "pluginlib/class_list_macros.hpp"
#include "rmw/qos_profiles.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/message_filter.h"

namespace deergo_nav2_plugins
{
namespace
{
using Node = rclcpp_lifecycle::LifecycleNode;
using Subscriber = message_filters::SubscriberBase<Node>;

template<typename Message>
bool boundSource(
  const std::shared_ptr<Subscriber> & source,
  const std::shared_ptr<tf2_ros::MessageFilterBase> & notifier,
  const Node::SharedPtr & node, uint32_t depth,
  const rclcpp::SubscriptionOptions & options)
{
  auto subscriber = std::dynamic_pointer_cast<message_filters::Subscriber<Message, Node>>(source);
  auto filter = std::dynamic_pointer_cast<tf2_ros::MessageFilter<Message>>(notifier);
  if (!subscriber || !filter) {
    return false;
  }
  filter->setQueueSize(depth);
  auto qos = rmw_qos_profile_sensor_data;
  qos.depth = depth;
  const auto topic = subscriber->getTopic();
  subscriber->subscribe(node, topic, qos, options);
  // Original lifecycle activate() resubscribes with this saved QoS and callback group.
  subscriber->unsubscribe();
  RCLCPP_INFO(node->get_logger(), "%s: DDS depth=%u, TF filter capacity=%u",
    topic.c_str(), depth, filter->getQueueSize());
  return true;
}

void boundQueues(
  const Node::SharedPtr & node, const std::string & name,
  const rclcpp::CallbackGroup::SharedPtr & group,
  const std::vector<std::shared_ptr<Subscriber>> & sources,
  const std::vector<std::shared_ptr<tf2_ros::MessageFilterBase>> & filters)
{
  if (!node || sources.size() != filters.size()) {
    throw std::runtime_error("Cannot configure costmap receiver queues");
  }
  const auto depth = node->get_parameter(name + ".scan_queue_size").as_int();
  if (depth < 1 || depth > std::numeric_limits<uint32_t>::max()) {
    throw std::runtime_error(name + ".scan_queue_size must be between 1 and UINT32_MAX");
  }
  rclcpp::SubscriptionOptions options;
  options.callback_group = group;
  for (size_t i = 0; i < sources.size(); ++i) {
    if (!boundSource<sensor_msgs::msg::LaserScan>(sources[i], filters[i], node, depth, options) &&
      !boundSource<sensor_msgs::msg::PointCloud2>(sources[i], filters[i], node, depth, options))
    {
      throw std::runtime_error("Unsupported costmap observation subscriber");
    }
  }
}
}  // namespace

void QueueObstacleLayer::onInitialize()
{
  nav2_costmap_2d::ObstacleLayer::onInitialize();
  declareParameter("scan_queue_size", rclcpp::ParameterValue(5));
  boundQueues(node_.lock(), name_, callback_group_, observation_subscribers_, observation_notifiers_);
}

void QueueVoxelLayer::onInitialize()
{
  nav2_costmap_2d::VoxelLayer::onInitialize();
  declareParameter("scan_queue_size", rclcpp::ParameterValue(5));
  boundQueues(node_.lock(), name_, callback_group_, observation_subscribers_, observation_notifiers_);
}
}  // namespace deergo_nav2_plugins

PLUGINLIB_EXPORT_CLASS(deergo_nav2_plugins::QueueObstacleLayer, nav2_costmap_2d::Layer)
PLUGINLIB_EXPORT_CLASS(deergo_nav2_plugins::QueueVoxelLayer, nav2_costmap_2d::Layer)
