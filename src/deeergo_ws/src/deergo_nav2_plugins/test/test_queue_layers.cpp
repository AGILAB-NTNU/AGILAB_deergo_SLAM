// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "deergo_nav2_plugins/queue_layers.hpp"
#include "message_filters/subscriber.h"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"

namespace
{
template<typename Layer, typename Message = sensor_msgs::msg::LaserScan>
class Probe : public Layer
{
public:
  auto subscriber()
  {
    return std::dynamic_pointer_cast<
      message_filters::Subscriber<Message, rclcpp_lifecycle::LifecycleNode>>(
      this->observation_subscribers_.at(0));
  }
  auto filter()
  {
    return std::dynamic_pointer_cast<tf2_ros::MessageFilter<Message>>(
      this->observation_notifiers_.at(0));
  }
  auto observations()
  {
    std::vector<nav2_costmap_2d::Observation> result;
    this->observation_buffers_.at(0)->getObservations(result);
    return result;
  }
};

template<typename Layer, typename Message = sensor_msgs::msg::LaserScan>
struct Scenario
{
  std::shared_ptr<nav2_util::LifecycleNode> node;
  std::unique_ptr<tf2_ros::Buffer> tf;
  nav2_costmap_2d::LayeredCostmap map{"map", false, true};
  Probe<Layer, Message> layer;

  explicit Scenario(std::optional<int64_t> depth = std::nullopt,
    const std::string & type = "LaserScan")
  {
    std::vector<rclcpp::Parameter> parameters{
      {"test_layer.observation_sources", "scan"},
      {"test_layer.scan.topic", "queue_test_scan"},
      {"test_layer.scan.data_type", type},
      {"test_layer.scan.observation_persistence", 10.0},
      {"test_layer.scan.min_obstacle_height", -0.01},
      {"test_layer.scan.max_obstacle_height", 2.0}};
    if (depth) {
      parameters.emplace_back("test_layer.scan_queue_size", *depth);
    }
    auto options = rclcpp::NodeOptions().parameter_overrides(parameters);
    node = std::make_shared<nav2_util::LifecycleNode>("queue_test", "", options);
    node->declare_parameter("track_unknown_space", true);
    node->declare_parameter("transform_tolerance", 0.3);
    auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    tf = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    // Tests preload transforms before releasing scans to timed layer lookups.
    tf->setUsingDedicatedThread(true);
    tf->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
      node->get_node_base_interface(), node->get_node_timers_interface(), group));
    map.resizeMap(100, 100, 0.05, -2.5, -2.5);
    layer.initialize(&map, "test_layer", tf.get(), node, group);
  }
};

class QueueLayers : public ::testing::Test
{
public:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

template<typename Layer>
void checkLifecycle()
{
  Scenario<Layer> setup;
  ASSERT_TRUE(setup.layer.filter());
  EXPECT_EQ(setup.layer.filter()->getQueueSize(), 5u);
  EXPECT_FALSE(setup.layer.subscriber()->getSubscriber());
  setup.layer.activate();
  ASSERT_TRUE(setup.layer.subscriber()->getSubscriber());
  auto qos = setup.layer.subscriber()->getSubscriber()->get_actual_qos();
  EXPECT_EQ(qos.depth(), 5u);
  EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  setup.layer.deactivate();
  EXPECT_FALSE(setup.layer.subscriber()->getSubscriber());
  setup.layer.activate();
  EXPECT_EQ(setup.layer.subscriber()->getSubscriber()->get_actual_qos().depth(), 5u);
  EXPECT_EQ(setup.layer.filter()->getQueueSize(), 5u);
  setup.layer.deactivate();
}

TEST_F(QueueLayers, ObstacleKeepsBothDepthsAcrossLifecycle)
{
  checkLifecycle<deergo_nav2_plugins::QueueObstacleLayer>();
}

TEST_F(QueueLayers, VoxelKeepsBothDepthsAcrossLifecycle)
{
  checkLifecycle<deergo_nav2_plugins::QueueVoxelLayer>();
}

TEST_F(QueueLayers, UsesYamlOverrideForBothQueues)
{
  Scenario<deergo_nav2_plugins::QueueObstacleLayer> setup(3);
  EXPECT_EQ(setup.layer.filter()->getQueueSize(), 3u);
  setup.layer.activate();
  EXPECT_EQ(setup.layer.subscriber()->getSubscriber()->get_actual_qos().depth(), 3u);
  setup.layer.deactivate();
}

TEST_F(QueueLayers, SupportsPointCloudSources)
{
  Scenario<deergo_nav2_plugins::QueueVoxelLayer, sensor_msgs::msg::PointCloud2> setup(
    5, "PointCloud2");
  EXPECT_EQ(setup.layer.filter()->getQueueSize(), 5u);
  setup.layer.activate();
  EXPECT_EQ(setup.layer.subscriber()->getSubscriber()->get_actual_qos().depth(), 5u);
  setup.layer.deactivate();
}

TEST_F(QueueLayers, RejectsUnboundedOrInvalidDepth)
{
  using Setup = Scenario<deergo_nav2_plugins::QueueObstacleLayer>;
  EXPECT_THROW(Setup(0), std::runtime_error);
  EXPECT_THROW(Setup(-1), std::runtime_error);
  EXPECT_THROW(Setup(4294967296LL), std::runtime_error);
}

TEST_F(QueueLayers, MissingTfDropsOldestScanAndProcessesNewestFive)
{
  Scenario<deergo_nav2_plugins::QueueObstacleLayer> setup;
  const auto start = setup.node->now();
  std::vector<int64_t> expected;
  for (int i = 0; i < 6; ++i) {
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    scan->header.frame_id = "laser";
    auto stamp = start + rclcpp::Duration::from_seconds(0.025 * i);
    scan->header.stamp = stamp;
    scan->angle_min = 0.0f;
    scan->angle_max = 0.1f;
    scan->angle_increment = 0.1f;
    scan->range_min = 0.05f;
    scan->range_max = 5.0f;
    scan->ranges = {1.0f, 1.0f};
    setup.layer.filter()->add(scan);
    if (i > 0) {
      expected.push_back(stamp.nanoseconds());
    }
  }
  EXPECT_TRUE(setup.layer.observations().empty());
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "laser";
  transform.transform.rotation.w = 1.0;
  transform.header.stamp = start - rclcpp::Duration::from_seconds(0.1);
  ASSERT_TRUE(setup.tf->setTransform(transform, "test"));
  transform.header.stamp = start + rclcpp::Duration::from_seconds(0.25);
  ASSERT_TRUE(setup.tf->setTransform(transform, "test"));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(setup.node->get_node_base_interface());
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  while (setup.layer.observations().size() < 5 && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
  }
  std::vector<int64_t> observed;
  for (const auto & observation : setup.layer.observations()) {
    observed.push_back(rclcpp::Time(observation.cloud_->header.stamp).nanoseconds());
  }
  std::sort(observed.begin(), observed.end());
  EXPECT_EQ(observed, expected);
}

TEST_F(QueueLayers, PluginlibLoadsBothInstalledPlugins)
{
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
    "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  EXPECT_TRUE(loader.createSharedInstance("deergo_nav2_plugins::QueueObstacleLayer"));
  EXPECT_TRUE(loader.createSharedInstance("deergo_nav2_plugins::QueueVoxelLayer"));
}
}  // namespace
