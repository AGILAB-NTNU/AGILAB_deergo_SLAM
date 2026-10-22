// SPDX-License-Identifier: Apache-2.0
#ifndef DEERGO_NAV2_PLUGINS__QUEUE_LAYERS_HPP_
#define DEERGO_NAV2_PLUGINS__QUEUE_LAYERS_HPP_

#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "nav2_costmap_2d/voxel_layer.hpp"

namespace deergo_nav2_plugins
{
class QueueObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  void onInitialize() override;
};

class QueueVoxelLayer : public nav2_costmap_2d::VoxelLayer
{
public:
  void onInitialize() override;
};
}  // namespace deergo_nav2_plugins

#endif  // DEERGO_NAV2_PLUGINS__QUEUE_LAYERS_HPP_
