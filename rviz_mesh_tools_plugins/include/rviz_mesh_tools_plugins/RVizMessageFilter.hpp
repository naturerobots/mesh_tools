#ifndef RVIZ_MESH_TOOLS_PLUGINS_RVIZ_MESSAGE_FILTER_HPP
#define RVIZ_MESH_TOOLS_PLUGINS_RVIZ_MESSAGE_FILTER_HPP

#include <tf2_ros/message_filter.h>
#include <rviz_common/transformation/frame_transformer.hpp>

namespace tf2_ros
{

template<typename T>
using RVizMessageFilter = tf2_ros::MessageFilter<T, rviz_common::transformation::FrameTransformer>;

template<typename T>
using RVizMessageFilterPtr = std::shared_ptr<RVizMessageFilter<T> >;

} // namespace tf2_ros

#endif // RVIZ_MESH_TOOLS_PLUGINS_RVIZ_MESSAGE_FILTER_HPP