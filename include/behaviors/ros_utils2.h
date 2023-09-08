#ifndef BEHAVIOR_TREE_BT_ROSUTILS2_
#define BEHAVIOR_TREE_BT_ROSUTILS2_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <memory>

#include <sstream>

namespace BT
{
    /// Method to register the ROS Node into a factory.
    /// It gives you the opportunity to set the ros::NodeHandle.
    template <class DerivedT>
    static void RegisterROSNode(BT::BehaviorTreeFactory &factory, const std::string &registration_ID,
                                ros::NodeHandle &node_handle)
    {
        NodeBuilder builder = [&node_handle](const std::string &name, const NodeConfiguration &config)
        {
            return std::make_unique<DerivedT>(node_handle, name, config);
        };

        TreeNodeManifest manifest;
        manifest.type = getType<DerivedT>();
        manifest.ports = DerivedT::providedPorts();
        manifest.registration_ID = registration_ID;
        factory.registerBuilder(manifest, builder);
    }
} // namespace BT

#endif
