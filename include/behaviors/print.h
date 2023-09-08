#ifndef BEHAVIOR_TREE_BT_Print_
#define BEHAVIOR_TREE_BT_Print_

#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>
#include <ros/service_client.h>

#include "ros_utils2.h"

namespace BT
{
    class Print : public BT::SyncActionNode
    {
    public:
        Print(ros::NodeHandle &nh, const std::string &name, const BT::NodeConfiguration &conf);

        Print() = delete;

        static PortsList providedPorts()
        {
            return {InputPort<std::string>("service_name", "name of the ROS service")};
        }

    protected:
        ros::NodeHandle &node_;

        BT::NodeStatus tick() override;
    };

} // namespace BT

#endif // BEHAVIOR_TREE_BT_Print_
