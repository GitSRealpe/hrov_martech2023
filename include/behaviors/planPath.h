#ifndef BEHAVIOR_TREE_BT_PLANPATH_
#define BEHAVIOR_TREE_BT_PLANPATH_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <std_msgs/String.h>
#include <hrov_martech2023/BaseGoal.h>
#include <hrov_martech2023/PlanGoal.h>
#include <ros/ros.h>
#include <thread>

// para registrar el nodo este
#include <behaviors/ros_utils2.h>

namespace BT
{
    class PlanPath : public BT::StatefulActionNode
    {
    public:
        PlanPath(ros::NodeHandle &nh, const std::string &name, const BT::NodeConfiguration &conf);
        // what is delete ?
        PlanPath() = delete;

        static PortsList providedPorts()
        {
            // return {InputPort<std::string>("msg_topic", "Topic of the msg")};
            return {OutputPort<bool>("success", "Planning succesfull?")};
            // return {InputPort("msg_topic", "Topic of the msg")};
        }

    protected:
        /// method to be called at the beginning.
        /// If it returns RUNNING, this becomes an asychronous node.
        NodeStatus onStart() override;

        /// method invoked by a RUNNING action.
        NodeStatus onRunning() override;

        /// when the method halt() is called and the action is RUNNING, this method is invoked.
        /// This is a convenient place todo a cleanup, if needed.
        void onHalted() override;

        ros::NodeHandle nh_;
        // ros::Subscriber sub_;
        // std::string msg_topic_;
        hrov_martech2023::BaseGoalConstPtr goal_;
    };
} // namespace BT

#endif // BEHAVIOR_TREE_BT_PLANPATH_