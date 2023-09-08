#ifndef BEHAVIOR_TREE_BT_WAITFORMSG_
#define BEHAVIOR_TREE_BT_WAITFORMSG_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <thread>

// para registrar el nodo este
#include <behaviors/ros_utils2.h>

namespace BT
{
    class WaitForMsg : public BT::StatefulActionNode
    {
    public:
        WaitForMsg(ros::NodeHandle &nh, const std::string &name, const BT::NodeConfiguration &conf);
        // what is delete ?
        WaitForMsg() = delete;

        static PortsList providedPorts()
        {
            return {InputPort<std::string>("msg_topic", "Topic of the msg")};
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

        // why virtual, why not override like CheckAttachment.h
        // void callback(const std_msgs::String &msg) override;
        // virtual void callback(const std_msgs::String &msg);

        ros::NodeHandle nh_;
        // ros::Subscriber sub_;
        std::string msg_topic_;
    };
} // namespace BT

#endif // BEHAVIOR_TREE_BT_WaitForMsg_