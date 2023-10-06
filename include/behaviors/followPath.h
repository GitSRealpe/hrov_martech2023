#ifndef BEHAVIOR_TREE_BT_FOLLOWPATH_
#define BEHAVIOR_TREE_BT_FOLLOWPATH_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <std_msgs/String.h>
// #include <girona_utils/PathStatus.h>
#include <girona_utils/PathAction.h>
#include <ros/ros.h>
#include <thread>

// para registrar el nodo este
#include <behaviors/ros_utils2.h>

namespace BT
{
    class FollowPath : public BT::StatefulActionNode
    {
    public:
        FollowPath(ros::NodeHandle &nh, const std::string &name, const BT::NodeConfiguration &conf);
        // what is delete ?
        FollowPath() = delete;

        static PortsList providedPorts()
        {
            // return {InputPort<std::string>("msg_topic", "Topic of the msg")};
            return {OutputPort<bool>("success", "Trajectory succesfull?")};
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
        // virtual void callback(girona_utils::PathStatusConstPtr);
        // virtual void feedback(girona_utils::PathFeedbackConstPtr);
        virtual void feedback(girona_utils::PathActionFeedbackConstPtr);

        ros::NodeHandle nh_;
        // ros::Subscriber sub_;
        ros::Subscriber subFeed;
        std::string msg_topic_;
        // girona_utils::PathFeedbackConstPtr pathStatus_;
        girona_utils::PathFeedback pathStatus_;
    };
} // namespace BT

#endif // BEHAVIOR_TREE_BT_FollowPath_