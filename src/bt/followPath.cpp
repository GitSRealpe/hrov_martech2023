#include <behaviors/followPath.h>
#include <std_srvs/Trigger.h>
// #include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
ros::ServiceClient moveClient;
std::shared_ptr<actionlib::SimpleActionClient<girona_utils::PathAction>> followClient;

namespace BT
{
    FollowPath::FollowPath(ros::NodeHandle &nh, const std::string &name, const BT::NodeConfiguration &conf)
        : BT::StatefulActionNode(name, conf), nh_(nh)
    {
        std::cout << "Instance of " << name << " created!" << std::endl; // Check when the object is built
        std::cout << "using rosnode: " << nh_.getNamespace() << "\n";
        // moveClient = nh.serviceClient<std_srvs::Trigger>("path_manager_server/startPath");
        // ros::Subscriber subFeed = nh.subscribe("path_manager_server/status", 10, &FollowPath::callback, this);
        followClient = std::make_shared<actionlib::SimpleActionClient<girona_utils::PathAction>>("path_manager_server");
    }

    void FollowPath::feedback(const girona_utils::PathFeedbackConstPtr feedMsg)
    {
        std::cout << feedMsg->status << "\n";
        pathStatus_ = feedMsg;
    }

    NodeStatus FollowPath::onStart()
    {
        // getInput<std::string>("pose_topic", msg_topic_).value();
        // getInput("msg_topic", msg_topic_);
        // std::cout << msg_topic_ << "\n";

        std::cout << "follow path server is " << followClient->waitForServer() << "\n";
        // ROS_INFO_STREAM(name() << " Starting to wait for msg on topic " << msg_topic_ << " yeh!");

        nav_msgs::PathConstPtr path_msg = ros::topic::waitForMessage<nav_msgs::Path>("planner/path_result");
        girona_utils::PathGoal goal;
        goal.path = *path_msg;

        followClient->sendGoal(goal,
                               actionlib::SimpleActionClient<girona_utils::PathAction>::SimpleDoneCallback(),
                               actionlib::SimpleActionClient<girona_utils::PathAction>::SimpleActiveCallback(),
                               boost::bind(&FollowPath::feedback, this, _1));

        return NodeStatus::RUNNING;
    }
    NodeStatus FollowPath::onRunning()
    {
        // ROS_INFO_STREAM(name() << " Waiting for user input on topic " << msg_topic_);
        switch (pathStatus_->status)
        {
        case girona_utils::PathFeedback::STARTED:
            return NodeStatus::RUNNING;
            break;
        case girona_utils::PathFeedback::END_REACHED:
            setOutput("success", true);
            return NodeStatus::SUCCESS;
        case girona_utils::PathFeedback::STOPPED:
            return NodeStatus::FAILURE;
        default:
            return NodeStatus::FAILURE;
            break;
        }
        return NodeStatus::SUCCESS;
    }
    void FollowPath::onHalted()
    {
        ROS_INFO_STREAM(name() << " Halted!");
    }

} // namespace BT