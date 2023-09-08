#include <behaviors/followPath.h>
#include <std_srvs/Trigger.h>

ros::ServiceClient moveClient;

namespace BT
{
    FollowPath::FollowPath(ros::NodeHandle &nh, const std::string &name, const BT::NodeConfiguration &conf)
        : BT::StatefulActionNode(name, conf), nh_(nh)
    {
        std::cout << "Instance of " << name << " created!" << std::endl; // Check when the object is built
        std::cout << "using rosnode: " << nh_.getNamespace() << "\n";
        moveClient = nh.serviceClient<std_srvs::Trigger>("path_manager_server/startPath");
        // ros::Subscriber subFeed = nh.subscribe("/pid_controller/feedback", 10, &callback, this);
        ros::Subscriber subFeed = nh.subscribe("/pid_controller/feedback", 10, &FollowPath::callback, this);
    }

    void FollowPath::callback(const girona_utils::PathStatusConstPtr pathMsg)
    {
        std::cout << pathMsg->status << "\n";
    }

    // void callback(const girona_utils::PathStatusConstPtr pathMsg)
    // {
    // }

    NodeStatus FollowPath::onStart()
    {
        // getInput<std::string>("pose_topic", msg_topic_).value();
        // getInput("msg_topic", msg_topic_);
        // std::cout << msg_topic_ << "\n";

        std::cout << "follow path server is " << moveClient.exists() << "\n";
        // ROS_INFO_STREAM(name() << " Starting to wait for msg on topic " << msg_topic_ << " yeh!");

        std_srvs::Trigger req;
        if (moveClient.call(req))
        {
            std::cout << "call was succesfull \n";
            return NodeStatus::RUNNING;
        }
        else
        {
            std::cout << "call did not succeed \n";
            return NodeStatus::FAILURE;
        }

        return NodeStatus::RUNNING;
    }
    NodeStatus FollowPath::onRunning()
    {
        // ROS_INFO_STREAM(name() << " Waiting for user input on topic " << msg_topic_);

        switch (pathStatus_.status)
        {
        case girona_utils::PathStatus::RUNNING:
            return NodeStatus::RUNNING;
            break;
        case girona_utils::PathStatus::END_REACHED:
            setOutput("success", true);
            return NodeStatus::SUCCESS;
        case girona_utils::PathStatus::STOPPED:
            return NodeStatus::FAILURE;
        default:
            return NodeStatus::FAILURE;
            break;
        }
    }
    void FollowPath::onHalted()
    {
        ROS_INFO_STREAM(name() << " Halted!");
    }

} // namespace BT