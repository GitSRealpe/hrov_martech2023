#include <behaviors/waitForMsg.h>

namespace BT
{
    WaitForMsg::WaitForMsg(ros::NodeHandle &nh, const std::string &name, const BT::NodeConfiguration &conf)
        : BT::StatefulActionNode(name, conf), nh_(nh)
    {
        std::cout << "Instance of " << name << " created!" << std::endl; // Check when the object is built
    }

    NodeStatus WaitForMsg::onStart()
    {
        // no hay necesidad de suscribirse, solo wait formessage maybe?
        // hmi_topic_ = nh_.subscribe("/pose_topic", 5, &WaitForMsg::callback, this);
        // getInput<std::string>("pose_topic", msg_topic_).value();
        getInput("msg_topic", msg_topic_);
        std::cout << msg_topic_ << "\n";
        ROS_INFO_STREAM(name() << " Starting to wait for msg on topic " << msg_topic_ << " yeh!");

        return NodeStatus::RUNNING;
    }
    NodeStatus WaitForMsg::onRunning()
    {
        // timeput para que no se quede bloqueado el arbol para siempre
        ROS_INFO_STREAM(name() << " Waiting for user input on topic " << msg_topic_);
        // wait for msg
        ros::topic::waitForMessage<std_msgs::String>(msg_topic_);
        return NodeStatus::SUCCESS;
    }
    void WaitForMsg::onHalted()
    {
        ROS_INFO_STREAM(name() << " Halted!");
    }

    // void WaitForMsg::callback(const std_msgs::String &msg)
    // {
    // }

} // namespace BT