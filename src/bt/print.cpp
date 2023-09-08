#include <behaviors/print.h>

namespace BT
{
    Print::Print(ros::NodeHandle &nh, const std::string &name, const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(name, conf), node_(nh)
    {
    }

    NodeStatus Print::tick()
    {
        setStatus(NodeStatus::RUNNING);
        std::vector<std::string> enable_tasks, disable_tasks;

        std::string server = getInput<std::string>("service_name").value();

        std::cout << server << "\n";

        if (server == "falla")
        {
            ROS_ERROR_NAMED(name(), "Server dice: (%s)", server);
            return NodeStatus::FAILURE;
        }

        if (server == "exito")
        {
            ROS_INFO_NAMED(name(), "Server dice: [%s]", server);
            return NodeStatus::SUCCESS;
        }
        else
        {
            ROS_ERROR_NAMED(name(), "Server dice: (%s)", server);
            return NodeStatus::FAILURE;
        }
    }

} // namespace BT