#include <behaviors/print.h>

namespace BT
{
    Print::Print(const std::string &name, const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(name, conf)
    {
    }

    NodeStatus Print::tick()
    {
        setStatus(NodeStatus::RUNNING);
        std::vector<std::string> enable_tasks, disable_tasks;

        std::string server = getInput<std::string>("service_name").value();

        // std::cout << server << "\n";
        ROS_INFO_STREAM(name() << "Server dice: " << server);
        return NodeStatus::SUCCESS;

        // if (server == "falla")
        // {
        //     ROS_INFO_STREAM(name() << "Server dice: " << server);
        //     return NodeStatus::FAILURE;
        // }

        // if (server == "exito")
        // {
        //     ROS_INFO_STREAM(name() << "Server dice: " << server);
        //     return NodeStatus::SUCCESS;
        // }
        // else
        // {
        //     ROS_INFO_STREAM(name() << "Server dice: " << server);
        //     return NodeStatus::FAILURE;
        // }
    }

} // namespace BT