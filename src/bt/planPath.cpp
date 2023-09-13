#include <behaviors/planPath.h>

ros::ServiceClient pathClient;

namespace BT
{
    PlanPath::PlanPath(ros::NodeHandle &nh, const std::string &name, const BT::NodeConfiguration &conf)
        : BT::StatefulActionNode(name, conf), nh_(nh)
    {
        std::cout << "Instance of " << name << " created!" << std::endl; // Check when the object is built
        std::cout << "using rosnode: " << nh_.getNamespace() << "\n";
        pathClient = nh.serviceClient<hrov_martech2023::PlanGoal>("getPath");
    }

    NodeStatus PlanPath::onStart()
    {
        std::cout << "planning path server is " << pathClient.exists() << "\n";

        return NodeStatus::RUNNING;
    }
    NodeStatus PlanPath::onRunning()
    {
        goal_ = ros::topic::waitForMessage<hrov_martech2023::BaseGoal>("/base_goal");
        std::cout << *goal_ << "\n";

        hrov_martech2023::PlanGoal req;
        req.request.position = goal_->position;
        req.request.yaw = goal_->yaw;
        pathClient.call(req);

        // base goal to plan goal then planreq call

        return NodeStatus::SUCCESS;
    }
    void PlanPath::onHalted()
    {
        ROS_INFO_STREAM(name() << " Halted!");
    }

} // namespace BT