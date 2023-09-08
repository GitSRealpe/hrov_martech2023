#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include <ros/ros.h>
#include <ros/console.h>

#include <behaviors/print.h>
#include <behaviors/waitForMsg.h>
#include <behaviors/followPath.h>

using namespace BT;

int main(int argc, char **argv)
{
    if (argc != 5)
    {
        ROS_ERROR("Usage: %s path_to_xml. Currently argc %i", argv[0], argc);
        for (uint i = 0; i < argc; i++)
        {
            ROS_ERROR("%s", argv[i]);
        }
        exit(-1);
    }

    ros::init(argc, argv, "behavior_tree1");
    ros::NodeHandle nh;

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    BehaviorTreeFactory factory;

    RegisterROSNode<FollowPath>(factory, "FollowPath", nh);
    RegisterROSNode<WaitForMsg>(factory, "WaitForMsg", nh);

    factory.registerBehaviorTreeFromFile(argv[1]);
    ROS_INFO_STREAM("Registered BT:");
    for (auto &x : factory.registeredBehaviorTrees())
    {
        ROS_INFO_STREAM("-- " << x);
    }

    auto tree = factory.createTree(std::string(argv[2]));
    // auto tree = factory.createTreeFromFile("./my_tree.xml");

    NodeStatus status = NodeStatus::IDLE;

    // This logger publish status changes using ZeroMQ. Used by Groot
    PublisherZMQ publisher_zmq(tree);
    // BT::RosoutLogger rosout_logger((&tree));

    ros::Duration(5.0).sleep();

    ROS_INFO_STREAM("[BehaviorTree] Start!");

    ros::Rate rate(50);
    while (ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING))
    {
        ros::spinOnce();
        ROS_DEBUG_THROTTLE_NAMED(2, "BT", "Tree tick");
        status = tree.tickRoot();
        rate.sleep();

        if (status == NodeStatus::IDLE)
        {
            ROS_WARN_STREAM("[BehaviorTree] Idle");
        }
    }

    ros::Duration(5.0).sleep();
    ROS_INFO_STREAM("[BehaviorTree] Finished!");

    return 0;
}