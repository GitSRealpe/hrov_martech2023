// ay jesus ayudame

// ROS stuff
#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
// #include <tf/transform_listener.h>

// OMPL STUFF
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/PlannerData.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

// FCL STUFF
#include <fcl/fcl.h>
#include <fcl/geometry/octree/octree.h>
// #include <fcl/narrowphase/collision.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state)
{
    std::cout << "validando, beep boop\n";
    double *val = state->as<ob::RealVectorStateSpace::StateType>()->values;
    std::cout << val[0] << "," << val[1] << "\n";
    // std::cout << "afuera\n";
    return true;
}

int main(int argc, char **argv)
{
    std::cout << "yeah planning\n";
    ros::init(argc, argv, "planning");
    ros::NodeHandle nh;

    octomap_msgs::OctomapConstPtr mapa_msg = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_binary");
    octomap::AbstractOcTree *abs_tree = octomap_msgs::msgToMap(*mapa_msg);
    std::shared_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree *>(abs_tree));

    // porque especificar el type?? fcl::OcTree<float> tree(octree);
    std::shared_ptr<fcl::OcTreef> tree(new fcl::OcTreef(octree));
    std::cout << tree->getFreeThres() << "\n";
    std::shared_ptr<fcl::CollisionGeometryf> geo(tree);
    fcl::CollisionObjectf tree_obj(geo);

    std::shared_ptr<fcl::Boxf> auv_box(new fcl::Boxf(1, 1, 1));

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped t;
    while (true)
    {
        try
        {
            t = tfBuffer.lookupTransform("world", "girona1000/base_link", ros::Time(0));
            break;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.2).sleep();
            continue;
        }
        // ros::Duration(0.2).sleep();
    }
    std::cout << t << "\n";

    tf2::transformToEigen(t);
    fcl::Transform3f auv_tf(tf2::transformToEigen(t));
    Eigen::IOFormat fmt(3, 0, ", ", ";\n", "[", "]", "[", "]");
    std::cout << auv_tf.matrix().format(fmt) << "\n";

    // fcl::CollisionObjectf *auv_co = new fcl::CollisionObjectf(auv_box, auv_tf);
    fcl::CollisionObjectf auv_co(auv_box, auv_tf);
    fcl::CollisionRequestf col_req;
    fcl::CollisionResultf col_res;

    fcl::collide(&tree_obj, &auv_co, col_req, col_res);

    std::cout << col_res.isCollision() << "\n";

    // auto r3(std::make_shared<ob::RealVectorStateSpace>(3));
    // r3->setName("Position");
    // ob::RealVectorBounds bounds(3);
    // bounds.setLow(0, -10);
    // bounds.setHigh(0, 10);
    // bounds.setLow(1, -10);
    // bounds.setHigh(1, 10);
    // bounds.setLow(2, 0.5);
    // bounds.setHigh(2, 5);
    // r3->setBounds(bounds);
    // auto so2(std::make_shared<ob::SO2StateSpace>());
    // so2->setName("Yaw");
    // auto navSpace = r3 + so2;
    // navSpace->setName("navSpace");

    // auto si(std::make_shared<ob::SpaceInformation>(navSpace));
    // si->printSettings();
    // std::cout << "\n";

    // ob::ScopedState<> start(navSpace);
    // start.random();
    // start.print();

    return 0;
}
