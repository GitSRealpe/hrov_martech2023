// ay jesus ayudame

// ROS stuff
#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <geometry_msgs/Pose.h>
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
// #include <fcl/fcl.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision_result.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// fcl::CollisionObjectf tree_obj;
std::shared_ptr<fcl::CollisionObjectf> tree_obj;
geometry_msgs::TransformStamped t;

class Validator : public ob::StateValidityChecker
{
public:
    Validator(const ob::SpaceInformationPtr &si) : ob::StateValidityChecker(si)
    {
        std::cout << "validator initializing\n";
        auv_box_ = std::shared_ptr<fcl::Boxf>(new fcl::Boxf(1, 1, 1));
        // fcl::Transform3f fr;
        auv_co_.reset(new fcl::CollisionObjectf(auv_box_));
        // tree_obj.reset(new fcl::CollisionObjectf(auv_box_));
        // auv_co_ = fcl::CollisionObjectf(auv_box_);
    }
    virtual bool isValid(const ob::State *state) const
    {
        std::cout << "validandoo beep boop\n";
        // si_->printState(state);
        ob::ScopedState<> stt(si_->getStateSpace(), state);
        stt.print();
        // for (auto x : stt.reals())
        // {
        //     std::cout << x << "\n";
        // }

        fcl::CollisionRequestf col_req_;
        fcl::CollisionResultf col_res_;
        fcl::Transform3f auv_tf(tf2::transformToEigen(t));
        auv_co_->setTransform(auv_tf);

        std::cout << auv_co_->getTranslation() << "\n";
        std::cout << tree_obj->getTranslation() << "\n";
        fcl::collide(tree_obj.get(), auv_co_.get(), col_req_, col_res_);
        std::cout << col_res_.isCollision() << "\n";
        return true;
    }

private:
    // fcl
    std::shared_ptr<fcl::Boxf> auv_box_;
    std::shared_ptr<fcl::CollisionObjectf> auv_co_;
    // fcl::CollisionObjectf auv_co_;
    // fcl::CollisionRequestf col_req_;
    // fcl::CollisionResultf col_res_;
    // std::shared_ptr<fcl::CollisionObjectf> tree_obj;
};

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
    // fcl::CollisionObjectf tree_obj(geo);
    // std::shared_ptr<fcl::CollisionObjectf> tree_obj;
    tree_obj.reset(new fcl::CollisionObjectf(geo));
    // tree_obj = fcl::CollisionObjectf(geo);

    std::shared_ptr<fcl::Boxf> auv_box(new fcl::Boxf(2, 2, 2));

    // esta tf sera el start state y ya
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    // geometry_msgs::TransformStamped t;
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

    // fcl::CollisionObjectf auv_co(auv_box, auv_tf);
    std::shared_ptr<fcl::CollisionObjectf> auv_co;
    auv_co.reset(new fcl::CollisionObjectf(auv_box, auv_tf));
    fcl::CollisionRequestf col_req;
    fcl::CollisionResultf col_res;
    // fcl::collide(tree_obj.get(), &auv_co, col_req, col_res);
    fcl::collide(tree_obj.get(), auv_co.get(), col_req, col_res);
    std::cout << col_res.isCollision() << "\n";

    auto r3(std::make_shared<ob::RealVectorStateSpace>(3));
    r3->setName("Position");
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, -10);
    bounds.setHigh(0, 10);
    bounds.setLow(1, -10);
    bounds.setHigh(1, 10);
    bounds.setLow(2, 0.5);
    bounds.setHigh(2, 5);
    r3->setBounds(bounds);
    auto so2(std::make_shared<ob::SO2StateSpace>());
    so2->setName("Yaw");
    auto navSpace = r3 + so2;
    navSpace->setName("navSpace");

    auto si(std::make_shared<ob::SpaceInformation>(navSpace));
    si->printSettings();
    std::cout << "\n";

    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new Validator(si)));

    // ob::ScopedState<> start(navSpace);
    // start.random();
    // start.print();
    // si->isValid(start.get());

    ob::ScopedState<> current(navSpace);
    current[0] = t.transform.translation.x;
    current[1] = t.transform.translation.y;
    current[2] = t.transform.translation.z;
    current[3] = tf2::transformToEigen(t).rotation().eulerAngles(2, 1, 0)[0];
    // current.print();
    si->isValid(current.get());

    return 0;
}
