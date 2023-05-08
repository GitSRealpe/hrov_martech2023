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
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <hrov_martech2023/PlanGoal.h>

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

class Validator : public ob::StateValidityChecker
{
public:
    Validator(const ob::SpaceInformationPtr &si, std::shared_ptr<fcl::CollisionObjectf> tree) : ob::StateValidityChecker(si) //, tree_obj_(tree)
    {
        std::cout << "validator initializing\n";
        auv_box_ = std::shared_ptr<fcl::Boxf>(new fcl::Boxf(2, 2, 2));
        auv_co_.reset(new fcl::CollisionObjectf(auv_box_));
        tree_obj_ = tree;
    }
    virtual bool isValid(const ob::State *state) const
    {
        // std::cout << "validandoo beep boop\n";
        ob::ScopedState<> stt(si_->getStateSpace(), state);
        // stt.print();
        auv_co_->setTranslation(Eigen::Vector3f(stt.reals().at(0), stt.reals().at(1), stt.reals().at(2)));
        auv_co_->setRotation(Eigen::AngleAxisf(stt.reals().at(3), Eigen::Vector3f::UnitZ()).matrix());
        // std::cout << auv_co_->getTranslation() << "\n";

        fcl::CollisionRequestf col_req_;
        fcl::CollisionResultf col_res_;
        fcl::collide(tree_obj_.get(), auv_co_.get(), col_req_, col_res_);

        if (col_res_.isCollision())
        {
            // std::cout << "colision at: \n";
            // std::cout << auv_co_->getTranslation() << "\n";
            return false;
        }
        else
        {
            return true;
        }
    }

private:
    // fcl
    std::shared_ptr<fcl::Boxf> auv_box_;
    std::shared_ptr<fcl::CollisionObjectf> auv_co_;
    std::shared_ptr<fcl::CollisionObjectf> tree_obj_;
};

bool sendWaypoints(hrov_martech2023::PlanGoal::Request &req, hrov_martech2023::PlanGoal::Response &res)
{
    ROS_INFO_STREAM("Sending los waypoints " << 3);
    return true;
}

int main(int argc, char **argv)
{
    std::cout << "yeah planning\n";
    ros::init(argc, argv, "planning");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("getWaypoints", sendWaypoints);

    octomap_msgs::OctomapConstPtr mapa_msg = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_binary");
    octomap::AbstractOcTree *abs_tree = octomap_msgs::msgToMap(*mapa_msg);
    std::shared_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree *>(abs_tree));

    std::shared_ptr<fcl::OcTreef> tree(new fcl::OcTreef(octree));
    std::cout << tree->getFreeThres() << "\n";
    std::shared_ptr<fcl::CollisionGeometryf> geo(tree);
    std::shared_ptr<fcl::CollisionObjectf> tree_obj(new fcl::CollisionObjectf(geo));

    auto r3(std::make_shared<ob::RealVectorStateSpace>(3));
    r3->setName("Position");
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, -10);
    bounds.setHigh(0, 10);
    bounds.setLow(1, -10);
    bounds.setHigh(1, 10);
    bounds.setLow(2, -5);
    bounds.setHigh(2, -0.2);
    r3->setBounds(bounds);
    auto so2(std::make_shared<ob::SO2StateSpace>());
    so2->setName("Yaw");
    auto navSpace = r3 + so2;
    navSpace->setName("navSpace");

    auto si(std::make_shared<ob::SpaceInformation>(navSpace));
    si->printSettings();
    std::cout << "\n";
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new Validator(si, tree_obj)));

    // esta tf sera el start state y ya
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
    }
    std::cout << t << "\n";

    ob::ScopedState<> current(navSpace);
    current[0] = t.transform.translation.x;
    current[1] = t.transform.translation.y;
    current[2] = t.transform.translation.z;
    current[3] = tf2::transformToEigen(t).rotation().eulerAngles(2, 1, 0)[0];
    si->isValid(current.get());

    ob::ScopedState<> goal(navSpace);
    goal[0] = 0;
    goal[1] = 6;
    goal[2] = -1;
    goal[3] = 0;
    si->isValid(goal.get());

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(current, goal);

    // create a planner for the defined space
    auto planner(std::make_shared<og::RRTConnect>(si));

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();
    planner->setRange(0.7);
    // planner->setIntermediateStates(false);
    planner->printSettings(std::cout);
    // print the problem settings
    std ::cout << "pdef print\n";
    pdef->print();

    // attempt to solve the problem
    ob::PlannerStatus solved = planner->ob::Planner::solve(60.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;

        // For visualizing things in rviz
        rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
        visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world", "/rviz_visual_markers"));
        visual_tools_->deleteAllMarkers();

        // path->print(std::cout);
        og::PathGeometric pathr = *pdef->getSolutionPath()->as<og::PathGeometric>();
        std::cout << pathr.getStateCount() << "\n";

        ob::PlannerData pd(si);
        planner->getPlannerData(pd);
        std::cout << pd.numVertices() << std::endl;

        EigenSTL::vector_Vector3d puntos;
        Eigen::Isometry3d punto;
        std::vector<rviz_visual_tools::colors> colorlist = {rviz_visual_tools::WHITE, rviz_visual_tools::BLUE};
        std::vector<rviz_visual_tools::colors> colors;
        int i = 0;
        for (auto stt : pathr.getStates())
        {
            ob::ScopedState<ob::CompoundStateSpace> sstt(navSpace, stt);

            punto = Eigen::AngleAxisd(sstt.reals().at(3), Eigen::Vector3d::UnitZ());
            punto.translation().x() = sstt.reals().at(0);
            punto.translation().y() = sstt.reals().at(1);
            punto.translation().z() = sstt.reals().at(2);
            puntos.push_back(punto.translation());
            colors.push_back(colorlist.at(i++ % colorlist.size()));
            visual_tools_->publishArrow(punto, rviz_visual_tools::RED, rviz_visual_tools::LARGE);
        }

        visual_tools_->publishPath(puntos, colors, 0.05);
        // Don't forget to trigger the publisher!
        visual_tools_->trigger();
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }

    return 0;
}
