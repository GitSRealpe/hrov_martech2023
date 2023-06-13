// ay jesus ayudame

// ROS stuff
#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <nav_msgs/Path.h>

// package stuff
#include <hrov_martech2023/PlanGoal.h>
#include <hrov_martech2023/PointArray.h>

// OMPL STUFF
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/PlannerData.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>

// FCL STUFF
// #include <fcl/fcl.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision_result.h>

// stuff
#include <chrono>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Validator : public ob::StateValidityChecker
{
private:
    // fcl
    std::shared_ptr<fcl::Boxf> auv_box_;
    std::shared_ptr<fcl::CollisionObjectf> auv_co_;
    std::shared_ptr<fcl::CollisionObjectf> tree_obj_;

public:
    Validator(const ob::SpaceInformationPtr &si, std::shared_ptr<fcl::CollisionObjectf> tree) : ob::StateValidityChecker(si) //, tree_obj_(tree)
    {
        std::cout << "validator initializing\n";
        auv_box_ = std::shared_ptr<fcl::Boxf>(new fcl::Boxf(1.7, 1.2, 1.3));
        auv_co_.reset(new fcl::CollisionObjectf(auv_box_));
        tree_obj_ = tree;
    }
    virtual bool isValid(const ob::State *state) const
    {
        // std::cout << "validandoo beep boop\n";
        ob::ScopedState<> stt(si_->getStateSpace(), state);
        // stt.print();
        auv_co_->setTranslation(Eigen::Vector3f(stt.reals().at(0), stt.reals().at(1), stt.reals().at(3)));
        auv_co_->setRotation(Eigen::AngleAxisf(stt.reals().at(2), Eigen::Vector3f::UnitZ()).matrix());
        // std::cout << auv_co_->getTranslation() << "\n";

        fcl::CollisionRequestf col_req_;
        fcl::CollisionResultf col_res_;
        // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        fcl::collide(tree_obj_.get(), auv_co_.get(), col_req_, col_res_);
        // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[us]" << std::endl;

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
};

fcl::CollisionObjectf getMapa()
{
    std::cout << "getting updated map\n";
    octomap_msgs::OctomapConstPtr mapa_msg = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_binary");
    octomap::AbstractOcTree *abs_tree = octomap_msgs::msgToMap(*mapa_msg);
    std::shared_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree *>(abs_tree));
    std::shared_ptr<fcl::OcTreef> tree(new fcl::OcTreef(octree));
    std::cout << tree->getFreeThres() << "\n";
    std::shared_ptr<fcl::CollisionGeometryf> geo(tree);
    std::shared_ptr<fcl::CollisionObjectf> tree_obj(new fcl::CollisionObjectf(geo));
    return *tree_obj;
}

class Planeador
{
private:
    ros::NodeHandle nh_;
    ros::Publisher pathPub;
    geometry_msgs::TransformStamped t;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    tf2_ros::Buffer tfBuffer;

    ob::StateSpacePtr navSpace_;
    ompl::base::ProblemDefinitionPtr pdef_;
    ob::PlannerPtr planner_;
    og::PathSimplifierPtr simply;
    og::SimpleSetupPtr ss_;

    std::shared_ptr<fcl::CollisionObjectf> tree_obj_;

    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
    std::vector<rviz_visual_tools::colors> colorlist_;

public:
    Planeador(ros::NodeHandle nh, std::shared_ptr<fcl::CollisionObjectf> tree) : nh_(nh)
    {
        tfListener.reset(new tf2_ros::TransformListener(tfBuffer));
        std::cout << "creando planer\n";
        std::cout << nh_.getNamespace() << "\n";
        pathPub = nh_.advertise<nav_msgs::Path>("planner/path_result", 1, true);

        auto dubss(std::make_shared<ob::DubinsStateSpace>(0.5, false));
        // auto dubss(std::make_shared<ob::ReedsSheppStateSpace>(1.0));
        dubss->setName("Dubins");
        ob::RealVectorBounds bounds(2);
        bounds.setLow(-100);
        bounds.setHigh(100);
        dubss->setBounds(bounds);
        auto zss(std::make_shared<ob::RealVectorStateSpace>(1));
        zss->setName("Z");
        ob::RealVectorBounds bounds2(1);
        bounds2.setLow(0.5);
        bounds2.setHigh(20);
        zss->setBounds(bounds2);
        navSpace_ = dubss + zss;
        navSpace_->setName("navSpace");
        navSpace_->setLongestValidSegmentFraction(0.1);

        std::cout << "setting simple setup\n";
        ss_ = std::make_shared<og::SimpleSetup>(navSpace_);
        tree_obj_ = tree;
        auto si = ss_->getSpaceInformation();
        std::cout << "setting validity checker setup\n";
        ss_->setStateValidityChecker(ob::StateValidityCheckerPtr(new Validator(si, tree_obj_)));
        std::cout << "setting planner\n";
        planner_ = (std::make_shared<og::RRT>(si));
        ss_->setPlanner(planner_);
        std::cout << "printing setup\n";
        ss_->setup();
        ss_->print();
        std::cout << "done printing setup\n";

        visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world_ned", "/rviz_visual_markers"));
        colorlist_ = {rviz_visual_tools::WHITE, rviz_visual_tools::BLUE};

        std::cout << "fin del constructor\n";
    }
    bool doPlan(hrov_martech2023::PlanGoal::Request &req, hrov_martech2023::PlanGoal::Response &res)
    {
        std::cout << "en el servicio\n";
        *tree_obj_ = getMapa();
        ss_->clear();
        visual_tools_->deleteAllMarkers();

        t = tfBuffer.lookupTransform("world_ned", "girona1000/base_link", ros::Time(0));
        std::cout << t << "\n";

        ob::ScopedState<> current(navSpace_);
        current[0] = t.transform.translation.x;
        current[1] = t.transform.translation.y;
        current[3] = t.transform.translation.z;
        current[2] = tf2::transformToEigen(t).rotation().eulerAngles(2, 1, 0)[0];

        ob::ScopedState<> goal(navSpace_);
        goal[0] = req.position.x;
        goal[1] = req.position.y;
        goal[3] = req.position.z;
        goal[2] = req.yaw;
        std::cout << "requesting solution to:\n\n";
        ss_->setStartAndGoalStates(current, goal);

        ss_->print();

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        // attempt to solve the problem
        ob::PlannerStatus status = ss_->solve(20.0);
        std::cout << status << "\n";
        if (status)
        {
            std::cout << "Found solution:\n";
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

            og::PathGeometric pathres = ss_->getSolutionPath();
            std::cout << pathres.getStateCount() << "\n";
            ss_->simplifySolution(5);
            std::cout << pathres.getStateCount() << "\n";
            pathres.interpolate(pathres.getStateCount() * 10);

            EigenSTL::vector_Vector3d puntos;
            Eigen::Isometry3d punto;
            nav_msgs::Path path;
            geometry_msgs::PoseStamped posestmp;
            std::vector<rviz_visual_tools::colors> colors;
            int i = 0;
            for (auto stt : pathres.getStates())
            {
                ob::ScopedState<ob::CompoundStateSpace> sstt(navSpace_, stt);
                punto = Eigen::AngleAxisd(sstt.reals().at(2), Eigen::Vector3d::UnitZ());
                punto.translation().x() = sstt.reals().at(0);
                punto.translation().y() = sstt.reals().at(1);
                punto.translation().z() = sstt.reals().at(3);
                puntos.push_back(punto.translation());
                colors.push_back(colorlist_.at(i++ % colorlist_.size()));
                visual_tools_->publishArrow(punto, rviz_visual_tools::RED, rviz_visual_tools::LARGE);

                posestmp.pose.position.x = sstt.reals().at(0);
                posestmp.pose.position.y = sstt.reals().at(1);
                posestmp.pose.position.z = sstt.reals().at(3);
                Eigen::Quaterniond q(Eigen::AngleAxisd(sstt.reals().at(2), Eigen::Vector3d::UnitZ()));
                posestmp.pose.orientation = tf2::toMsg(q);
                path.poses.push_back(posestmp);
            }

            pathPub.publish(path);
            visual_tools_->publishPath(puntos, colors, 0.05);
            // Don't forget to trigger the publisher!
            visual_tools_->trigger();

            res.success = true;
        }
        else
        {
            std::cout << "Solution not found\n";
            res.success = false;
        }

        return true;
    }
};

int main(int argc, char **argv)
{
    std::cout << "yeah planning\n";
    ros::init(argc, argv, "planning");
    ros::NodeHandle nh;

    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    Planeador plnr(nh, std::make_shared<fcl::CollisionObjectf>(getMapa()));
    ros::ServiceServer service = nh.advertiseService("getPath", &Planeador::doPlan, &plnr);

    while (ros::ok())
    {
        ros::spinOnce();
        ros::Duration(1).sleep();
        // ros::spin();
    }

    return 0;
}
