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
private:
    // fcl
    std::shared_ptr<fcl::Boxf> auv_box_;
    std::shared_ptr<fcl::CollisionObjectf> auv_co_;
    std::shared_ptr<fcl::CollisionObjectf> tree_obj_;

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
};

class Planeador
{
private:
    ob::StateSpacePtr navSpace_;
    ompl::base::ProblemDefinitionPtr pdef_;
    std::shared_ptr<ompl::geometric::RRTConnect> planner_;
    std::shared_ptr<fcl::CollisionObjectf> tree_obj_;

    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
    std::vector<rviz_visual_tools::colors> colorlist_;

public:
    Planeador(std::shared_ptr<fcl::CollisionObjectf> tree)
    {

        std::cout << "creando planer\n";
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
        navSpace_ = r3 + so2;
        navSpace_->setName("navSpace");
        auto si(std::make_shared<ob::SpaceInformation>(navSpace_));
        si->printSettings();

        tree_obj_ = tree;
        std::cout << "\n";
        si->setStateValidityChecker(ob::StateValidityCheckerPtr(new Validator(si, tree_obj_)));

        // create a problem instance
        pdef_ = std::make_shared<ob::ProblemDefinition>(si);
        // create a planner for the defined space
        planner_ = (std::make_shared<og::RRTConnect>(si));
        // set the problem we are trying to solve for the planner_
        planner_->setProblemDefinition(pdef_);
        // perform setup steps for the planner_
        planner_->setup();
        planner_->setRange(0.7);
        // planner_->setIntermediateStates(false);
        planner_->printSettings(std::cout);

        visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world", "/rviz_visual_markers"));
        colorlist_ = {rviz_visual_tools::WHITE, rviz_visual_tools::BLUE};

        std::cout << "fin del constructor\n";
    }
    bool doPlan(hrov_martech2023::PlanGoal::Request &req, hrov_martech2023::PlanGoal::Response &res)
    {
        std::cout << "en el servicio\n";
        planner_->clear();
        pdef_->clearSolutionPaths();
        visual_tools_->deleteAllMarkers();

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

        ob::ScopedState<> current(navSpace_);
        current[0] = t.transform.translation.x;
        current[1] = t.transform.translation.y;
        current[2] = t.transform.translation.z;
        current[3] = tf2::transformToEigen(t).rotation().eulerAngles(2, 1, 0)[0];

        ob::ScopedState<> goal(navSpace_);
        goal[0] = req.position.x;
        goal[1] = req.position.y;
        goal[2] = req.position.z;
        goal[3] = req.yaw;
        std::cout << "requesting solution to:\n\n";
        pdef_->setStartAndGoalStates(current, goal);
        pdef_->print();

        // attempt to solve the problem
        if (planner_->ob::Planner::solve(1))
        {
            std::cout << "Found solution:\n";

            og::PathGeometric pathr = *pdef_->getSolutionPath()->as<og::PathGeometric>();
            std::cout << pathr.getStateCount() << "\n";

            EigenSTL::vector_Vector3d puntos;
            Eigen::Isometry3d punto;
            std::vector<rviz_visual_tools::colors> colors;
            int i = 0;
            for (auto stt : pathr.getStates())
            {
                ob::ScopedState<ob::CompoundStateSpace> sstt(navSpace_, stt);
                punto = Eigen::AngleAxisd(sstt.reals().at(3), Eigen::Vector3d::UnitZ());
                punto.translation().x() = sstt.reals().at(0);
                punto.translation().y() = sstt.reals().at(1);
                punto.translation().z() = sstt.reals().at(2);
                puntos.push_back(punto.translation());
                colors.push_back(colorlist_.at(i++ % colorlist_.size()));
                visual_tools_->publishArrow(punto, rviz_visual_tools::RED, rviz_visual_tools::LARGE);
            }

            visual_tools_->publishPath(puntos, colors, 0.05);
            // Don't forget to trigger the publisher!
            visual_tools_->trigger();

            res.success = true;
        }
        else
        {
            std::cout << "Solution not found:\n";
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

    octomap_msgs::OctomapConstPtr mapa_msg = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_binary");
    octomap::AbstractOcTree *abs_tree = octomap_msgs::msgToMap(*mapa_msg);
    std::shared_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree *>(abs_tree));

    std::shared_ptr<fcl::OcTreef> tree(new fcl::OcTreef(octree));
    std::cout << tree->getFreeThres() << "\n";
    std::shared_ptr<fcl::CollisionGeometryf> geo(tree);
    std::shared_ptr<fcl::CollisionObjectf> tree_obj(new fcl::CollisionObjectf(geo));

    Planeador plnr(tree_obj);
    ros::ServiceServer service = nh.advertiseService("getPath", &Planeador::doPlan, &plnr);

    while (ros::ok())
    {
        ros::spinOnce();
        ros::Duration(1).sleep();
        // ros::spin();
    }

    return 0;
}