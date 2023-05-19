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

// package stuff
#include <hrov_martech2023/PlanGoal.h>
#include <hrov_martech2023/PointArray.h>

// OMPL STUFF
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/PlannerData.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/tools/benchmark/Benchmark.h>

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
        auv_box_ = std::shared_ptr<fcl::Boxf>(new fcl::Boxf(1.8, 1.2, 1.3));
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
    ros::NodeHandle nh_;
    ros::Publisher pathPub;

    ob::StateSpacePtr navSpace_;
    og::SimpleSetupPtr ss_;
    ompl::base::ProblemDefinitionPtr pdef_;
    // std::shared_ptr<og::RRTConnect> planner_;
    std::shared_ptr<og::RRT> planner_;
    std::shared_ptr<fcl::CollisionObjectf> tree_obj_;

    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
    std::vector<rviz_visual_tools::colors> colorlist_;

public:
    Planeador(ros::NodeHandle nh, std::shared_ptr<fcl::CollisionObjectf> tree) : nh_(nh)
    {
        std::cout << "creando planer\n";
        std::cout << nh_.getNamespace() << "\n";
        pathPub = nh_.advertise<hrov_martech2023::PointArray>("planner/path_result", 1, true);

        auto dubss(std::make_shared<ob::DubinsStateSpace>(0.5, true));
        dubss->setName("Dubins");
        ob::RealVectorBounds bounds(2);
        bounds.setLow(-10);
        bounds.setHigh(10);
        dubss->setBounds(bounds);
        auto zss(std::make_shared<ob::RealVectorStateSpace>(1));
        zss->setName("Z");
        ob::RealVectorBounds bounds2(1);
        bounds2.setHigh(0.5);
        bounds2.setLow(-5);
        zss->setBounds(bounds2);
        navSpace_ = dubss + zss;
        navSpace_->setName("navSpace");
        ss_.reset(new og::SimpleSetup(navSpace_));
        auto si(std::make_shared<ob::SpaceInformation>(navSpace_));
        // si->printSettings();

        tree_obj_ = tree;
        std::cout << "\n";
        // si->setStateValidityChecker(ob::StateValidityCheckerPtr(new Validator(si, tree_obj_)));
        ss_->setStateValidityChecker(ob::StateValidityCheckerPtr(new Validator(si, tree_obj_)));

        std::cout << "fin del constructor\n";
    }
    bool doPlan(hrov_martech2023::PlanGoal::Request &req, hrov_martech2023::PlanGoal::Response &res)
    {
        std::cout << "en el servicio\n";
        ss_->clear();
        ss_->clearStartStates();

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
        current[3] = t.transform.translation.z;
        current[2] = tf2::transformToEigen(t).rotation().eulerAngles(2, 1, 0)[0];

        ob::ScopedState<> goal(navSpace_);
        goal[0] = req.position.x;
        goal[1] = req.position.y;
        goal[3] = req.position.z;
        goal[2] = req.yaw;
        std::cout << "requesting solution to:\n\n";

        ss_->setStartAndGoalStates(current, goal);
        ss_->setup();
        ss_->print();

        // First we create a benchmark class:
        ompl::tools::Benchmark b(*ss_.get(), "my experiment");
        // We add the planners to evaluate.
        b.addPlanner(ob::PlannerPtr(new og::KPIECE1(ss_->getSpaceInformation())));
        b.addPlanner(ob::PlannerPtr(new og::RRT(ss_->getSpaceInformation())));
        b.addPlanner(ob::PlannerPtr(new og::SBL(ss_->getSpaceInformation())));
        b.addPlanner(ob::PlannerPtr(new og::LBKPIECE1(ss_->getSpaceInformation())));

        ompl::tools::Benchmark::Request test;
        test.maxTime = 5.0;
        test.maxMem = 100.0;
        test.runCount = 50;
        test.displayProgress = true;
        b.benchmark(test);

        // This will generate a file of the form ompl_host_time.log
        b.saveResultsToFile();

        res.success = true;

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

    Planeador plnr(nh, tree_obj);
    ros::ServiceServer service = nh.advertiseService("getPath", &Planeador::doPlan, &plnr);

    while (ros::ok())
    {
        ros::spinOnce();
        ros::Duration(1).sleep();
        // ros::spin();
    }

    return 0;
}
