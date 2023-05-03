#include <robowflex_library/builder.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/io/broadcaster.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/log.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>

#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>

#include <octomap_msgs/conversions.h>

using namespace robowflex;

static const std::string BASE = "base";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    // Create a robot.
    auto gr1000 = std::make_shared<Robot>("girona1000");
    gr1000->initializeFromYAML("package://hrov_martech2023/config/girona1000.yaml");

    for (auto joint : gr1000->getJointNames())
    {
        std::cout << joint << "\n";
    }
    std::cout << "\n";

    // no hay jointmodel 0 por alguna razon
    // std::vector<moveit_msgs::JointLimits> limits = gr1000->getModel()->getJointModel(2)->getVariableBoundsMsg();
    for (auto jointModel : gr1000->getModel()->getJointModels())
    {
        for (moveit_msgs::JointLimits limit : jointModel->getVariableBoundsMsg())
        {
            std::cout << limit << "\n";
        }
    }
    // for (moveit_msgs::JointLimits limit : limits)
    // {
    //     std::cout << limit << "\n";
    // }

    // Create an RViz visualization helper. Publishes all topics and parameter under `/robowflex` by
    // default.
    IO::RVIZHelper rviz(gr1000);
    // maybe no usar el broadcaster?

    RBX_INFO("RViz Initialized! Press enter to continue (after your RViz is setup)...");
    std::cin.get();

    octomap_msgs::OctomapConstPtr mapa_msg = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_binary");
    octomap::AbstractOcTree *abs_tree = octomap_msgs::binaryMsgToMap(*mapa_msg);
    std::cout << "resolution " << abs_tree->getResolution() << "\n";
    std::cout << "type " << abs_tree->getTreeType() << "\n";
    std::unique_ptr<octomap::OcTree> tree(dynamic_cast<octomap::OcTree *>(abs_tree));

    auto scene = std::make_shared<Scene>(gr1000);
    // scene->fromYAMLFile("package://robowflex_scripts/resources/mobile_test/test_bot_scene.yaml");
    scene->getScene()->processOctomapMsg(*mapa_msg);
    scene->toYAMLFile("gr1000_scene.yaml");

    // robot_state::RobotStatePtr any_state(new moveit::core::RobotState(gr1000->getModel()));
    // any_state->setVariablePositions({-3, 0, 4, 0});
    // any_state->printStatePositions();
    // any_state->updateCollisionBodyTransforms();
    // std::cout << scene->checkCollision(*any_state).collision << "\n";

    // state del robot en la planning scene
    RBX_INFO("setting scene state");
    std::cin.get();
    // scene->getScene()->setCurrentState(*any_state);
    scene->getScene()->getPlanningFrame();
    // Visualize the scene in RViz.
    rviz.updateScene(scene);

    // Create the default planner for the gr1000.
    auto planner = std::make_shared<OMPL::OMPLPipelinePlanner>(gr1000);
    planner->initialize(
        "package://girona1000_moveit/config/ompl_planning.yaml" // planner config
    );

    // Create a motion planning request with a pose goal.
    MotionRequestBuilder request(planner, BASE);
    request.setAllowedPlanningTime(60 * 5);
    request.setNumPlanningAttempts(10);
    moveit_msgs::WorkspaceParameters wp;
    geometry_msgs::Vector3 v;
    v.x = v.y = v.z = -10;
    wp.min_corner = v;
    v.x = v.y = v.z = 10;
    wp.max_corner = v;
    request.setWorkspaceBounds(wp);
    std::cout << request.getRequest().workspace_parameters << "\n";
    request.setStartConfiguration({0, 0, 0, 0});
    request.setGoalConfiguration({-3, 0, 5, 0});

    // request.toYAMLFile("package://hrov_martech2023/gr1000_request.yaml");

    // Display the goal region in RViz.
    rviz.addGoalMarker("goal", request);
    rviz.updateMarkers();

    RBX_INFO("Scene and Goal displayed! Press enter to plan...");
    std::cin.get();

    // Do motion planning!
    planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());
    if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return 1;

    // Publish the trajectory to a topic to display in RViz.
    rviz.updateTrajectory(res);

    // Create a trajectory object for better manipulation.
    auto trajectory = std::make_shared<Trajectory>(res.trajectory_);
    trajectory->vectorize();

    // // Output path to a file for visualization.
    // trajectory->toYAMLFile("package://robowflex_resources/mobile_test/gr1000_traj.yaml");

    // RBX_INFO("Press enter to remove goal and scene.");
    // std::cin.get();

    // // Clean up RViz.
    // rviz.removeMarker("goal");
    // rviz.updateMarkers();
    // rviz.removeScene();

    RBX_INFO("Press enter to exit.");
    std::cin.get();

    return 0;
}
