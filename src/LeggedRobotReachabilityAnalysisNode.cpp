// #include <ocs2_quadruped/LeggedRobotInterface.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
// #include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
// #include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

// #include "ocs2_quadruped_ros/visualization/LeggedRobotVisualizer.h"

#include <ocs2_mpc/MPC_Settings.h>

#include <ocs2_anymal_mpc/AnymalInterface.h>
#include <ocs2_quadruped_interface/QuadrupedVisualizer.h>

#include <reachability_analysis/ReachabilityAnalyzer.h>

// using namespace ocs2;
// using namespace quadruped;

int main(int argc, char** argv) 
{
    const std::string robotName = "anymal";

    // Initialize ros node
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("anymal_reachability_analysis",
                                                                rclcpp::NodeOptions()
                                                                .allow_undeclared_parameters(true)
                                                                .automatically_declare_parameters_from_overrides(true));

    // Get node parameters
    std::string taskFile, urdfFile, referenceFile, frameFile; 

    node->get_parameter("taskFile", taskFile);
    node->get_parameter("frameFile", frameFile);
    node->get_parameter("urdfFile", urdfFile);
    // nodeHandle.getParam("/envFile", envFile);
    // nodeHandle.getParam("/volumeFlag", volumeFlag);

    // Robot interface
    std::string urdfString = anymal::getUrdfString(urdfFile);
    auto anymalInterface = anymal::getAnymalInterface(urdfString, taskFile, frameFile); // , envFile
    ocs2::mpc::Settings mpcSettings = ocs2::mpc::loadSettings(taskFile); // new

    // MRT
    ocs2::MRT_ROS_Interface mrt(robotName);
    mrt.initRollout(&anymalInterface->getRollout());
    mrt.launchNodes(node);

    // Visualization
    auto visualizer = std::make_shared<switched_model::QuadrupedVisualizer>(anymalInterface->getKinematicModel(), 
                                                                            anymalInterface->getJointNames(), 
                                                                            anymalInterface->getBaseName(), 
                                                                            node);


    // Dummy legged robot
    ocs2::MRT_ROS_Dummy_Loop leggedRobotDummySimulator(mrt, 
                                                        mpcSettings.mrtDesiredFrequency_,
                                                        mpcSettings.mpcDesiredFrequency_);
    leggedRobotDummySimulator.subscribeObservers({visualizer});

    ReachabilityAnalyzer * reachabilityAnalyzer = new ReachabilityAnalyzer(node, anymalInterface, visualizer);


    rclcpp::Rate rate(100);
    rclcpp::Duration sleepDuration = rclcpp::Duration(rate.period());

    rclcpp::Time timeStamp = node->get_clock()->now();

    while (rclcpp::ok())
    {
        reachabilityAnalyzer->runReachabilityAnalysis(timeStamp);        
        rate.sleep();
        timeStamp += sleepDuration;
        rclcpp::spin_some(node);
    }

    // delete reachabilityAnalyzer;

    rclcpp::shutdown();
    return 0;
}