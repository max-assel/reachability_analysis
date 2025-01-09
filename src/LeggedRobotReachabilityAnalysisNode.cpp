#include <ros/init.h>
#include <ocs2_quadruped/LeggedRobotInterface.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include "ocs2_quadruped_ros/visualization/LeggedRobotVisualizer.h"

#include <reachability_analysis/ReachabilityAnalyzer.h>

using namespace ocs2;
using namespace quadruped;


int main(int argc, char** argv) 
{
    const std::string robotName = "legged_robot";

    // Initialize ros node
    ros::init(argc, argv, robotName + "_reachability_analysis");
    ros::NodeHandle nodeHandle;

    // Get node parameters
    std::string taskFile, urdfFile, referenceFile, volumeFlag; // envFile, 

    nodeHandle.getParam("/taskFile", taskFile);
    nodeHandle.getParam("/referenceFile", referenceFile);
    nodeHandle.getParam("/urdfFile", urdfFile);
    // nodeHandle.getParam("/envFile", envFile);
    nodeHandle.getParam("/volumeFlag", volumeFlag);

    // Robot interface
    LeggedRobotInterface interface(taskFile, urdfFile, referenceFile);
  
    // MRT
    MRT_ROS_Interface mrt(robotName);
    mrt.initRollout(&interface.getRollout());
    mrt.launchNodes(nodeHandle);

    // Visualization
    CentroidalModelPinocchioMapping pinocchioMapping(interface.getCentroidalModelInfo());
    PinocchioEndEffectorKinematics endEffectorKinematics(interface.getPinocchioInterface(), pinocchioMapping,
                                                        interface.modelSettings().contactNames3DoF);
    std::shared_ptr<LeggedRobotVisualizer> leggedRobotVisualizer(
        new LeggedRobotVisualizer(interface.getPinocchioInterface(), interface.getCentroidalModelInfo(), endEffectorKinematics, nodeHandle));

    // Dummy legged robot
    MRT_ROS_Dummy_Loop leggedRobotDummySimulator(mrt, interface.mpcSettings().mrtDesiredFrequency_,
                                                interface.mpcSettings().mpcDesiredFrequency_);
    leggedRobotDummySimulator.subscribeObservers({leggedRobotVisualizer});

    ReachabilityAnalyzer * reachabilityAnalyzer = new ReachabilityAnalyzer(nodeHandle);

    dynamic_reconfigure::Server<reachability_analysis::ParametersConfig> server;
    dynamic_reconfigure::Server<reachability_analysis::ParametersConfig>::CallbackType serverCallback;

    serverCallback = boost::bind(&ReachabilityAnalyzer::reconfigureCallback, reachabilityAnalyzer, _1, _2);
    server.setCallback(serverCallback);

    endEffectorKinematics.setPinocchioInterface(interface.getPinocchioInterface());


    ros::Rate rate(10);

    while (ros::ok())
    {
        ros::spinOnce(); // need because we are receiving callbacks
        reachabilityAnalyzer->runReachabilityAnalysis(interface, 
                                                        leggedRobotVisualizer,
                                                        endEffectorKinematics,
                                                        volumeFlag);        
        rate.sleep();
    }

    delete reachabilityAnalyzer;

    return 0;
}