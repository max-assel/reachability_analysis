#pragma once

#include <rclcpp/rclcpp.hpp>

#include <random>

// #include <ocs2_quadruped/LeggedRobotInterface.h>
#include <ocs2_custom_quadruped_interface/CustomQuadrupedVisualizer.h>
// #include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <ocs2_custom_quadruped_interface/CustomQuadrupedInterface.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// #include <geometry_msgs/Point.h>

// #include <dynamic_reconfigure/server.h>
// #include <reachability_analysis/ParametersConfig.h>

// namespace ocs2 {
// namespace quadruped {

static constexpr size_t CONFIG_DIM = 18;

class ReachabilityAnalyzer
{
public:
    ReachabilityAnalyzer(const rclcpp::Node::SharedPtr& node,
                            std::shared_ptr<switched_model::CustomQuadrupedInterface> & interface);
    // void runProjectionAnalysis(LeggedRobotInterface & interface, 
    //                             std::shared_ptr<LeggedRobotVisualizer> & leggedRobotVisualizer,
    //                             PinocchioEndEffectorKinematics & endEffectorKinematics);
    
    // LeggedRobotInterface & interface, 
    // std::shared_ptr<LeggedRobotVisualizer> & leggedRobotVisualizer,
    // PinocchioEndEffectorKinematics & endEffectorKinematics,
    // const std::string & volumeFlag
    
    void runReachabilityAnalysis();
    // void reconfigureCallback(reachability_analysis::ParametersConfig &config, uint32_t level);

private:
    // // bool IKProjection(Eigen::VectorXd & new_q,
    // //                     const Eigen::VectorXd & v,
    // //                     Eigen::Vector3d torso_pose,
    // //                     LeggedRobotInterface & interface,
    // //                     std::shared_ptr<LeggedRobotVisualizer> & leggedRobotVisualizer,
    // //                     bool final);
    // void publishState(LeggedRobotInterface & interface, Eigen::VectorXd & x, std::shared_ptr<LeggedRobotVisualizer> & leggedRobotVisualizer);
    // void publishEEPositions(Eigen::VectorXd & q, 
    //                         PinocchioEndEffectorKinematics & endEffectorKinematics);
    // void visualize3DSuperquadrics(const Eigen::Vector3d & p_torso);
    // void visualize3DSuperquadric(const int & legIdx,
    //                                 const Eigen::Vector3d & p_torso);                            
    // // void visualizeSuperquadric(std::shared_ptr<LeggedRobotVisualizer> & leggedRobotVisualizer);

    std::shared_ptr<switched_model::CustomQuadrupedInterface> interface_; /**< Go2 interface */

    rclcpp::Node::SharedPtr node_;

    double sqCurvX = 0.5;
    double sqCurvY = 0.5;
    double sqCurvZ = 0.5;

    double sqDimX = 0.175;
    double sqDimY = 0.15;
    double sqDimZ = 0.30;

    double x_offset_front = 0.0;
    double x_offset_back = 0.0;
    double y_offset_left = 0.0;
    double y_offset_right = 0.0;
    double z_offset = 0.0;

    double roll = 0.0; // left/right
    double pitch = 0.0; // front/back
    double yaw = 0.0; // ??

    // double FL_hip_manual_pos = 0.0;
    // double FL_thigh_manual_pos = 0.0;
    // double FL_calf_manual_pos = 0.0;

    // double FR_hip_manual_pos = 0.0;
    // double FR_thigh_manual_pos = 0.0;
    // double FR_calf_manual_pos = 0.0;

    // double BL_hip_manual_pos = 0.0;
    // double BL_thigh_manual_pos = 0.0;
    // double BL_calf_manual_pos = 0.0;

    // double BR_hip_manual_pos = 0.0;
    // double BR_thigh_manual_pos = 0.0;
    // double BR_calf_manual_pos = 0.0;        

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr projectionPublisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr superquadricPublisher;
    int marker_counter;
    std::vector<ocs2::Color> feetColorMap_;

    std::default_random_engine generator;

};

// }  // namespace quadruped
// }  // namespace ocs2
