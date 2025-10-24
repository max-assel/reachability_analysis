#pragma once

#include <rclcpp/rclcpp.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <random>

// #include <ocs2_quadruped/LeggedRobotInterface.h>
#include <ocs2_quadruped_interface/QuadrupedVisualizer.h>
// #include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

#include <ocs2_quadruped_interface/QuadrupedInterface.h>

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
static constexpr size_t JOINT_DIM = 12;

enum LegIndex {FL = 0, FR = 1, BL = 2, BR = 3}; /**< Enum for leg indices */

class ReachabilityAnalyzer
{
public:
    ReachabilityAnalyzer(const rclcpp::Node::SharedPtr& node,
                            std::shared_ptr<switched_model::QuadrupedInterface> & interface,
                            std::shared_ptr<switched_model::QuadrupedVisualizer> & visualizer);

    void runReachabilityAnalysis(const rclcpp::Time & timeStamp);

private:
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

    void publishState(const Eigen::VectorXd & q, const rclcpp::Time & timeStamp);
    void publishEEPositions(const Eigen::VectorXd & q);
    void visualize3DSuperquadrics(const Eigen::Vector3d & p_torso);
    void visualize3DSuperquadric(const int & legIdx,
                                    const Eigen::Vector3d & p_torso);                            

    std::shared_ptr<switched_model::QuadrupedInterface> interface_; /**< Anymal interface */
    std::shared_ptr<switched_model::QuadrupedVisualizer> visualizer_; /**< Anymal visualizer */

    rclcpp::Node::SharedPtr node_;

    // front set
    double sqDimX_front = 0.40;
    double sqDimY_front = 0.35;
    double sqDimZ_front = 0.2125;

    double sqCurvX_front = 1.5;
    double sqCurvY_front = 2.0;
    double sqCurvZ_front = 2.0;

    double x_offset_front = 0.20;
    double y_offset_front = 0.025;
    double z_offset_front = -0.25;

    double roll_front = 0.30; // left/right
    double pitch_front = 0.0; // front/back
    double yaw_front = 0.0; // ??   

    // back set
    double sqDimX_back = 0.40;
    double sqDimY_back = 0.35;
    double sqDimZ_back = 0.2125;

    double sqCurvX_back = 1.5;
    double sqCurvY_back = 2.0;
    double sqCurvZ_back = 2.0;

    double x_offset_back = -0.20;
    double y_offset_back = -0.025;
    double z_offset_back = -0.25;

    double roll_back = 0.30; // left/right
    double pitch_back = 0.0; // front/back
    double yaw_back = 0.0; // ??   

    // Callback handle for parameter changes
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;    

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr FLCloudPublisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr FRCloudPublisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr BLCloudPublisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr BRCloudPublisher;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr FLSuperquadricPublisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr FRSuperquadricPublisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr BLSuperquadricPublisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr BRSuperquadricPublisher;

    int marker_counter;
    std::vector<ocs2::Color> feetColorMap_;

    std::default_random_engine generator;

};

// }  // namespace quadruped
// }  // namespace ocs2
