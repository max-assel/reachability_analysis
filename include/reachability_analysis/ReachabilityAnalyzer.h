#pragma once

#include <ocs2_quadruped/LeggedRobotInterface.h>
#include "ocs2_quadruped_ros/visualization/LeggedRobotVisualizer.h"
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

namespace ocs2 {
namespace quadruped {

class ReachabilityAnalyzer
{
public:
    ReachabilityAnalyzer(ros::NodeHandle& nodeHandle);
    void runReachabilityAnalysis(LeggedRobotInterface & interface, 
                                    std::shared_ptr<LeggedRobotVisualizer> & leggedRobotVisualizer,
                                    PinocchioEndEffectorKinematics & endEffectorKinematics);


private:
    bool IKProjection(Eigen::VectorXd & new_q,
                        const Eigen::VectorXd & v,
                        Eigen::Vector3d torso_pose,
                        LeggedRobotInterface & interface,
                        std::shared_ptr<LeggedRobotVisualizer> & leggedRobotVisualizer,
                        bool final);
    void publishProjection(LeggedRobotInterface & interface, Eigen::VectorXd & x, std::shared_ptr<LeggedRobotVisualizer> & leggedRobotVisualizer);
    void publishContact(Eigen::VectorXd & q, 
                        PinocchioEndEffectorKinematics & endEffectorKinematics);
    void visualizeSuperquadric(std::shared_ptr<LeggedRobotVisualizer> & leggedRobotVisualizer);

    ros::Publisher projectionPublisher;
    ros::Publisher superquadricPublisher;
    int marker_counter;
    std::vector<Color> feetColorMap_;

};

}  // namespace quadruped
}  // namespace ocs2
