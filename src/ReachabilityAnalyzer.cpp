// #include <pinocchio/fwd.hpp> // NEED TO KEEP THIS AT TOP

// #include "pinocchio/algorithm/frames.hpp"
// #include "pinocchio/algorithm/kinematics.hpp"
// #include "pinocchio/algorithm/model.hpp"

// #include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <reachability_analysis/ReachabilityAnalyzer.h>

// namespace ocs2 {
// namespace quadruped {


// void enforceJointLimits(const pinocchio::Model & model, Eigen::VectorXd & q, Eigen::Vector3d torso_pose)
// {
//     // std::cout << "  model.lowerPositionLimit: " << model.lowerPositionLimit.transpose() << std::endl;
//     // std::cout << "  model.upperPositionLimit: " << model.upperPositionLimit.transpose() << std::endl;

//     // need to artificially set z height constraint

//     for (int i = 0; i < model.nq; i++) // 18
//     {
//         if (i >= 0 && i < 3) // robot torso position
//         {
//             q[0] = torso_pose[0];
//             q[1] = torso_pose[1];
//             q[2] = torso_pose[2];
//             /*
//             if (i == 2)
//             {
//                 // robot z position
//                 double z_min = 0.25;
//                 double z_max = 0.35;
//                 q[i] = std::max(z_min, std::min(q[i], z_max));
//             }
//             */
            
            
//         } else if (i >= 3 && i < 6) // robot torso orientation
//         {
//             q[3] = 0.0; 
//             q[4] = 0.0; 
//             q[5] = 0.0;
//         } else // joint angles
//         {
//             // unwind
//             while (q[i] < -2.0 * M_PI) 
//                 q[i] += 2.0 * M_PI;

//             while (q[i] > 2.0 * M_PI) 
//                 q[i] -= 2.0 * M_PI;         

//             // clamp with limits
//             q[i] = std::max(model.lowerPositionLimit[i], std::min(q[i], model.upperPositionLimit[i]));
//         }
//     }
// }

// void ReachabilityAnalyzer::reconfigureCallback(reachability_analysis::ParametersConfig &config, uint32_t level) 
// {
//     sqCurvX = config.sqCurvX;
//     sqCurvY = config.sqCurvY;
//     sqCurvZ = config.sqCurvZ;

//     sqDimX = config.sqDimX;
//     sqDimY = config.sqDimY;
//     sqDimZ = config.sqDimZ;

//     x_offset_front = config.x_offset_front;
//     x_offset_back = config.x_offset_back;
//     y_offset_left = config.y_offset_left;
//     y_offset_right = config.y_offset_right;
//     z_offset = config.z_offset;
 
//     FL_hip_manual_pos = config.FL_hip;
//     FL_thigh_manual_pos = config.FL_thigh;
//     FL_calf_manual_pos = config.FL_calf;

//     FR_hip_manual_pos = config.FR_hip;
//     FR_thigh_manual_pos = config.FR_thigh;
//     FR_calf_manual_pos = config.FR_calf;

//     BL_hip_manual_pos = config.BL_hip;
//     BL_thigh_manual_pos = config.BL_thigh;
//     BL_calf_manual_pos = config.BL_calf;

//     BR_hip_manual_pos = config.BR_hip;
//     BR_thigh_manual_pos = config.BR_thigh;
//     BR_calf_manual_pos = config.BR_calf;     

//     roll = config.roll;
//     pitch = config.pitch;
//     yaw = config.yaw;   
// }

// bool ReachabilityAnalyzer::IKProjection(Eigen::VectorXd & new_q,
//                     const Eigen::VectorXd & v,
//                     Eigen::Vector3d torso_pose,
//                     LeggedRobotInterface & interface,
//                     std::shared_ptr<LeggedRobotVisualizer> & leggedRobotVisualizer,
//                     bool final)
// {
//     // std::cout << "[IKProjection]" << std::endl;
//     const auto& model = interface.getPinocchioInterface().getModel();
//     auto& data = interface.getPinocchioInterface().getData();

   
//     Eigen::MatrixXd J_FL, J_FR, J_BL, J_BR;
//     Eigen::MatrixXd J_total;

//     J_FL = pinocchio::Data::Matrix6x::Zero(6, 18);
//     J_FR = pinocchio::Data::Matrix6x::Zero(6, 18);
//     J_BL = pinocchio::Data::Matrix6x::Zero(6, 18);
//     J_BR = pinocchio::Data::Matrix6x::Zero(6, 18);
//     J_total = Eigen::MatrixXd::Zero(12, 18); 

//     pinocchio::FrameIndex FL_foot_frame_id = model.getBodyId("FL_foot");
//     pinocchio::FrameIndex FR_foot_frame_id = model.getBodyId("FR_foot");
//     pinocchio::FrameIndex BL_foot_frame_id = model.getBodyId("RL_foot");
//     pinocchio::FrameIndex BR_foot_frame_id = model.getBodyId("RR_foot");     

//     bool foundTransition = false;
//     double epsilon = 0.005;
//     int num_projection_iterations = 100;

//     // std::cout << "  beginning q: " << new_q.transpose() << std::endl;

//     // project into contact
//     for (int projection_iteration = 0; projection_iteration < num_projection_iterations; projection_iteration++)
//     {
//         // std::cout << "  projection " << projection_iteration << std::endl;
//         // publishState(interface, new_q, leggedRobotVisualizer);

//         // update model based on current configuration
//         pinocchio::forwardKinematics(model, data, new_q, v);
//         pinocchio::computeJointJacobians(model, data);        
//         pinocchio::updateFramePlacements(model, data);

//         // calculate target footholds
//         Eigen::Vector3d FL_target_position, FR_target_position, BL_target_position, BR_target_position;
//         if (final)
//         {
//             FL_target_position = Eigen::Vector3d(0.175, 0.20, 0.0);
//             FR_target_position = Eigen::Vector3d(0.175, -0.20, 0.0);
//             BL_target_position = Eigen::Vector3d(-0.25, 0.20, 0.0);
//             BR_target_position = Eigen::Vector3d(-0.25, -0.20, 0.0);            
//         } else
//         {
//             FL_target_position = data.oMf[FL_foot_frame_id].translation();
//             FL_target_position[2] = 0.0;
//             FR_target_position = data.oMf[FR_foot_frame_id].translation();
//             FR_target_position[2] = 0.0;
//             BL_target_position = data.oMf[BL_foot_frame_id].translation();
//             BL_target_position[2] = 0.0;
//             BR_target_position = data.oMf[BR_foot_frame_id].translation();
//             BR_target_position[2] = 0.0;
//         }
        

//         // std::cout << "      FL_target_position: " << FL_target_position.transpose() << std::endl;
//         // std::cout << "      FR_target_position: " << FR_target_position.transpose() << std::endl;
//         // std::cout << "      BL_target_position: " << BL_target_position.transpose() << std::endl;
//         // std::cout << "      BR_target_position: " << BR_target_position.transpose() << std::endl;

//         Eigen::VectorXd f_x(12); // 3 Dof per foot, constraining position
//         f_x << (data.oMf[FL_foot_frame_id].translation() - FL_target_position), 
//                (data.oMf[FR_foot_frame_id].translation() - FR_target_position),
//                (data.oMf[BL_foot_frame_id].translation() - BL_target_position), 
//                (data.oMf[BR_foot_frame_id].translation() - BR_target_position);
//         // std::cout << "      error: " << f_x.norm() << std::endl;

//         foundTransition = (f_x.norm() < epsilon);
//         if (foundTransition)
//             break;

//         // calculate Jacobians
//         J_FL.setZero(); J_FR.setZero(); J_BL.setZero(); J_BR.setZero();

//         pinocchio::getFrameJacobian(model, data, FL_foot_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_FL); 
//         pinocchio::getFrameJacobian(model, data, FR_foot_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_FR);
//         pinocchio::getFrameJacobian(model, data, BL_foot_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_BL);
//         pinocchio::getFrameJacobian(model, data, BR_foot_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_BR);

//         // std::cout << "J_FL: " << J_FL << std::endl;
//         // std::cout << "J_FR: " << J_FR << std::endl;
//         // std::cout << "J_BL: " << J_BL << std::endl;
//         // std::cout << "J_BR: " << J_BR << std::endl;

//         J_total.setZero();
//         J_total.block(0, 0, 3, model.nq) = J_FL.template topRows<3>(); // extract position rows of jacobian
//         J_total.block(3, 0, 3, model.nq) = J_FR.template topRows<3>(); // extract position rows of jacobian
//         J_total.block(6, 0, 3, model.nq) = J_BL.template topRows<3>(); // extract position rows of jacobian
//         J_total.block(9, 0, 3, model.nq) = J_BR.template topRows<3>(); // extract position rows of jacobian

//         // std::cout << "J_total size: " << J_total.rows() << ", " << J_total.cols() << std::endl;

//         // std::cout << "J_total" << J_total << std::endl;

//         Eigen::MatrixXd pinv = J_total.completeOrthogonalDecomposition().pseudoInverse();

//         // std::cout << "pinv size: " << pinv.rows() << ", " << pinv.cols() << std::endl;
//         // std::cout << "pinv: " << pinv << std::endl;

//         // pinv size: 18 x 12
//         double alpha = 1.0; // + 1.0 * (1.0 - double(projection_iteration) / num_projection_iterations); // learning rate
//         Eigen::VectorXd temp_q = new_q - alpha * pinv * f_x;
//         new_q = temp_q;
//         // std::cout << "  projected q:     " << temp_q.transpose() << std::endl;

//         // NEED TO ENFORCE JOINT LIMITS
//         enforceJointLimits(model, new_q, torso_pose);
//         // std::cout << "  joint limited q: " << temp_q.transpose() << std::endl;


//         projection_iteration++;

//         // std::cout << "  projection_iteration: " << projection_iteration << std::endl;
//         // std::cout << "      FL_foot position: " << data.oMf[FL_foot_frame_id].translation().transpose() << std::endl;
//         // std::cout << "      FR_foot position: " << data.oMf[FR_foot_frame_id].translation().transpose() << std::endl;
//         // std::cout << "      BL_foot position: " << data.oMf[BL_foot_frame_id].translation().transpose() << std::endl;
//         // std::cout << "      BR_foot position: " << data.oMf[BR_foot_frame_id].translation().transpose() << std::endl;
//         // std::cout << "      error: " << x.norm() << std::endl;
//     }  

//     return foundTransition;
// }

ReachabilityAnalyzer::ReachabilityAnalyzer(const rclcpp::Node::SharedPtr& node,
                                            std::shared_ptr<switched_model::CustomQuadrupedInterface> & interface,
                                            std::shared_ptr<switched_model::CustomQuadrupedVisualizer> & visualizer)
{
    node_ = node;
    interface_ = interface;
    visualizer_ = visualizer;
    projectionPublisher = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/projections", 1);
    superquadricPublisher = node_->create_publisher<visualization_msgs::msg::Marker>("/superquadrics", 1);
    marker_counter = 0;
    feetColorMap_ = {ocs2::Color::blue, ocs2::Color::orange, ocs2::Color::yellow, ocs2::Color::purple};  // Colors for markers per feet
}

// LeggedRobotInterface & interface, 
// std::shared_ptr<LeggedRobotVisualizer> & leggedRobotVisualizer,
// PinocchioEndEffectorKinematics & endEffectorKinematics,
// const std::string & volumeFlag

void ReachabilityAnalyzer::runReachabilityAnalysis()
{
    // std::cout << "[runReachabilityAnalysis()]" << std::endl;
    // int num_projections = 1000;

    // const auto& model = interface.getPinocchioInterface().getModel();
    // auto& data = interface.getPinocchioInterface().getData();

    Eigen::VectorXd q = Eigen::VectorXd::Zero(CONFIG_DIM);
    // Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);

    Eigen::Vector3d torso_pose(0.0, 0.0, 0.0);
    Eigen::VectorXd defaultState = interface_->getInitialState();

    // set torso pose
    q[0] = torso_pose[0]; 
    q[1] = torso_pose[1]; 
    q[2] = torso_pose[2];
    q[3] = 0.0; 
    q[4] = 0.0; 
    q[5] = 0.0;

    // for (int i = 0; i < num_projections; i++)
    // {
        // std::cout << "projection " << i << std::endl;

    // set joint poses
    for (int j = 6; j < CONFIG_DIM; j++)
    {
        double min_posn = -1.0;
        double max_posn = -1.0;

        // if (volumeFlag == "full")
        // {
        min_posn = interface_->modelSettings().lowerJointLimits_[j];
        max_posn = interface_->modelSettings().upperJointLimits_[j];

        std::uniform_real_distribution<double> joint_distribution(min_posn, max_posn);

        // randomly sample with limits
        q[j] = joint_distribution(generator);            
        // } else if (volumeFlag == "conservative")
        // {
        //     if (j == 6 || j == 9 || j == 12 || j == 15) // hip
        //     {
        //         // Hip position limits: [-0.863, 0.863], default position: 0.0
        //         min_posn = -0.2; // -0.430;
        //         max_posn = 0.2; // 0.430;
        //     } else if (j == 7 || j == 10 || j == 13 || j == 16) // thigh
        //     {
        //         // Thigh position limits: [-0.686, 4.501], default position: 0.72
        //         min_posn = 0.5;
        //         max_posn = 1.0; // 1.44;
        //     } else if (j == 8 || j == 11 || j == 14 || j == 17) // calf
        //     {
        //         // Calf position limits: [-2.818, -0.888], default position: -1.44
        //         min_posn = -1.88;
        //         max_posn = -1.00;
        //     }

        //     std::uniform_real_distribution<double> joint_distribution(min_posn, max_posn);

        //     // randomly sample with limits
        //     q[j] = joint_distribution(generator);    
        // } else if (volumeFlag == "manual")
        // {
        //     if (j == 6)
        //         q[j] = FL_hip_manual_pos;
        //     else if (j == 7)
        //         q[j] = FL_thigh_manual_pos;
        //     else if (j == 8)
        //         q[j] = FL_calf_manual_pos;
        //     else if (j == 9)
        //         q[j] = FR_hip_manual_pos;
        //     else if (j == 10)
        //         q[j] = FR_thigh_manual_pos;
        //     else if (j == 11)
        //         q[j] = FR_calf_manual_pos;
        //     else if (j == 12)
        //         q[j] = BL_hip_manual_pos;
        //     else if (j == 13)
        //         q[j] = BL_thigh_manual_pos;
        //     else if (j == 14)
        //         q[j] = BL_calf_manual_pos;
        //     else if (j == 15)
        //         q[j] = BR_hip_manual_pos;
        //     else if (j == 16)
        //         q[j] = BR_thigh_manual_pos;
        //     else if (j == 17)
        //         q[j] = BR_calf_manual_pos;                
        // } else
        // {
        //     throw std::runtime_error("volume flag '" + volumeFlag + "' is not identified");
        // }
    }    

    // // update model based on current configuration
    // pinocchio::forwardKinematics(model, data, q, v);
    // pinocchio::computeJointJacobians(model, data);        
    // pinocchio::updateFramePlacements(model, data);

    publishEEPositions(q);       
    publishState(q);

    // // fit superquadrics to dataset
    // visualize3DSuperquadrics(torso_pose);

    return;
}

void ReachabilityAnalyzer::publishState(const Eigen::VectorXd & q)
{
    Eigen::VectorXd x = Eigen::VectorXd::Zero(q.size() + 6);
    x.block(6, 0, 18, 1) = q;

    double real_time_factor = 0.1;

    ocs2::SystemObservation sol;
    sol.state = x;
    sol.input = Eigen::VectorXd::Zero(q.size());
    const auto timeStamp = node_->get_clock()->now();

    visualizer_->publishObservation(timeStamp, sol);
    // rclcpp::Rate(real_time_factor * 1.0 / interface_.getRollout().settings().timeStep).sleep();
}

void ReachabilityAnalyzer::publishEEPositions(const Eigen::VectorXd & q)
{
    assert(q.size() == CONFIG_DIM && "q must have size 18");

    // Eigen::VectorXd x = Eigen::VectorXd::Zero(q.size() + 6);
    // x.block(6, 0, 18, 1) = q;

    visualization_msgs::msg::MarkerArray markerArray;
    // const auto feetPositions = endEffectorKinematics.getPosition(x);

    for (int leg_idx = 0; leg_idx < 4; leg_idx++)
    {
        const switched_model::base_coordinate_t basePose = q.head(6);
        const switched_model::joint_coordinate_t qJoints = q.tail(12);

        Eigen::Vector3d foot_posn_world = interface_->getKinematicModel().footPositionInOriginFrame(leg_idx, basePose, qJoints);

        // std::cout << "  footPosition: " << feetPositions[leg_idx] << std::endl;

        // prune configurations with potential for self-collision
        // if (leg_idx == 0) // FL
        // {
        //     // if (feetPositions[leg_idx][0] <= 0.0 || feetPositions[leg_idx][1] <= 0.0)
        //     //     continue;
        // } else
        // {
        //     continue;
        // }
        
        // if (leg_idx == 1) // FR
        // {
        //     if (feetPositions[leg_idx][0] <= 0.0 || feetPositions[leg_idx][1] >= 0.0)
        //         continue;
        // }
        
        // if (leg_idx == 2) // BL
        // {
        //     if (feetPositions[leg_idx][0] >= 0.0 || feetPositions[leg_idx][1] <= 0.0)
        //         continue;
        // }

        // if (leg_idx == 3) // BR
        // {
        //     if (feetPositions[leg_idx][0] >= 0.0 || feetPositions[leg_idx][1] >= 0.0)
        //         continue;
        // }

        // prune ee positions with z >= 0.0
        // if (feetPositions[leg_idx][2] >= 0.0)
        //     continue;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = node_->get_clock()->now();
        marker.ns = "projections";
        marker.id = marker_counter++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // std::cout << "  marker.id: " << marker.id << std::endl;

        marker.pose.position.x = foot_posn_world[0];
        marker.pose.position.y = foot_posn_world[1];
        marker.pose.position.z = foot_posn_world[2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // std::cout << "  marker.pose: " << marker.pose << std::endl;

        marker.scale.x = 0.01; 
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color = ocs2::getColor(feetColorMap_[leg_idx]);
        marker.color.a = 1.0;
        // marker.color.r = 0.0;
        // marker.color.g = 1.0;
        // marker.color.b = 0.0;
        // std::cout << "  pre-add" << std::endl;
        markerArray.markers.push_back(marker); // [marker.id] = 
        // std::cout << "  post-add" << std::endl;
    }
    projectionPublisher->publish(markerArray);
    // ros::Duration(0.1).sleep();
}

// int signum(double value)
// {
//     if (value < 0)
//         return -1;
//     else if (value == 0)
//         return 0;
//     else   
//         return 1;
// }   

// void ReachabilityAnalyzer::visualize3DSuperquadrics(const Eigen::Vector3d & p_torso)
// {
//     for (int leg_idx = 0; leg_idx < 4; leg_idx++)
//     {
//         visualize3DSuperquadric(leg_idx, p_torso);
//     }
// }

// void ReachabilityAnalyzer::visualize3DSuperquadric(const int & legIdx,
//                                                     const Eigen::Vector3d & p_torso)
// {
//     // std::cout << "                                          yaw: " << yaw << std::endl;
//     // std::cout << "                                          pitch: " << pitch << std::endl;
//     // std::cout << "                                          roll: " << roll << std::endl;
    
//     Eigen::Vector3d sqDims(sqDimX, sqDimY, sqDimZ);
//     Eigen::Vector3d sqCurvature(sqCurvX, sqCurvY, sqCurvZ);

//     Eigen::Vector3d sqCenter;
//     Eigen::Vector3d sqOrientationZyx;
//     if (legIdx == 0) // FL
//     {
//         sqCenter << x_offset_front, y_offset_left, z_offset;
//         sqOrientationZyx << yaw, pitch, roll;
//     } else
//     {
//         return;
//     }
    
//     // } else if (legIdx == 1) // FR
//     // {
//     //     sqCenter << x_offset_front, y_offset_right, z_offset;
//     //     sqOrientationZyx << yaw, pitch, -roll;
//     // } else if (legIdx == 2) // BL
//     // {
//     //     sqCenter << x_offset_back, y_offset_left, z_offset;
//     //     sqOrientationZyx << yaw, -pitch, roll;
//     // } else                  // BR
//     // {
//     //     sqCenter << x_offset_back, y_offset_right, z_offset;
//     //     sqOrientationZyx << yaw, -pitch, -roll;    
//     // }

//     Eigen::Quaterniond q = ocs2::getQuaternionFromEulerAnglesZyx(sqOrientationZyx);

//     visualization_msgs::msg::Marker marker;
//     marker.header.frame_id = "odom";
//     marker.header.stamp = ros::Time();
//     marker.ns = "superquadric";
//     marker.id = legIdx;
//     marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST; // visualization_msgs::msg::Marker::LINE_STRIP;
//     marker.action = visualization_msgs::msg::Marker::ADD;
//     marker.pose.position.x = p_torso[0];
//     marker.pose.position.y = p_torso[1];
//     marker.pose.position.z = p_torso[2];
//     marker.pose.orientation.x = q.x();
//     marker.pose.orientation.y = q.y();
//     marker.pose.orientation.z = q.z();
//     marker.pose.orientation.w = q.w();
//     marker.scale.x = 1.0; // 0.01;
//     marker.scale.y = 1.0;
//     marker.scale.z = 1.0;
//     marker.color = getColor(feetColorMap_[legIdx]);
//     marker.color.a = 0.8;

//     // Eigen::Vector3d epsilon(0.5, 0.5, 0.5);
//     // Eigen::Vector3d C(0.15, 0.175, 0.15);

//     // Eigen::Vector3d center(1.0, 0.0, 0.0);

//     int n = 60;

//     Eigen::VectorXd x(n * n);
//     Eigen::VectorXd y(n * n);
//     Eigen::VectorXd z(n * n);

//     double etamax = M_PI/2;
//     double etamin = -M_PI/2;
//     double wmax = M_PI;
//     double wmin = -M_PI;
//     double deta = (etamax-etamin)/n;
//     double dw = (wmax-wmin)/n;

//     for (int i = 0; i <= n; i++)
//     {
//         for (int j = 0; j <= n; j++)
//         {
//             geometry_msgs::Point p1;
//             double eta = etamin + (i-1) * deta;
//             double w   = wmin + (j-1) * dw;            
//             p1.x = sqCenter[0] + sqDims[0] * signum(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[0]) * signum(std::cos(w)) * std::pow(std::abs(std::cos(w)), 2.0 / sqCurvature[0]);
//             p1.y = sqCenter[1] + sqDims[1] * signum(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[1]) * signum(sin(w)) * std::pow(std::abs(sin(w)), 2.0 / sqCurvature[1]);
//             p1.z = sqCenter[2] + sqDims[2] * signum(sin(eta)) * std::pow(std::abs(sin(eta)), 2.0 / sqCurvature[2]);

//             geometry_msgs::Point p2;
//             eta = etamin + ((i+1)-1) * deta;
//             w = wmin + (j-1) * dw;            
//             p2.x = sqCenter[0] + sqDims[0] * signum(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[0]) * signum(std::cos(w)) * std::pow(std::abs(std::cos(w)), 2.0 / sqCurvature[0]);
//             p2.y = sqCenter[1] + sqDims[1] * signum(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[1]) * signum(sin(w)) * std::pow(std::abs(sin(w)), 2.0 / sqCurvature[1]);
//             p2.z = sqCenter[2] + sqDims[2] * signum(sin(eta)) * std::pow(std::abs(sin(eta)), 2.0 / sqCurvature[2]);

//             geometry_msgs::Point p3;
//             eta = etamin + (i-1) * deta;
//             w = wmin + ((j+1)-1) * dw;            
//             p3.x = sqCenter[0] + sqDims[0] * signum(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[0]) * signum(std::cos(w)) * std::pow(std::abs(std::cos(w)), 2.0 / sqCurvature[0]);
//             p3.y = sqCenter[1] + sqDims[1] * signum(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[1]) * signum(sin(w)) * std::pow(std::abs(sin(w)), 2.0 / sqCurvature[1]);
//             p3.z = sqCenter[2] + sqDims[2] * signum(sin(eta)) * std::pow(std::abs(sin(eta)), 2.0 / sqCurvature[2]);

//             geometry_msgs::Point p4;
//             eta = etamin + ((i+1)-1) * deta;
//             w = wmin + ((j+1)-1) * dw;            
//             p4.x = sqCenter[0] + sqDims[0] * signum(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[0]) * signum(std::cos(w)) * std::pow(std::abs(std::cos(w)), 2.0 / sqCurvature[0]);
//             p4.y = sqCenter[1] + sqDims[1] * signum(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[1]) * signum(sin(w)) * std::pow(std::abs(sin(w)), 2.0 / sqCurvature[1]);
//             p4.z = sqCenter[2] + sqDims[2] * signum(sin(eta)) * std::pow(std::abs(sin(eta)), 2.0 / sqCurvature[2]);    

//             marker.points.push_back(p3);
//             marker.points.push_back(p2);
//             marker.points.push_back(p1); 

//             marker.points.push_back(p2);
//             marker.points.push_back(p3);
//             marker.points.push_back(p4);             
//         }
//     }

//     geometry_msgs::Point p1;
//     double eta = etamin + n * deta;
//     double w   = wmin + n * dw;            
//     p1.x = sqCenter[0] + sqDims[0] * signum(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[0]) * signum(std::cos(w)) * std::pow(std::abs(std::cos(w)), 2.0 / sqCurvature[0]);
//     p1.y = sqCenter[1] + sqDims[1] * signum(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[1]) * signum(sin(w)) * std::pow(std::abs(sin(w)), 2.0 / sqCurvature[1]);
//     p1.z = sqCenter[2] + sqDims[2] * signum(sin(eta)) * std::pow(std::abs(sin(eta)), 2.0 / sqCurvature[2]);

//     geometry_msgs::Point p2;
//     eta = etamin + (0.0) * deta;
//     w = wmin + n * dw;            
//     p2.x = sqCenter[0] + sqDims[0] * signum(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[0]) * signum(std::cos(w)) * std::pow(std::abs(std::cos(w)), 2.0 / sqCurvature[0]);
//     p2.y = sqCenter[1] + sqDims[1] * signum(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[1]) * signum(sin(w)) * std::pow(std::abs(sin(w)), 2.0 / sqCurvature[1]);
//     p2.z = sqCenter[2] + sqDims[2] * signum(sin(eta)) * std::pow(std::abs(sin(eta)), 2.0 / sqCurvature[2]);

//     geometry_msgs::Point p3;
//     eta = etamin + n * deta;
//     w = wmin + (0.0) * dw;            
//     p3.x = sqCenter[0] + sqDims[0] * signum(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[0]) * signum(std::cos(w)) * std::pow(std::abs(std::cos(w)), 2.0 / sqCurvature[0]);
//     p3.y = sqCenter[1] + sqDims[1] * signum(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[1]) * signum(sin(w)) * std::pow(std::abs(sin(w)), 2.0 / sqCurvature[1]);
//     p3.z = sqCenter[2] + sqDims[2] * signum(sin(eta)) * std::pow(std::abs(sin(eta)), 2.0 / sqCurvature[2]);

//     geometry_msgs::Point p4;
//     eta = etamin + (0.0) * deta;
//     w = wmin + (0.0) * dw;            
//     p4.x = sqCenter[0] + sqDims[0] * signum(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[0]) * signum(std::cos(w)) * std::pow(std::abs(std::cos(w)), 2.0 / sqCurvature[0]);
//     p4.y = sqCenter[1] + sqDims[1] * signum(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[1]) * signum(sin(w)) * std::pow(std::abs(sin(w)), 2.0 / sqCurvature[1]);
//     p4.z = sqCenter[2] + sqDims[2] * signum(sin(eta)) * std::pow(std::abs(sin(eta)), 2.0 / sqCurvature[2]);    

//     marker.points.push_back(p3);
//     marker.points.push_back(p2);
//     marker.points.push_back(p1); 

//     marker.points.push_back(p2);
//     marker.points.push_back(p3);
//     marker.points.push_back(p4);     

//     superquadricPublisher.publish(marker);
// }

// void ReachabilityAnalyzer::visualizeSuperquadric(std::shared_ptr<LeggedRobotVisualizer> & leggedRobotVisualizer)
// {
//     visualization_msgs::msg::MarkerArray markerArray;

//     // order: FL, FR, BL, BR
//     std::vector<double> x0s = {x_offset_front, x_offset_front, x_offset_back, x_offset_back};
//     std::vector<double> y0s = {y_offset_left, y_offset_right, y_offset_left, y_offset_right};
    
//     for (int leg_idx = 0; leg_idx < 4; leg_idx++)
//     {

//         int num_points = 100;
//         double theta_min = -M_PI;
//         double theta_max = M_PI;
//         double delta_theta = (theta_max - theta_min) / num_points;

//         visualization_msgs::msg::Marker marker;
//         marker.header.frame_id = "odom";
//         marker.header.stamp = ros::Time();
//         marker.ns = "superquadric";
//         marker.id = leg_idx;
//         marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
//         marker.action = visualization_msgs::msg::Marker::ADD;

//         std::cout << "  marker.id: " << marker.id << std::endl;

//         marker.pose.position.x = 0.0;
//         marker.pose.position.y = 0.0;
//         marker.pose.position.z = 0.0;
//         marker.pose.orientation.x = 0.0;
//         marker.pose.orientation.y = 0.0;
//         marker.pose.orientation.z = 0.0;
//         marker.pose.orientation.w = 1.0;

//         std::cout << "  marker.pose: " << marker.pose << std::endl;

//         marker.scale.x = 0.01; 
//         marker.color = getColor(feetColorMap_[leg_idx]);
//         marker.color.a = 1.0;     

//         for (int i = 0; i <= num_points; i++)
//         {
//             double theta = theta_min + (i) * delta_theta;
            
//             double x = x0s[leg_idx] + sqDimX * signum(std::cos(theta)) * std::pow(std::abs(std::cos(theta)), sqCurvX);
//             double y = y0s[leg_idx] + sqDimY * signum(std::sin(theta)) * std::pow(std::abs(std::sin(theta)), sqCurvY);

//             geometry_msgs::Point p;
//             p.x = x;
//             p.y = y;
//             p.z = 0.0;
//             marker.points.push_back(p);
//         }

//         markerArray.markers.push_back(marker);
//     }

//     superquadricPublisher.publish(markerArray);
//     // how to plot?
// }

// }  // namespace quadruped
// }  // namespace ocs2