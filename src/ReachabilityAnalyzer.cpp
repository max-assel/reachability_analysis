#include <pinocchio/fwd.hpp> // NEED TO KEEP THIS AT TOP

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/model.hpp"

#include <reachability_analysis/ReachabilityAnalyzer.h>

namespace ocs2 {
namespace quadruped {


void enforceJointLimits(const pinocchio::Model & model, Eigen::VectorXd & q, Eigen::Vector3d torso_pose)
{
    // std::cout << "  model.lowerPositionLimit: " << model.lowerPositionLimit.transpose() << std::endl;
    // std::cout << "  model.upperPositionLimit: " << model.upperPositionLimit.transpose() << std::endl;

    // need to artificially set z height constraint

    for (int i = 0; i < model.nq; i++) // 18
    {
        if (i >= 0 && i < 3) // robot torso position
        {
            q[0] = torso_pose[0];
            q[1] = torso_pose[1];
            q[2] = torso_pose[2];
            /*
            if (i == 2)
            {
                // robot z position
                double z_min = 0.25;
                double z_max = 0.35;
                q[i] = std::max(z_min, std::min(q[i], z_max));
            }
            */
            
            
        } else if (i >= 3 && i < 6) // robot torso orientation
        {
            q[3] = 0.0; 
            q[4] = 0.0; 
            q[5] = 0.0;
        } else // joint angles
        {
            // unwind
            while (q[i] < -2.0 * M_PI) 
                q[i] += 2.0 * M_PI;

            while (q[i] > 2.0 * M_PI) 
                q[i] -= 2.0 * M_PI;         

            // clamp with limits
            q[i] = std::max(model.lowerPositionLimit[i], std::min(q[i], model.upperPositionLimit[i]));
        }
    }
}


bool ReachabilityAnalyzer::IKProjection(Eigen::VectorXd & new_q,
                    const Eigen::VectorXd & v,
                    Eigen::Vector3d torso_pose,
                    LeggedRobotInterface & interface,
                    std::shared_ptr<LeggedRobotVisualizer> & leggedRobotVisualizer,
                    bool final)
{
    // std::cout << "[IKProjection]" << std::endl;
    const auto& model = interface.getPinocchioInterface().getModel();
    auto& data = interface.getPinocchioInterface().getData();

   
    Eigen::MatrixXd J_FL, J_FR, J_BL, J_BR;
    Eigen::MatrixXd J_total;

    J_FL = pinocchio::Data::Matrix6x::Zero(6, 18);
    J_FR = pinocchio::Data::Matrix6x::Zero(6, 18);
    J_BL = pinocchio::Data::Matrix6x::Zero(6, 18);
    J_BR = pinocchio::Data::Matrix6x::Zero(6, 18);
    J_total = Eigen::MatrixXd::Zero(12, 18); 

    pinocchio::FrameIndex FL_foot_frame_id = model.getBodyId("FL_foot");
    pinocchio::FrameIndex FR_foot_frame_id = model.getBodyId("FR_foot");
    pinocchio::FrameIndex BL_foot_frame_id = model.getBodyId("RL_foot");
    pinocchio::FrameIndex BR_foot_frame_id = model.getBodyId("RR_foot");     

    bool foundTransition = false;
    double epsilon = 0.005;
    int num_projection_iterations = 100;

    // std::cout << "  beginning q: " << new_q.transpose() << std::endl;

    // project into contact
    for (int projection_iteration = 0; projection_iteration < num_projection_iterations; projection_iteration++)
    {
        // std::cout << "  projection " << projection_iteration << std::endl;
        // publishProjection(interface, new_q, leggedRobotVisualizer);

        // update model based on current configuration
        pinocchio::forwardKinematics(model, data, new_q, v);
        pinocchio::computeJointJacobians(model, data);        
        pinocchio::updateFramePlacements(model, data);

        // calculate target footholds
        Eigen::Vector3d FL_target_position, FR_target_position, BL_target_position, BR_target_position;
        if (final)
        {
            FL_target_position = Eigen::Vector3d(0.175, 0.20, 0.0);
            FR_target_position = Eigen::Vector3d(0.175, -0.20, 0.0);
            BL_target_position = Eigen::Vector3d(-0.25, 0.20, 0.0);
            BR_target_position = Eigen::Vector3d(-0.25, -0.20, 0.0);            
        } else
        {
            FL_target_position = data.oMf[FL_foot_frame_id].translation();
            FL_target_position[2] = 0.0;
            FR_target_position = data.oMf[FR_foot_frame_id].translation();
            FR_target_position[2] = 0.0;
            BL_target_position = data.oMf[BL_foot_frame_id].translation();
            BL_target_position[2] = 0.0;
            BR_target_position = data.oMf[BR_foot_frame_id].translation();
            BR_target_position[2] = 0.0;
        }
        

        // std::cout << "      FL_target_position: " << FL_target_position.transpose() << std::endl;
        // std::cout << "      FR_target_position: " << FR_target_position.transpose() << std::endl;
        // std::cout << "      BL_target_position: " << BL_target_position.transpose() << std::endl;
        // std::cout << "      BR_target_position: " << BR_target_position.transpose() << std::endl;

        Eigen::VectorXd f_x(12); // 3 Dof per foot, constraining position
        f_x << (data.oMf[FL_foot_frame_id].translation() - FL_target_position), 
               (data.oMf[FR_foot_frame_id].translation() - FR_target_position),
               (data.oMf[BL_foot_frame_id].translation() - BL_target_position), 
               (data.oMf[BR_foot_frame_id].translation() - BR_target_position);
        // std::cout << "      error: " << f_x.norm() << std::endl;

        foundTransition = (f_x.norm() < epsilon);
        if (foundTransition)
            break;

        // calculate Jacobians
        J_FL.setZero(); J_FR.setZero(); J_BL.setZero(); J_BR.setZero();

        pinocchio::getFrameJacobian(model, data, FL_foot_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_FL); 
        pinocchio::getFrameJacobian(model, data, FR_foot_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_FR);
        pinocchio::getFrameJacobian(model, data, BL_foot_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_BL);
        pinocchio::getFrameJacobian(model, data, BR_foot_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_BR);

        // std::cout << "J_FL: " << J_FL << std::endl;
        // std::cout << "J_FR: " << J_FR << std::endl;
        // std::cout << "J_BL: " << J_BL << std::endl;
        // std::cout << "J_BR: " << J_BR << std::endl;

        J_total.setZero();
        J_total.block(0, 0, 3, model.nq) = J_FL.template topRows<3>(); // extract position rows of jacobian
        J_total.block(3, 0, 3, model.nq) = J_FR.template topRows<3>(); // extract position rows of jacobian
        J_total.block(6, 0, 3, model.nq) = J_BL.template topRows<3>(); // extract position rows of jacobian
        J_total.block(9, 0, 3, model.nq) = J_BR.template topRows<3>(); // extract position rows of jacobian

        // std::cout << "J_total size: " << J_total.rows() << ", " << J_total.cols() << std::endl;

        // std::cout << "J_total" << J_total << std::endl;

        Eigen::MatrixXd pinv = J_total.completeOrthogonalDecomposition().pseudoInverse();

        // std::cout << "pinv size: " << pinv.rows() << ", " << pinv.cols() << std::endl;
        // std::cout << "pinv: " << pinv << std::endl;

        // pinv size: 18 x 12
        double alpha = 1.0; // + 1.0 * (1.0 - double(projection_iteration) / num_projection_iterations); // learning rate
        Eigen::VectorXd temp_q = new_q - alpha * pinv * f_x;
        new_q = temp_q;
        // std::cout << "  projected q:     " << temp_q.transpose() << std::endl;

        // NEED TO ENFORCE JOINT LIMITS
        enforceJointLimits(model, new_q, torso_pose);
        // std::cout << "  joint limited q: " << temp_q.transpose() << std::endl;


        projection_iteration++;

        // std::cout << "  projection_iteration: " << projection_iteration << std::endl;
        // std::cout << "      FL_foot position: " << data.oMf[FL_foot_frame_id].translation().transpose() << std::endl;
        // std::cout << "      FR_foot position: " << data.oMf[FR_foot_frame_id].translation().transpose() << std::endl;
        // std::cout << "      BL_foot position: " << data.oMf[BL_foot_frame_id].translation().transpose() << std::endl;
        // std::cout << "      BR_foot position: " << data.oMf[BR_foot_frame_id].translation().transpose() << std::endl;
        // std::cout << "      error: " << x.norm() << std::endl;
    }  

    return foundTransition;
}

ReachabilityAnalyzer::ReachabilityAnalyzer(ros::NodeHandle& nodeHandle)
{
    projectionPublisher = nodeHandle.advertise<visualization_msgs::MarkerArray>("/projections", 1);
    superquadricPublisher = nodeHandle.advertise<visualization_msgs::MarkerArray>("/superquadrics", 1);
    marker_counter = 0;
    feetColorMap_ = {Color::blue, Color::orange, Color::yellow, Color::purple};  // Colors for markers per feet
}

void ReachabilityAnalyzer::runReachabilityAnalysis(LeggedRobotInterface & interface, 
                                                    std::shared_ptr<LeggedRobotVisualizer> & leggedRobotVisualizer,
                                                    PinocchioEndEffectorKinematics & endEffectorKinematics)
{
    int num_projections = 10000;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(18);

    int num_sample_iterations = 25;

    Eigen::Vector3d torso_pose(0.0, 0.0, 0.30);
    Eigen::VectorXd defaultState = interface.getInitialState();

    std::default_random_engine generator;
    std::uniform_real_distribution<double> joint_distribution(-M_PI/8, M_PI/8);
    
    bool foundTransition = false;

    for (int i = 0; i < num_projections; i++)
    {
        // std::cout << "projection " << i << std::endl;

        Eigen::VectorXd new_q = q;

        new_q[0] = torso_pose[0]; new_q[1] = torso_pose[1]; new_q[2] = torso_pose[2];
        new_q[3] = 0.0; new_q[4] = 0.0; new_q[5] = 0.0;

        new_q.block(6, 0, 12, 1) = defaultState.block(12, 0, 12, 1);

        // randomly sample leg joints

        for (int j = 0; j < 12; j++)
            new_q[6 + j] += joint_distribution(generator);

        foundTransition = IKProjection(new_q, v, torso_pose, interface, leggedRobotVisualizer, false);

        if (foundTransition)
            publishContact(new_q, endEffectorKinematics);       
        visualizeSuperquadric(leggedRobotVisualizer);
    }

    Eigen::VectorXd new_q = q;

    new_q[0] = torso_pose[0]; new_q[1] = torso_pose[1]; new_q[2] = torso_pose[2];
    new_q[3] = 0.0; new_q[4] = 0.0; new_q[5] = 0.0;

    new_q.block(6, 0, 12, 1) = defaultState.block(12, 0, 12, 1);

    foundTransition = IKProjection(new_q, v, torso_pose, interface, leggedRobotVisualizer, true);

    publishProjection(interface, new_q, leggedRobotVisualizer);

}

void ReachabilityAnalyzer::publishProjection(LeggedRobotInterface & interface, Eigen::VectorXd & q, 
                                             std::shared_ptr<LeggedRobotVisualizer> & leggedRobotVisualizer)
{
    Eigen::VectorXd x = Eigen::VectorXd::Zero(q.size() + 6);
    x.block(6, 0, 18, 1) = q;

    double real_time_factor = 0.1;

    SystemObservation sol;
    sol.state = x;
    sol.input = x;
    const auto timeStamp = ros::Time::now();

    leggedRobotVisualizer->publishObservation(timeStamp, sol);
    ros::Rate(real_time_factor * 1.0 / interface.getRollout().settings().timeStep).sleep();
}

void ReachabilityAnalyzer::publishContact(Eigen::VectorXd & q, 
                                            PinocchioEndEffectorKinematics & endEffectorKinematics)
{
    Eigen::VectorXd x = Eigen::VectorXd::Zero(q.size() + 6);
    x.block(6, 0, 18, 1) = q;

    visualization_msgs::MarkerArray markerArray;
    const auto feetPositions = endEffectorKinematics.getPosition(x);

    for (int leg_idx = 0; leg_idx < 4; leg_idx++)
    {
        // std::cout << "  footPosition: " << feetPositions[leg_idx] << std::endl;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time();
        marker.ns = "projections";
        marker.id = marker_counter++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        // std::cout << "  marker.id: " << marker.id << std::endl;

        marker.pose.position.x = feetPositions[leg_idx][0];
        marker.pose.position.y = feetPositions[leg_idx][1];
        marker.pose.position.z = feetPositions[leg_idx][2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // std::cout << "  marker.pose: " << marker.pose << std::endl;

        marker.scale.x = 0.01; 
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color = getColor(feetColorMap_[leg_idx]);
        marker.color.a = 1.0;
        // marker.color.r = 0.0;
        // marker.color.g = 1.0;
        // marker.color.b = 0.0;
        // std::cout << "  pre-add" << std::endl;
        markerArray.markers.push_back(marker); // [marker.id] = 
        // std::cout << "  post-add" << std::endl;
    }
    projectionPublisher.publish(markerArray);
    ros::Duration(0.1).sleep();
}

int signum(double value)
{
    if (value < 0)
        return -1;
    else if (value == 0)
        return 0;
    else   
        return 1;
}   

void ReachabilityAnalyzer::visualizeSuperquadric(std::shared_ptr<LeggedRobotVisualizer> & leggedRobotVisualizer)
{
    visualization_msgs::MarkerArray markerArray;

    // order: FL, FR, BL, BR
    std::vector<double> x0s = {0.175, 0.175, -0.25, -0.25};
    std::vector<double> y0s = {0.20, -0.20, 0.20, -0.20};

    double a = 0.5;
    double b = 0.5;

    double A = 0.175;
    double B = 0.15;

    for (int leg_idx = 0; leg_idx < 4; leg_idx++)
    {

        int num_points = 100;
        double theta_min = -M_PI;
        double theta_max = M_PI;
        double delta_theta = (theta_max - theta_min) / num_points;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time();
        marker.ns = "superquadric";
        marker.id = leg_idx;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;

        // std::cout << "  marker.id: " << marker.id << std::endl;

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // std::cout << "  marker.pose: " << marker.pose << std::endl;

        marker.scale.x = 0.01; 
        marker.color = getColor(feetColorMap_[leg_idx]);
        marker.color.a = 1.0;     

        for (int i = 0; i <= num_points; i++)
        {
            double theta = theta_min + (i) * delta_theta;
            
            double x = x0s[leg_idx] + A * signum(std::cos(theta)) * std::pow(std::abs(std::cos(theta)), a);
            double y = y0s[leg_idx] + B * signum(std::sin(theta)) * std::pow(std::abs(std::sin(theta)), b);

            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = 0.0;
            marker.points.push_back(p);
        }

        markerArray.markers.push_back(marker);
    }

    superquadricPublisher.publish(markerArray);
    // how to plot?
}

}  // namespace quadruped
}  // namespace ocs2