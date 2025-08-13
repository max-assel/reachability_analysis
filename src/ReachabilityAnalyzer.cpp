// #include <pinocchio/fwd.hpp> // NEED TO KEEP THIS AT TOP

// #include "pinocchio/algorithm/frames.hpp"
// #include "pinocchio/algorithm/kinematics.hpp"
// #include "pinocchio/algorithm/model.hpp"

// #include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <reachability_analysis/ReachabilityAnalyzer.h>

ReachabilityAnalyzer::ReachabilityAnalyzer(const rclcpp::Node::SharedPtr& node,
                                            std::shared_ptr<switched_model::CustomQuadrupedInterface> & interface,
                                            std::shared_ptr<switched_model::CustomQuadrupedVisualizer> & visualizer)
{
    node_ = node;
    interface_ = interface;
    visualizer_ = visualizer;
    FLCloudPublisher = node_->create_publisher<visualization_msgs::msg::Marker>("/FLPointCloud", 1);
    FRCloudPublisher = node_->create_publisher<visualization_msgs::msg::Marker>("/FRPointCloud", 1);
    BLCloudPublisher = node_->create_publisher<visualization_msgs::msg::Marker>("/BLPointCloud", 1);
    BRCloudPublisher = node_->create_publisher<visualization_msgs::msg::Marker>("/BRPointCloud", 1);
    FLSuperquadricPublisher = node_->create_publisher<visualization_msgs::msg::Marker>("/FLSuperquadrics", 1);
    FRSuperquadricPublisher = node_->create_publisher<visualization_msgs::msg::Marker>("/FRSuperquadrics", 1);
    BLSuperquadricPublisher = node_->create_publisher<visualization_msgs::msg::Marker>("/BLSuperquadrics", 1);
    BRSuperquadricPublisher = node_->create_publisher<visualization_msgs::msg::Marker>("/BRSuperquadrics", 1);

    marker_counter = 0;
    feetColorMap_ = {ocs2::Color::blue, ocs2::Color::orange, ocs2::Color::yellow, ocs2::Color::purple};  // Colors for markers per feet


    node->declare_parameter("sqDimX", 0.0);
    node->declare_parameter("sqDimY", 0.0);
    node->declare_parameter("sqDimZ", 0.0);
    node->declare_parameter("sqCurvX", 2.0);
    node->declare_parameter("sqCurvY", 2.0);
    node->declare_parameter("sqCurvZ", 2.0);
    node->declare_parameter("x_offset", 0.20);
    node->declare_parameter("y_offset", 0.025);
    node->declare_parameter("z_offset", -0.25);
    node->declare_parameter("roll", 0.30);
    node->declare_parameter("pitch", 0.0);
    node->declare_parameter("yaw", 0.0);

    node->get_parameter("sqDimX", sqDimX);
    node->get_parameter("sqDimY", sqDimY);
    node->get_parameter("sqDimZ", sqDimZ);
    node->get_parameter("sqCurvX", sqCurvX);
    node->get_parameter("sqCurvY", sqCurvY);
    node->get_parameter("sqCurvZ", sqCurvZ);
    node->get_parameter("x_offset", x_offset);
    node->get_parameter("y_offset", y_offset);
    node->get_parameter("z_offset", z_offset);
    node->get_parameter("roll", roll);
    node->get_parameter("pitch", pitch);
    node->get_parameter("yaw", yaw);

    callback_handle_ = node_->add_on_set_parameters_callback(
        std::bind(&ReachabilityAnalyzer::parametersCallback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult ReachabilityAnalyzer::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &param: parameters)
    {
        if (param.get_name() == "sqDimX")
        {
            sqDimX = param.get_value<double>();
        } else if (param.get_name() == "sqDimY")
        {
            sqDimY = param.get_value<double>();
        } else if (param.get_name() == "sqDimZ")
        {
            sqDimZ = param.get_value<double>();
        } else if (param.get_name() == "sqCurvX")
        {
            sqCurvX = param.get_value<double>();
        } else if (param.get_name() == "sqCurvY")
        {
            sqCurvY = param.get_value<double>();
        } else if (param.get_name() == "sqCurvZ")
        {
            sqCurvZ = param.get_value<double>();
        } else if (param.get_name() == "x_offset")
        {
            x_offset = param.get_value<double>();
        } else if (param.get_name() == "y_offset")
        {
            y_offset = param.get_value<double>();
        } else if (param.get_name() == "z_offset")
        {
            z_offset = param.get_value<double>();
        } else if (param.get_name() == "roll")
        {
            roll = param.get_value<double>();
        } else if (param.get_name() == "pitch")
        {
            pitch = param.get_value<double>();
        } else if (param.get_name() == "yaw")
        {
            yaw = param.get_value<double>();
        }
    }
 
    return result;
}

void ReachabilityAnalyzer::runReachabilityAnalysis(const rclcpp::Time & timeStamp)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[runReachabilityAnalysis()]");
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
    }    


    publishEEPositions(q);       
    publishState(q, timeStamp);

    // fit superquadrics to dataset
    visualize3DSuperquadrics(torso_pose);

    return;
}

void ReachabilityAnalyzer::publishState(const Eigen::VectorXd & q, const rclcpp::Time & timeStamp)
{
    // RCLCPP_INFO_STREAM(node_->get_logger(), "[publishState()]");
    Eigen::VectorXd x = Eigen::VectorXd::Zero(q.size() + 6);
    x.block(6, 0, 18, 1) = q;

    double real_time_factor = 0.1;

    ocs2::SystemObservation sol;
    sol.state = x;
    sol.input = Eigen::VectorXd::Zero(q.size());
    // rclcpp::Time timeStamp = node_->get_clock()->now();

    // RCLCPP_INFO_STREAM(node_->get_logger(), "  timeStamp: " << timeStamp.seconds() << " seconds");

    visualizer_->publishObservation(timeStamp, sol);
}

void ReachabilityAnalyzer::publishEEPositions(const Eigen::VectorXd & q)
{
    assert(q.size() == CONFIG_DIM && "q must have size 18");

    // visualization_msgs::msg::MarkerArray markerArray;
    // const auto feetPositions = endEffectorKinematics.getPosition(x);

    for (int leg_idx = 0; leg_idx < 4; leg_idx++)
    {
        const switched_model::base_coordinate_t basePose = q.head(6);
        const switched_model::joint_coordinate_t qJoints = q.tail(12);

        Eigen::Vector3d foot_posn_world = interface_->getKinematicModel().footPositionInOriginFrame(leg_idx, basePose, qJoints);

        // std::cout << "  footPosition: " << feetPositions[leg_idx] << std::endl;

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

        if (leg_idx == FL)
        {
            FLCloudPublisher->publish(marker);
        } else if (leg_idx == FR)
        {
            FRCloudPublisher->publish(marker);
        } else if (leg_idx == BL)
        {
            BLCloudPublisher->publish(marker);
        } else if (leg_idx == BR)
        {
            BRCloudPublisher->publish(marker);
        } else
        {
            throw std::invalid_argument("Invalid leg index.");
        }

        // markerArray.markers.push_back(marker); // [marker.id] = 
        // std::cout << "  post-add" << std::endl;
    }
    // projectionPublisher->publish(markerArray);
    // ros::Duration(0.1).sleep();
}

int sign(const double & input)
{
    if (input > 0)
        return 1;
    else if (input < 0)
        return -1;
    else
        return 0;
}

void ReachabilityAnalyzer::visualize3DSuperquadrics(const Eigen::Vector3d & p_torso)
{
    for (int leg_idx = 0; leg_idx < 4; leg_idx++)
    {
        visualize3DSuperquadric(leg_idx, p_torso);
    }
}

void ReachabilityAnalyzer::visualize3DSuperquadric(const int & legIdx,
                                                    const Eigen::Vector3d & p_torso)
{
    // std::cout << "                                          yaw: " << yaw << std::endl;
    // std::cout << "                                          pitch: " << pitch << std::endl;
    // std::cout << "                                          roll: " << roll << std::endl;
    
    Eigen::Vector3d sqDims(sqDimX, sqDimY, sqDimZ);
    Eigen::Vector3d sqCurvature(sqCurvX, sqCurvY, sqCurvZ);

    Eigen::Vector3d sqCenter;
    Eigen::Vector3d sqOrientation;
    if (legIdx == FL)
    {
        sqCenter << x_offset, y_offset, z_offset;
        sqOrientation << roll, pitch, yaw;
    } else if (legIdx == FR)
    {
        sqCenter << x_offset, -y_offset, z_offset;
        sqOrientation << -roll, pitch, yaw;        
    } else if (legIdx == BL)
    {
        sqCenter << -x_offset, y_offset, z_offset;
        sqOrientation << roll, -pitch, yaw;
    } else if (legIdx == BR)
    {
        sqCenter << -x_offset, -y_offset, z_offset;
        sqOrientation << -roll, -pitch, yaw;
    } else    
    {
        throw std::invalid_argument("Invalid leg index.");
    }

    Eigen::Quaterniond q = ocs2::getQuaternionFromEulerAnglesXyz(sqOrientation);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = node_->get_clock()->now();
    marker.ns = "superquadric";
    marker.id = legIdx;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST; // visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = p_torso[0];
    marker.pose.position.y = p_torso[1];
    marker.pose.position.z = p_torso[2];
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = 1.0; // 0.01;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color = ocs2::getColor(feetColorMap_[legIdx]);
    marker.color.a = 0.8;

    // Eigen::Vector3d epsilon(0.5, 0.5, 0.5);
    // Eigen::Vector3d C(0.15, 0.175, 0.15);

    // Eigen::Vector3d center(1.0, 0.0, 0.0);

    int n = 60;

    Eigen::VectorXd x(n * n);
    Eigen::VectorXd y(n * n);
    Eigen::VectorXd z(n * n);

    double etamax = M_PI/2;
    double etamin = -M_PI/2;
    double wmax = M_PI;
    double wmin = -M_PI;
    double deta = (etamax-etamin)/n;
    double dw = (wmax-wmin)/n;

    for (int i = 0; i <= n; i++)
    {
        for (int j = 0; j <= n; j++)
        {
            geometry_msgs::msg::Point p1;
            double eta = etamin + (i-1) * deta;
            double w   = wmin + (j-1) * dw;            
            p1.x = sqCenter[0] + sqDims[0] * sign(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[0]) * sign(std::cos(w)) * std::pow(std::abs(std::cos(w)), 2.0 / sqCurvature[0]);
            p1.y = sqCenter[1] + sqDims[1] * sign(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[1]) * sign(sin(w)) * std::pow(std::abs(sin(w)), 2.0 / sqCurvature[1]);
            p1.z = sqCenter[2] + sqDims[2] * sign(sin(eta)) * std::pow(std::abs(sin(eta)), 2.0 / sqCurvature[2]);

            geometry_msgs::msg::Point p2;
            eta = etamin + ((i+1)-1) * deta;
            w = wmin + (j-1) * dw;            
            p2.x = sqCenter[0] + sqDims[0] * sign(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[0]) * sign(std::cos(w)) * std::pow(std::abs(std::cos(w)), 2.0 / sqCurvature[0]);
            p2.y = sqCenter[1] + sqDims[1] * sign(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[1]) * sign(sin(w)) * std::pow(std::abs(sin(w)), 2.0 / sqCurvature[1]);
            p2.z = sqCenter[2] + sqDims[2] * sign(sin(eta)) * std::pow(std::abs(sin(eta)), 2.0 / sqCurvature[2]);

            geometry_msgs::msg::Point p3;
            eta = etamin + (i-1) * deta;
            w = wmin + ((j+1)-1) * dw;            
            p3.x = sqCenter[0] + sqDims[0] * sign(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[0]) * sign(std::cos(w)) * std::pow(std::abs(std::cos(w)), 2.0 / sqCurvature[0]);
            p3.y = sqCenter[1] + sqDims[1] * sign(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[1]) * sign(sin(w)) * std::pow(std::abs(sin(w)), 2.0 / sqCurvature[1]);
            p3.z = sqCenter[2] + sqDims[2] * sign(sin(eta)) * std::pow(std::abs(sin(eta)), 2.0 / sqCurvature[2]);

            geometry_msgs::msg::Point p4;
            eta = etamin + ((i+1)-1) * deta;
            w = wmin + ((j+1)-1) * dw;            
            p4.x = sqCenter[0] + sqDims[0] * sign(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[0]) * sign(std::cos(w)) * std::pow(std::abs(std::cos(w)), 2.0 / sqCurvature[0]);
            p4.y = sqCenter[1] + sqDims[1] * sign(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[1]) * sign(sin(w)) * std::pow(std::abs(sin(w)), 2.0 / sqCurvature[1]);
            p4.z = sqCenter[2] + sqDims[2] * sign(sin(eta)) * std::pow(std::abs(sin(eta)), 2.0 / sqCurvature[2]);    

            marker.points.push_back(p3);
            marker.points.push_back(p2);
            marker.points.push_back(p1); 

            marker.points.push_back(p2);
            marker.points.push_back(p3);
            marker.points.push_back(p4);             
        }
    }

    geometry_msgs::msg::Point p1;
    double eta = etamin + n * deta;
    double w   = wmin + n * dw;            
    p1.x = sqCenter[0] + sqDims[0] * sign(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[0]) * sign(std::cos(w)) * std::pow(std::abs(std::cos(w)), 2.0 / sqCurvature[0]);
    p1.y = sqCenter[1] + sqDims[1] * sign(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[1]) * sign(sin(w)) * std::pow(std::abs(sin(w)), 2.0 / sqCurvature[1]);
    p1.z = sqCenter[2] + sqDims[2] * sign(sin(eta)) * std::pow(std::abs(sin(eta)), 2.0 / sqCurvature[2]);

    geometry_msgs::msg::Point p2;
    eta = etamin + (0.0) * deta;
    w = wmin + n * dw;            
    p2.x = sqCenter[0] + sqDims[0] * sign(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[0]) * sign(std::cos(w)) * std::pow(std::abs(std::cos(w)), 2.0 / sqCurvature[0]);
    p2.y = sqCenter[1] + sqDims[1] * sign(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[1]) * sign(sin(w)) * std::pow(std::abs(sin(w)), 2.0 / sqCurvature[1]);
    p2.z = sqCenter[2] + sqDims[2] * sign(sin(eta)) * std::pow(std::abs(sin(eta)), 2.0 / sqCurvature[2]);

    geometry_msgs::msg::Point p3;
    eta = etamin + n * deta;
    w = wmin + (0.0) * dw;            
    p3.x = sqCenter[0] + sqDims[0] * sign(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[0]) * sign(std::cos(w)) * std::pow(std::abs(std::cos(w)), 2.0 / sqCurvature[0]);
    p3.y = sqCenter[1] + sqDims[1] * sign(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[1]) * sign(sin(w)) * std::pow(std::abs(sin(w)), 2.0 / sqCurvature[1]);
    p3.z = sqCenter[2] + sqDims[2] * sign(sin(eta)) * std::pow(std::abs(sin(eta)), 2.0 / sqCurvature[2]);

    geometry_msgs::msg::Point p4;
    eta = etamin + (0.0) * deta;
    w = wmin + (0.0) * dw;            
    p4.x = sqCenter[0] + sqDims[0] * sign(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[0]) * sign(std::cos(w)) * std::pow(std::abs(std::cos(w)), 2.0 / sqCurvature[0]);
    p4.y = sqCenter[1] + sqDims[1] * sign(std::cos(eta)) * std::pow(std::abs(std::cos(eta)), 2.0 / sqCurvature[1]) * sign(sin(w)) * std::pow(std::abs(sin(w)), 2.0 / sqCurvature[1]);
    p4.z = sqCenter[2] + sqDims[2] * sign(sin(eta)) * std::pow(std::abs(sin(eta)), 2.0 / sqCurvature[2]);    

    marker.points.push_back(p3);
    marker.points.push_back(p2);
    marker.points.push_back(p1); 

    marker.points.push_back(p2);
    marker.points.push_back(p3);
    marker.points.push_back(p4);     

    if (legIdx == FL)
    {
        FLSuperquadricPublisher->publish(marker);
    } else if (legIdx == FR)
    {
        FRSuperquadricPublisher->publish(marker);
    } else if (legIdx == BL)
    {
        BLSuperquadricPublisher->publish(marker);
    } else if (legIdx == BR)
    {
        BRSuperquadricPublisher->publish(marker);
    } else
    {
        throw std::invalid_argument("Invalid leg index.");
    }

    // superquadricPublisher->publish(marker);
}

// }  // namespace quadruped
// }  // namespace ocs2