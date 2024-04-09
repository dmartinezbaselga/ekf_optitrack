#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <tf/tf.h>
#include <ekf/PoseVel.h>
#include <ekf/PoseVelArray.h>

class EKFVelocity
{
private:
    Eigen::MatrixXd Q, R, P, H, K, F;
    Eigen::Vector4d state; // (x, y, vx, vy)
    ros::Time last_time, current_time;
    ros::Subscriber pose_sub;
    double x_new, y_new, theta_new;
    bool initialized;
    std::string id_;

public:
    EKFVelocity(const std::string id);
    ~EKFVelocity();
    Eigen::Vector4d predictState(double dt);
    Eigen::MatrixXd calculateKalmanGain(Eigen::MatrixXd H, Eigen::MatrixXd Q, Eigen::MatrixXd R);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    ekf::PoseVel getFiltered();
};

EKFVelocity::EKFVelocity(const std::string id)
{
    ROS_INFO_STREAM("Init constructor " << initialized);
    this->initialized = false;
    // Initialize state and covariance
    state << 0, 0, 0, 0; // Initial position and zero velocity
    this->P = Eigen::MatrixXd::Zero(4, 4);
    P(0, 0) = 0.001; // Variance in x position
    P(1, 1) = 0.001; // Variance in y position
    P(2, 2) = 0.01;  // Variance in x velocity
    P(3, 3) = 0.01;  // Variance in y velocity

    this->Q = Eigen::MatrixXd::Zero(4, 4);
    Q(0, 0) = 0.01; // Variance in x position
    Q(1, 1) = 0.01; // Variance in y position
    Q(2, 2) = 0.1;  // Variance in x velocity
    Q(3, 3) = 0.1;  // Variance in y velocity

    id_ = id;

    this->R = Eigen::MatrixXd::Zero(2, 2);
    R(0, 0) = 0.0001; // Variance in measured x position
    R(1, 1) = 0.0001; // Variance in measured y position

    // Measurement matrix (assuming constant velocity model)
    this->H = Eigen::MatrixXd::Zero(2, 4);
    H << 1, 0, 0, 0,
        0, 1, 0, 0;

    this->F = Eigen::MatrixXd::Zero(4, 4);
    
    ROS_INFO_STREAM("Finished constructor");
}

// Function to perform system model prediction
Eigen::Vector4d EKFVelocity::predictState(double dt) {
    P = F*P*F.transpose() + Q;
    return F*state;
}

// Function to update Kalman gain
Eigen::MatrixXd EKFVelocity::calculateKalmanGain(Eigen::MatrixXd H, Eigen::MatrixXd Q, Eigen::MatrixXd R) {
    return P * H.transpose() * (H * P * H.transpose() + R).inverse();
}

void EKFVelocity::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!initialized){
        initialized = true;
        state[0] = msg->pose.position.x;
        state[1] = msg->pose.position.y;
        last_time = msg->header.stamp;
    }
    x_new = msg->pose.position.x;
    y_new = msg->pose.position.y;
    current_time = msg->header.stamp;
    tf::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    theta_new = yaw;
}

ekf::PoseVel EKFVelocity::getFiltered(){
    ekf::PoseVel msg;
    if (initialized){        
        // Prediction step
        double dt = (current_time - last_time).toSec();
        last_time = current_time;
        F << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;
        Eigen::Vector4d state_pred = predictState(dt); // dt = current time - previous time

        // Update step (assuming constant noise covariances)
        Eigen::MatrixXd K = calculateKalmanGain(H, Q, R); 

        // Measurement vector
        Eigen::Vector2d z(x_new, y_new);

        state = state_pred + K * (z - H * state_pred);
        P = (Eigen::MatrixXd::Identity(4, 4) - K * H) * P;

        msg.pose.x = state[0];
        msg.pose.y = state[1];
        msg.pose.z = theta_new;
        msg.twist.x = state[2];
        msg.twist.y = state[3];
        msg.id = id_;
    }
    return msg;
}

EKFVelocity::~EKFVelocity()
{
}
