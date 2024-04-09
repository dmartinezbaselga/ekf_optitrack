#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <tf/tf.h>
#include <ekf/PoseVelArray.h>
#include <ros/package.h>
#include <chrono>
#include <fstream>

class ExpManager
{
private:
    ros::NodeHandle nh;  
    int exp_id, group_id;
    double goals_x[3][3] = {
        2.32, -2.13, 2.35,
        1.98, -2.02, 0.04,
        2.53, 0.14, 0.06
    };
    double goals_y[3][3] = {
        2.05, 0.05, -1.73,
        -1.57, -1.55, 2.39,
        0.61, -2.07, 2.38
    };
    std::vector<ekf::PoseVelArray> poses_list;

public:
    ExpManager(const int exp_id, const int group_id);
    ~ExpManager();
    void posesCallback(const ekf::PoseVelArray& msg);
    void publishGoal(const int exp_id, const int robot_id, ros::Publisher& goal_pub);
};

ExpManager::ExpManager(const int exp_id, const int group_id)
{
    this->exp_id = exp_id;
    this->group_id = group_id;
}

void ExpManager::publishGoal(const int exp_id, const int robot_id, ros::Publisher& goal_pub){
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = goals_x[exp_id][robot_id];
    msg.pose.position.y = goals_y[exp_id][robot_id];
    goal_pub.publish(msg);
    ROS_INFO("PUBLISHEEED");
}

void ExpManager::posesCallback(const ekf::PoseVelArray& msg) {
    bool finished = true;
    poses_list.push_back(msg);
    for (auto& agent : msg.poses){
        if (agent.id.find("robot_") == 0){
            int id = agent.id.back() - '0';
            id -= 1;
            double dx = agent.pose.x - goals_x[exp_id][id];
            double dy = agent.pose.y - goals_y[exp_id][id];
            if (sqrt(dx*dx + dy*dy) > 0.15){
                finished = false;
            }
        }
    }
    if (finished){
        ros::shutdown();
    }
}

ExpManager::~ExpManager()
{
    const auto p1 = std::chrono::system_clock::now();
    std::ofstream f(ros::package::getPath("ekf") + "/data/group_" + std::to_string(group_id) + "_exp_" + std::to_string(exp_id) + "_" + 
                    std::to_string(std::chrono::duration_cast<std::chrono::seconds>(p1.time_since_epoch()).count()) + ".txt");
    for (auto poses : poses_list){
        for (auto pose : poses.poses){
            f << "(" << pose.pose.x << ", " << pose.pose.y << ") ";
        }
        f << std::endl;
    }
}
