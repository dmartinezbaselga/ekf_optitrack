#include <exp_manager.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "exp_manager");
    ros::NodeHandle nh("~");
    int exp_n, group_id;
    if (nh.getParam("exp_n", exp_n) && nh.getParam("group_id", group_id))
    {
        ExpManager exp = ExpManager(exp_n, group_id);
        std::vector<ros::Publisher> goals_pub;
        for (int i = 0; i < 3; i++){
            ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/robot_" + std::to_string(i+1) + "/goal", 1);
            goals_pub.push_back(goal_pub);
            // exp.publishGoal(exp_n, i, goal_pub);
        }
        ros::Subscriber poses_sub = nh.subscribe("/ekf_poses", 1, &ExpManager::posesCallback, &exp);
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            ros::spinOnce();
            for (int i = 0; i < 3; i++){
                exp.publishGoal(exp_n, i, goals_pub[i]);
            }
            loop_rate.sleep();
        }
    }
    else{
        ROS_ERROR("The experiment number must be specified in exp_n.");
    }
    return 0;
}