#include <ekf.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "ekf_velocity_estimator");
    ros::NodeHandle nh("~");

    std::vector<EKFVelocity*> ekf_vector;
    std::vector<ros::Subscriber> ekf_subs;
    int n_peds, n_rob;
    // sleep(5);
    if (nh.getParam("n_peds", n_peds) && nh.getParam("n_rob", n_rob))
    {
        ros::Publisher poses_pub = nh.advertise<ekf::PoseVelArray>("/ekf_poses", 1);
        for (int i = 1; i <= n_peds; i++){
            EKFVelocity* ekf_vel = new EKFVelocity("pedestrian_" + std::to_string(i));
            ekf_vector.push_back(ekf_vel);
            ekf_subs.push_back(nh.subscribe(("/vrpn_client_node/pedestrian_" + std::to_string(i) + "/pose").c_str(), 1, 
			&EKFVelocity::poseCallback, ekf_vel));
        }
        for (int i = 1; i <= n_peds; i++){
            EKFVelocity* ekf_vel = new EKFVelocity("robot_" + std::to_string(i));
            ekf_vector.push_back(ekf_vel);
            ekf_subs.push_back(nh.subscribe(("/vrpn_client_node/robot_" + std::to_string(i) + "/pose").c_str(), 1, 
			&EKFVelocity::poseCallback, ekf_vel));
        }
        ros::Rate loop_rate(20);
        while (ros::ok())
        {
            ekf::PoseVelArray msg;
            ros::spinOnce();
            for (int i = 0; i < ekf_vector.size(); i++){
                ekf::PoseVel pose = ekf_vector[i]->getFiltered();
                msg.poses.push_back(pose);
            }
            msg.stamp = ros::Time::now();
            poses_pub.publish(msg);
            loop_rate.sleep();
        }
    }
    else{
        ROS_ERROR("The number of pedestrians and robots must be specified in n_peds and n_rob.");
    }

        // odom_pub = nh_.advertise<nav_msgs::Odometry>(("/pedestrian_" + std::to_string(ped_id) + "/odom").c_str(), 5);


    return 0;
}