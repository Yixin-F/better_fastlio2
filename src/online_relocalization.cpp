#include "online-relo/pose_estimator_new.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "OR_LIO");

    ROS_INFO("\033[1;32m----> Online Relocalization Started.\033[0m");

    pose_estimator *lol = new pose_estimator("123");
    std::thread opt_thread(&pose_estimator::run, lol);
    
    ros::spin();

    return 0;
}