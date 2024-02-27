#include "online-relo/pose_estimator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "OR_LIO");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug); 
    ROS_INFO("\033[1;32m----> Online Relocalization Started.\033[0m");

    pose_estimator *lol = new pose_estimator();
    std::thread opt_thread(&pose_estimator::run, lol);
    std::thread pub_thread(&pose_estimator::publishThread, lol);
    
    ros::spin();

    return 0;
}