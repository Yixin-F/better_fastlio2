#include "dynamic-remove/tgrs.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "DY-Remover");
    ROS_INFO("\033[1;32m----> dynamic remove.\033[0m");

    pcl::PointCloud<PointType>::Ptr test(new pcl::PointCloud<PointType>());
    pcl::io::loadPCDFile("/home/yixin-f/fast-lio2/src/data_dy/000077.pcd", *test);
    SSC ssc(test, 1);
    std::cout << test->points.size() << std::endl;
    std::cout << ssc.cloud_use->points.size() << std::endl;
    std::cout << ssc.apri_vec.size() << std::endl;
    std::cout << ssc.hash_cloud.size() << std::endl;

    TGRS remover;
    remover.cluster(ssc.apri_vec, ssc.hash_cloud, ssc.cluster_vox);
    std::cout << ssc.cluster_vox.size() << std::endl;
    remover.saveColorCloud(ssc, "/home/yixin-f/fast-lio2/src/data_dy/000077_color.pcd");

    return 0;
}