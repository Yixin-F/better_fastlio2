#include "dynamic-remove/tgrs.h"

std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<PointType>::Ptr> detect(const pcl::PointCloud<PointType>::Ptr& local_, const pcl::PointCloud<PointType>::Ptr& global_, const float& search_dis_){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>());  // result
    pcl::PointCloud<PointType>::Ptr result_use(new pcl::PointCloud<PointType>());  // result
    int local_num = local_->points.size();
    std::cout << "Second local scan points num is: " << local_num << std::endl;

    // select update area after registeration
    PointType point_min, point_max;
    pcl::getMinMax3D(*local_, point_min, point_max);
    float min_x = point_min.x;
    float min_y = point_min.y;
    float min_z = point_min.z;
    float max_x = point_max.x;
    float max_y = point_max.y;
    float max_z = point_max.z;

    int global_num = global_->points.size();
    std::cout << "First global scan points num is: " << global_num << std::endl;

    pcl::PointCloud<PointType>::Ptr global_select(new pcl::PointCloud<PointType>());
    for(size_t i = 0; i < global_num; i ++){
        PointType pt = global_->points[i];
        if(pt.x < min_x || pt.x > max_x || pt.y < min_y || pt.y > max_y || pt.z < min_z || pt.z > max_z)
            continue;
        global_select->points.emplace_back(pt);
    }
    int select_num = global_select->points.size();

    std::cout << "First glocal has been selected as num : " << select_num << " by scaled " << "\n" 
                       << "x: " << min_x << " to  " << max_x << "\n"
                       << "y: " << min_y << " to  " << max_y << "\n"
                       << "z: " << min_z << " to  " << max_z << std::endl;

    // local search based on global
    pcl::KdTreeFLANN<PointType>::Ptr kdtree(new pcl::KdTreeFLANN<PointType>());
    kdtree->setInputCloud(global_select);   // global select input

    std::vector<std::pair<int, std::vector<int>>> local_find;
    std::vector<int> local_nofind;
    std::vector<int> global_match;
    std::vector<int> global_nomatch;
    for(size_t i = 0; i < local_num; i++){
        std::vector<int> id;
        std::vector<float> dis;
        kdtree->radiusSearch(local_->points[i], search_dis_, id, dis);  // local find

        if(id.size() != 0){
            local_find.emplace_back(std::make_pair(i, id));  // local find 
        }
        else{
            local_nofind.emplace_back(i);  // local not find
        }

        global_match.insert(global_match.end(), id.begin(), id.end());  
    }

    std::sort(global_match.begin(), global_match.end());
    std::vector<int>::iterator it = unique(global_match.begin(), global_match.end());
    global_match.erase(it, global_match.end()); // global match

    for(size_t i = 0; i < select_num; i++){
        if(std::find(global_match.begin(), global_match.end(), i) == global_match.end()){
            global_nomatch.emplace_back(i);  // global match
        }
    }

    // local find and glocal match -> point fusion (green)
    for(auto& pr :local_find){
        pcl::PointXYZRGB pt_rgb;
        PointType pt_use;

        int fusion_num = pr.second.size() + 1;
        float local_ratio = 1 / fusion_num + 1;
        float global_ratio = 1- 1 / fusion_num / pr.second.size();

        float x_sum = local_->points[pr.first].x * local_ratio ;
        float y_sum = local_->points[pr.first].y * local_ratio;
        float z_sum = local_->points[pr.first].z * local_ratio;
        for(auto& pr0 : pr.second){
            x_sum += global_select->points[pr0].x * global_ratio;
            y_sum += global_select->points[pr0].y * global_ratio;
            z_sum += global_select->points[pr0].z * global_ratio;
        }
        pt_rgb.x = x_sum / fusion_num;
        pt_rgb.y = y_sum / fusion_num;
        pt_rgb.z = z_sum / fusion_num;
        pt_use.x = x_sum / fusion_num;
        pt_use.y = y_sum / fusion_num;
        pt_use.z = z_sum / fusion_num;
        pt_rgb.r = 0;
        pt_rgb.g = 0;
        pt_rgb.b = 255.0;
        result->points.emplace_back(pt_rgb);  // save
        result_use->points.emplace_back(pt_use);  // save
    }
    std::cout << "local find and glocal match -> fusion" << std::endl; 

    // local not find -> new
    for(auto& vi : local_nofind){
        PointType pt_use;
        pcl::PointXYZRGB pt_rgb;
        pt_rgb.x = local_->points[vi].x;
        pt_rgb.y = local_->points[vi].y;
        pt_rgb.z = local_->points[vi].z;
        pt_rgb.r = 0;
        pt_rgb.g = 0;
        pt_rgb.b = 255.0;
        pt_use.x = local_->points[vi].x;
        pt_use.y = local_->points[vi].y;
        pt_use.z = local_->points[vi].z;

        result->points.emplace_back(pt_rgb);  // save
        result_use->points.emplace_back(pt_use);
    }
    std::cout << "local not find -> new" << std::endl; 

    // global not match -> old
    for(auto& vi : global_nomatch){
        pcl::PointXYZRGB pt_rgb;
        pt_rgb.x = global_select->points[vi].x;
        pt_rgb.y = global_select->points[vi].y;
        pt_rgb.z = global_select->points[vi].z;
        pt_rgb.r = 255.0;
        pt_rgb.g = 0;
        pt_rgb.b = 0;
        result->points.emplace_back(pt_rgb);  // save
    }
    std::cout << "global not match -> old" << std::endl; 

    result->height = 1;
    result->width = result->points.size();
    std::cout << "result num: " << result->points.size() << std::endl;
    
    return std::make_pair(result, result_use);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "DY-Remover");
    ROS_INFO("\033[1;32m----> dynamic remove.\033[0m");

    // // TODO: dynamio removal text
    // pcl::PointCloud<PointType>::Ptr test(new pcl::PointCloud<PointType>());
    // pcl::io::loadPCDFile("/home/yixin-f/fast-lio2/src/data_dy/000077.pcd", *test);
    // SSC ssc(test, 1);
    // std::cout << test->points.size() << std::endl;
    // std::cout << ssc.cloud_use->points.size() << std::endl;
    // std::cout << ssc.apri_vec.size() << std::endl;
    // std::cout << ssc.hash_cloud.size() << std::endl;

    TGRS remover;
    // remover.cluster(ssc.apri_vec, ssc.hash_cloud, ssc.cluster_vox);
    // std::cout << ssc.cluster_vox.size() << std::endl;
    // remover.saveColorCloud(ssc, "/home/yixin-f/fast-lio2/src/data_dy/000077_color.pcd");

    // remover.recognizePD(ssc);
    // std::cout << ssc.PD_cluster.size() << std::endl;
    // std::vector<int> pdVox;
    // pcl::PointCloud<PointType>::Ptr pcCloudGrab(new pcl::PointCloud<PointType>());
    // for(auto& pd : ssc.PD_cluster){
    //     for(auto& pt : ssc.cluster_vox[pd]){
    //         pdVox.emplace_back(ssc.hash_cloud[pt].ptVoxIdx);
    //     }
    // }
    // *pcCloudGrab += *remover.getCloudByVec(pdVox, ssc.cloud_vox);
    // pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_dy/000077_pd.pcd", *pcCloudGrab);

    // PointTypePose pose_prior;
    // pose_prior.x = 0.0;
    // pose_prior.y = 0.0;
    // pose_prior.z = 0.0;
    // pose_prior.roll = 0.0;
    // pose_prior.pitch = 0.0;
    // pose_prior.yaw = 0.0;

    // PointTypePose pose_cur;
    // pose_cur.x = 0.0;
    // pose_cur.y = 0.0;
    // pose_cur.z = 0.0;
    // pose_cur.roll = 0.0;
    // pose_cur.pitch = 0.0;
    // pose_cur.yaw = 0.0;

    // remover.trackPD(ssc, &pose_prior, ssc, &pose_cur);
    // TODO: dynamio removal text

    Eigen::Matrix<double, 3, 3> ext_r;
    ext_r << -1, 0, 0, 0, -1, 0, 0, 0, 1;
    std::vector<double> ext_t{1.77, 0.0, -0.05};
    Eigen::Matrix<double, 3, 1> euler_r = RotMtoEuler(ext_r);
    Eigen::Affine3f ext_rr = pcl::getTransformation(ext_t[0], ext_t[1], ext_t[2], euler_r(0, 0), euler_r(1, 0), euler_r(2, 0));
    PointTypePose pose_ext;
    pose_ext.x = ext_t[0];
    pose_ext.y = ext_t[1];
    pose_ext.z = ext_t[2];
    pose_ext.roll = euler_r(0, 0);
    pose_ext.pitch = euler_r(1, 0);
    pose_ext.yaw = euler_r(2, 0);

    std::string prior_path = "/home/yixin-f/fast-lio2/src/parkinglot/05/PCDs";
    std::string prior_pose = "/home/yixin-f/fast-lio2/src/parkinglot/0105/aft_tansformation2.pcd";
    std::string cur_path = "/home/yixin-f/fast-lio2/src/parkinglot/06/PCDs";
    std::string cur_pose = "/home/yixin-f/fast-lio2/src/parkinglot/0106/aft_tansformation2.pcd";

    pcl::PointCloud<PointTypePose>::Ptr pri_pose(new pcl::PointCloud<PointTypePose>());
    pcl::PointCloud<PointTypePose>::Ptr aft_pose(new pcl::PointCloud<PointTypePose>());
    pcl::PointCloud<PointType>::Ptr pri_kd(new pcl::PointCloud<PointType>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::io::loadPCDFile(cur_pose, *aft_pose);
    std::cout << "session size: " << aft_pose->points.size() << std::endl;

    pcl::io::loadPCDFile(prior_pose, *pri_pose);
    std::cout << "session size: " << pri_pose->points.size() << std::endl;
    for(size_t i = 0; i < pri_pose->points.size(); i ++){
        PointType pt;
        pt.x = pri_pose->points[i].x;
        pt.y = pri_pose->points[i].y;
        pt.z = pri_pose->points[i].z;
        pri_kd->points.emplace_back(pt);
    }

    // TODO: a good demo to use the multi-session and segmentation result !! 
    pcl::PointCloud<PointType>::Ptr prior_map(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prior_map_ext(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prior_map_ext2(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr cur_map(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr pd_detect1(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr pd_detect2(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr g_detect1(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr g_detect2(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr use(new pcl::PointCloud<PointType>());

    // seq. 01 at parkinglot start
    // 02 -> 01 --- 0~30/3 : 0~50/5
    // 03 -> 02 --- 50~80/3 : 0~30/3
    // 04 -> 03 --- 80~110/3 : 50~80/3
    // 05 -> 04 --- 0~30/3 : 80~110/3
    // 06 -> 05 --- 0~30/3 : 0~30/3

    // seq. 02 at parkinglot middle
    // 02 -> 01 --- 340~370/3 : 370~400/3
    // 03 -> 02 --- 390~420/3 : 340~370/3
    // 04 -> 03 --- 340~370/3 : 390~420/3
    // 05 -> 04 --- 420~450/3 : 340~370/3
    // 06 -> 05 --- 390~420/3 : 420~450/3

    for(size_t i = 390; i < 420; i += 3){
        pcl::PointCloud<PointType>::Ptr cur(new pcl::PointCloud<PointType>());
        std::string name_cur = (boost::format("%s/%06d.pcd") % cur_path % i).str();
        std::cout << name_cur << std::endl;
        // std::string name_cur = cur_path + std::to_string(i);
        pcl::io::loadPCDFile(name_cur, *cur);

        SSC ssc_i(cur, i);
        remover.cluster(ssc_i.apri_vec, ssc_i.hash_cloud, ssc_i.cluster_vox);
        remover.recognizePD(ssc_i);

        std::vector<int> ptid;
        for(auto& it : ssc_i.PD_cluster){
            for(auto& is : ssc_i.cluster_vox[it]){
                addVec(ptid, ssc_i.hash_cloud[is].ptIdx);
            }
        }
        pcl::PointCloud<PointType>::Ptr pdcloud(new pcl::PointCloud<PointType>());
        pdcloud = remover.getCloudByVec(ptid, ssc_i.cloud_use);

        pcl::PointCloud<PointType>::Ptr tmp2(new pcl::PointCloud<PointType>());
        *tmp2 += *transformPointCloud(ssc_i.cloud_ng, &pose_ext);
        *cur_map += *transformPointCloud(tmp2, &aft_pose->points[i]);

        pcl::PointCloud<PointType>::Ptr tmp3(new pcl::PointCloud<PointType>());
        *tmp3 += *transformPointCloud(pdcloud, &pose_ext);
        *pd_detect1 += *transformPointCloud(tmp3, &aft_pose->points[i]);

        pcl::PointCloud<PointType>::Ptr tmp4(new pcl::PointCloud<PointType>());
        *tmp4 += *transformPointCloud(ssc_i.cloud_g, &pose_ext);
        *g_detect1 += *transformPointCloud(tmp4, &aft_pose->points[i]);

    }

    for(size_t i = 420; i < 450; i += 3){
        pcl::PointCloud<PointType>::Ptr prior(new pcl::PointCloud<PointType>());
        std::string name_prior = (boost::format("%s/%06d.pcd") % prior_path % i).str();
        std::cout << name_prior << std::endl;
        // std::string name_prior = prior_path + std::to_string(i);
        pcl::io::loadPCDFile(name_prior, *prior);
        

        // Eigen::Affine3f trans_prior_i = pcl::getTransformation(pri_pose->points[i].x, pri_pose->points[i].y, pri_pose->points[i].z, 
        //                                                         pri_pose->points[i].roll, pri_pose->points[i].pitch, pri_pose->points[i].yaw);
        // Eigen::Affine3f trans = trans_prior_i * ext_rr;
        // pcl::PointCloud<PointType>::Ptr prior_ext(new pcl::PointCloud<PointType>());
        // transformPointCloud(prior, trans, prior_ext);
        // *prior_map_ext += *prior_ext;

        SSC ssc_i(prior, i);
        remover.cluster(ssc_i.apri_vec, ssc_i.hash_cloud, ssc_i.cluster_vox);
        remover.recognizePD(ssc_i);

        std::vector<int> ptid;
        for(auto& it : ssc_i.PD_cluster){
            for(auto& is : ssc_i.cluster_vox[it]){
                addVec(ptid, ssc_i.hash_cloud[is].ptIdx);
            }
        }
        pcl::PointCloud<PointType>::Ptr pdcloud(new pcl::PointCloud<PointType>());
        pdcloud = remover.getCloudByVec(ptid, ssc_i.cloud_use);
        pcl::PointCloud<PointType>::Ptr tmp5(new pcl::PointCloud<PointType>());
        *tmp5 += *transformPointCloud(pdcloud, &pose_ext);
        *pd_detect2 += *transformPointCloud(tmp5, &pri_pose->points[i]);

        pcl::PointCloud<PointType>::Ptr tmp(new pcl::PointCloud<PointType>());
        *tmp += *transformPointCloud(ssc_i.cloud_ng, &pose_ext);
        *prior_map_ext2 += *transformPointCloud(tmp, &pri_pose->points[i]);

        pcl::PointCloud<PointType>::Ptr tmp6(new pcl::PointCloud<PointType>());
        *tmp6 += *transformPointCloud(ssc_i.cloud_g, &pose_ext);
        *g_detect2 += *transformPointCloud(tmp6, &pri_pose->points[i]);
    }

    std::pair<PointType, PointType> heightPair_p = remover.getBoundingBoxOfCloud(pd_detect2);
    std::pair<PointType, PointType> heightPair_c = remover.getBoundingBoxOfCloud(pd_detect1);
    pcl::PointCloud<PointType>::Ptr prior_select(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr cur_select(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prig_select(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr curg_select(new pcl::PointCloud<PointType>());
    PointType min, max;
    min.x = (heightPair_p.first.x > heightPair_c.first.x) ? heightPair_p.first.x : heightPair_c.first.x;
    min.y = (heightPair_p.first.y > heightPair_c.first.y) ? heightPair_p.first.y : heightPair_c.first.y;
    min.z = (heightPair_p.first.z > heightPair_c.first.z) ? heightPair_p.first.z : heightPair_c.first.z;
    max.x = (heightPair_p.second.x < heightPair_c.second.x) ? heightPair_p.second.x : heightPair_c.second.x;
    max.y = (heightPair_p.second.y < heightPair_c.second.y) ? heightPair_p.second.y : heightPair_c.second.y;
    max.z = (heightPair_p.second.z < heightPair_c.second.z) ? heightPair_p.second.z : heightPair_c.second.z;
    for(size_t i = 0; i < prior_map_ext2->points.size(); i++){
        if(prior_map_ext2->points[i].x < min.x || prior_map_ext2->points[i].x > max.x){
            continue;
        }
        if(prior_map_ext2->points[i].y < min.y || prior_map_ext2->points[i].y > max.y){
            continue;
        }
        if(prior_map_ext2->points[i].z < min.z || prior_map_ext2->points[i].z > max.z){
            continue;
        }
        prior_select->points.emplace_back(prior_map_ext2->points[i]);
    }
    for(size_t i = 0; i < cur_map->points.size(); i++){
        if(cur_map->points[i].x < min.x || cur_map->points[i].x > max.x){
            continue;
        }
        if(cur_map->points[i].y < min.y || cur_map->points[i].y > max.y){
            continue;
        }
        if(cur_map->points[i].z < min.z || cur_map->points[i].z > max.z){
            continue;
        }
        cur_select->points.emplace_back(cur_map->points[i]);
    }
    for(size_t i = 0; i < g_detect1->points.size(); i++){
        if(g_detect1->points[i].x < min.x || g_detect1->points[i].x > max.x){
            continue;
        }
        if(g_detect1->points[i].y < min.y || g_detect1->points[i].y > max.y){
            continue;
        }
        if(g_detect1->points[i].z < min.z || g_detect1->points[i].z > max.z){
            continue;
        }
        prig_select->points.emplace_back(g_detect1->points[i]);
    }
    for(size_t i = 0; i < g_detect2->points.size(); i++){
        if(g_detect2->points[i].x < min.x || g_detect2->points[i].x > max.x){
            continue;
        }
        if(g_detect2->points[i].y < min.y || g_detect2->points[i].y > max.y){
            continue;
        }
        if(g_detect2->points[i].z < min.z || g_detect2->points[i].z > max.z){
            continue;
        }
        curg_select->points.emplace_back(g_detect2->points[i]);
    }


    cur_select->height = 1;
    cur_select->width = cur_select->points.size();
    prior_select->height = 1;
    prior_select->width = prior_select->points.size();
    pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_dy/cur_map_select.pcd", *cur_select);
    pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_dy/prior_map_select.pcd", *prior_select);
    // pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_dy/prior_g.pcd", *g_detect2);
    // pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_dy/cur_g.pcd", *g_detect1);

    curg_select->height = 1;
    curg_select->width = curg_select->points.size();
    prig_select->height = 1;
    prig_select->width = prig_select->points.size();
    // pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_dy/curg_select.pcd", *curg_select);
    // pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_dy/prig_select.pcd", *prig_select);

    // pcl::VoxelGrid<PointType> filter;
    // filter.setInputCloud(cur_select);
    // filter.setLeafSize(0.2f, 0.2f, 0.2f);
    // filter.filter(*cur_select);
    // filter.setInputCloud(prior_select);
    // filter.setLeafSize(0.2f, 0.2f, 0.2f);
    // filter.filter(*prior_select);

    result = detect(pd_detect1, pd_detect2, 0.4).first;
    use = detect(pd_detect1, pd_detect2, 0.4).second;
    result->height = 1;
    result->width = result->points.size();
    use->height = 1;
    use->width = use->points.size();
    pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_dy/result.pcd", *result);
    pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_dy/use.pcd", *use);

    pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_dy/cur_map_ext2.pcd", *cur_map);
    // pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_dy/prior_map.pcd", *prior_map);
    // pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_dy/prior_map_ext.pcd", *prior_map_ext);
    pcl::io::savePCDFile("/home/yixin-f/fast-lio2/src/data_dy/prior_map_ext2.pcd", *prior_map_ext2);
    // TODO: a good demo to use the multi-session and segmentation result !! 

    // // TODO: object-level fusion and update
    // pcl::KdTreeFLANN<PointType>::Ptr kdtreePriorMapPoses(new pcl::KdTreeFLANN<PointType>());
    // kdtreePriorMapPoses->setInputCloud(pri_kd);

    // for(size_t i = 0; i < 20; i ++){
    //     pcl::PointCloud<PointType>::Ptr cur(new pcl::PointCloud<PointType>());
    //     std::string name_cur = (boost::format("%s/%06d.pcd") % cur_path % i).str();
    //     std::cout << name_cur << std::endl;
    //     pcl::io::loadPCDFile(name_cur, *cur);

    //     SSC ssc_i(cur, i);
    //     remover.cluster(ssc_i.apri_vec, ssc_i.hash_cloud, ssc_i.cluster_vox);
    //     std::string name_ssc = (boost::format("%s/%06dscc.pcd") % "/home/yixin-f/fast-lio2/src/data_dy" % i).str();
    //     remover.saveColorCloud(ssc_i, name_ssc);
    //     remover.recognizePD(ssc_i);

    //     Eigen::Affine3f trans_cur_i = pcl::getTransformation(aft_pose->points[i].x, aft_pose->points[i].y, aft_pose->points[i].z, 
    //                                                 aft_pose->points[i].roll, aft_pose->points[i].pitch, aft_pose->points[i].yaw);
    //     Eigen::Affine3f aft_ = trans_cur_i * ext_rr;

    //     float aft[6];
    //     pcl::getTranslationAndEulerAngles(aft_, aft[0], aft[1], aft[2], aft[3], aft[4], aft[5]);
    //     PointTypePose pose_aft;
    //     pose_aft.x = aft[0];
    //     pose_aft.y = aft[1];
    //     pose_aft.z = aft[2];
    //     pose_aft.roll = aft[3];
    //     pose_aft.pitch = aft[4];
    //     pose_aft.yaw = aft[5];

    //     std::vector<int> idxs;
    //     std::vector<float> dis;
    //     PointType pt;
    //     pt.x = aft_pose->points[i].x;
    //     pt.y = aft_pose->points[i].y;
    //     pt.z = aft_pose->points[i].z;
    //     kdtreePriorMapPoses->radiusSearch(pt, 5, idxs, dis);
    //     if(idxs.size() == 0){
    //         continue;
    //     }

    // }
    // // TODO: object-level fusion and update

    return 0;
}