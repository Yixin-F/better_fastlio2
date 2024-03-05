#include "tgrs.h"
#include "tictoc.hpp"

void TGRS::mergeClusters(std::vector<int>& clusterIdxs_, const int& idx1_, const int& idx2_){
    for(int i = 0; i < clusterIdxs_.size(); i++){
        if(clusterIdxs_[i] == idx1_){
            clusterIdxs_[i] = idx2_;
        }
    }
}

std::vector<int> TGRS::findVoxelNeighbors(const int& range_idx_, const int& sector_idx_, const int& azimuth_idx_, int size_){
    std::vector<int> neighborIdxs;
    if(range_idx_ > RANGE_NUM * 0.6){
        size_ = 1;
    }
    for(int x = range_idx_ - size_; x <= range_idx_ + size_; x++){
        if(x > RANGE_NUM -1 || x < 0) {continue;}
        for(int y = sector_idx_ - size_; y <= sector_idx_ + size_; y++){
            if(y > SECTOR_NUM -1 || y < 0) {continue;}
            for(int z = azimuth_idx_ - size_; z <= azimuth_idx_ + size_; z++){
                if(z > AZIMUTH_NUM - 1 || z < 0) {continue;}
                neighborIdxs.emplace_back(x * SECTOR_NUM + y + z * RANGE_NUM * SECTOR_NUM);  
            }
        }
    }
    return neighborIdxs;
}

void TGRS::cluster(const std::vector<PointAPRI>& apri_vec_, 
                   std::unordered_map<int, Voxel>& hash_cloud_,
                   std::unordered_map<int, std::vector<int>>& cluster_vox)
{
    int cluster_name = 4;
    std::vector<int> clusterIdxs = std::vector<int>(apri_vec_.size(), -1);

    TicToc cluster_t;

    // vec cluster
    for(int i = 0; i < apri_vec_.size(); i++){
        PointAPRI apri = apri_vec_[i];
        std::unordered_map<int, Voxel>::iterator it_find1;
        std::unordered_map<int, Voxel>::iterator it_find2;
        std::vector<int> neighbors;  // restore a lot of apri-neighbors idxs

        it_find1 = hash_cloud_.find(apri.voxel_idx);
        if(it_find1 != hash_cloud_.end()){
            std::vector<int> neighbor = findVoxelNeighbors(apri.range_idx, apri.sector_idx, apri.azimuth_idx, 1);   
            for(int k = 0; k < neighbor.size(); k++){
                it_find2 =  hash_cloud_.find(neighbor[k]);   
                if(it_find2 != hash_cloud_.end()){
                    addVec(neighbors, it_find2->second.ptIdx);
                }
            }
        }

        neighbors.swap(neighbors);
        if(neighbors.size() > 0){
            for(int n = 0; n < neighbors.size(); n++){
                int oc = clusterIdxs[i];
                int nc = clusterIdxs[neighbors[n]];

                if(oc != -1 && nc != -1){
                    if(oc != nc){
                        mergeClusters(clusterIdxs, oc, nc);  // merge
                    }
                }
                else{
                    if(nc != -1){
                        clusterIdxs[i] = nc;
                    }
                    else{
                        if(oc != -1){
                            clusterIdxs[neighbors[n]] = oc;
                        }
                    }
                }    
            }
        }

        if(clusterIdxs[i] == -1){
            cluster_name ++;   //   a new class
            clusterIdxs[i] = cluster_name;  // just encode the cluster name
            for(int m = 0; m < neighbors.size(); m++){
                clusterIdxs[neighbors[m]] = cluster_name;
            }
        }
    }

    // voxel cluster
    std::unordered_map<int, std::vector<int>>::iterator it_v;
    for(size_t i = 0; i < clusterIdxs.size(); i++){
        it_v = cluster_vox.find(clusterIdxs[i]);
        if(it_v != cluster_vox.end()){
            it_v->second.emplace_back(apri_vec_[i].voxel_idx);
            hash_cloud_[apri_vec_[i].voxel_idx].label = it_v->first;
        }
        else{
            std::vector<int> vox_vec;
            vox_vec.emplace_back(apri_vec_[i].voxel_idx);
            cluster_vox.insert(std::make_pair(clusterIdxs[i], vox_vec));
            hash_cloud_[apri_vec_[i].voxel_idx].label = clusterIdxs[i];
        }
    }
    
    for(auto& it : cluster_vox){
        sampleVec(it.second);
    }
}

std::pair<PointType, PointType> TGRS::getBoundingBoxOfCloud(const pcl::PointCloud<PointType>::Ptr& cloud_){
    PointType point_min, point_max;
    pcl::getMinMax3D(*cloud_, point_min, point_max);
    return std::make_pair(point_min, point_max);
}

pcl::PointCloud<PointType>::Ptr TGRS::getCloudByVec(const std::vector<int>& vec_, const pcl::PointCloud<PointType>::Ptr& cloud_){
    pcl::PointCloud<PointType>::Ptr cloudReturn(new pcl::PointCloud<PointType>());
    for(auto &it : vec_){
        cloudReturn->points.emplace_back(cloud_->points[it]);
    }
    return cloudReturn;
}

void TGRS::recognizePD(SSC& ssc){
    for(auto& it : ssc.cluster_vox){
        std::vector<int> voxIdx;
        for(auto& id : it.second){
            voxIdx.emplace_back(id);
        }
        pcl::PointCloud<PointType>::Ptr voxels(new pcl::PointCloud<PointType>());
        *voxels += *getCloudByVec(voxIdx, ssc.cloud_vox);
        std::pair<PointType, PointType> heightPair = getBoundingBoxOfCloud(voxels);
        if(heightPair.first.z <= -(SENSOR_HEIGHT - 0.2) && (heightPair.second.z + SENSOR_HEIGHT) <= PD_HEIGHT){
            ssc.PD_cluster.emplace_back(it.first);
        }
    }
    std::cout << "There are " << ssc.PD_cluster.size() << " PD objects." << std::endl;
}

void TGRS::trackHD(SSC& ssc_pre, PointTypePose* pose_pre, SSC& ssc_next, PointTypePose* pose_next){
    
}

