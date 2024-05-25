// head for registeration writted by @Yixin Fang

#pragma once

#include <iostream>
#include "ICP.h"
#include "io_pc.h"
#include "FRICP.h"
#include "../tool_color_printf.h"
#include "../common_lib.h"

typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Vertices;
typedef Eigen::Matrix<Scalar, 3, 1> VectorN;
ofstream fout_time;

class Registeration{
public:
    Eigen::MatrixXd res_trans;
    enum Method{ICP, AA_ICP, FICP, RICP, FR_ICP, PPL, RPPL, SparseICP, SICPPPL} method=RICP;
    int dim = 3;

    // please see here for mode details
    Registeration(int mode_){
        std::cout << "Method :\n"
                  << "0: ICP\n1: AA-ICP\n2: Our Fast ICP\n3: Our Robust ICP\n4: Our Fast and Robust ICP\n5: ICP Point-to-plane\n"
                  << "6: Our Fast and Robust ICP point to plane\n7: Sparse ICP\n8: Sparse ICP point to plane\n" 
                  << "search radius(< 0.5) for difference detection"<< std::endl;
        method = Method(mode_);
        std::cout << ANSI_COLOR_GREEN << "register by Method " << mode_ << ANSI_COLOR_RESET << std::endl;
        fout_time.open("/home/yixin-f/fast-lio2/src/data_loc/relo_time.txt", ios::out);

    }
    ~Registeration() {}

    Eigen::MatrixXd run(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& source,
             const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& target)
    {   
        //--- Model that will be rigidly transformed
        Vertices vertices_source, normal_source, src_vert_colors;
        read_pcd_online(vertices_source, normal_source, src_vert_colors, source, dim);

        //--- Model that source will be aligned to
        Vertices vertices_target, normal_target, tar_vert_colors;
        read_pcd_online(vertices_target, normal_target, tar_vert_colors, target, dim);

        // scaling
        Eigen::Vector3d source_scale, target_scale;
        source_scale = vertices_source.rowwise().maxCoeff() - vertices_source.rowwise().minCoeff();
        target_scale = vertices_target.rowwise().maxCoeff() - vertices_target.rowwise().minCoeff();
        double scale = std::max(source_scale.norm(), target_scale.norm());
        vertices_source /= scale;
        vertices_target /= scale;

        /// De-mean
        VectorN source_mean, target_mean;
        source_mean = vertices_source.rowwise().sum() / double(vertices_source.cols());
        target_mean = vertices_target.rowwise().sum() / double(vertices_target.cols());
        vertices_source.colwise() -= source_mean;
        vertices_target.colwise() -= target_mean;

        double time;
        // set ICP parameters
        ICP::Parameters pars;

        // set Sparse-ICP parameters
        SICP::Parameters spars;
        spars.p = 0.4;
        spars.print_icpn = false;

        ///--- Execute registration
        std::cout << "execute registration -> ";
        FRICP<3> fricp;
        double begin_reg = omp_get_wtime();
        double converge_rmse = 0;
        switch(method)
        {
            case ICP:
            {
                pars.f = ICP::NONE;
                pars.use_AA = false;
                fricp.point_to_point(vertices_source, vertices_target, source_mean, target_mean, pars);
                res_trans = pars.res_trans;
                break;
            }
            case AA_ICP:
            {
                AAICP::point_to_point_aaicp(vertices_source, vertices_target, source_mean, target_mean, pars);
                res_trans = pars.res_trans;
                break;
            }
            case FICP:
            {
                pars.f = ICP::NONE;
                pars.use_AA = true;
                fricp.point_to_point(vertices_source, vertices_target, source_mean, target_mean, pars);
                res_trans = pars.res_trans;
                break;
            }   
            case RICP:
            {
                pars.f = ICP::WELSCH;
                pars.use_AA = false;
                fricp.point_to_point(vertices_source, vertices_target, source_mean, target_mean, pars);
                res_trans = pars.res_trans;
                break;
            }
            case FR_ICP:
            {
                pars.f = ICP::WELSCH;
                pars.use_AA = true;
                fricp.point_to_point(vertices_source, vertices_target, source_mean, target_mean, pars);
                res_trans = pars.res_trans;
                break;
            }
            case PPL:
            {
                pars.f = ICP::NONE;
                pars.use_AA = false;
                if(normal_target.size() == 0)
                {
                    std::cout << "Warning! The target model without normals can't run Point-to-plane method!" << std::endl;
                    exit(0);
                }
                fricp.point_to_plane(vertices_source, vertices_target, normal_source, normal_target, source_mean, target_mean, pars);
                res_trans = pars.res_trans;
                break;
            }
            case RPPL:
            {
                pars.nu_end_k = 1.0/6;
                pars.f = ICP::WELSCH;
                pars.use_AA = true;
                if(normal_target.size()== 0)
                {
                    std::cout << "Warning! The target model without normals can't run Point-to-plane method!" << std::endl;
                    exit(0);
                }
                fricp.point_to_plane_GN(vertices_source, vertices_target, normal_source, normal_target, source_mean, target_mean, pars);
                res_trans = pars.res_trans;
                break;
            }
            case SparseICP:
            {
                SICP::point_to_point(vertices_source, vertices_target, source_mean, target_mean, spars);
                res_trans = spars.res_trans;
                break;
            }   
            case SICPPPL:
            {   
                if(normal_target.size() == 0)
                {
                    std::cout << "Warning! The target model without normals can't run Point-to-plane method!" << std::endl;
                    exit(0);
                }
                SICP::point_to_plane(vertices_source, vertices_target, normal_target, source_mean, target_mean, spars);
                res_trans = spars.res_trans;
                break;
            }
        }
        double end_reg = omp_get_wtime();
        time = end_reg - begin_reg;
        std::cout << "Registration cost(s): " << time << std::endl;
        fout_time << std::fixed << time << std::endl;

        // vertices_source = scale * vertices_source;
        Eigen::Affine3d res_T;
        res_T.linear() = res_trans.block(0,0,3,3);
        res_T.translation() = res_trans.block(0,3,3,1);
        res_trans.block(0,3,3,1) *= scale;
        std::cout << "scale: " << scale << std::endl;
        std::cout << "res_trans: " << res_trans << std::endl;

        return res_trans;
    }
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr detectDiff(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& source_res,
                                                      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& target,
                                                      const float& radius)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>());  
        result = detect(source_res, target, radius);
    }
};