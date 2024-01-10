///////////////////////////////////////////////////////////////////////////////
///   "Sparse Iterative Closest Point"
///   by Sofien Bouaziz, Andrea Tagliasacchi, Mark Pauly
///   Copyright (C) 2013  LGG, EPFL
///////////////////////////////////////////////////////////////////////////////
///   1) This file contains different implementations of the ICP algorithm.
///   2) This code requires EIGEN and NANOFLANN.
///   3) If OPENMP is activated some part of the code will be parallelized.
///   4) This code is for now designed for 3D registration
///   5) Two main input types are Eigen::Matrix3Xd or Eigen::Map<Eigen::Matrix3Xd>
///////////////////////////////////////////////////////////////////////////////
///   namespace nanoflann: NANOFLANN KD-tree adaptor for EIGEN
///   namespace RigidMotionEstimator: functions to compute the rigid motion
///   namespace SICP: sparse ICP implementation
///   namespace ICP: reweighted ICP implementation
///////////////////////////////////////////////////////////////////////////////
#ifndef ICP_H
#define ICP_H
#include "../nanoflann.hpp"
#include "AndersonAcceleration.h"
#include <time.h>
#include <fstream>
#include <algorithm>
#include "median.h"
#include <iostream>

#define TUPLE_SCALE	  0.95
#define TUPLE_MAX_CNT 1000


///////////////////////////////////////////////////////////////////////////////
namespace nanoflann {
    /// KD-tree adaptor for working with data directly stored in an Eigen Matrix, without duplicating the data storage.
    /// This code is adapted from the KDTreeEigenMatrixAdaptor class of nanoflann.hpp
    template <class MatrixType, int DIM = -1, class Distance = nanoflann::metric_L2, typename IndexType = int>
    struct KDTreeAdaptor {
        typedef KDTreeAdaptor<MatrixType, DIM, Distance> self_t;
        typedef typename MatrixType::Scalar              num_t;
        typedef typename Distance::template traits<num_t, self_t>::distance_t metric_t;
        typedef KDTreeSingleIndexAdaptor< metric_t, self_t, DIM, IndexType>  index_t;
        index_t* index;
        KDTreeAdaptor(const MatrixType &mat, const int leaf_max_size = 10) : m_data_matrix(mat) {
            const size_t dims = mat.rows();
            index = new index_t(dims, *this, nanoflann::KDTreeSingleIndexAdaptorParams(leaf_max_size, dims));
            index->buildIndex();
        }
        ~KDTreeAdaptor() { delete index; }
        const MatrixType &m_data_matrix;
        /// Query for the num_closest closest points to a given point (entered as query_point[0:dim-1]).
        inline void query(const num_t *query_point, const size_t num_closest, IndexType *out_indices, num_t *out_distances_sq) const {
            nanoflann::KNNResultSet<typename MatrixType::Scalar, IndexType> resultSet(num_closest);
            resultSet.init(out_indices, out_distances_sq);
            index->findNeighbors(resultSet, query_point, nanoflann::SearchParams());
        }
        /// Query for the closest points to a given point (entered as query_point[0:dim-1]).
        inline IndexType closest(const num_t *query_point) const {
            IndexType out_indices;
            num_t out_distances_sq;
            query(query_point, 1, &out_indices, &out_distances_sq);
            return out_indices;
        }
        const self_t & derived() const { return *this; }
        self_t & derived() { return *this; }
        inline size_t kdtree_get_point_count() const { return m_data_matrix.cols(); }
        /// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
        inline num_t kdtree_distance(const num_t *p1, const size_t idx_p2, size_t size) const {
            num_t s = 0;
            for (size_t i = 0; i<size; i++) {
                num_t d = p1[i] - m_data_matrix.coeff(i, idx_p2);
                s += d*d;
            }
            return s;
        }
        /// Returns the dim'th component of the idx'th point in the class:
        inline num_t kdtree_get_pt(const size_t idx, int dim) const {
            return m_data_matrix.coeff(dim, idx);
        }
        /// Optional bounding-box computation: return false to default to a standard bbox computation loop.
        template <class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
    };
}
///////////////////////////////////////////////////////////////////////////////
/// Compute the rigid motion for point-to-point and point-to-plane distances
namespace RigidMotionEstimator {
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Confidence weights
    template <typename Derived1, typename Derived2, typename Derived3>
    Eigen::Affine3d point_to_point(Eigen::MatrixBase<Derived1>& X,
        Eigen::MatrixBase<Derived2>& Y,
        const Eigen::MatrixBase<Derived3>& w) {
        int dim = X.rows();
        /// Normalize weight vector
        Eigen::VectorXd w_normalized = w / w.sum();
        /// De-mean
        Eigen::VectorXd X_mean(dim), Y_mean(dim);
        for (int i = 0; i<dim; ++i) {
            X_mean(i) = (X.row(i).array()*w_normalized.transpose().array()).sum();
            Y_mean(i) = (Y.row(i).array()*w_normalized.transpose().array()).sum();
        }
        X.colwise() -= X_mean;
        Y.colwise() -= Y_mean;

        /// Compute transformation
        Eigen::Affine3d transformation;
        MatrixXX sigma = X * w_normalized.asDiagonal() * Y.transpose();
        Eigen::JacobiSVD<MatrixXX> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
        if (svd.matrixU().determinant()*svd.matrixV().determinant() < 0.0) {
            VectorX S = VectorX::Ones(dim); S(dim-1) = -1.0;
            transformation.linear() = svd.matrixV()*S.asDiagonal()*svd.matrixU().transpose();
        }
        else {
            transformation.linear() = svd.matrixV()*svd.matrixU().transpose();
        }
        transformation.translation() = Y_mean - transformation.linear()*X_mean;
        /// Re-apply mean
        X.colwise() += X_mean;
        Y.colwise() += Y_mean;
        /// Apply transformation
//        X = transformation*X;
        /// Return transformation
        return transformation;
    }
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    template <typename Derived1, typename Derived2>
    inline Eigen::Affine3d point_to_point(Eigen::MatrixBase<Derived1>& X,
        Eigen::MatrixBase<Derived2>& Y) {
        return point_to_point(X, Y, Eigen::VectorXd::Ones(X.cols()));
    }
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Confidence weights
    /// @param Right hand side
    template <typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5>
    Eigen::Affine3d point_to_plane(Eigen::MatrixBase<Derived1>& X,
        Eigen::MatrixBase<Derived2>& Y,
        Eigen::MatrixBase<Derived3>& N,
        const Eigen::MatrixBase<Derived4>& w,
        const Eigen::MatrixBase<Derived5>& u) {
        typedef Eigen::Matrix<double, 6, 6> Matrix66;
        typedef Eigen::Matrix<double, 6, 1> Vector6;
        typedef Eigen::Block<Matrix66, 3, 3> Block33;
        /// Normalize weight vector
        Eigen::VectorXd w_normalized = w / w.sum();
        /// De-mean
        Eigen::Vector3d X_mean;
        for (int i = 0; i<3; ++i)
            X_mean(i) = (X.row(i).array()*w_normalized.transpose().array()).sum();
        X.colwise() -= X_mean;
        Y.colwise() -= X_mean;
        /// Prepare LHS and RHS
        Matrix66 LHS = Matrix66::Zero();
        Vector6 RHS = Vector6::Zero();
        Block33 TL = LHS.topLeftCorner<3, 3>();
        Block33 TR = LHS.topRightCorner<3, 3>();
        Block33 BR = LHS.bottomRightCorner<3, 3>();
        Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3, X.cols());
#pragma omp parallel
        {
#pragma omp for
            for (int i = 0; i<X.cols(); i++) {
                C.col(i) = X.col(i).cross(N.col(i));
            }
#pragma omp sections nowait
            {
#pragma omp section
                for (int i = 0; i<X.cols(); i++) TL.selfadjointView<Eigen::Upper>().rankUpdate(C.col(i), w(i));
#pragma omp section
                for (int i = 0; i<X.cols(); i++) TR += (C.col(i)*N.col(i).transpose())*w(i);
#pragma omp section
                for (int i = 0; i<X.cols(); i++) BR.selfadjointView<Eigen::Upper>().rankUpdate(N.col(i), w(i));
#pragma omp section
                for (int i = 0; i<C.cols(); i++) {
                    double dist_to_plane = -((X.col(i) - Y.col(i)).dot(N.col(i)) - u(i))*w(i);
                    RHS.head<3>() += C.col(i)*dist_to_plane;
                    RHS.tail<3>() += N.col(i)*dist_to_plane;
                }
            }
        }
        LHS = LHS.selfadjointView<Eigen::Upper>();
        /// Compute transformation
        Eigen::Affine3d transformation;
        Eigen::LDLT<Matrix66> ldlt(LHS);
        RHS = ldlt.solve(RHS);
        transformation = Eigen::AngleAxisd(RHS(0), Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(RHS(1), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(RHS(2), Eigen::Vector3d::UnitZ());
        transformation.translation() = RHS.tail<3>();
        /// Apply transformation
        X = transformation*X;
        /// Re-apply mean
        X.colwise() += X_mean;
        Y.colwise() += X_mean;
        transformation.translation() += -transformation.linear() * X_mean + X_mean;
        /// Return transformation
        return transformation;
    }
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Confidence weights
    template <typename Derived1, typename Derived2, typename Derived3, typename Derived4>
    inline Eigen::Affine3d point_to_plane(Eigen::MatrixBase<Derived1>& X,
        Eigen::MatrixBase<Derived2>& Yp,
        Eigen::MatrixBase<Derived3>& Yn,
        const Eigen::MatrixBase<Derived4>& w) {
        return point_to_plane(X, Yp, Yn, w, Eigen::VectorXd::Zero(X.cols()));
    }
}
///////////////////////////////////////////////////////////////////////////////
/// ICP implementation using ADMM/ALM/Penalty method
namespace SICP {
    struct Parameters {
        bool use_penalty = false; /// if use_penalty then penalty method else ADMM or ALM (see max_inner)
        double p = 1.0;           /// p norm
        double mu = 10.0;         /// penalty weight
        double alpha = 1.2;       /// penalty increase factor
        double max_mu = 1e5;      /// max penalty
        int max_icp = 100;        /// max ICP iteration
        int max_outer = 100;      /// max outer iteration
        int max_inner = 1;        /// max inner iteration. If max_inner=1 then ADMM else ALM
        double stop = 1e-5;       /// stopping criteria
        bool print_icpn = false;  /// (debug) print ICP iteration
        Eigen::Matrix4d init_trans = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d gt_trans = Eigen::Matrix4d::Identity();
        bool has_groundtruth = false;
        int convergence_iter = 0;
        double convergence_mse = 0.0;
        double convergence_gt_mse = 0.0;
        Eigen::Matrix4d res_trans = Eigen::Matrix4d::Identity();
        std::string file_err = "";
        std::string out_path = "";
        int total_iters = 0;
    };
    /// Shrinkage operator (Automatic loop unrolling using template)
    template<unsigned int I>
    inline double shrinkage(double mu, double n, double p, double s) {
        return shrinkage<I - 1>(mu, n, p, 1.0 - (p / mu)*std::pow(n, p - 2.0)*std::pow(s, p - 1.0));
    }
    template<>
    inline double shrinkage<0>(double, double, double, double s) { return s; }
    /// 3D Shrinkage for point-to-point
    template<unsigned int I>
    inline void shrink(Eigen::Matrix3Xd& Q, double mu, double p) {
        double Ba = std::pow((2.0 / mu)*(1.0 - p), 1.0 / (2.0 - p));
        double ha = Ba + (p / mu)*std::pow(Ba, p - 1.0);
#pragma omp parallel for
        for (int i = 0; i<Q.cols(); ++i) {
            double n = Q.col(i).norm();
            double w = 0.0;
            if (n > ha) w = shrinkage<I>(mu, n, p, (Ba / n + 1.0) / 2.0);
            Q.col(i) *= w;
        }
    }
    /// 1D Shrinkage for point-to-plane
    template<unsigned int I>
    inline void shrink(Eigen::VectorXd& y, double mu, double p) {
        double Ba = std::pow((2.0 / mu)*(1.0 - p), 1.0 / (2.0 - p));
        double ha = Ba + (p / mu)*std::pow(Ba, p - 1.0);
#pragma omp parallel for
        for (int i = 0; i<y.rows(); ++i) {
            double n = std::abs(y(i));
            double s = 0.0;
            if (n > ha) s = shrinkage<I>(mu, n, p, (Ba / n + 1.0) / 2.0);
            y(i) *= s;
        }
    }
    /// Sparse ICP with point to point
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Parameters
    template <typename Derived1, typename Derived2>
    void point_to_point(Eigen::MatrixBase<Derived1>& X,
        Eigen::MatrixBase<Derived2>& Y, Eigen::Vector3d& source_mean, Eigen::Vector3d& target_mean,
        Parameters& par) {
        /// Build kd-tree
        nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree(Y);
        /// Buffers
        Eigen::Matrix3Xd Q = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::Matrix3Xd Z = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::Matrix3Xd C = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::Matrix3Xd ori_X = X;
        Eigen::Affine3d T(par.init_trans);
        Eigen::Matrix3Xd X_gt;
        int nPoints = X.cols();
        X = T * X;
        Eigen::Matrix3Xd Xo1 = X;
        Eigen::Matrix3Xd Xo2 = X;
        double  gt_mse = 0.0, run_time;
        double begin_time, end_time;
        std::vector<double> gt_mses, times;

        if(par.has_groundtruth)
        {
            Eigen::Vector3d temp_trans = par.gt_trans.block(0, 3, 3, 1);
            X_gt = ori_X;
            X_gt.colwise() += source_mean;
            X_gt = par.gt_trans.block(0, 0, 3, 3) * X_gt;
            X_gt.colwise() += temp_trans - target_mean;
        }

        begin_time = omp_get_wtime();

        /// ICP
        int icp;
        for (icp = 0; icp<par.max_icp; ++icp) {
            /// Find closest point
#pragma omp parallel for
            for (int i = 0; i<X.cols(); ++i) {
                Q.col(i) = Y.col(kdtree.closest(X.col(i).data()));
            }

            end_time = omp_get_wtime();
            run_time = end_time - begin_time;
            ///calc mse and gt_mse
            if(par.has_groundtruth)
            {
                gt_mse = (X - X_gt).squaredNorm() / nPoints;
            }
            times.push_back(run_time);
            gt_mses.push_back(gt_mse);
//            if(par.print_icpn)
//                std::cout << "iter = " << icp << ", time = " << run_time << ", mse = " << mse << ", gt_mse = " << gt_mse << std::endl;

            /// Computer rotation and translation
            double mu = par.mu;
            for (int outer = 0; outer<par.max_outer; ++outer) {
                double dual = 0.0;
                for (int inner = 0; inner<par.max_inner; ++inner) {
                    /// Z update (shrinkage)
                    Z = X - Q + C / mu;
                    shrink<3>(Z, mu, par.p);
                    /// Rotation and translation update
                    Eigen::Matrix3Xd U = Q + Z - C / mu;
                    Eigen::Affine3d cur_T = RigidMotionEstimator::point_to_point(X, U);
                    X = cur_T * X;
                    T = cur_T * T;
                    /// Stopping criteria
                    dual = pow((X - Xo1).norm(),2) / nPoints;
                    Xo1 = X;
                    if (dual < par.stop) break;
                }
                /// C update (lagrange multipliers)
                Eigen::Matrix3Xd P = X - Q - Z;
                if (!par.use_penalty) C.noalias() += mu*P;
                /// mu update (penalty)
                if (mu < par.max_mu) mu *= par.alpha;
                /// Stopping criteria
                double primal = P.colwise().norm().maxCoeff();
                if (primal < par.stop && dual < par.stop) break;
            }

            /// Stopping criteria
            double stop = (X-Xo2).colwise().norm().maxCoeff();
            Xo2 = X;
            if (stop < par.stop) break;
        }
        if(par.has_groundtruth)
            gt_mse = (X-X_gt).squaredNorm()/nPoints;

        if(par.print_icpn)
        {
            std::ofstream out_res(par.out_path);
            for(int i = 0; i<times.size(); i++)
            {
                out_res << times[i] << " " << gt_mses[i] << std::endl;
            }
            out_res.close();
        }

        T.translation().noalias() += -T.rotation()*source_mean + target_mean;
        X.colwise() += target_mean;

        ///save convergence result
        par.convergence_gt_mse = gt_mse;
        par.convergence_iter = icp;
        par.res_trans = T.matrix();
    }
    /// Sparse ICP with point to plane
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Parameters
    template <typename Derived1, typename Derived2, typename Derived3>
    void point_to_plane(Eigen::MatrixBase<Derived1>& X,
        Eigen::MatrixBase<Derived2>& Y,
        Eigen::MatrixBase<Derived3>& N, Eigen::Vector3d source_mean, Eigen::Vector3d target_mean,
        Parameters &par) {
        /// Build kd-tree
        nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree(Y);
        /// Buffers
        Eigen::Matrix3Xd Qp = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::Matrix3Xd Qn = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::VectorXd Z = Eigen::VectorXd::Zero(X.cols());
        Eigen::VectorXd C = Eigen::VectorXd::Zero(X.cols());
        Eigen::Matrix3Xd ori_X = X;
        Eigen::Affine3d T(par.init_trans);
        Eigen::Matrix3Xd X_gt;
        int nPoints = X.cols();
        X = T*X;
        Eigen::Matrix3Xd Xo1 = X;
        Eigen::Matrix3Xd Xo2 = X;
        double gt_mse = 0.0, run_time;
        double begin_time, end_time;
        std::vector<double> gt_mses, times;

        if(par.has_groundtruth)
        {
            Eigen::Vector3d temp_trans = par.gt_trans.block(0, 3, 3, 1);
            X_gt = ori_X;
            X_gt.colwise() += source_mean;
            X_gt = par.gt_trans.block(0, 0, 3, 3) * X_gt;
            X_gt.colwise() += temp_trans - target_mean;
        }

        begin_time = omp_get_wtime();

        /// ICP
        int icp;
        int total_iters = 0;
        for (icp = 0; icp<par.max_icp; ++icp) {

            /// Find closest point
#pragma omp parallel for
            for (int i = 0; i<X.cols(); ++i) {
                int id = kdtree.closest(X.col(i).data());
                Qp.col(i) = Y.col(id);
                Qn.col(i) = N.col(id);
            }

            end_time = omp_get_wtime();
            run_time = end_time - begin_time;
            ///calc mse and gt_mse
            if(par.has_groundtruth)
            {
                gt_mse = (X - X_gt).squaredNorm() / nPoints;
            }
            times.push_back(run_time);
            gt_mses.push_back(gt_mse);

            if(par.print_icpn)
                std::cout << "iter = " << icp << ", time = " << run_time << ", gt_mse = " << gt_mse << std::endl;

            /// Computer rotation and translation
            double mu = par.mu;
            for (int outer = 0; outer<par.max_outer; ++outer) {
                double dual = 0.0;
                for (int inner = 0; inner<par.max_inner; ++inner) {
                    total_iters++;
                    /// Z update (shrinkage)
                    Z = (Qn.array()*(X - Qp).array()).colwise().sum().transpose() + C.array() / mu;
                    shrink<3>(Z, mu, par.p);
                    /// Rotation and translation update
                    Eigen::VectorXd U = Z - C / mu;
                    T = RigidMotionEstimator::point_to_plane(X, Qp, Qn, Eigen::VectorXd::Ones(X.cols()), U)*T;
                    /// Stopping criteria
                    dual = (X - Xo1).colwise().norm().maxCoeff();
                    Xo1 = X;
                    if (dual < par.stop) break;
                }
                /// C update (lagrange multipliers)
                Eigen::VectorXd P = (Qn.array()*(X - Qp).array()).colwise().sum().transpose() - Z.array();
                if (!par.use_penalty) C.noalias() += mu*P;
                /// mu update (penalty)
                if (mu < par.max_mu) mu *= par.alpha;
                /// Stopping criteria
                double primal = P.array().abs().maxCoeff();
                if (primal < par.stop && dual < par.stop) break;
            }
            /// Stopping criteria
            double stop = (X - Xo2).colwise().norm().maxCoeff();
            Xo2 = X;
            if (stop < par.stop) break;
        }
        if(par.has_groundtruth)
        {
            gt_mse = (X-X_gt).squaredNorm()/nPoints;
        }
        if(par.print_icpn)
        {
            std::ofstream out_res(par.out_path);
            for(int i = 0; i<times.size(); i++)
            {
                out_res << times[i] << " " <<gt_mses[i] << std::endl;
            }
            out_res.close();
        }

        T.translation() += - T.rotation() * source_mean + target_mean;
        X.colwise() += target_mean;
        ///save convergence result
        par.convergence_gt_mse = gt_mse;
        par.convergence_iter = icp;
        par.res_trans = T.matrix();
        par.total_iters = total_iters;
    }
}
///////////////////////////////////////////////////////////////////////////////
/// ICP implementation using iterative reweighting
namespace ICP {
    enum Function {
        PNORM,
        TUKEY,
        FAIR,
        LOGISTIC,
        TRIMMED,
        WELSCH,
        AUTOWELSCH,
        NONE
    };
    enum AAElementType {
        EULERANGLE,
        QUATERNION,
        LOG_TRANS,
        ROTATION,
        FPFH,
        DUAL_QUATERNION
    };
    class Parameters {
    public:
        Parameters() : f(NONE),
            p(0.1),
            max_icp(100),
            max_outer(1),
            stop(1e-5),
            use_AA(false),
            print_energy(false),
            print_output(false),
            anderson_m(5),
            beta_(1.0),
            error_overflow_threshold_(0.05),
            has_groundtruth(false),
            gt_trans(Eigen::Matrix4d::Identity()),
            convergence_energy(0.0),
            convergence_iter(0),
            convergence_gt_mse(0.0),
            nu_begin_k(3),
            nu_end_k(1.0/(3.0*sqrt(3.0))),
            use_init(false),
            nu_alpha(1.0/2) {}
        /// Parameters
        Function f;     /// robust function type
        double p;       /// paramter of the robust function/// para k
        int max_icp;    /// max ICP iteration
        int max_outer;  /// max outer iteration
        double stop;    /// stopping criteria
        bool use_AA;  /// whether using anderson acceleration
        std::string out_path;
        bool print_energy;///whether print energy
        bool print_output; ///whether write result to txt
        int anderson_m;
        double beta_;
        double error_overflow_threshold_;
        MatrixXX init_trans;
        MatrixXX gt_trans;
        bool has_groundtruth;
        double convergence_energy;
        int convergence_iter;
        double convergence_gt_mse;
        MatrixXX res_trans;
        double nu_begin_k;
        double nu_end_k;
        bool use_init;
        double nu_alpha;
    };
    /// Weight functions
    /// @param Residuals
    /// @param Parameter
    void uniform_weight(Eigen::VectorXd& r) {
        r = Eigen::VectorXd::Ones(r.rows());
    }
    /// @param Residuals
    /// @param Parameter
    void pnorm_weight(Eigen::VectorXd& r, double p, double reg = 1e-8) {
        for (int i = 0; i<r.rows(); ++i) {
            r(i) = p / (std::pow(r(i), 2 - p) + reg);
        }
    }
    /// @param Residuals
    /// @param Parameter
    void tukey_weight(Eigen::VectorXd& r, double p) {
        for (int i = 0; i<r.rows(); ++i) {
            if (r(i) > p) r(i) = 0.0;
            else r(i) = std::pow((1.0 - std::pow(r(i) / p, 2.0)), 2.0);
        }
    }
    /// @param Residuals
    /// @param Parameter
    void fair_weight(Eigen::VectorXd& r, double p) {
        for (int i = 0; i<r.rows(); ++i) {
            r(i) = 1.0 / (1.0 + r(i) / p);
        }
    }
    /// @param Residuals
    /// @param Parameter
    void logistic_weight(Eigen::VectorXd& r, double p) {
        for (int i = 0; i<r.rows(); ++i) {
            r(i) = (p / r(i))*std::tanh(r(i) / p);
        }
    }
    struct sort_pred {
        bool operator()(const std::pair<int, double> &left,
            const std::pair<int, double> &right) {
            return left.second < right.second;
        }
    };
    /// @param Residuals
    /// @param Parameter
    void trimmed_weight(Eigen::VectorXd& r, double p) {
        std::vector<std::pair<int, double> > sortedDist(r.rows());
        for (int i = 0; i<r.rows(); ++i) {
            sortedDist[i] = std::pair<int, double>(i, r(i));
        }
        std::sort(sortedDist.begin(), sortedDist.end(), sort_pred());
        r.setZero();
        int nbV = r.rows()*p;
        for (int i = 0; i<nbV; ++i) {
            r(sortedDist[i].first) = 1.0;
        }
    }
    /// @param Residuals
    /// @param Parameter
    void welsch_weight(Eigen::VectorXd& r, double p) {
        for (int i = 0; i<r.rows(); ++i) {
            r(i) = std::exp(-r(i)*r(i)/(2*p*p));
        }
    }

    /// @param Residuals
    /// @param Parameter
    void autowelsch_weight(Eigen::VectorXd& r, double p) {
        double median;
        igl::median(r, median);
        welsch_weight(r, p*median/(std::sqrt(2)*2.3));
        //welsch_weight(r,p);
    }

    /// Energy functions
    /// @param Residuals
    /// @param Parameter
    double uniform_energy(Eigen::VectorXd& r) {
        double energy = 0;
        for (int i = 0; i<r.rows(); ++i) {
            energy += r(i)*r(i);
        }
        return energy;
    }
    /// @param Residuals
    /// @param Parameter
    double pnorm_energy(Eigen::VectorXd& r, double p, double reg = 1e-8) {
        double energy = 0;
        for (int i = 0; i<r.rows(); ++i) {
            energy += (r(i)*r(i))*p / (std::pow(r(i), 2 - p) + reg);
        }
        return energy;
    }
    /// @param Residuals
    /// @param Parameter
    double tukey_energy(Eigen::VectorXd& r, double p) {
        double energy = 0;
        double w;
        for (int i = 0; i<r.rows(); ++i) {
            if (r(i) > p) w = 0.0;
            else w = std::pow((1.0 - std::pow(r(i) / p, 2.0)), 2.0);

            energy += (r(i)*r(i))*w;
        }
        return energy;
    }
    /// @param Residuals
    /// @param Parameter
    double fair_energy(Eigen::VectorXd& r, double p) {
        double energy = 0;
        for (int i = 0; i<r.rows(); ++i) {
            energy += (r(i)*r(i))*1.0 / (1.0 + r(i) / p);
        }
        return energy;
    }
    /// @param Residuals
    /// @param Parameter
    double logistic_energy(Eigen::VectorXd& r, double p) {
        double energy = 0;
        for (int i = 0; i<r.rows(); ++i) {
            energy += (r(i)*r(i))*(p / r(i))*std::tanh(r(i) / p);
        }
        return energy;
    }
    /// @param Residuals
    /// @param Parameter
    double trimmed_energy(Eigen::VectorXd& r, double p) {
        std::vector<std::pair<int, double> > sortedDist(r.rows());
        for (int i = 0; i<r.rows(); ++i) {
            sortedDist[i] = std::pair<int, double>(i, r(i));
        }
        std::sort(sortedDist.begin(), sortedDist.end(), sort_pred());
        Eigen::VectorXd t = r;
        t.setZero();
        double energy = 0;
        int nbV = r.rows()*p;
        for (int i = 0; i<nbV; ++i) {
            energy += r(i)*r(i);
        }
        return energy;
    }

    /// @param Residuals
    /// @param Parameter
    double welsch_energy(Eigen::VectorXd& r, double p) {
        double energy = 0;
        for (int i = 0; i<r.rows(); ++i) {
            energy += 1.0 - std::exp(-r(i)*r(i)/(2*p*p));
        }
        return energy;
    }
    /// @param Residuals
    /// @param Parameter
    double autowelsch_energy(Eigen::VectorXd& r, double p) {
        double energy = 0;
        energy = welsch_energy(r, 0.5);
        return energy;
    }
    /// @param Function type
    /// @param Residuals
    /// @param Parameter
    void robust_weight(Function f, Eigen::VectorXd& r, double p) {
        switch (f) {
        case PNORM: pnorm_weight(r, p); break;
        case TUKEY: tukey_weight(r, p); break;
        case FAIR: fair_weight(r, p); break;
        case LOGISTIC: logistic_weight(r, p); break;
        case TRIMMED: trimmed_weight(r, p); break;
        case WELSCH: welsch_weight(r, p); break;
        case AUTOWELSCH: autowelsch_weight(r,p); break;
        case NONE: uniform_weight(r); break;
        default: uniform_weight(r); break;
        }
    }

    //Cacl energy
    double get_energy(Function f, Eigen::VectorXd& r, double p) {
        double energy = 0;
        switch (f) {
            //case PNORM: pnorm_weight(r,p); break;
        case TUKEY: energy = tukey_energy(r, p); break;
        case FAIR: energy = fair_energy(r, p); break;
        case LOGISTIC: energy = logistic_energy(r, p); break;
        case TRIMMED: energy = trimmed_energy(r, p); break;
        case WELSCH: energy = welsch_energy(r, p); break;
        case AUTOWELSCH: energy = autowelsch_energy(r, p); break;
        case NONE: energy = uniform_energy(r); break;
        default: energy = uniform_energy(r); break;
        }
        return energy;
    }
}
    namespace AAICP{
        typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
            typedef Eigen::Matrix<Scalar, 6, 1> Vector6;
            typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
            typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;
            typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorX;
            typedef Eigen::Matrix<Scalar, 6, Eigen::Dynamic> Matrix6X;

            ///aaicp
            ///////////////////////////////////////////////////////////////////////////////////////////
            Vector6 Matrix42Vector6 (const Matrix4 m)
            {
              Vector6 v;
              Matrix3 s = m.block(0,0,3,3);
              v.head(3) = s.eulerAngles(0, 1, 2);
              v.tail(3) = m.col(3).head(3);
              return v;
            }

            ///////////////////////////////////////////////////////////////////////////////////////////
            Matrix4 Vector62Matrix4 (const Vector6 v)
            {
              Matrix3 s (Eigen::AngleAxis<Scalar>(v(0), Vector3::UnitX())
                * Eigen::AngleAxis<Scalar>(v(1), Vector3::UnitY())
                * Eigen::AngleAxis<Scalar>(v(2), Vector3::UnitZ()));
              Matrix4 m = Matrix4::Zero();
              m.block(0,0,3,3) = s;
              m(3,3) = 1;
              m.col(3).head(3) = v.tail(3);
              return m;
            }

            ///////////////////////////////////////////////////////////////////////////////////////////
            int alphas_cond (VectorX alphas)
            {
              double alpha_limit_min_ = -10;
              double alpha_limit_max_ = 10;
              return alpha_limit_min_ < alphas.minCoeff() && alphas.maxCoeff() < alpha_limit_max_ && alphas(alphas.size()-1) > 0;
            }

            ///////////////////////////////////////////////////////////////////////////////////////////
            VectorX get_alphas_lstsq (const Matrix6X f)
            {
              Matrix6X A = f.leftCols(f.cols()-1);
              A *= -1;
              A += f.rightCols(1) * VectorX::Constant(f.cols()-1, 1).transpose();
              VectorX sol = A.colPivHouseholderQr().solve(f.rightCols(1));
              sol.conservativeResize(sol.size()+1);
              sol[sol.size()-1] = 0;
              sol[sol.size()-1] = 1-sol.sum();
              return sol;
            }

            ///////////////////////////////////////////////////////////////////////////////////////////
            VectorX get_next_u (const Matrix6X u, const Matrix6X g, const Matrix6X f, std::vector<double> & save_alphas)
            {
              int i = 1;
              double beta_ = 1.0;
              save_alphas.clear();
              Vector6 sol = ((1-beta_)*u.col(u.cols()-1) + beta_*g.col(g.cols()-1));
              VectorX sol_alphas(1);
              sol_alphas << 1;

              i = 2;
              for (; i <= f.cols(); i++)
              {
                VectorX alphas = get_alphas_lstsq(f.rightCols(i));
                if (!alphas_cond(alphas))
                {
                    break;
                }
                sol = (1-beta_)*u.rightCols(i)*alphas + beta_*g.rightCols(i)*alphas;
                sol_alphas = alphas;
              }
              for(int i= 0; i<sol_alphas.rows(); i++)
              {
                  save_alphas.push_back(sol_alphas[i]);
              }
              return sol;
            }


            template <typename Derived1, typename Derived2, typename Derived3>
            void point_to_point_aaicp(Eigen::MatrixBase<Derived1>& X,Eigen::MatrixBase<Derived2>& Y, Eigen::MatrixBase<Derived3>& source_mean,
                               Eigen::MatrixBase<Derived3>& target_mean,
                ICP::Parameters& par) {
                /// Build kd-tree
                nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree(Y);
                /// Buffers
                Eigen::Matrix3Xd Q = Eigen::Matrix3Xd::Zero(3, X.cols());
                Eigen::VectorXd W = Eigen::VectorXd::Zero(X.cols());
                Eigen::Matrix3Xd ori_X = X;

                double prev_energy = std::numeric_limits<double>::max(), energy = std::numeric_limits<double>::max();
                Eigen::Affine3d T;
                if(par.use_init)
                {
                    T.linear() = par.init_trans.block(0,0,3,3);
                    T.translation() = par.init_trans.block(0,3,3,1);
                }
                else
                    T = Eigen::Affine3d::Identity();
                Eigen::Matrix3Xd X_gt = X;
                ///stop criterion paras
                MatrixXX To1 =T.matrix();
                MatrixXX To2 = T.matrix();

                 ///AA paras
                 Matrix6X u(6,0), g(6,0), f(6,0);
                 Vector6 u_next, u_k;
                 Matrix4 transformation = Matrix4::Identity();
                 Matrix4 final_transformation = Matrix4::Identity();

                 ///output para
                 std::vector<double> times, energys, gt_mses;
                 double gt_mse;
                 double begin_time, end_time, run_time;
                 begin_time = omp_get_wtime();

                 ///output coeffs
                 std::vector<std::vector<double>> coeffs;
                 coeffs.clear();
                 std::vector<double> alphas;

                 X = T * X;
                 ///groud truth target point cloud
                 if(par.has_groundtruth)
                 {
                     Eigen::Vector3d temp_trans = par.gt_trans.block(0, 3, 3, 1);
                     X_gt = ori_X;
                     X_gt.colwise() += source_mean;
                     X_gt = par.gt_trans.block(0, 0, 3, 3) * X_gt;
                     X_gt.colwise() += temp_trans - target_mean;
                 }

                 ///begin ICP
                 int icp = 0;
                 for (; icp<par.max_icp; ++icp)
                 {
                     bool accept_aa = false;
                     int nPoints = X.cols();
                     end_time = omp_get_wtime();
                     run_time = end_time - begin_time;
                     /// Find closest point
         #pragma omp parallel for
                     for (int i = 0; i<nPoints; ++i) {
                         Q.col(i) = Y.col(kdtree.closest(X.col(i).data()));
                     }

                     ///calc time

                     times.push_back(run_time);
                     if(par.has_groundtruth)
                     {
                         gt_mse = pow((X - X_gt).norm(),2) / nPoints;
                     }
                     gt_mses.push_back(gt_mse);

                     /// Computer rotation and translation
                     /// Compute weights
                     W = (X - Q).colwise().norm();
                     robust_weight(par.f, W, par.p);

                     /// Rotation and translation update
                     T = RigidMotionEstimator::point_to_point(X, Q, W) * T;
                     final_transformation = T.matrix();

                     ///Anderson acceleration
                     if(icp)
                     {
                         Vector6 g_k = Matrix42Vector6(transformation * final_transformation);
                         // Calculate energy
                         W = (X - Q).colwise().norm();
                         energy = get_energy(par.f, W, par.p);

                         ///The first heuristic
                         if ((energy - prev_energy)/prev_energy > par.error_overflow_threshold_) {
                             u_next = u_k = g.rightCols(1);
                             prev_energy = std::numeric_limits<double>::max();
                             u = u.rightCols(2);
                             g = g.rightCols(1);
                             f = f.rightCols(1);
                         }
                         else
                         {
                             prev_energy = energy;

                             g.conservativeResize(g.rows(),g.cols()+1);
                             g.col(g.cols()-1) = g_k;

                             Vector6 f_k = g_k - u_k;
                             f.conservativeResize(f.rows(),f.cols()+1);
                             f.col(f.cols()-1) = f_k;

                             u_next = get_next_u(u, g, f, alphas);
                             u.conservativeResize(u.rows(),u.cols()+1);
                             u.col(u.cols()-1) = u_next;

                             u_k = u_next;
                             accept_aa = true;
                         }
                     }
                     ///init
                     else
                     {
                         // Calculate energy
                         W = (X - Q).colwise().norm();
                         prev_energy = get_energy(par.f, W, par.p);
                         Vector6 u0 = Matrix42Vector6(Matrix4::Identity());
                         u.conservativeResize(u.rows(),u.cols()+1);
                         u.col(0)=u0;

                         Vector6 u1 = Matrix42Vector6(transformation * final_transformation);
                         g.conservativeResize(g.rows(),g.cols()+1);
                         g.col(0)=u1;

                         u.conservativeResize(u.rows(),u.cols()+1);

                         u.col(1)=u1;

                         f.conservativeResize(f.rows(),f.cols()+1);
                         f.col(0)=u1 - u0;

                         u_next = u1;
                         u_k = u1;

                         energy = prev_energy;
                     }

                     transformation = Vector62Matrix4(u_next)*(final_transformation.inverse());
                     final_transformation = Vector62Matrix4(u_next);
                     X = final_transformation.block(0,0,3,3) * ori_X;
                     Vector3 trans = final_transformation.block(0,3,3,1);
                     X.colwise() += trans;

                     energys.push_back(energy);

                     if (par.print_energy)
                         std::cout << "icp iter = " << icp << ", Energy = " << energy << ", gt_mse = " << gt_mse<< std::endl;

                     /// Stopping criteria
                     double stop2 = (final_transformation - To2).norm();
                     To2 = final_transformation;
                     if (stop2 < par.stop && icp) break;
                 }

                 W = (X - Q).colwise().norm();
                 double last_energy = get_energy(par.f, W, par.p);
                 gt_mse = pow((X - X_gt).norm(),2) / X.cols();

                 final_transformation.block(0,3,3,1) += -final_transformation.block(0, 0, 3, 3)*source_mean + target_mean;
                 X.colwise() += target_mean;

                 ///save convergence result
                 par.convergence_energy = last_energy;
                 par.convergence_gt_mse = gt_mse;
                 par.convergence_iter = icp;
                 par.res_trans = final_transformation;

                 ///output
                 if (par.print_output)
                 {
                     std::ofstream out_res(par.out_path);
                     if (!out_res.is_open())
                     {
                         std::cout << "Can't open out file " << par.out_path << std::endl;
                     }
                     ///output time and energy
                     out_res.precision(16);
                     for (int i = 0; i<icp; i++)
                     {
                         out_res << times[i] << " " << energys[i] <<" " << gt_mses[i] << std::endl;
                     }
                     out_res.close();
                 }
    }
    }
#endif
