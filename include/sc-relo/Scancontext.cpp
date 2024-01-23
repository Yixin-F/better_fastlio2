#include "Scancontext.h"
#include <fstream>
#include <iomanip>

namespace ScanContext
{

    void coreImportTest(void)
    {
        cout << "scancontext lib is successfully imported." << endl;
    } // coreImportTest

    float rad2deg(float radians)
    {
        return radians * 180.0 / M_PI;
    }

    float deg2rad(float degrees)
    { 
        return degrees * M_PI / 180.0;
    }

    float xy2theta(const float &_x, const float &_y)
    {
        if (_x >= 0 & _y >= 0)
            return (180 / M_PI) * atan(_y / _x);

        else if (_x < 0 & _y >= 0)
            return 180 - ((180 / M_PI) * atan(_y / (-_x)));

        else if (_x < 0 & _y < 0)
            return 180 + ((180 / M_PI) * atan(_y / _x));

        else
            return 360 - ((180 / M_PI) * atan((-_y) / _x));
    } // xy2theta

    /**
     * @brief  对矩阵进行循环右移
     *
     * @param[in] _mat  输入矩阵
     * @param[in] _num_shift   循环右移的列数
     * @return MatrixXd  移动之后最终的矩阵
     */
    MatrixXd circshift(MatrixXd &_mat, int _num_shift)
    {
        // shift columns to right direction
        assert(_num_shift >= 0);

        if (_num_shift == 0)
        {
            MatrixXd shifted_mat(_mat);
            return shifted_mat; // Early return
        }

        MatrixXd shifted_mat = MatrixXd::Zero(_mat.rows(), _mat.cols());
        for (int col_idx = 0; col_idx < _mat.cols(); col_idx++)
        {
            int new_location = (col_idx + _num_shift) % _mat.cols();
            shifted_mat.col(new_location) = _mat.col(col_idx);
        }

        return shifted_mat;

    } // circshift

    std::vector<float> eig2stdvec(MatrixXd _eigmat)
    {
        std::vector<float> vec(_eigmat.data(), _eigmat.data() + _eigmat.size());
        return vec;
    } // eig2stdvec

    /**
     * @brief 输入两个_sc矩阵，计算他们之间的SC距离
     *
     * @param[in] _sc1
     * @param[in] _sc2
     * @return double
     */
    double SCManager::distDirectSC(MatrixXd &_sc1, MatrixXd &_sc2)
    {
        int num_eff_cols = 0; // i.e., to exclude all-nonzero sector
        double sum_sector_similarity = 0;
        // 遍历两个SC矩阵的所有列
        for (int col_idx = 0; col_idx < _sc1.cols(); col_idx++)
        {
            VectorXd col_sc1 = _sc1.col(col_idx);
            VectorXd col_sc2 = _sc2.col(col_idx);

            // 如果其中有一列一个点云都没有，那么直接不比较
            if (col_sc1.norm() == 0 | col_sc2.norm() == 0)
                continue; // don't count this sector pair.

            // 求两个列向量之间的 cos(\theta)
            double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());

            sum_sector_similarity = sum_sector_similarity + sector_similarity;
            num_eff_cols = num_eff_cols + 1;
        }

        // 越相似，cos越大，得分越大
        double sc_sim = sum_sector_similarity / num_eff_cols;
        return 1.0 - sc_sim; // 然后1-cos，变成如果越相似，则值越小

    } // distDirectSC

    /**
     * @brief 输入两个sector key，寻找让他们两个最匹配的水平偏移
     *
     * @param[in] _vkey1
     * @param[in] _vkey2
     * @return int  _vkey2右移几个sector，结果和_vkey1最匹配
     */
    int SCManager::fastAlignUsingVkey(MatrixXd &_vkey1, MatrixXd &_vkey2)
    {
        int argmin_vkey_shift = 0;
        double min_veky_diff_norm = 10000000;
        for (int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++)
        {
            // 矩阵的列，循环右移shift个单位
            MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx);

            // 直接相减，sector key是1xN的矩阵，即一个行向量
            MatrixXd vkey_diff = _vkey1 - vkey2_shifted;

            double cur_diff_norm = vkey_diff.norm(); // 算范数
            // 查找最小的偏移量
            if (cur_diff_norm < min_veky_diff_norm)
            {
                argmin_vkey_shift = shift_idx;
                min_veky_diff_norm = cur_diff_norm;
            }
        }

        return argmin_vkey_shift;

    } // fastAlignUsingVkey

    /**
     * @brief 输入两个Scan-Context矩阵，计算它们之间的相似度得分
     *
     * @param[in] _sc1
     * @param[in] _sc2
     * @return std::pair<double, int>  <最小的SC距离，此时_sc2应该右移几列>
     */
    std::pair<double, int> SCManager::distanceBtnScanContext(MatrixXd &_sc1, MatrixXd &_sc2)
    {
        // Step 1. 使用sector-key快速对齐，把矩阵的列进行移动
        // 1. fast align using variant key (not in original IROS18)
        // 计算sector Key,也就是sector最大高度均值组成的数组，1xN
        MatrixXd vkey_sc1 = makeSectorkeyFromScancontext(_sc1);
        MatrixXd vkey_sc2 = makeSectorkeyFromScancontext(_sc2);
        // 这里将_vkey2循环右移，然后跟_vkey1作比较，找到一个最相似（二者做差最小）的时候，记下循环右移的量
        int argmin_vkey_shift = fastAlignUsingVkey(vkey_sc1, vkey_sc2);

        // 上面用sector key匹配，找到一个初始的偏移量，但肯定不是准确的，再在这个偏移量左右扩展一下搜索空间
        // 注意这个SEARCH_RADIUS是区间的一半，即左右偏移。这里是0.5* 10% * 60 = 3，也就是左右扩展3列
        const int SEARCH_RADIUS = round(0.5 * SEARCH_RATIO * _sc1.cols()); // a half of search range
        std::vector<int> shift_idx_search_space{argmin_vkey_shift};
        for (int ii = 1; ii < SEARCH_RADIUS + 1; ii++)
        {
            //! 疑问：这里感觉+ii的时候不用再加_sc1.cols()？
            shift_idx_search_space.push_back((argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols());
            shift_idx_search_space.push_back((argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols());
        }
        std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

        // Step 2. 对_sc2循环右移，计算最相近的scan context
        // 2. fast columnwise diff
        int argmin_shift = 0;
        double min_sc_dist = 10000000;
        for (int num_shift : shift_idx_search_space)
        {
            // sc2循环右移几位，注意这里是对矩阵进行右移，而不是移动sector-key
            MatrixXd sc2_shifted = circshift(_sc2, num_shift);
            // 计算两个SC之间的距离
            double cur_sc_dist = distDirectSC(_sc1, sc2_shifted);
            if (cur_sc_dist < min_sc_dist)
            {
                argmin_shift = num_shift;
                min_sc_dist = cur_sc_dist;
            }
        }

        return make_pair(min_sc_dist, argmin_shift);

    } // distanceBtnScanContext

    /**
     * @brief 输入一帧点云，生成Scan-Context
     *
     * @param[in] _scan_down, PointType类型，是pcl::PointXYZI
     * @return MatrixXd, 生成的Scan-Context矩阵
     */
    MatrixXd SCManager::makeScancontext(pcl::PointCloud<PointType> &_scan_down)
    {
        // TicToc t_making_desc;

        // 这帧点云的点的数量
        int num_pts_scan_down = _scan_down.points.size();

        // Step 1. 创建bin的矩阵，并把每个bin内点云最大高度初始化为-1000
        // main
        const int NO_POINT = -1000; // 标记格子中是否有点，如果没有点高度设置成-1000, 是一个肯定没有的值
        // ring行、sector列的矩阵，和论文中一致
        MatrixXd desc = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);

        PointType pt;
        float azim_angle, azim_range; // wihtin 2d plane
        int ring_idx, sctor_idx;
        // Step 2. 遍历每个点，往bin中赋值点云最大高度
        for (size_t pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++)
        {
            pt.x = _scan_down.points[pt_idx].x;
            pt.y = _scan_down.points[pt_idx].y;
            //! 疑问：这里把所有的高度+2，让高度>0，为什么要这么做？
            // 解答：目前感觉就是对于地面机器人这种场景，安装高度不变，然后把安装高度加上去之后，
            //      让安装高度之下的点云的高度从负数也都变成正数
            pt.z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT; // naive adding is ok (all points should be > 0).

            // xyz to ring, sector
            azim_range = sqrt(pt.x * pt.x + pt.y * pt.y); // 距离
            azim_angle = xy2theta(pt.x, pt.y);            // 角度

            // if range is out of roi, pass
            // 距离超过80米的点不考虑
            if (azim_range > PC_MAX_RADIUS)
                continue;

            // 计算这个点落到那个bin中，下标从1开始数。注意下面先min再max，其实就是把结果限制到1~PC_NUM之间
            ring_idx = std::max(std::min(PC_NUM_RING, int(ceil((azim_range / PC_MAX_RADIUS) * PC_NUM_RING))), 1);
            sctor_idx = std::max(std::min(PC_NUM_SECTOR, int(ceil((azim_angle / 360.0) * PC_NUM_SECTOR))), 1);

            // taking maximum z
            // 用z值，也就是高度来更新这个格子，存最大的高度。
            // 注意之这里为什么-1，以为数组的索引从0开始，上面的索引是[1, PC_NUM]，而在编程中数组的索引应该是[0, PC_NUM-1]
            if (desc(ring_idx - 1, sctor_idx - 1) < pt.z) // -1 means cpp starts from 0
                desc(ring_idx - 1, sctor_idx - 1) = pt.z; // update for taking maximum value at that bin
        }

        // Step 3. 把bin中没有点的那些，高度设置成0
        // reset no points to zero (for cosine dist later)
        for (int row_idx = 0; row_idx < desc.rows(); row_idx++)
            for (int col_idx = 0; col_idx < desc.cols(); col_idx++)
                if (desc(row_idx, col_idx) == NO_POINT)
                    desc(row_idx, col_idx) = 0;

        // t_making_desc.toc("PolarContext making");

        return desc;
    } // SCManager::makeScancontext

    /**
     * @brief 输入构造的SC矩阵，计算ring-key。其实就是对于矩阵的每一行（对应每一个环），
     *        计算这一行的平均值（即计算一个环中点云最大高度的平均值）
     *
     * @param[in] _desc
     * @return MatrixXd， ring行的向量，每个值都是每个环的平均值
     */
    MatrixXd SCManager::makeRingkeyFromScancontext(Eigen::MatrixXd &_desc)
    {
        /*
         * summary: rowwise mean vector
         */
        Eigen::MatrixXd invariant_key(_desc.rows(), 1);
        for (int row_idx = 0; row_idx < _desc.rows(); row_idx++)
        {
            Eigen::MatrixXd curr_row = _desc.row(row_idx);
            // 计算平均值。注意这个命名很有意思，说ring-key是不变的key，这是由于ring具有旋转不变性
            invariant_key(row_idx, 0) = curr_row.mean();
        }

        return invariant_key;
    } // SCManager::makeRingkeyFromScancontext

    /**
     * @brief 输入构造的SC矩阵，计算sector-key。其实就是对于矩阵的每一列（对应每一个扇区），
     *        计算这一列的平均值（即计算一个扇区中点云最大高度的平均值）
     *
     * @param[in] _desc
     * @return MatrixXd
     */
    MatrixXd SCManager::makeSectorkeyFromScancontext(Eigen::MatrixXd &_desc)
    {
        /*
         * summary: columnwise mean vector
         */
        Eigen::MatrixXd variant_key(1, _desc.cols());
        for (int col_idx = 0; col_idx < _desc.cols(); col_idx++)
        {
            Eigen::MatrixXd curr_col = _desc.col(col_idx);
            // 计算平均值，这里说sector是变化的key，以为旋转一下之后，variant_key中相当于不同位置之间进行了交换
            variant_key(0, col_idx) = curr_col.mean();
        }

        return variant_key;
    } // SCManager::makeSectorkeyFromScancontext

    /**
     * @brief 输入一帧点云，生成ScanContext，并计算ring-key和sector-key，然后存到数据库中
     *
     * @param[in] _scan_down
     */
    void SCManager::makeAndSaveScancontextAndKeys(pcl::PointCloud<PointType> &_scan_down)
    {
        // Step 1. 对输入点云计算Scan-Context矩阵
        Eigen::MatrixXd sc = makeScancontext(_scan_down); // v1

        // Step 2. 使用计算的Scan-Context矩阵，计算ring-key和sector-key
        Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(sc);     // ring-key旋转不变
        Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext(sc); // sector-key旋转变化
        // 把ring-key的Eigen向量，转成std::vector的数据格式
        // 最终就是使用ring-key在历史帧中查询相同的ring-key来得到候选匹配，然后计算Scan-Context距离
        std::vector<float> polarcontext_invkey_vec = eig2stdvec(ringkey);

        // Step 3. 把这帧的数据存到类成员变量中，即存到数据库中
        polarcontexts_.push_back(sc);
        polarcontext_invkeys_.push_back(ringkey);
        polarcontext_vkeys_.push_back(sectorkey);
        polarcontext_invkeys_mat_.push_back(polarcontext_invkey_vec);

        // cout <<polarcontext_vkeys_.size() << endl;

    } // SCManager::makeAndSaveScancontextAndKeys

    void SCManager::saveScancontextAndKeys( Eigen::MatrixXd _scd ){
        Eigen::MatrixXd ringkey = makeRingkeyFromScancontext( _scd );
        Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext( _scd );
        std::vector<float> polarcontext_invkey_vec = eig2stdvec( ringkey );

        polarcontexts_.push_back( _scd ); 
        polarcontext_invkeys_.push_back( ringkey );
        polarcontext_vkeys_.push_back( sectorkey );
        polarcontext_invkeys_mat_.push_back( polarcontext_invkey_vec );
    } // SCManager::SaveScancontextAndKeys

    std::pair<int, float> SCManager::detectLoopClosureIDBetweenSession (std::vector<float>& _curr_key, Eigen::MatrixXd& _curr_desc){
        int loop_id { -1 }; // init with -1, -1 means no loop (== LeGO-LOAM's variable "closestHistoryFrameID")

        auto& curr_key = _curr_key;
        auto& curr_desc = _curr_desc; // current observation (query)

        // step 0: if first, construct the tree in batch
        if( ! is_tree_batch_made ) // run only once
        {
            polarcontext_invkeys_to_search_.clear();
            polarcontext_invkeys_to_search_.assign( polarcontext_invkeys_mat_.begin(), polarcontext_invkeys_mat_.end() ) ;

            polarcontext_tree_batch_.reset(); 
            polarcontext_tree_batch_ = std::make_unique<InvKeyTree>(PC_NUM_RING /* dim */, polarcontext_invkeys_to_search_, 10 /* max leaf */ );

            is_tree_batch_made = true; // for running this block only once
        }
        
        double min_dist = 10000000; // init with somthing large
        int nn_align = 0;
        int nn_idx = 0;

        // step 1: knn search
        std::vector<size_t> candidate_indexes( NUM_CANDIDATES_FROM_TREE ); 
        std::vector<float> out_dists_sqr( NUM_CANDIDATES_FROM_TREE );

        nanoflann::KNNResultSet<float> knnsearch_result( NUM_CANDIDATES_FROM_TREE );
        knnsearch_result.init( &candidate_indexes[0], &out_dists_sqr[0] );
        polarcontext_tree_batch_->index->findNeighbors( knnsearch_result, &curr_key[0] /* query */, nanoflann::SearchParams(10) ); // error here

        // step 2: pairwise distance (find optimal columnwise best-fit using cosine distance)
        TicToc t_calc_dist;   
        for ( int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++ )
        {
            MatrixXd polarcontext_candidate = polarcontexts_[ candidate_indexes[candidate_iter_idx] ];
            std::pair<double, int> sc_dist_result = distanceBtnScanContext( curr_desc, polarcontext_candidate ); 
        
            double candidate_dist = sc_dist_result.first;
            int candidate_align = sc_dist_result.second;

            if( candidate_dist < min_dist ){
                min_dist = candidate_dist;
                nn_align = candidate_align;

                nn_idx = candidate_indexes[candidate_iter_idx];
            }
        }
        t_calc_dist.toc("Distance calc");

        // step 3: similarity threshold
        if( min_dist < SC_DIST_THRES )
            loop_id = nn_idx; 

        // To do: return also nn_align (i.e., yaw diff)
        float yaw_diff_rad = deg2rad(nn_align * PC_UNIT_SECTORANGLE);
        std::pair<int, float> result {loop_id, yaw_diff_rad};

        return result;

    } // SCManager::detectLoopClosureIDBetweenSession

    const Eigen::MatrixXd& SCManager::getConstRefRecentSCD(void){
        return polarcontexts_.back();
    }

    std::pair<int, float> SCManager::detectClosestKeyframeID(int num_exclude_recent, const std::vector<float> &curr_key, Eigen::MatrixXd &curr_desc)
    {
        int loop_id{-1}; // init with -1, -1 means no loop (== LeGO-LOAM's variable "closestHistoryFrameID")

        /*
         * step 1: candidates from ringkey tree_
         */
        // Step 1. 数据库中关键帧数量太少，则不检测回环？
        if (polarcontext_invkeys_mat_.size() < num_exclude_recent + 1)
        {
            std::pair<int, float> result{loop_id, 0.0};
            return result; // Early return
        }

        // Step 2. 经过一段时间之后，就重新构造 ring-key的kdtree
        // tree_ reconstruction (not mandatory to make everytime)
        if (tree_making_period_conter % TREE_MAKING_PERIOD_ == 0) // to save computation cost
        {
            // TicToc t_tree_construction;

            polarcontext_invkeys_to_search_.clear();
            // 最近50帧很难构成回环，因此构造kdtree的数据不包括最近的50帧
            polarcontext_invkeys_to_search_.assign(polarcontext_invkeys_mat_.begin(), polarcontext_invkeys_mat_.end() - num_exclude_recent);

            // 重新构造kdtree
            polarcontext_tree_.reset();
            polarcontext_tree_ = std::make_shared<InvKeyTree>(PC_NUM_RING /* dim */, polarcontext_invkeys_to_search_, 10 /* max leaf */);
            // tree_ptr_->index->buildIndex(); // inernally called in the constructor of InvKeyTree (for detail, refer the nanoflann and KDtreeVectorOfVectorsAdaptor)
            // t_tree_construction.toc("Tree construction");
        }
        tree_making_period_conter = tree_making_period_conter + 1;

        double min_dist = 10000000; // init with somthing large
        int nn_align = 0;
        int nn_idx = 0;

        // Step 3. 使用kdtree进行knn的最近邻查找
        // knn search
        // 从kdtree中寻找10个最相似的候选帧
        std::vector<size_t> candidate_indexes(NUM_CANDIDATES_FROM_TREE); // 10个最相似候选帧的索引
        std::vector<float> out_dists_sqr(NUM_CANDIDATES_FROM_TREE);      // 10个最相似候选帧的距离

        // TicToc t_tree_search;
        nanoflann::KNNResultSet<float> knnsearch_result(NUM_CANDIDATES_FROM_TREE);
        knnsearch_result.init(&candidate_indexes[0], &out_dists_sqr[0]);
        // 调用接口查找距离最近的10个候选帧
        polarcontext_tree_->index->findNeighbors(knnsearch_result, &curr_key[0] /* query */, nanoflann::SearchParams(10));
        // t_tree_search.toc("Tree search");

        // Step 4. 遍历最相似候选帧，计算Scan-Context距离
        /*
         *  step 2: pairwise distance (find optimal columnwise best-fit using cosine distance)
         */
        // TicToc t_calc_dist;
        for (int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++)
        {
            // 每个相似候选帧的SC矩阵
            MatrixXd polarcontext_candidate = polarcontexts_[candidate_indexes[candidate_iter_idx]];
            // 当前帧和SC矩阵计算相似得分，返回结果是 <最近的sc距离， _sc2右移的列数>
            std::pair<double, int> sc_dist_result = distanceBtnScanContext(curr_desc, polarcontext_candidate);

            double candidate_dist = sc_dist_result.first;
            int candidate_align = sc_dist_result.second;

            if (candidate_dist < min_dist)
            {
                min_dist = candidate_dist;
                nn_align = candidate_align;

                nn_idx = candidate_indexes[candidate_iter_idx]; // 找到最匹配的关键帧的索引
            }
        }
        // t_calc_dist.toc("Distance calc");

        // Step 5. 计算的最小距离要小于设定的阈值
        /*
         * loop threshold check
         */
        if (min_dist < SC_DIST_THRES)
        {
            loop_id = nn_idx;

            // std::cout.precision(3);
            // cout << "[Loop found] Nearest distance: " << min_dist << " btn " << polarcontexts_.size() - 1 << " and " << nn_idx << "." << endl;
            // cout << "[Loop found] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
        }
        else
        {
            // std::cout.precision(3);
            // cout << "[Not loop] Nearest distance: " << min_dist << " btn " << polarcontexts_.size() - 1 << " and " << nn_idx << "." << endl;
            // cout << "[Not loop] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
        }

        // To do: return also nn_align (i.e., yaw diff)
        float yaw_diff_rad = deg2rad(nn_align * PC_UNIT_SECTORANGLE);
        std::pair<int, float> result{loop_id, yaw_diff_rad};

        return result;
    }

    /**
     * @brief 检测闭环对，就是检测数据库中最新的那个点云帧和历史上所有帧之间的回环关系
     *
     * @return std::pair<int, float>
     */
    std::pair<int, float> SCManager::detectLoopClosureID(int num_exclude_recent)
    {
        auto curr_key = polarcontext_invkeys_mat_.back(); // current observation (query)
        auto curr_desc = polarcontexts_.back();           // current observation (query)

        return detectClosestKeyframeID(num_exclude_recent, curr_key, curr_desc);
    } // SCManager::detectLoopClosureID

    void SCManager::saveCurrentSCD(const std::string &save_path, int num_digits, const std::string &delimiter)
    {
        const auto &curr_scd = polarcontexts_.back();
        std::ostringstream out;
        out << std::internal << std::setfill('0') << std::setw(num_digits) << polarcontexts_.size() - 1;
        std::string curr_scd_node_idx = out.str();

        // delimiter: ", " or " " etc.
        int precision = 3; // or Eigen::FullPrecision, but SCD does not require such accruate precisions so 3 is enough.
        const static Eigen::IOFormat the_format(precision, Eigen::DontAlignCols, delimiter, "\n");

        std::ofstream file(save_path + "/" + curr_scd_node_idx + ".scd");
        if (file.is_open())
        {
            file << curr_scd.format(the_format);
            file.close();
        }
    }

    void SCManager::loadPriorSCD(const std::string &path, int num_digits, int num_keyframe)
    {
        for (auto i = 0; i < num_keyframe; ++i)
        {
            std::ostringstream out;
            out << std::internal << std::setfill('0') << std::setw(num_digits) << i;
            std::string curr_scd_node_idx = out.str();
            std::ifstream file(path + "/" + curr_scd_node_idx + ".scd");
            Eigen::MatrixXd curr_scd;
            curr_scd.resize(PC_NUM_RING, PC_NUM_SECTOR);

            if (file.is_open())
            {
                for (int i = 0; i < PC_NUM_RING; ++i)
                    for (int j = 0; j < PC_NUM_SECTOR; ++j)
                        file >> curr_scd(i, j);

                file.close();
            }

            Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(curr_scd);
            Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext(curr_scd);
            std::vector<float> polarcontext_invkey_vec = eig2stdvec(ringkey);

            polarcontexts_.push_back(curr_scd);
            polarcontext_invkeys_.push_back(ringkey);
            polarcontext_vkeys_.push_back(sectorkey);
            polarcontext_invkeys_mat_.push_back(polarcontext_invkey_vec);
        }
    }

    std::pair<int, float> SCManager::relocalize(pcl::PointCloud<PointType> &scan_down)
    {
        if (polarcontexts_.empty())
        {
            std::pair<int, float> result{-1, 0.0};
            return result;
        }

        Eigen::MatrixXd sc = makeScancontext(scan_down);
        Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(sc);
        // Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext(sc);
        std::vector<float> polarcontext_invkey_vec = eig2stdvec(ringkey);

        return detectClosestKeyframeID(0, polarcontext_invkey_vec, sc);
    }

} // namespace SC2
