#include "utility.h"

bool cout_debug = false;

// 读取每一帧激光扫描的bin文件
void readBin(std::string _bin_path, pcl::PointCloud<PointType>::Ptr _pcd_ptr)
{
 	std::fstream input(_bin_path.c_str(), ios::in | ios::binary);
	if(!input.good()){
		cerr << "Could not read file: " << _bin_path << endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, ios::beg);
  
	for (int ii=0; input.good() && !input.eof(); ii++) {
		PointType point;

		input.read((char *) &point.x, sizeof(float));
		input.read((char *) &point.y, sizeof(float));
		input.read((char *) &point.z, sizeof(float));
		input.read((char *) &point.intensity, sizeof(float));

		_pcd_ptr->push_back(point);
	}
	input.close();
}

namespace LTslamParam {
    // NOTE: kSessionStartIdxOffset in utility.h

    int ungenGlobalNodeIdx (const int& _session_idx, const int& _idx_in_graph)
    {
        return (_idx_in_graph - 1) / (_session_idx * kSessionStartIdxOffset);
    } // ungenGlobalNodeIdx

    int genGlobalNodeIdx (const int& _session_idx, const int& _node_offset)
    {
        // return (_session_idx * LTslam::kSessionStartIdxOffset) + _node_offset + 1;
        return (_session_idx * kSessionStartIdxOffset) + _node_offset + 1;
    } // genGlobalNodeIdx

    int genAnchorNodeIdx (const int& _session_idx)
    {
        return (_session_idx * kSessionStartIdxOffset);
        // return (_session_idx * LTslam::kSessionStartIdxOffset) + (LTslam::kSessionStartIdxOffset - 1);
    } // genAnchorNodeIdx

} // namespace LTslamParam


inline float rad2deg(float radians)
{ 
    return radians * 180.0 / M_PI; 
}

inline float deg2rad(float degrees)
{ 
    return degrees * M_PI / 180.0; 
}

Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
{ 
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
}

void fsmkdir(std::string _path)
{
    if (!fs::is_directory(_path) || !fs::exists(_path)) 
        fs::create_directories(_path); // create src folder
} //fsmkdir


pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, 
                                                transformIn->roll, transformIn->pitch, transformIn->yaw);
    
    int numberOfCores = 8; // TODO move to yaml
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        pointFrom = &cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom->intensity;
    }
    return cloudOut;
}

std::vector<std::pair<double, int>> sortVecWithIdx(const std::vector<double>& arr) 
{ 
    std::vector<std::pair<double, int> > vp; 
    for (int i = 0; i < arr.size(); ++i)
        vp.push_back(std::make_pair(arr[i], i)); 
  
    std::sort(vp.begin(), vp.end(), std::greater<>()); 
    return vp;
} 

sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}


std::vector<double> splitPoseLine(std::string _str_line, char _delimiter) {
    std::vector<double> parsed;
    std::stringstream ss(_str_line);
    std::string temp;
    while (getline(ss, temp, _delimiter)) {
        parsed.push_back(std::stod(temp)); // convert string to "double"
    }
    return parsed;
}

// 读取g2o的存储格式
// 举例4：VERTEX_SE3:QUAT 99 -61.332581 -9.253125 0.131973 -0.004256 -0.005810 -0.625732 0.780005
G2oLineInfo splitG2oFileLine(std::string _str_line) {

    std::stringstream ss(_str_line);

	std::vector<std::string> parsed_elms ;
    std::string elm;
	char delimiter = ' ';
    while (getline(ss, elm, delimiter)) {
        parsed_elms.push_back(elm); // convert string to "double"
    }

	G2oLineInfo parsed;
    // 判断是结点还是边
	if( isTwoStringSame(parsed_elms.at(0), G2oLineInfo::kVertexTypeName) )
	{
		parsed.type = parsed_elms.at(0);
		parsed.curr_idx = std::stoi(parsed_elms.at(1));
		parsed.trans.push_back(std::stod(parsed_elms.at(2)));
		parsed.trans.push_back(std::stod(parsed_elms.at(3)));
		parsed.trans.push_back(std::stod(parsed_elms.at(4)));
		parsed.quat.push_back(std::stod(parsed_elms.at(5)));
		parsed.quat.push_back(std::stod(parsed_elms.at(6)));
		parsed.quat.push_back(std::stod(parsed_elms.at(7)));
		parsed.quat.push_back(std::stod(parsed_elms.at(8)));
	}
	if( isTwoStringSame(parsed_elms.at(0), G2oLineInfo::kEdgeTypeName) )
	{
		parsed.type = parsed_elms.at(0);
		parsed.prev_idx = std::stoi(parsed_elms.at(1));
		parsed.curr_idx = std::stoi(parsed_elms.at(2));
		parsed.trans.push_back(std::stod(parsed_elms.at(3)));
		parsed.trans.push_back(std::stod(parsed_elms.at(4)));
		parsed.trans.push_back(std::stod(parsed_elms.at(5)));
		parsed.quat.push_back(std::stod(parsed_elms.at(6)));
		parsed.quat.push_back(std::stod(parsed_elms.at(7)));
		parsed.quat.push_back(std::stod(parsed_elms.at(8)));
		parsed.quat.push_back(std::stod(parsed_elms.at(9)));
	}

	return parsed;
}

bool isTwoStringSame(std::string _str1, std::string _str2)
{
	return !(_str1.compare(_str2));
}

void collect_digits(std::vector<int>& digits, int num) {
    if (num > 9) {
        collect_digits(digits, num / 10);
    }
    digits.push_back(num % 10);
}

// 记录位姿
void writePose3ToStream(std::fstream& _stream, gtsam::Pose3 _pose)
{
    gtsam::Point3 t = _pose.translation();
    gtsam::Rot3 R = _pose.rotation();

    // r1 means column 1 (see https://gtsam.org/doxygen/a02759.html)
    std::string sep = " "; // separator
    _stream << R.r1().x() << sep << R.r2().x() << sep << R.r3().x() << sep << t.x() << sep 
            << R.r1().y() << sep << R.r2().y() << sep << R.r3().y() << sep << t.y() << sep
            << R.r1().z() << sep << R.r2().z() << sep << R.r3().z() << sep << t.z() << std::endl;
}

std::vector<int> linspace(int a, int b, int N) {
    int h = (b - a) / static_cast<int>(N-1);
    std::vector<int> xs(N);
    typename std::vector<int>::iterator x;
    int val;
    for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
        *x = val;
    return xs;
}

void saveSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter = " ")
{
    // delimiter: ", " or " " etc.

    int precision = 3; // or Eigen::FullPrecision, but SCD does not require such accruate precisions so 3 is enough.
    const static Eigen::IOFormat the_format(precision, Eigen::DontAlignCols, delimiter, "\n");
 
    std::ofstream file(fileName);
    if (file.is_open())
    {
        file << matrix.format(the_format);
        file.close();
    }
}

Eigen::MatrixXd readSCD(std::string fileToOpen)
{
	// ref: https://aleksandarhaber.com/eigen-matrix-library-c-tutorial-saving-and-loading-data-in-from-a-csv-file/
	using namespace Eigen;

    std::vector<double> matrixEntries;
    std::ifstream matrixDataFile(fileToOpen);
    std::string matrixRowString;
	std::string matrixEntry;

    int matrixRowNumber = 0;
    while (getline(matrixDataFile, matrixRowString)) {
        std::stringstream matrixRowStringStream(matrixRowString); //convert matrixRowString that is a string to a stream variable.
        while (getline(matrixRowStringStream, matrixEntry, ' ')) // here we read pieces of the stream matrixRowStringStream until every comma, and store the resulting character into the matrixEntry
            matrixEntries.push_back(stod(matrixEntry));   //here we convert the string to double and fill in the row vector storing all the matrix entries

        matrixRowNumber++; //update the column numbers
    }
    return Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);
}

float poseDistance(const gtsam::Pose3& p1, const gtsam::Pose3& p2)
{
    auto p1x = p1.translation().x();
    auto p1y = p1.translation().y();
    auto p1z = p1.translation().z();
    auto p2x = p2.translation().x();
    auto p2y = p2.translation().y();
    auto p2z = p2.translation().z();
    
    return sqrt((p1x-p2x)*(p1x-p2x) + (p1y-p2y)*(p1y-p2y) + (p1z-p2z)*(p1z-p2z));
}




// SphericalPoint cart2sph(const PointType & _cp)
// { // _cp means cartesian point

//     if(cout_debug){
//         cout << "Cartesian Point [x, y, z]: [" << _cp.x << ", " << _cp.y << ", " << _cp.z << endl;
//     }

//     SphericalPoint sph_point {
//          std::atan2(_cp.y, _cp.x), 
//          std::atan2(_cp.z, std::sqrt(_cp.x*_cp.x + _cp.y*_cp.y)),
//          std::sqrt(_cp.x*_cp.x + _cp.y*_cp.y + _cp.z*_cp.z)
//     };    
//     return sph_point;
// }

// ! 多分辨率range image
// std::pair<int, int> resetRimgSize(const std::pair<float, float> _fov, const float _resize_ratio)
// {
//     // default is 1 deg x 1 deg 
//     float alpha_vfov = _resize_ratio;    
//     float alpha_hfov = _resize_ratio;    

//     float V_FOV = _fov.first;
//     float H_FOV = _fov.second;

//     int NUM_RANGE_IMG_ROW = std::round(V_FOV*alpha_vfov);
//     int NUM_RANGE_IMG_COL = std::round(H_FOV*alpha_hfov);

//     std::pair<int, int> rimg {NUM_RANGE_IMG_ROW, NUM_RANGE_IMG_COL};
//     return rimg;
// }


// std::set<int> convertIntVecToSet(const std::vector<int> & v) 
// { 
//     std::set<int> s; 
//     for (int x : v) { 
//         s.insert(x); 
//     } 
//     return s; 
// } 

// void pubRangeImg(cv::Mat& _rimg, 
//                 sensor_msgs::ImagePtr& _msg,
//                 image_transport::Publisher& _publiser,
//                 std::pair<float, float> _caxis)
// {
//     cv::Mat scan_rimg_viz = convertColorMappedImg(_rimg, _caxis);
//     _msg = cvmat2msg(scan_rimg_viz);
//     _publiser.publish(_msg);    
// } // pubRangeImg


// sensor_msgs::ImagePtr cvmat2msg(const cv::Mat &_img)
// {
//   sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", _img).toImageMsg();
//   return msg;
// }


// void publishPointcloud2FromPCLptr(const ros::Publisher& _scan_publisher, const pcl::PointCloud<PointType>::Ptr _scan)
// {
//     sensor_msgs::PointCloud2 tempCloud;
//     pcl::toROSMsg(*_scan, tempCloud);
//     tempCloud.header.stamp = ros::Time::now();
//     tempCloud.header.frame_id = "ltmapper";
//     _scan_publisher.publish(tempCloud);
// } // publishPointcloud2FromPCLptr


// sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
// {
//     sensor_msgs::PointCloud2 tempCloud;
//     pcl::toROSMsg(*thisCloud, tempCloud);
//     tempCloud.header.stamp = thisStamp;
//     tempCloud.header.frame_id = thisFrame;
//     if (thisPub->getNumSubscribers() != 0)
//         thisPub->publish(tempCloud);
//     return tempCloud;
// }

// float pointDistance(PointType p)
// {
//     return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
// }

// float pointDistance(PointType p1, PointType p2)
// {
//     return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
// }

