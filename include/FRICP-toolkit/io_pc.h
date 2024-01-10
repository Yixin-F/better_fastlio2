#ifndef IO_H
#define IO_H
///--- hacked from OpenGP obj reader
#include <cstdio>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/eigen.h>
#include <pcl/common/centroid.h>
#include <pcl/common/impl/common.hpp>
#include<pcl/filters/voxel_grid.h>


template <class MatrixType>
bool read_pcd(MatrixType& vertices, MatrixType& normals, MatrixType& vert_colors, const std::string& filename, int D) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::io::loadPCDFile(filename, *cloud);
    pcl::VoxelGrid<pcl::PointXYZI> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(0.05f, 0.05f, 0.05f);
    filter.filter(*cloud_down);
    
    float  x, y, z;

    int num = cloud_down->points.size();
    vertices.resize(D, num);
    normals.resize(D, num);
    vert_colors.resize(D, num);
    for(size_t i = 0; i < num; i++){
        vertices(0, i) = cloud_down->points[i].x;
        vertices(1, i) = cloud_down->points[i].y;
        vertices(2, i) = cloud_down->points[i].z;

        normals(0, i) = 0;  // normals and colors are not useful !!
        normals(1, i) = 0;
        normals(2, i) = 0;
        vert_colors(0, i) = 0;
        vert_colors(1, i) = 0;
        vert_colors(2, i) = 0;
    }

    return true;
}

template <class MatrixType>
bool read_pcd_online(MatrixType& vertices, MatrixType& normals, MatrixType& vert_colors, const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_down, int D) {
    int num = cloud_down->points.size();
    vertices.resize(D, num);
    normals.resize(D, num);
    vert_colors.resize(D, num);
    for(size_t i = 0; i < num; i++){
        vertices(0, i) = cloud_down->points[i].x;
        vertices(1, i) = cloud_down->points[i].y;
        vertices(2, i) = cloud_down->points[i].z;

        normals(0, i) = 0;  // normals and colors are not useful !!
        normals(1, i) = 0;
        normals(2, i) = 0;
        vert_colors(0, i) = 0;
        vert_colors(1, i) = 0;
        vert_colors(2, i) = 0;
    }

    return true;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr detect(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& local_, const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& global_, const float& search_dis_){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>());  // result
    int local_num = local_->points.size();
    std::cout << "Second local scan points num is: " << local_num << std::endl;

    // select update area after registeration
    pcl::PointXYZINormal point_min, point_max;
    pcl::getMinMax3D(*local_, point_min, point_max);
    float min_x = point_min.x;
    float min_y = point_min.y;
    float min_z = point_min.z;
    float max_x = point_max.x;
    float max_y = point_max.y;
    float max_z = point_max.z;

    int global_num = global_->points.size();
    std::cout << "First global scan points num is: " << global_num << std::endl;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr global_select(new pcl::PointCloud<pcl::PointXYZINormal>());
    for(size_t i = 0; i < global_num; i ++){
        pcl::PointXYZINormal pt = global_->points[i];
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
    pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZINormal>());
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
        pt_rgb.r = 0;
        pt_rgb.g = 255.0;
        pt_rgb.b = 0;
        result->points.emplace_back(pt_rgb);  // save
    }
    std::cout << "local find and glocal match -> fusion" << std::endl; 

    // local not find -> new
    for(auto& vi : local_nofind){
        pcl::PointXYZRGB pt_rgb;
        pt_rgb.x = local_->points[vi].x;
        pt_rgb.y = local_->points[vi].y;
        pt_rgb.z = local_->points[vi].z;
        pt_rgb.r = 255.0;
        pt_rgb.g = 0;
        pt_rgb.b = 0;
        result->points.emplace_back(pt_rgb);  // save
    }
    std::cout << "local not find -> new" << std::endl; 

    // global not match -> old
    for(auto& vi : global_nomatch){
        pcl::PointXYZRGB pt_rgb;
        pt_rgb.x = global_select->points[vi].x;
        pt_rgb.y = global_select->points[vi].y;
        pt_rgb.z = global_select->points[vi].z;
        pt_rgb.r = 0;
        pt_rgb.g = 0;
        pt_rgb.b = 255.0;
        result->points.emplace_back(pt_rgb);  // save
    }
    std::cout << "global not match -> old" << std::endl; 

    result->height = 1;
    result->width = result->points.size();
    std::cout << "result num: " << result->points.size() << std::endl;
    
    return result;
}

template <class MatrixType>
bool read_obj(MatrixType& vertices, MatrixType& normals, MatrixType& vert_colors, const std::string& filename, int D) {
    char   s[200];
    float  x, y, z, cx, cy, cz;

    // open file (in ASCII mode)
    FILE* in = fopen(filename.c_str(), "r");
    if (!in) return false;

    // clear line once
    memset(&s, 0, 200);

    //--- First pass, counts vertices
    int n_vertices = 0;
    int n_normals = 0;
    while (in && !feof(in) && fgets(s, 200, in)) {
        // comment
        if (s[0] == '#' || isspace(s[0])) continue;
        // vertex
        else if (strncmp(s, "v ", 2) == 0)
            n_vertices++;
        else if (strncmp(s, "vn ", 2) == 0)
            n_normals++;
    }
    fseek(in, 0, 0); ///< rewind
    vertices.resize(D, n_vertices);
    if(n_normals > 0)
        normals.resize(D, n_vertices);
    bool runonce = true;

    //--- Second pass, fills in
    int curr_vertex=0;
    while (in && !feof(in) && fgets(s, 200, in)) {
        // comment
        if (s[0] == '#' || isspace(s[0])) continue;

        // normal
        else if (strncmp(s, "vn ", 3) == 0) {
            if (sscanf(s, "vn %f %f %f", &x, &y, &z)) {
                if (runonce)
                {
                    normals.resize(D, n_vertices);
                    runonce = false;
                }
                normals(0, curr_vertex) = x;
                normals(1, curr_vertex) = y;
                normals(2, curr_vertex) = z;
            }
        }

        // vertex
        else if (strncmp(s, "v ", 2) == 0) {
            if (sscanf(s, "v %f %f %f %f %f %f", &x, &y, &z, &cx, &cy, &cz)) {
                            vertices(0,curr_vertex) = x;
                            vertices(1,curr_vertex) = y;
                            vertices(2,curr_vertex) = z;

                            if(vert_colors.size()==0)
                                vert_colors.resize(D, n_vertices);

                            vert_colors(0, curr_vertex) = cx;
                            vert_colors(1, curr_vertex) = cy;
                            vert_colors(2, curr_vertex) = cz;
                            curr_vertex++;
                        }
            else if (sscanf(s, "v %f %f %f", &x, &y, &z)) {
                vertices(0,curr_vertex) = x;
                vertices(1,curr_vertex) = y;
                vertices(2,curr_vertex) = z;

                curr_vertex++;
            }
        }
        // face
        else if (strncmp(s, "f ", 2) == 0) {
            continue;
        }

        // clear line
        memset(&s, 0, 200);
    }

    fclose(in);
    return true;
}
//-----------------------------------------------------------------------------

///--- Replaces vertices in prev_filename with content of vertices, saves in filename
template <class MatrixType>
bool write_obj_replaceverts(const std::string& prev_filename, const MatrixType& vertices, const MatrixType& normals,
                            const MatrixType& vert_colors, const std::string& filename) {
    typedef Eigen::Vector3d Texture_coordinate;

    char   s[200];

    FILE* out = fopen(filename.c_str(), "w");
    FILE* in = fopen(prev_filename.c_str(), "r");
    if (!in || !out)
        return false;

    // clear line once
    memset(&s, 0, 200);

    //--- Second pass, fills in
    int curr_vertex=0;
    while (in && !feof(in) && fgets(s, 200, in)) {
        // vertex
        if (!isspace(s[0]) && strncmp(s, "v ", 2) == 0) {
            fprintf(out, "vn %f %f %f\n", normals(0,curr_vertex), normals(1,curr_vertex), normals(2,curr_vertex));
            fprintf(out, "v %f %f %f ", vertices(0,curr_vertex), vertices(1,curr_vertex), vertices(2,curr_vertex));
            if(vert_colors.size())
                fprintf(out, "%f %f %f\n", vert_colors(0,curr_vertex), vert_colors(1, curr_vertex), vert_colors(2, curr_vertex));
            else
                fprintf(out, "\n");
            curr_vertex++;
        } else {
            fprintf(out, "%s", s);
        }

        // clear line
        memset(&s, 0, 200);
    }


    fclose(in);
    fclose(out);
    return true;
}


template <class MatrixType>
bool read_transMat(MatrixType& trans, const std::string& filename)
{
	std::ifstream input(filename);
	std::string line;
	int rows, cols;
	std::vector<std::vector<double>> total_data;
	while (getline(input, line)) {
        if(line[0] == 'V' || line[0] == 'M')
            continue;
		std::istringstream iss(line);
		std::vector<double> lineVec;
		while (iss) {
			double item;
			if (iss >> item)
				lineVec.push_back(item);
		}
		cols = lineVec.size();
		total_data.push_back(lineVec);
	}
	if (total_data.size() == 0)
	{
		std::cout << filename << " is empty !! " << std::endl;
		return false;
	}
	rows = total_data.size();
	trans.resize(rows, cols);
    std::cout << "rows = " << rows << " cols = " << cols << std::endl;
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			trans(i, j) = total_data[i][j];
		}
	}
	input.close();
    std::cout << "read trans = \n" << trans << std::endl;
	return true;
}

template <class MatrixType>
bool read_ply(MatrixType& vertices, MatrixType& normals, MatrixType& colors, const std::string& filename, int D) {
    char   s[200];
    float  x, y, z, nx, ny, nz;
    int r, g, b;
    int dim = 0;
    int n_vertices, curr_vertex;
    Eigen::Vector3i ID;
    ID.setZero();

    // open file (in ASCII mode)
    FILE* in = fopen(filename.c_str(), "r");
    if (!in) return false;

    // clear line once
    memset(&s, 0, 200);

    //--- First pass, counts vertices
    while (in && !feof(in) && fgets(s, 200, in)) {

        // comment
        if (strncmp(s, "element ", 8) == 0)
        {
            sscanf(s, "element vertex %d", &n_vertices);
        }
        if (strncmp(s, "property float x", 16) == 0)
        {
            vertices.resize(D,n_vertices);
            ID[0]=1;
            dim += 3;
        }
        if(strncmp(s, "property float nx", 17) == 0)
        {
            normals.resize(D,n_vertices);
            ID[1]=1;
            dim += 3;
        }
        if(strncmp(s, "property uchar red", 18) == 0)
        {
            colors.resize(3,n_vertices);
            ID[2]=1;
            dim += 3;
        }
        if(strncmp(s, "end_header", 10) == 0)
        {
            break;
        }
        memset(&s, 0, 200);
    }

    // clear line
    memset(&s, 0, 200);
    curr_vertex = 0;
    while (in && !feof(in) && fgets(s, 200, in) && curr_vertex<n_vertices)
    {
        if(dim == 3)
        {
            if(sscanf(s, "%f %f %f",&x, &y, &z))
            {
                vertices(0,curr_vertex) = x;
                vertices(1,curr_vertex) = y;
                vertices(2,curr_vertex) = z;
                curr_vertex++;
            }
        }
        else if(dim==6)
        {
            if(ID[1])
            {
                if(sscanf(s, "%f %f %f %f %f %f",&x, &y, &z, &nx, &ny, &nz))
                {
                    vertices(0,curr_vertex) = x;
                    vertices(1,curr_vertex) = y;
                    vertices(2,curr_vertex) = z;
                    normals(0, curr_vertex) = nx;
                    normals(1, curr_vertex) = ny;
                    normals(2, curr_vertex) = nz;
                    curr_vertex++;
                }
            }
            else
            {
                 if(sscanf(s, "%f %f %f %d %d %d",&x, &y, &z, &r, &g, &b))
                 {
                     vertices(0,curr_vertex) = x;
                     vertices(1,curr_vertex) = y;
                     vertices(2,curr_vertex) = z;
                     colors(0, curr_vertex) = r;
                     colors(1, curr_vertex) = g;
                     colors(2, curr_vertex) = b;
                     curr_vertex++;
                 }
            }

        }
        else if(dim == 9)
        {
            if(sscanf(s, "%f %f %f %f %f %f %d %d %d", &x, &y, &z, &nx, &ny, &nz, &r, &g, &b))
            {
                vertices(0,curr_vertex) = x;
                vertices(1,curr_vertex) = y;
                vertices(2,curr_vertex) = z;
                normals(0, curr_vertex) = nx;
                normals(1, curr_vertex) = ny;
                normals(2, curr_vertex) = nz;
                colors(0, curr_vertex) = r;
                colors(1, curr_vertex) = g;
                colors(2, curr_vertex) = b;
                curr_vertex ++;
            }

        }
        if(curr_vertex > n_vertices)
        {
            n_vertices = curr_vertex;
            vertices.resize(Eigen::NoChange, n_vertices);
            if(normals.size())
            {
                normals.resize(Eigen::NoChange, n_vertices);
            }
            break;
        }
        // clear line
        memset(&s, 0, 200);
    }
    fclose(in);
    return true;
}

template <class MatrixType>
bool write_ply(MatrixType& vertices, MatrixType& normals, MatrixType& colors, const std::string& filename) {
    char   s[200];
    int n_vertices, curr_vertex;
    n_vertices = vertices.cols();
    Eigen::Vector3d ID; // whether there are normal or color
    ID.setZero();
    if (vertices.cols())
    {
        ID[0] = 1;
    }
    else
    {
        std::cout << "Warning : No points!!!" << std::endl;
        return false;
    }
    if (normals.cols())
    {
        ID[1] = 1;
//        std::cout << "output file has normals !" << std::endl;
    }
    if (colors.cols())
    {
        ID[2] = 1;
//        std::cout << "output file has colors !" << std::endl;
    }

    FILE* out = fopen(filename.c_str(), "w");
    if (!out)
        return false;
    // clear line once
    memset(&s, 0, 200);

    fprintf(out, "ply\nformat ascii 1.0\nelement vertex %d\n", n_vertices);
    fprintf(out, "property float x \nproperty float y\nproperty float z\n");
    if(ID[1])	fprintf(out, "property float nx \nproperty float ny\nproperty float nz\n");
    if(ID[2])	fprintf(out, "property uchar red\nproperty uchar green\nproperty uchar blue\n");
    fprintf(out, "end_header\n");

    // clear line
    memset(&s, 0, 200);
    curr_vertex = 0;
    while (curr_vertex<n_vertices)
    {
        fprintf(out, "%f %f %f ", vertices(0, curr_vertex), vertices(1, curr_vertex), vertices(2, curr_vertex));
        if (ID[1])	fprintf(out, "%f %f %f ", normals(0, curr_vertex), normals(1, curr_vertex), normals(2, curr_vertex));
        if (ID[2])	fprintf(out, "%d %d %d ", (int)colors(0, curr_vertex), (int)colors(1, curr_vertex), (int)colors(2, curr_vertex));
        fprintf(out, "\n");
        // clear line
        memset(&s, 0, 200);
        curr_vertex++;
    }
    fclose(out);
    return true;
}

template <class MatrixType>
bool write_pcd(MatrixType& vertices,  pcl::PointCloud<pcl::PointXYZI>::Ptr& res_cloud_, const std::string& filename){
    for(size_t i = 0; i < vertices.cols(); i++){
        pcl::PointXYZI pt;
        pt.x = vertices(0, i);
        pt.y = vertices(1, i);
        pt.z = vertices(2, i);
        pt.intensity = 0.0;
        res_cloud_->points.emplace_back(pt);
    }
    return true;
}

template <class MatrixType>
bool read_file(MatrixType& vertices, MatrixType& normals, MatrixType& vert_colors,
               const std::string& filename) {
    if(strcmp(filename.substr(filename.size()-4,4).c_str(), ".obj") == 0)
    {
        return read_obj(vertices, normals, vert_colors, filename, 3);
    }
    else if(strcmp(filename.substr(filename.size()-4, 4).c_str(),".ply")==0)
    {
        return read_ply(vertices, normals, vert_colors,  filename, 3);
    }
    else if(strcmp(filename.substr(filename.size()-4, 4).c_str(),".pcd")==0){
        return read_pcd(vertices, normals, vert_colors,  filename, 3);
    }
    else
    {
        std::cout << "Can't read file " << filename << std::endl;
    }
}

template <class MatrixType>
bool write_file(const std::string& prev_filename, const MatrixType& vertices, pcl::PointCloud<pcl::PointXYZI>::Ptr& res_cloud_, const MatrixType& normals,
                const MatrixType& vert_colors, const std::string& filename) {
    if(strcmp(filename.substr(filename.size()-4,4).c_str(),".obj")==0)
    {
        return write_obj_replaceverts(prev_filename, vertices, normals, vert_colors, filename);
    }
    else if(strcmp(filename.substr(filename.size()-4,4).c_str(),".ply")==0)
    {
        return write_ply(vertices, normals, vert_colors, filename);
    }
    else if(strcmp(filename.substr(filename.size()-4,4).c_str(),".pcd")==0){
        return write_pcd(vertices, res_cloud_, filename);
    }
    else
    {
        std::cout << "Can't write to file "<< filename << std::endl;
    }
}


#endif // IO_H
