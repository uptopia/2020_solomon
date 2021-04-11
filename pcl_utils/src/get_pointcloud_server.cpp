#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/PoseStamped.h"
#include <typeinfo>

#include <iostream>
#include <fstream>
#include <algorithm>

#include "pcl_ros/filters/filter.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/ia_ransac.h>//采样一致性

#include "pcl_utils/snapshot.h"
#include "pcl_utils/ROI.h"
#include "pcl_utils/ROI_array.h"

//boost
#include <boost/make_shared.hpp>

using pcl::NormalEstimation;
using pcl::search::KdTree;
using namespace std;

typedef boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> CloudPointRGBPtr;

struct ICPResult
{
  float icp_error;
  CloudPointRGBPtr cloud;
  Eigen::Matrix4d tf_matrix;
  float id;
};

std::vector<ICPResult> icp_result;

pcl_utils::ROI_array yolo_ROI_array;

bool compare_highest_points(const ICPResult &result_1, const ICPResult &result_2) //minimum z is the higheset!!!!
{ 
    return result_1.icp_error < result_2.icp_error;
}

std::string open_path_image = "../Solomon_ws/src/pcl_utils/data/save_img/test_img.jpg";   // [1] img_file
std::string save_path_rgb = "../Solomon_ws/src/pcl_utils/data/save_img/rgb_";             // [2] img_rs_rgb; [6] all_rs
std::string save_path_depth = "../Solomon_ws/src/pcl_utils/data/save_img/depth_";         // [3] img_rs_depth; [6] all_rs
std::string open_path_cloud = "../Solomon_ws/src/pcl_utils/data/save_cloud/test_cloud.pcd";// [4] cloud_file
std::string save_path_cloud = "../Solomon_ws/src/pcl_utils/data/save_cloud/cloud";       // [5] cloud_rs; [6] all_rs

ros::Publisher pubFilter, pubSource, pubSACSegmentation, pubExtractRegionSAC, pubICPAlign, pubFPFH;

//===================Data===================
//宣告輸出給ROS的點雲的格式
sensor_msgs::PointCloud2 Filter_output;   
sensor_msgs::PointCloud2 FPFH_output;   
sensor_msgs::PointCloud2 SACsegmentation_output;   
sensor_msgs::PointCloud2 ICP_align_output;   
sensor_msgs::PointCloud2 Source_output;   
sensor_msgs::PointCloud2 Source_CAD_output;   
sensor_msgs::PointCloud2 Source_CAD_Transform_output;

//Origin Pointcloud
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//filter Pointcloud
pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//PassThrough PointCloud
pcl::PointCloud<pcl::PointXYZ>::Ptr passthrough_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//VoxelGrid PointCloud
pcl::PointCloud<pcl::PointXYZ>::Ptr voxelgrid_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//SAC
pcl::PointCloud<pcl::PointXYZ>::Ptr SACSegmentation_Extract_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//color filter after Extract RegionGrowing with SACSegmentation
pcl::PointCloud<pcl::PointXYZ>::Ptr color_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//Result after do ICP
pcl::PointCloud<pcl::PointXYZ>::Ptr ICP_filtered (new pcl::PointCloud<pcl::PointXYZ>);

std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > Extract_vector;

std::vector <float> icp_score;

std::string object_type = "non";

bool is_YOLO_detect = false;
bool catch_image = false;
bool YOLO_go = false;
//===================Data===================/

//===================Config===================
float x_coordinate_min,y_coordinate_min,z_coordinate_min;
float x_coordinate_max,y_coordinate_max,z_coordinate_max;

int cnt_cloud = 0;
int cnt_SACSegmentation = 0;
int cnt_ColorFiltered = 0;
int cnt_ExtractColor = 0;
//===================Config===================/

// void file_formated()
// {
//   pcl::PointCloud<pcl::PointNormal>::Ptr source_normal(new pcl::PointCloud<pcl::PointNormal>);

//   std::string source_normal_file = "/home/lucaubuntu18/Solomon_TransparentObject_ws/src/open3d_utils/PCD_Data/cad_data/solomon_part_12000.pcd";
//   cout<<"source_file :" << source_normal_file << endl;

//   string file_path_cloud = save_path_cloud + "_" + source_normal_file + "_formated";
//   file_path_cloud = file_path_cloud + ".pcd";
//   pcl::io::loadPCDFile<pcl::PointNormal>(source_normal_file, *source_normal);
//   pcl::io::savePCDFileASCII<pcl::PointXYZ>(file_path_cloud, *source_normal);
// }

void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  cout<<matrix<<endl;
}

void set_coordinate_limit_min(const geometry_msgs::Point& input)
{
  x_coordinate_min = input.x;
  y_coordinate_min = input.y;
  z_coordinate_min = input.z;
  printf("coordinatee min   x:%f, y:%f, z:%f\n", x_coordinate_min, y_coordinate_min, z_coordinate_min);
}

void set_coordinate_limit_max(const geometry_msgs::Point& input)
{
  x_coordinate_max = input.x;
  y_coordinate_max = input.y;
  z_coordinate_max = input.z;
  printf("coordinatee max   x:%f, y:%f, z:%f\n", x_coordinate_max, y_coordinate_max, z_coordinate_max);
}

bool savePointCloud(pcl_utils::snapshot::Request &req, pcl_utils::snapshot::Response &res)
{
  // req.call;
  // res.back;
  ostringstream os;
  os << cnt_cloud;
  string file_path_cloud = save_path_cloud + os.str();
  file_path_cloud = file_path_cloud + ".pcd";

  pcl::io::savePCDFileASCII<pcl::PointXYZ>(file_path_cloud, *cloud);
  
  cnt_cloud++;

  return true;
}

void do_savePCDFileASCII(std::string cloud_FileName, 
                         int *cnt, 
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud)
{
  ostringstream os;
  os << *cnt;
  string file_path_cloud = save_path_cloud + "_" + cloud_FileName + "_" + os.str();
  file_path_cloud = file_path_cloud + ".pcd";

  pcl::io::savePCDFileASCII<pcl::PointXYZ>(file_path_cloud, *input_cloud);
  *cnt = *cnt + 1;
}

void do_Passthrough(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, 
                    pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                    std::string dim, float min, float max)
{
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (input_cloud);
  pass.setFilterFieldName (dim);
  pass.setFilterLimits (min, max);
    //pass.setFilterLimitsNegative (true);
  pass.filter (*output_cloud);
}

void do_VoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, 
                    pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud)
{
  // 進行一個濾波處理
  pcl::VoxelGrid<pcl::PointXYZ> sor;   //例項化濾波
  sor.setInputCloud (input_cloud);     //設定輸入的濾波
  sor.setLeafSize (0.0003, 0.0003, 0.0003);   //設定體素網格的大小
  sor.filter (*output_cloud);      //儲存濾波後的點雲
}

void PointCloud_cd(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // ROS to PCL
  pcl::fromROSMsg(*cloud_msg, *cloud);

  do_Passthrough(cloud, filter_cloud, "x", x_coordinate_min, x_coordinate_max);
  do_Passthrough(filter_cloud, filter_cloud, "y", y_coordinate_min, y_coordinate_max);
  do_Passthrough(filter_cloud, filter_cloud, "z", z_coordinate_min, z_coordinate_max);
  do_VoxelGrid(filter_cloud, filter_cloud);
  
  // PCL to ROS
  pcl::toROSMsg(*filter_cloud, Filter_output);    //第一個引數是輸入，後面的是輸出
  //Specify the frame that you want to publish
  Filter_output.header.frame_id = "camera_depth_optical_frame";
  //釋出命令
  pubFilter.publish (Filter_output);
}

void do_SACSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, 
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud)
{
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.005);//origin : 0.01
  seg.setInputCloud (input_cloud);
  seg.segment (*inliers, *coefficients);
  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
  // 提取地面以外點雲
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (input_cloud);
  extract.setNegative (true);
  extract.setIndices (inliers);
  extract.filter (*output_cloud);

  // std::cerr << "Ground cloud after filtering: " << std::endl;
  // std::cerr << *output_cloud << std::endl;

 // PCL to ROS
  pcl::toROSMsg(*output_cloud, SACsegmentation_output);    //第一個引數是輸入，後面的是輸出

  //Specify the frame that you want to publish
  SACsegmentation_output.header.frame_id = "camera_depth_optical_frame";

  //釋出命令
  pubSACSegmentation.publish (SACsegmentation_output);
}

Eigen::Vector4f do_ComputeLocation(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
  Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*input_cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*input_cloud, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //校正主方向间垂直
	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
 
	// std::cout << "特徵值va(3x1):\n" << eigenValuesPCA << std::endl;
	// std::cout << "特徵向量ve(3x3):\n" << eigenVectorsPCA << std::endl;
	// std::cout << "質心点(4x1):\n" << pcaCentroid << std::endl;

  return pcaCentroid;
}

void do_Extract(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, 
                std::vector <pcl::PointCloud<pcl::PointXYZ>::Ptr> &output_cloud,
                std::vector <pcl::PointIndices> &clusters)
{
  output_cloud.clear();

  for(int kk = 0; kk < clusters.size(); kk++)
  {
      if(clusters[kk].indices.size() > 2000 && clusters[kk].indices.size() < 12000)
      {
          cout << "Cluster #" << kk << " size = " << clusters[kk].indices.size() << endl;
          pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
          tmp_cloud->clear();
          int counter = 0;

          while (counter < clusters[kk].indices.size ())
          {
            int idx = clusters[kk].indices[counter];
            tmp_cloud->push_back(input_cloud->points[idx]);
            counter++;        
          }
          output_cloud.push_back(tmp_cloud);
      }
  }
  std::cout<<"do_Extract END!!"<<endl;
}

Eigen::Matrix4d  
do_ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr &target, 
       pcl::PointCloud<pcl::PointXYZ>::Ptr &source)
{
  ICPResult tmp_result;

  //ICP
  int iterations = 50000;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; //创建ICP对象，用于ICP配准

  icp.setInputSource(source); //设置输入点云
  icp.setInputTarget(target); //设置目标点云（输入点云进行仿射变换，得到目标点云）
  // icp.setMaxCorrespondenceDistance(1);// Set the max correspondence distance (e.g., correspondences with higher distances will be ignored)
  icp.setMaximumIterations(iterations);    //criterion1: 设置最大迭代次数iterations=true
  icp.setTransformationEpsilon(1e-8);     //criterion2: transformation epsilon 
  icp.setEuclideanFitnessEpsilon(0.01);   // Set the euclidean distance difference epsilon (criterion 3)    

  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
  output_cloud->clear();

  icp.align(*output_cloud);          //匹配后源点云
  
  // //icp.setMaximumIterations(1);  // 设置为1以便下次调用
  std::cout << "Applied " << iterations << " ICP iteration(s)"<< std::endl;
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
  transformation_matrix = icp.getFinalTransformation().cast<double>();

  if (icp.hasConverged())//icp.hasConverged ()=1（true）输出变换矩阵的适合性评估
  {
      std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
      transformation_matrix = icp.getFinalTransformation().cast<double>();
      tmp_result.icp_error = icp.getFitnessScore();
      tmp_result.cloud = output_cloud;
      tmp_result.tf_matrix = transformation_matrix;
      icp_result.push_back(tmp_result);
  }
  else
  {
      PCL_ERROR("\nICP has not converged.\n");
  }  
  return transformation_matrix;
}

Eigen::Matrix4d 
do_FPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr  &target,
        pcl::PointCloud<pcl::PointXYZ>::Ptr  &output_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr  &source)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt_o(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src_o(new pcl::PointCloud<pcl::PointXYZ>);

  cloud_tgt_o = target;
  cloud_src_o = source;

  // pcl::io::loadPCDFile<pcl::PointXYZ>(source, *cloud_src_o);

  //去除NAN点
	std::vector<int> indices_src; //保存去除的点的索引
	pcl::removeNaNFromPointCloud(*cloud_src_o, *cloud_src_o, indices_src);
	std::cout << "remove *cloud_src_o nan" << endl;

	std::vector<int> indices_tgt;
	pcl::removeNaNFromPointCloud(*cloud_tgt_o, *cloud_tgt_o, indices_tgt);
	std::cout << "remove *cloud_tgt_o nan" << endl;

  //下采样滤波
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setLeafSize(0.002, 0.002, 0.002);
	voxel_grid.setInputCloud(cloud_src_o);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
	voxel_grid.filter(*cloud_src);
	std::cout << "down size *cloud_src_o from " << cloud_src_o->size() << "to" << cloud_src->size() << endl;

	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_2;
	voxel_grid_2.setLeafSize(0.002, 0.002, 0.002);
	voxel_grid_2.setInputCloud(cloud_tgt_o);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt(new pcl::PointCloud<pcl::PointXYZ>);
	voxel_grid_2.filter(*cloud_tgt);
	std::cout << "down size *cloud_tgt_o.pcd from " << cloud_tgt_o->size() << "to" << cloud_tgt->size() << endl;

	//计算表面法线
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_src;
	ne_src.setInputCloud(cloud_src);
	pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree< pcl::PointXYZ>());
	ne_src.setSearchMethod(tree_src);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
	ne_src.setRadiusSearch(0.01);
	ne_src.compute(*cloud_src_normals);
	std::cout << "compute *cloud_src_normals" << endl;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_tgt;
	ne_tgt.setInputCloud(cloud_tgt);
	pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree< pcl::PointXYZ>());
	ne_tgt.setSearchMethod(tree_tgt);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
	//ne_tgt.setKSearch(20);
	ne_tgt.setRadiusSearch(0.01);
	ne_tgt.compute(*cloud_tgt_normals);
	std::cout << "compute *cloud_tgt_normals" << endl;

	//计算FPFH
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_src;
	fpfh_src.setInputCloud(cloud_src);
	fpfh_src.setInputNormals(cloud_src_normals);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_src_fpfh(new pcl::search::KdTree<pcl::PointXYZ>);
	fpfh_src.setSearchMethod(tree_src_fpfh);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh_src.setRadiusSearch(0.1);
	fpfh_src.compute(*fpfhs_src);
	std::cout << "compute *cloud_src fpfh" << endl;

	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_tgt;
	fpfh_tgt.setInputCloud(cloud_tgt);
	fpfh_tgt.setInputNormals(cloud_tgt_normals);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_tgt_fpfh(new pcl::search::KdTree<pcl::PointXYZ>);
	fpfh_tgt.setSearchMethod(tree_tgt_fpfh);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh_tgt.setRadiusSearch(0.1);
	fpfh_tgt.compute(*fpfhs_tgt);
	std::cout << "compute *cloud_tgt fpfh" << endl;

	//SAC配准
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
	scia.setInputSource(cloud_src);
	scia.setInputTarget(cloud_tgt);
	scia.setSourceFeatures(fpfhs_src);
	scia.setTargetFeatures(fpfhs_tgt);
	//scia.setMinSampleDistance(1);
	//scia.setNumberOfSamples(2);
	//scia.setCorrespondenceRandomness(20);
  pcl::PointCloud<pcl::PointXYZ>::Ptr sac_result(new pcl::PointCloud<pcl::PointXYZ>);

	scia.align(*sac_result);
	std::cout << "sac has converged:" << scia.hasConverged() << "  score: " << scia.getFitnessScore() << endl;
	Eigen::Matrix4d sac_trans;
	sac_trans = scia.getFinalTransformation().cast<double>();
	std::cout << sac_trans << endl;

  output_cloud = sac_result;
	//pcl::io::savePCDFileASCII("bunny_transformed_sac.pcd", *sac_result);
  pcl::toROSMsg(*output_cloud, FPFH_output);    //第一個引數是輸入，後面的是輸出
  //Specify the frame that you want to publish
  FPFH_output.header.frame_id = "camera_depth_optical_frame";
  //釋出命令
  pubFPFH.publish (FPFH_output);

  return sac_trans;
}

void get_RegionYOLO(pcl_utils::ROI_array ROI_array)
{
  if (ROI_array.ROI_list.size()>0 && YOLO_go)
  {
    // ===========do some transform here===========
    // 1 pixel = 0.06666667 cm, at z = 61 cm
    // center pixel = (320, 240), at resolution of raw image = (640, 480)
    // float pixeltocm = 0.06666667;
    float pixeltocm = 0.10067114;
    float d = 10;
    float center_x = 320;
    float center_y = 240;
    center_x = center_x + d;
    center_y = center_y + d;

    x_coordinate_min = (ROI_array.ROI_list[0].min_x-center_x)*pixeltocm/100;
    x_coordinate_max = (ROI_array.ROI_list[0].Max_x-center_x)*pixeltocm/100;
    y_coordinate_min = (ROI_array.ROI_list[0].min_y-center_y)*pixeltocm/100;
    y_coordinate_max = (ROI_array.ROI_list[0].Max_y-center_y)*pixeltocm/100;

    z_coordinate_min = 0;
    z_coordinate_max = 0.6025;

    object_type = ROI_array.ROI_list[0].object_name;
    // ===========do some transform here===========/

    is_YOLO_detect = true;
    catch_image = true;
  }
  else
  {
    // x_coordinate_min = 0;
    // x_coordinate_max = 0;
    // y_coordinate_min = 0;
    // y_coordinate_max = 0;
    // z_coordinate_min = 0;
    // z_coordinate_max = 0;

    is_YOLO_detect = false;
  }

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (x_coordinate_min, x_coordinate_max);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (y_coordinate_min, y_coordinate_max);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (z_coordinate_min, z_coordinate_max);
  pass.filter (*cloud); 
}

void set_InputFile(std::string file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud)
{
  // cout<<"source_file : " << file_name << endl;
  pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *output_cloud);
}

template <typename T> 
vector<size_t> argsort(const vector<T> &v) 
{ 
  // 建立下标数组
  vector<size_t> idx(v.size()); 
  iota(idx.begin(), idx.end(), 0); 
  // 调用sort函数，匿名函数自动捕获待排序数组
  sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) {return v[i1] < v[i2];}); 
  return idx;
}

bool Align(pcl_utils::snapshot::Request &req, pcl_utils::snapshot::Response &res)
{
  YOLO_go = true;
  if (catch_image)
  {
    YOLO_go = false;
    std::string source_name_front = "/home/robotarm/Documents/solomon_ws/src/pcl_utils/data/source/final/solomon_part_A.pcd";
    std::string source_name_back = "/home/robotarm/Documents/solomon_ws/src/pcl_utils/data/source/final/solomon_part_B.pcd";
    std::string source_name_down = "/home/robotarm/Documents/solomon_ws/src/pcl_utils/data/source/final/solomon_part_C.pcd";
    std::string source_name_up = "/home/robotarm/Documents/solomon_ws/src/pcl_utils/data/source/final/solomon_part_D.pcd";
    std::string source_name_CAD = "/home/robotarm/Documents/solomon_ws/src/pcl_utils/data/source/final/solomon_part.pcd";

    std::string source_name = "non";
    
    do_Passthrough(cloud, cloud, "x", x_coordinate_min, x_coordinate_max);
    do_Passthrough(cloud, cloud, "y", y_coordinate_min, y_coordinate_max);
    do_Passthrough(cloud, cloud, "z", z_coordinate_min, z_coordinate_max);

    if(cloud->size()>1)
    {
      std::cout<<"cloud->size() "<<cloud->size()<<endl;

      if(object_type=="front")
      {
          source_name = source_name_front;
      }
      else if(object_type=="back")
      {
          source_name = source_name_back;
      }
      else if(object_type=="up")
      {
          source_name = source_name_down;
      }
      else if(object_type=="down")
      {
          source_name = source_name_up;
      }
      else
      {
          source_name = "unknown";
          res.doit = false;
          return true;
      }
      
      std::cout<<"source_name -----> "<<object_type<<endl;

      pcl::PointCloud<pcl::PointXYZ>::Ptr Source(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr Source_CAD(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr FPFH_out(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr Final_Transform_Cloud(new pcl::PointCloud<pcl::PointXYZ>);
      
      Eigen::Matrix4d FPFH_Transform;
      Eigen::Matrix4d ICP_Transform;
      Eigen::Matrix4d Final_Transform;

      pcl::io::loadPCDFile<pcl::PointXYZ>(source_name, *Source);
      pcl::io::loadPCDFile<pcl::PointXYZ>(source_name_CAD, *Source_CAD);

      float scale = 0.001;

      for(int n = 0; n < Source_CAD->size(); n ++)
      {
        Source_CAD->points[n].x = Source_CAD->points[n].x * scale;
        Source_CAD->points[n].y = Source_CAD->points[n].y * scale;
        Source_CAD->points[n].z = Source_CAD->points[n].z * scale;
      }

      Eigen::Vector4f source_CAD_center;
      source_CAD_center = do_ComputeLocation(Source_CAD);
      // std::cout<<"Source_CAD_center "<<source_CAD_center<<endl;
      
      for(int n = 0; n < Source->size(); n ++)
      {
        
        Source->points[n].x = Source->points[n].x * scale;
        Source->points[n].y = Source->points[n].y * scale;
        Source->points[n].z = Source->points[n].z * scale;
      }
      
      Eigen::Matrix4d SourceToOrigin_Transform = Eigen::Matrix4d::Identity();
      SourceToOrigin_Transform(0,3) = -1*source_CAD_center(0);
      SourceToOrigin_Transform(1,3) = -1*source_CAD_center(1);
      SourceToOrigin_Transform(2,3) = -1*source_CAD_center(2);

      // std::cout<<"SourceToOrigin_Transform "<<SourceToOrigin_Transform<<endl;

      pcl::transformPointCloud(*Source, *Source, SourceToOrigin_Transform);

      FPFH_Transform = do_FPFH(cloud, FPFH_out, Source);
      ICP_Transform = do_ICP(cloud, FPFH_out);

      Final_Transform = ICP_Transform*FPFH_Transform;

      switch (object_type)
      {
      case "front":
        if(Final_Transform.coeff(2, 2) > -0.85) //cos(30) ~= 0.85
        {
          res.doit = false;
          return true;
        }
        break;
      case "back":
        if(Final_Transform.coeff(2, 2) < 0.85)
        {
          res.doit = false;
          return true;
        }
        break;
      case "up":
        if(Final_Transform.coeff(1, 2) < 0.7) //cos(45) ~= 0.7
        {
          res.doit = false;
          return true;
        }
        break;
      case "down":
        if(Final_Transform.coeff(1, 2) > -0.7)
        {
          res.doit = false;
          return true;
        }
        break;
      default:
        res.doit = false;
        return true;
        break;
      }

      pcl::transformPointCloud(*Source_CAD, *Source_CAD, SourceToOrigin_Transform);

      // PCL to ROS
      pcl::toROSMsg(*Source_CAD, Source_CAD_output);    //第一個引數是輸入，後面的是輸出
      // Specify the frame that you want to publish
      Source_CAD_output.header.frame_id = "camera_depth_optical_frame";
      // 釋出命令
      pubSource.publish (Source_CAD_output);

      // do Source to Target Transform
      pcl::transformPointCloud(*Source, *Final_Transform_Cloud, Final_Transform);

      std::cout<<"Final_Transform "<<endl<<Final_Transform<<endl;
      
      // PCL to ROS
      pcl::toROSMsg(*Final_Transform_Cloud, ICP_align_output);    //第一個引數是輸入，後面的是輸出
      // Specify the frame that you want to publish
      ICP_align_output.header.frame_id = "camera_depth_optical_frame";
      // 釋出命令
      pubICPAlign.publish (ICP_align_output);

      catch_image = false;
      res.doit = true;
      std::vector<double> transform;
      transform.resize(16);
      for(int i=0; i<4; i++)
      {
        for(int j=0; j<4; j++)
        {
          transform[i*4+j] = Final_Transform.coeff(i, j);
        }
      }

      res.type = object_type;
      res.trans = transform;
    }
    else
    {
      std::cout<<"cloud->size() < 1000 "<<endl;
      res.doit = false;
    }
  }
  else
  {
    std::cout<<"YOLO detect No objects."<<endl;
    res.doit = false;
  }
  
  // do_FPFH(cloud, FPFH_out_2, source_name_2);
  // do_FPFH(cloud, FPFH_out_3, source_name_3);
  

  // do_VoxelGrid(cloud, cloud);

  // set_InputFile(target_name, cloud);
  //=================Filters=================/

  // //=================PlaneModelSegmentation and ExtractGround=================
  // do_SACSegmentation(cloud, SACSegmentation_Extract_filtered);
  // do_savePCDFileASCII("SAC_Extract", &cnt_SACSegmentation, SACSegmentation_Extract_filtered);
  // //=================PlaneModelSegmentation=================/

  // //=================RegionGrowing=================
  // std::vector <pcl::PointIndices> clusters = do_RegionGrowingRGB(SACSegmentation_Extract_filtered, color_filtered);
  // do_savePCDFileASCII("Color_Region", &cnt_ColorFiltered, color_filtered);
  // //=================RegionGrowing=================/

  // //=================Extract RegionGrowing=================
  // // do_Extract(color_filtered, Extract_vector, clusters);
  // do_Extract(SACSegmentation_Extract_filtered, Extract_vector, clusters);//No RegionGrowingRGB!!

  // for (int i=0; i<Extract_vector.size(); i++ )
  // {
  //   do_savePCDFileASCII("Extract_vector", &cnt_ExtractColor, Extract_vector[i]);
  // }
  // //=================Extract RegionGrowing=================/

  //=================ICP1=================
  // Eigen::Matrix4d Metal_Transform1;
  // Metal_Transform1 = do_ICP_2(cloud, FPFH_out);

  // std::cout<<"Final transform list 1: "<<endl;

  // for (int i=0; i<Metal_Transform1.size(); i++)
  // {
  //   std::cout<<"Metal Object["<<i<<"] :"<<endl;
  //   std::cout<<icp_result[i].icp_error<<endl;
  //   print4x4Matrix(Metal_Transform1);
  // }
  
  // std::sort(icp_result.begin(), icp_result.end(), compare_highest_points);

  // for(int i =0; i<icp_result.size();i++)
  // {
  //   std::cout<<icp_result[i].icp_error<<endl;
  // }
  // //=================ICP1=================/
  // if (icp_result.size()!= 0)
  // {
  //   temp_final.icp_error = icp_result[0].icp_error;
  //   temp_final.cloud = icp_result[0].cloud;
  //   temp_final.tf_matrix = icp_result[0].tf_matrix;
  //   temp_final.id = 1;
  //   final_output_cloud.push_back(temp_final);
  // }
  //=================ICP2=================
  // Eigen::Matrix4d Metal_Transform2;
  // Metal_Transform2 = do_ICP_2(cloud, FPFH_out);

  // std::cout<<"Final transform list 2: "<<endl;

  // for (int i=0; i<Metal_Transform2.size(); i++)
  // {
  //   std::cout<<"Metal Object["<<i<<"] :"<<endl;
  //   print4x4Matrix(Metal_Transform2);
  // }

  // std::sort(icp_result.begin(), icp_result.end(), compare_highest_points);

  // for(int i =0; i<icp_result.size();i++)
  // {
  //   std::cout<<icp_result[i].icp_error<<endl;
  // }
  // //=================ICP2=================/
  // if (icp_result.size()!= 0)
  // {
  //   temp_final.icp_error = icp_result[0].icp_error;
  //   temp_final.cloud = icp_result[0].cloud;
  //   temp_final.tf_matrix = icp_result[0].tf_matrix;
  //   temp_final.id = 2;
  //   final_output_cloud.push_back(temp_final);
  // }

  //=================ICP3=================
  // Eigen::Matrix4d Metal_Transform3;
  // Metal_Transform3 = do_ICP_2(cloud, FPFH_out);

  // std::cout<<"Final transform list 3: "<<endl;

  // for (int i=0; i<Metal_Transform3.size(); i++)
  // {
  //   std::cout<<"Metal Object["<<i<<"] :"<<endl;
  //   print4x4Matrix(Metal_Transform3);
  // }

  // std::sort(icp_result.begin(), icp_result.end(), compare_highest_points);

  // for(int i =0; i<icp_result.size();i++)
  // {
  //   std::cout<<icp_result[i].icp_error<<endl;
  // }
  // //=================ICP3=================/
  // if (icp_result.size()!= 0)
  // {
  //   temp_final.icp_error = icp_result[0].icp_error;
  //   temp_final.cloud = icp_result[0].cloud;
  //   temp_final.tf_matrix = icp_result[0].tf_matrix;
  //   temp_final.id = 3;
  //   final_output_cloud.push_back(temp_final);
  // }

  // std::sort(final_output_cloud.begin(), final_output_cloud.end(), compare_highest_points);

  // std::cout<<"final_output_cloud size "<<final_output_cloud.size()<<endl;

  // for(int i=0; i<final_output_cloud.size(); i++)
  // {
  //    std::cout<<final_output_cloud[i].icp_error<<endl;
  // }

  // Eigen::Vector4f final_location;
  // final_location = do_ComputeLocation(final_output_cloud[0].cloud);
  // std::cout << "質心点(4x1):\n" << final_location << std::endl;
  // std::cout << "id:\n" << final_output_cloud[0].id << std::endl;

  //   // // PCL to ROS
  // CloudPointRGBPtr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // *output_cloud = *final_output_cloud[0].cloud +*final_output_cloud[1].cloud;
  // *output_cloud = *output_cloud + *final_output_cloud[2].cloud;

  // pcl::toROSMsg(*final_output_cloud[0].cloud, ICP_align_output);    //第一個引數是輸入，後面的是輸出
  // //Specify the frame that you want to publish
  // ICP_align_output.header.frame_id = "camera_depth_optical_frame";
  // //釋出命令
  // pubICPAlign.publish(ICP_align_output);
  

  return true;
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "get_pointcloud_server");

  ros::NodeHandle nh;

  // Create ROS preccess pointcloud publisher for the output point cloud
  pubFilter = nh.advertise<sensor_msgs::PointCloud2> ("/process_pubFilter", 1);
  pubSource = nh.advertise<sensor_msgs::PointCloud2> ("/process_pubSource", 1);
  pubFPFH = nh.advertise<sensor_msgs::PointCloud2> ("/process_FPFH_cloud", 1);
  pubSACSegmentation = nh.advertise<sensor_msgs::PointCloud2> ("/process_SACSegmentation_cloud", 1);
  pubICPAlign = nh.advertise<sensor_msgs::PointCloud2> ("/process_pubICPAlign_cloud", 1);

  // Create ROS config publisher for the passthrough limits
  ros::Publisher pubCoordinateLimitMin = nh.advertise<geometry_msgs::Point> ("/coordinate_limit_min", 1);
  ros::Publisher pubCoordinateLimitMax = nh.advertise<geometry_msgs::Point> ("/coordinate_limit_max", 1);

  // Create ROS subscriber for the input point cloud
  ros::Subscriber subSaveCloud = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/color/points", 1, PointCloud_cd);
  ros::Subscriber subCoordinateMin = nh.subscribe ("/coordinate_limit_min", 1, set_coordinate_limit_min);
  ros::Subscriber subCoordinateMax = nh.subscribe ("/coordinate_limit_max", 1, set_coordinate_limit_max);
  ros::Subscriber subYOLOv4 = nh.subscribe ("/object/ROI_array", 1, get_RegionYOLO);

  // Create a ROS ServiceServer for save pointcloud service 
  ros::ServiceServer savePointCloud_service = nh.advertiseService("snapshot", savePointCloud);
  ros::ServiceServer Align_service = nh.advertiseService("AlignPointCloud", Align);

  // geometry_msgs::Point min, max;

  // if (argc==1)
  // {
  //   min.x = -0.5;
  //   min.y = -0.5;
  //   min.z = -0.5;

  //   max.x = 0.5;
  //   max.y = 0.5;
  //   max.z = 0.5;
  // }
  // else if(argc==7)
  // {
  //   min.x = atof(argv[1]);
  //   max.x = atof(argv[2]);

  //   min.y = atof(argv[3]);
  //   max.y = atof(argv[4]);

  //   min.z = atof(argv[5]);
  //   max.z = atof(argv[6]);
  // }
  // else
  // {
  //   min.x = -1.0;
  //   min.y = -1.0;
  //   min.z = -1.0;

  //   max.x = 1.0;
  //   max.y = 1.0;
  //   max.z = 0.3;
  // }
  // pubCoordinateLimitMin.publish(min);
  // pubCoordinateLimitMax.publish(max);

  ros::spin();
}