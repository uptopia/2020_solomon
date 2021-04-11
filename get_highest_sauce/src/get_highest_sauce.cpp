// get_highest_sauce.cpp
//
//
// 
// written by Shang-Wen, Wong(2020.9.18)


#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// pcl
#include <pcl/console/time.h>
#include <pcl/common/angles.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/registration/icp.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing_rgb.h>

//#include< pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h>

//boost
#include <boost/make_shared.hpp>

//realsense_msgs
#include <realsense2_camera/Extrinsics.h>

//darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

// //server, client
#include "get_highest_sauce/HighestSauce.h"

//C++ 
#include <vector>
#include <iostream>
#include <algorithm>


typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointTRGB;
typedef boost::shared_ptr<pcl::PointCloud<PointTRGB>> PointCloudTRGBPtr;

using namespace std;

struct Center2D
{
    int x;
    int y;
};

struct Center3D
{
    float x;
    float y;
    float z;
};

struct Box2D
{
    int xmin;
    int ymin;
    int xmax;
    int ymax;
};

struct Sauce
{
    std::string sauce_class;
    float probability;
    Box2D box_pixel;
    Center2D center_pixel;
    Center3D center_point;
    PointCloudTRGBPtr depth_cloud;
    PointCloudTRGBPtr rgb_cloud;
    int pick_ranking;
    int num_of_clusters;
};

int img_width;
int img_height;

bool data_received_yolo = false;
bool data_received_depth = false;
bool data_received_rgb = false;
bool cloud_saved_depth = false;
bool cloud_saved_rgb = false;

bool input_cloud_from_file = false;

std::string file_path_cloud_depth = "../solomon_ws/src/get_highest_sauce/depth_cloud_tmp.pcd";
std::string file_path_cloud_rgb = "../solomon_ws/src/get_highest_sauce/rgb_cloud_tmp.pcd";

//all information in one scene
//int total_num_of_sauce_type_in_a_scene = 0;//
std::vector<std::string> sauce_type{};
int num_sauce_types = 0;
std::vector<Sauce> sauces_all{};

Eigen::Matrix4f matrix_depth2color = Eigen::Matrix4f::Identity();

pcl::PointCloud<PointTRGB>::Ptr depth_cloud_ori(new pcl::PointCloud<PointTRGB>);
pcl::PointCloud<PointTRGB>::Ptr rgb_cloud_ori(new pcl::PointCloud<PointTRGB>);
pcl::PointCloud<PointTRGB>::Ptr cloud_rgb_aligned2depth(new pcl::PointCloud<PointTRGB>);

pcl::PointCloud<PointTRGB>::Ptr tmp_cloud(new pcl::PointCloud<PointTRGB>);
pcl::PointCloud<PointTRGB>::Ptr target_sauce_center(new pcl::PointCloud<PointTRGB>);

//sauce clusters
std::vector<std::string> text_list{};

bool enable_pcl_visualizer = false; //[true] pcl_visualizer; [false] rviz
//boost::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer ("Top 3 Sauces"));

ros::Publisher top3_cloud_pub, target_sauce_center_pub;
sensor_msgs::PointCloud2 top3_clouds_msg;
sensor_msgs::PointCloud2 target_sauce_center_msg;

//Higheset Sauce Infomation
std::string highest_sauce_class;
float highest_sauce_3D_point_x;
float highest_sauce_3D_point_y;
float highest_sauce_3D_point_z;



// bool compare_num_of_clusters(const Sauce &sauce1, const Sauce &sauce2)
// {
//     return sauce1.num_of_clusters.size() > sauce2.num_of_clusters.size();
// }

bool compare_highest_points(const Sauce &sauce1, const Sauce &sauce2) //minimum z is the higheset!!!!
{ 
    return sauce1.center_point.z < sauce2.center_point.z;
}

bool compare_num_of_cluster_points(const pcl::PointIndices &cluster1, const pcl::PointIndices &cluster2)
{
    return cluster1.indices.size() > cluster2.indices.size();
}

void camera_extrinsic_cb (const realsense2_camera::Extrinsics& extrinsicsMsg)
{
    //=================================================================//
    // Transformation from realsense depth to color coordinate system
    // Subscribe "/camera/extrinsic/depth_to_color" topic
    //=================================================================//

    cout<< "\ncamera_extrinsic_cb\n";
    
    matrix_depth2color = Eigen::Matrix4f::Identity();
    matrix_depth2color <<
    extrinsicsMsg.rotation[0], extrinsicsMsg.rotation[1], extrinsicsMsg.rotation[2], extrinsicsMsg.translation[0],
    extrinsicsMsg.rotation[3], extrinsicsMsg.rotation[4], extrinsicsMsg.rotation[5], extrinsicsMsg.translation[1],
    extrinsicsMsg.rotation[6], extrinsicsMsg.rotation[7], extrinsicsMsg.rotation[8], extrinsicsMsg.translation[2],
    0.0, 0.0, 0.0, 1.0;
    
    cout << "frame id =" << extrinsicsMsg.header.frame_id << endl;
    cout << "matrix_depth2color = \n" << matrix_depth2color << endl;
}

void yolo_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& boxes_msg)
{
    //==================================================//
    // Subscribe "/darknet_ros/bounding_boxes" topic
    //==================================================//
    //cout << "\nyolo_callback\n";
    //cout << "Bouding Boxes (header):" << boxes_msg->header << endl;
    //cout << "Bouding Boxes (image_header):" << boxes_msg->image_header << endl;

    int obj_num = boxes_msg->bounding_boxes.size();
    
    if(data_received_yolo == false)
    {
        sauce_type = {};
        sauces_all.resize(obj_num);

        for(int k = 0; k < obj_num; ++k)
        {
            std::string sauce_class = boxes_msg->bounding_boxes[k].Class;
            float probability = boxes_msg->bounding_boxes[k].probability;
            int xmin = boxes_msg->bounding_boxes[k].xmin;
            int xmax = boxes_msg->bounding_boxes[k].xmax;
            int ymin = boxes_msg->bounding_boxes[k].ymin;
            int ymax = boxes_msg->bounding_boxes[k].ymax;
            int center_x = int((xmin +xmax)/2.0);
            int center_y = int((ymin +ymax)/2.0);
            
            sauces_all[k].sauce_class = sauce_class;
            sauces_all[k].probability = probability;
            sauces_all[k].box_pixel.xmin = xmin;
            sauces_all[k].box_pixel.xmax = xmax;
            sauces_all[k].box_pixel.ymin = ymin;
            sauces_all[k].box_pixel.ymax = ymax;
            sauces_all[k].center_pixel.x = center_x;
            sauces_all[k].center_pixel.y = center_y;

            sauce_type.push_back(sauce_class);
        }

        std::sort(sauce_type.begin(), sauce_type.end());
        sauce_type.erase(std::unique(sauce_type.begin(), sauce_type.end()), sauce_type.end());
        num_sauce_types = sauce_type.size();

        if(boxes_msg->bounding_boxes.empty())
            obj_num = 0;
        if(sauce_type.empty())
            num_sauce_types = 0;

        cout << "Total Yolo clusters = " << obj_num << endl;   //ERROR: display 1 even if no obj detected
        cout << "Total num of sauce types = " << num_sauce_types << endl; //ERROR: display 1 even if no obj detected

        //data_received_yolo = true;
    }
}

void depth_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& depth_cloud_msg)
{
    //==================================================//
    // Organized Point Cloud; Depth Point Cloud
    // Subscribe "/camera/depth_registered/points" topic
    //==================================================//
    //cout << "depth_cloud_cb\n";
    cout<<"depth_cloud_cb"<<endl;
    int points = depth_cloud_msg->height * depth_cloud_msg->width;

    if((points!=0) && (data_received_depth == false) && (cloud_saved_depth ==false))
    {
        img_width = depth_cloud_msg->width;
        img_height = depth_cloud_msg->height;

        // 將點雲格式由sensor_msgs/PointCloud2轉成pcl/PointCloud(PointXYZ, PointXYZRGB)
        depth_cloud_ori->clear();
        pcl::fromROSMsg(*depth_cloud_msg, *depth_cloud_ori);
        //data_received_depth = true;

        if(input_cloud_from_file == true)
        {
            pcl::io::savePCDFileBinary<PointTRGB>(file_path_cloud_depth, *depth_cloud_ori);
            cout << "depth_cloud_ori saved: " << file_path_cloud_depth << "; (width, height) = " << depth_cloud_ori->width << ", " << depth_cloud_ori->height << endl;
            //cloud_saved_depth = true;
        }
    }
}

void rgb_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& rgb_cloud_msg)
{
    //==================================================//
    // Unorganized Point Cloud; RGB Point Cloud
    // Subscribe "/camera/depth/color/points" topic
    //==================================================//
    //cout<<"rgb_cloud_cb\n";

    int points = rgb_cloud_msg->height * rgb_cloud_msg->width;

    if((points!=0) && (data_received_rgb ==false) && (cloud_saved_rgb ==false))
    {
        // 將點雲格式由sensor_msgs/PointCloud2轉成pcl/PointCloud(PointXYZ, PointXYZRGB)
        rgb_cloud_ori->clear();
        pcl::fromROSMsg(*rgb_cloud_msg, *rgb_cloud_ori);

        //data_received_rgb = true;

        if(input_cloud_from_file == true)
        {
            pcl::io::savePCDFileBinary<PointTRGB>(file_path_cloud_rgb, *rgb_cloud_ori);
            cout << "rgb_cloud_ori saved: " << file_path_cloud_rgb << "; (width, height) = " << rgb_cloud_ori->width << ", " << rgb_cloud_ori->height << endl;
            //cloud_saved_rgb = true;
        }
    }    
}

bool pick_highest_sauce()
{
    tmp_cloud->clear();
    
    // if((data_received_depth == true) && (data_received_rgb == true))
    // {
    //     //===================================//
    //     // Input Depth, RGB Point Cloud: 
    //     // (1) from file; or (2) from buffer
    //     //===================================//
    //     if((input_cloud_from_file == true) && (cloud_saved_depth == true) && (cloud_saved_rgb == true))
    //     {
    //         pcl::io::loadPCDFile<PointTRGB>(file_path_cloud_depth, *depth_cloud_ori);
    //         pcl::io::loadPCDFile<PointTRGB>(file_path_cloud_rgb, *rgb_cloud_ori);
    //         cout << "Load depth_cloud, rgb_cloud from file!\n";
    //     }
        cout << "[rgb_cloud_ori] total points = " << rgb_cloud_ori->size() << endl;    
        cout << "[depth_cloud_ori] total points= " << depth_cloud_ori->size() << endl;

        cout << "\n\n\n"
                << "//======================================//\n"
                << "*****Find Highest Sauce, START!*****\n"
                << "//======================================//\n\n";
        
        //===================================//
        // Align RGB cloud to Depth cloud
        // via matrix_depth2color
        //===================================//
        pcl::transformPointCloud(*rgb_cloud_ori, *cloud_rgb_aligned2depth, matrix_depth2color);
        //view_cloudRGB2("aligned result", depth_cloud_ori, cloud_rgb_aligned2depth);

        //========================================//
        // Go over all EVERY Yolov4 detected sauces
        // save 2D, 3D sauce information
        //========================================//
        for(int n = 0; n < sauces_all.size(); ++n)
        {
            cout << "Sauce #" << n << endl;
            
            //=========================================//
            // Extract Sauce's Depth Cloud(Orgainized)
            // 2D pixel mapping to 3D points
            //=========================================//
            sauces_all[n].depth_cloud = boost::make_shared<pcl::PointCloud<PointTRGB>>();

            int xmin = sauces_all[n].box_pixel.xmin;
            int xmax = sauces_all[n].box_pixel.xmax;
            int ymin = sauces_all[n].box_pixel.ymin;
            int ymax = sauces_all[n].box_pixel.ymax;
            
            //Ensure the 2D pixels are inside image's max width, height
            if(xmin < 0) xmin = 114;//186;//0;
            if(ymin < 0) ymin = 40;//74;//0;
            if(xmax > img_width-1) xmax = 723;//1085;//img_width-1;
            if(ymax > img_height-1) ymax = 424;//648;//img_height-1;
            cout<<"\timgwidth, imgHeight = "<< img_width <<",  "<< img_height<<endl;
            cout<< "\tPixel (xmin, xmax, ymin, ymax) = "<< xmin << ", " << xmax <<", " << ymin << ", " << ymax << endl;

            //Map 2D pixel to 3D points
            for(int i = xmin; i <= xmax; i++)
            {
                for(int j = ymin; j<= ymax; j++)
                {
                    PointTRGB depth_pt = depth_cloud_ori->at(i, j);
                    if(pcl_isfinite(depth_pt.x) && pcl_isfinite(depth_pt.y) && pcl_isfinite(depth_pt.z))
                    {
                        sauces_all[n].depth_cloud->push_back(depth_pt);
                        //tmp_cloud->push_back(depth_pt);
                    }
                }
            }
            cout << "\tExtract [depth_cloud] = " << sauces_all[n].depth_cloud->size() << endl;

            //==================================================//
            // Extract Sauce's RGB Cloud(Unorgainized)
            // Use Sauce's depth_cloud to find nearest neighbors
            // in rgb_cloud
            //==================================================//
            // Find neighbors within radius
            float search_radius = 0.005;    //(i.e. 0.005 = 5mm)

            pcl::KdTreeFLANN<PointTRGB> kdtree;
            kdtree.setInputCloud(cloud_rgb_aligned2depth);
            
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            std::vector<int> IdxRadiusSearchAll_rgb_sauces;

            for(int nn =0; nn < sauces_all[n].depth_cloud->size(); ++nn)
            {
                pointIdxRadiusSearch.clear();
                pointRadiusSquaredDistance.clear();

                PointTRGB search_pt = sauces_all[n].depth_cloud->points[nn];
                                        
                if(pcl_isfinite(search_pt.x) && pcl_isfinite(search_pt.y) && pcl_isfinite(search_pt.z))
                {
                    if(kdtree.radiusSearch(search_pt, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
                    {
                        IdxRadiusSearchAll_rgb_sauces.insert(IdxRadiusSearchAll_rgb_sauces.end(), pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end());
                    }
                }                        
            }

            // Remove duplicate indices
            //std::vector<int> last;
            sort(IdxRadiusSearchAll_rgb_sauces.begin(), IdxRadiusSearchAll_rgb_sauces.end(), greater<int>());
            IdxRadiusSearchAll_rgb_sauces.erase(std::unique(IdxRadiusSearchAll_rgb_sauces.begin(), IdxRadiusSearchAll_rgb_sauces.end()), IdxRadiusSearchAll_rgb_sauces.end());
                            
            sauces_all[n].rgb_cloud = boost::make_shared<pcl::PointCloud<PointTRGB>>();
            for(int mm = 0; mm < IdxRadiusSearchAll_rgb_sauces.size(); mm++)
            {
                PointTRGB rgb_pt = cloud_rgb_aligned2depth->points[IdxRadiusSearchAll_rgb_sauces[mm]];
                sauces_all[n].rgb_cloud->push_back(rgb_pt);
                //tmp_cloud->push_back(rgb_pt);
            }
            cout << "\tExtract [rgb_cloud] = " << sauces_all[n].rgb_cloud->size() << endl;
    
            //==========================================//
            // Get Center 3D Points
            // map 2D center_pixel to 3D center_point
            //==========================================//
            int center_x = sauces_all[n].center_pixel.x;
            int center_y = sauces_all[n].center_pixel.y;

            PointTRGB center_pt_3d = depth_cloud_ori->at(center_x, center_y);
            cout << "\tCenter_pt_3d = " << center_pt_3d.x << ", " << center_pt_3d.y << ", " << center_pt_3d.z << endl;

            // if Center_pt_3d is NAN, use all cluster's points
            if(!pcl_isfinite(center_pt_3d.x) || !pcl_isfinite(center_pt_3d.y) || !pcl_isfinite(center_pt_3d.z))
            {
                int total_points = sauces_all[n].depth_cloud->size();
                center_pt_3d.x = 0;
                center_pt_3d.y = 0;
                center_pt_3d.z = 0;

                for(int kk = 0; kk < total_points; ++kk)
                {
                    PointTRGB pt = sauces_all[n].depth_cloud->points[kk];
                    center_pt_3d.x += pt.x;
                    center_pt_3d.y += pt.y;
                    center_pt_3d.z += pt.z;
                }

                center_pt_3d.x /= total_points;
                center_pt_3d.y /= total_points;
                center_pt_3d.z /= total_points;

                cout << "\t**Center_pt_3d = " << center_pt_3d.x << ", " << center_pt_3d.y << ", " << center_pt_3d.z << endl;
            }
            sauces_all[n].center_point.x = center_pt_3d.x;
            sauces_all[n].center_point.y = center_pt_3d.y;
            sauces_all[n].center_point.z = center_pt_3d.z;         
        }
        
        //======================================================//
        // Select highest sauce
        // Highest sauce criteria:
        // (1) smallest z value (closest to the camera)
        //======================================================//
        std::sort(sauces_all.begin(), sauces_all.end(), compare_highest_points);

        //========================================//
        // Visualize highest sauce on rgb_cloud
        //========================================//
        if(enable_pcl_visualizer == true)
        {
            // cout << "\n\n";          
            // cout<< "*****Find Sauces info*****\n";
            // for(int index = 0; index < text_list.size()/2; ++index)
            // {   
            //     view->removePointCloud("target_sauce_center");
            //     view->removePointCloud(text_list[2*index]);
            //     view->removeText3D(text_list[2*index+1]);
            // }
        
            // view->addCoordinateSystem(0.100);
            // view->setBackgroundColor(1, 1, 1);

            // text_list.clear();
            // for(int idx = 0; idx < sauces_all.size(); ++idx)
            // {   
            //     std::string cloud_name = "cloud" + to_string(idx);
            //     std::string text = "Rank # " + to_string(idx+1);
            //     text = text + "; ";
            //     text = text + sauces_all[idx].sauce_class;
            //     text = text + "; z = ";
            //     text = text + to_string(sauces_all[idx].center_point.z);

            //     cout << text << endl;

            //     //Show top 3 on the viewer
            //     if(idx <3)
            //     {                    
            //         PointTRGB pt_target;
            //         pt_target.x = sauces_all[idx].center_point.x;
            //         pt_target.y = sauces_all[idx].center_point.y;
            //         pt_target.z = sauces_all[idx].center_point.z;

            //         text_list.push_back(cloud_name);              
            //         view->addPointCloud<PointTRGB>(sauces_all[idx].rgb_cloud, pcl::visualization::PointCloudColorHandlerRGBField<PointTRGB>(sauces_all[idx].rgb_cloud), cloud_name);
                    
            //         text_list.push_back(text);
            //         view->addText3D(text, pt_target, 0.01, 1.0, 0.0, 0.0); //text, position, textscale, r, g, b

            //         //Plot Center Point on Top1 sauce
            //         if(idx == 0)
            //         {
            //             target_sauce_center->clear();
            //             target_sauce_center->push_back(pt_target);
            //             view->addPointCloud<PointTRGB>(target_sauce_center, pcl::visualization::PointCloudColorHandlerCustom<PointTRGB>(target_sauce_center, 255, 0 ,0), "target_sauce_center");
            //             view->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "target_sauce_center");
            //         }   
            //     }
            // }
            // view->setCameraPosition(
            //     0.0, 0.0, -1.00,     // camera位置
            //     0.0, 0.0, 1.0,       // view向量--相机朝向
            //     0.0, -1.0, 0.0       // up向量
            // );
            // //view->spin();
            // view->spinOnce(10);
        }
        else
        {
            //Display Top 3 Sauces via Rviz
            pcl::PointCloud<PointTRGB>::Ptr top3_clouds(new pcl::PointCloud<PointTRGB>);            

            for(int idx = 0; idx < sauces_all.size(); ++idx)
            {   
                if(idx <3)
                {                    
                    PointTRGB pt_target;
                    pt_target.x = sauces_all[idx].center_point.x;
                    pt_target.y = sauces_all[idx].center_point.y;
                    pt_target.z = sauces_all[idx].center_point.z;

                    *top3_clouds = *top3_clouds + *(sauces_all[idx].rgb_cloud);
             
                    //Plot Center Point on Top1 sauce
                    if(idx == 0)
                    {
                        target_sauce_center->clear();
                        target_sauce_center->push_back(pt_target);
                    }
                }
            }
            
            //Publish pcl::PointCloud to ROS sensor::PointCloud2, and to topic
            pcl::toROSMsg(*top3_clouds, top3_clouds_msg);
            pcl::toROSMsg(*target_sauce_center, target_sauce_center_msg);

            top3_clouds_msg.header.frame_id = "camera_depth_optical_frame";
            target_sauce_center_msg.header.frame_id = "camera_depth_optical_frame";

            top3_cloud_pub.publish(top3_clouds_msg);
            target_sauce_center_pub.publish(target_sauce_center_msg);
        }
        
        //========================================//
        // Output highest sauce information
        //========================================//
        if(sauces_all.size()!=0)
        {
            highest_sauce_class = sauces_all[0].sauce_class;
            highest_sauce_3D_point_x = sauces_all[0].center_point.x;
            highest_sauce_3D_point_y = sauces_all[0].center_point.y;
            highest_sauce_3D_point_z = sauces_all[0].center_point.z;    

            cout << "\n\n"
                << "*****Find Highest Sauce, Done!*****\n"
                << "\thighest_sauce_class = " << highest_sauce_class << endl
                << "\thighest_sauce_3D_point_x = " << highest_sauce_3D_point_x << endl
                << "\thighest_sauce_3D_point_y = " << highest_sauce_3D_point_y << endl
                << "\thighest_sauce_3D_point_z = " << highest_sauce_3D_point_z << "\n\n\n";
            
            // data_received_yolo = false;
            // data_received_depth = false;
            // data_received_rgb = false;
            // cloud_saved_rgb = false;
            // cloud_saved_depth = false;
            sauces_all = {};

            return true;
        }
        else
        {
            return false;
        }
        
    //}    
    // ros::spinOnce();
    // ros::Duration(5).sleep();
}

bool response_highest(get_highest_sauce::HighestSauce::Request& req, get_highest_sauce::HighestSauce::Response& res)
{

    cout<<"start"<<endl;
    res.is_done = false;
    
    if(req.picture_pose_reached == true)
    {
        cout << "req.picture_pose_reached = " << int(req.picture_pose_reached) << endl;

        data_received_yolo = false;
        data_received_depth = false;
        data_received_rgb = false;
        cloud_saved_rgb = false;
        cloud_saved_depth = false;

        bool highest_get = pick_highest_sauce();

        if(highest_get == true)
        {
            cout<<"highest_get == true"<<endl;
            res.multiple_sauce_type = ((num_sauce_types == 0) || (num_sauce_types == 1))? false: true;
            res.sauce_class = highest_sauce_class;
            res.center_point_x = highest_sauce_3D_point_x;
            res.center_point_y = highest_sauce_3D_point_y;
            res.center_point_z = highest_sauce_3D_point_z;
            res.is_done = true;
            cout << "[SERVER] sending back response (highest_sauce)" << res.sauce_class << endl;
        }
        else
        {
            //NO SAUCE OBJECT; OR UNABLE TO DETECT SAUCES
            cout << "!!!!!!!!!!!!!!NO SAUCE OBJECT; OR UNABLE TO DETECT SAUCES!!!!!!!!!!!!!!\n" << endl;
            cout << "!!!!!!!!!!!!!!NO SAUCE OBJECT; OR UNABLE TO DETECT SAUCES!!!!!!!!!!!!!!\n" << endl;
            cout << "!!!!!!!!!!!!!!NO SAUCE OBJECT; OR UNABLE TO DETECT SAUCES!!!!!!!!!!!!!!\n" << endl;
        }            
    }
    else
    {
        cout << "req.picture_pose_reached = " << req.picture_pose_reached << endl;
    }
    cout<<"end"<<endl;
    return true;
}

// void view_cloudRGB(std::string window_name, pcl::PointCloud<PointTRGB>::Ptr& cloud)
// {
//     pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (window_name));
//     pcl::visualization::PointCloudColorHandlerRGBField<PointTRGB> rgb(cloud);
//     viewer->addPointCloud<PointTRGB>(cloud, rgb, "cloud");
//     viewer->addCoordinateSystem(0.100);
//     viewer->spin();
// }

// void view_cloudRGB2(std::string window_name, pcl::PointCloud<PointTRGB>::Ptr& cloud1, pcl::PointCloud<PointTRGB>::Ptr& cloud2)
// {
//     pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (window_name));
//     pcl::visualization::PointCloudColorHandlerRGBField<PointTRGB> rgb1(cloud1);
//     pcl::visualization::PointCloudColorHandlerRGBField<PointTRGB> rgb2(cloud2);
//     viewer->addPointCloud<PointTRGB>(cloud1, rgb1, "cloud1");
//     viewer->addPointCloud<PointTRGB>(cloud2, rgb2, "cloud2");
//     viewer->addCoordinateSystem(0.100);
//     viewer->spin();
// }


int main(int argc, char** argv)
{   
    //====================//
    // Get Highest Sauce
    //====================//
    ros::init(argc, argv, "get_highest_sauce_server");
    
    //====================//
    // Subscribe topics
    //====================//
    ros::NodeHandle nh;

    cout << "\n\n\n"
        << "//================================//\n"
        << "*****Subscribe ROS Topics*****\n"
        << "//================================//\n\n";
    ros::Subscriber sub_cam_extrinsic = nh.subscribe("/camera/extrinsics/depth_to_color", 1, camera_extrinsic_cb);  //depth_to_color_trans_matrix
    ros::Subscriber sub_depth_cloud = nh.subscribe("/camera/depth_registered/points", 1, depth_cloud_cb);           //depth point cloud (organized)
    ros::Subscriber sub_rgb_cloud = nh.subscribe("/camera/depth/color/points", 1, rgb_cloud_cb);                    //color point cloud (unorganized)
    ros::Subscriber sub_yolov4 = nh.subscribe("/darknet_ros/bounding_boxes", 1, yolo_cb);                           //yolov4 detection result
 
    top3_cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/top3_cloud_pub", 1);
    target_sauce_center_pub = nh.advertise<sensor_msgs::PointCloud2> ("/target_sauce_center_pub", 1);
    ros::ServiceServer service = nh.advertiseService("/get_highest_sauce", response_highest);

    ros::spin();

    return 0;    
}