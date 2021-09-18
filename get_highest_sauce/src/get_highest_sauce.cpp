// get_highest_sauce.cpp
//
//
// 
// written by Shang-Wen, Wong(2021.5.11)

//ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

//ROS Service, Client
#include "get_highest_sauce/HighestSauce.h"

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/distances.h>

//boost
#include <boost/make_shared.hpp>

//Yolov4 darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

//C++ 
#include <vector>
#include <iostream>
#include <algorithm>

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
    PointCloudTRGBPtr cloud;    
};

int img_width;
int img_height;

std::vector<std::string> sauce_type{};
int num_sauce_types = 0;
std::vector<Sauce> sauces_all{};

pcl::PointCloud<PointTRGB>::Ptr organized_cloud_ori(new pcl::PointCloud<PointTRGB>);
pcl::PointCloud<PointTRGB>::Ptr top3_sauce(new pcl::PointCloud<PointTRGB>);            
pcl::PointCloud<PointTRGB>::Ptr top1_sauce_center(new pcl::PointCloud<PointTRGB>);

sensor_msgs::PointCloud2 top3_sauce_msg;
sensor_msgs::PointCloud2 top1_sauce_center_msg;
ros::Publisher top3_sauce_pub, top1_sauce_center_pub;

//Higheset Sauce Infomation
std::string highest_sauce_class;
float highest_sauce_3D_point_x;
float highest_sauce_3D_point_y;
float highest_sauce_3D_point_z;

bool compare_highest_points(const Sauce &sauce1, const Sauce &sauce2)
{ 
    //Object with 'minimum z' is the 'higheset' in the [Camera] coordinate system
    return sauce1.center_point.z < sauce2.center_point.z;
}

void yolo_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& boxes_msg)
{
    //==================================================//
    // Subscribe "/darknet_ros/bounding_boxes" topic
    //==================================================//
    // cout << "\nYolo_callback\n";
    //cout << "Bouding Boxes (header):" << boxes_msg->header << endl;
    //cout << "Bouding Boxes (image_header):" << boxes_msg->image_header << endl;

    int obj_num = boxes_msg->bounding_boxes.size();
    
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

    // cout << "Total Yolo objects = " << obj_num << endl;
    // cout << "Total # of sauce types = " << num_sauce_types << endl;
}

void organized_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& organized_cloud_msg)
{
    //==================================================//
    // Organized Point Cloud; Depth Point Cloud
    // Subscribe "/camera/depth_registered/points" topic
    //==================================================//
    // cout << "organized_cloud_cb" << endl;

    int img_width = organized_cloud_msg->width;
    int img_height = organized_cloud_msg->height;
    int points = img_height * img_width;

    if((points!=0))
    {
        //將點雲格式由sensor_msgs/PointCloud2轉成pcl/PointCloud(PointXYZ, PointXYZRGB)
        organized_cloud_ori->clear();
        pcl::fromROSMsg(*organized_cloud_msg, *organized_cloud_ori);
        
        // cout << "organized_cloud_ori (width, height) = " << img_width << ", " << img_height << endl;
    }
}

bool pick_highest_sauce()
{  
    //========================================//
    // Go over all EVERY Yolov4 detected sauces
    // save 2D, 3D sauce information
    //========================================//
    for(int n = 0; n < sauces_all.size(); ++n)
    {
        // cout << "Sauce #" << n << endl;
        
        //=========================================//
        // Extract Sauce's Depth Cloud(Orgainized)
        // 2D pixel mapping to 3D points
        //=========================================//
        sauces_all[n].cloud = boost::make_shared<pcl::PointCloud<PointTRGB>>();

        int xmin = sauces_all[n].box_pixel.xmin;
        int xmax = sauces_all[n].box_pixel.xmax;
        int ymin = sauces_all[n].box_pixel.ymin;
        int ymax = sauces_all[n].box_pixel.ymax;
        
        // //Ensure the 2D pixels are inside image's max width, height
        // if(xmin < 0) xmin = 114;//186;//0;
        // if(ymin < 0) ymin = 40;//74;//0;
        // if(xmax > img_width-1) xmax = 723;//1085;//img_width-1;
        // if(ymax > img_height-1) ymax = 424;//648;//img_height-1;
        // cout<<"\timgwidth, imgHeight = "<< img_width <<",  "<< img_height<<endl;
        // cout<< "\tPixel (xmin, xmax, ymin, ymax) = "<< xmin << ", " << xmax <<", " << ymin << ", " << ymax << endl;

        //Map 2D pixel to 3D points
        for(int i = xmin; i <= xmax; i++)
        {
            for(int j = ymin; j<= ymax; j++)
            {
                PointTRGB mapped_pt = organized_cloud_ori->at(i, j);
                if(pcl_isfinite(mapped_pt.x) && pcl_isfinite(mapped_pt.y) && pcl_isfinite(mapped_pt.z))
                {
                    sauces_all[n].cloud->push_back(mapped_pt);
                }
            }
        }
        // cout << "\tExtracted sauce cloud pt size = " << sauces_all[n].cloud->size() << endl;
            
        //==========================================//
        // Get Center 3D Points
        // map 2D center_pixel to 3D center_point
        //==========================================//
        int center_x = sauces_all[n].center_pixel.x;
        int center_y = sauces_all[n].center_pixel.y;

        //=============//
        // new method //
        //=============//
        PointTRGB center_pt_3d_ori = organized_cloud_ori->at(center_x, center_y);
        PointTRGB center_pt_3d_avg = organized_cloud_ori->at(center_x, center_y);
        // cout << "\tCenter_pt_3d = " << center_pt_3d_ori.x << ", " << center_pt_3d_ori.y << ", " << center_pt_3d_ori.z << endl;

        // no matter center_pt_3d_ori is NAN or not, avg with points less than 1 cm
        int total_points = sauces_all[n].cloud->size();
        center_pt_3d_avg.x = 0;
        center_pt_3d_avg.y = 0;
        center_pt_3d_avg.z = 0;

        int point_use_to_avg = 0;
        for(int kk = 0; kk < total_points; ++kk)
        {
            PointTRGB pt = sauces_all[n].cloud->points[kk];
            if(pcl_isfinite(pt.x) && pcl_isfinite(pt.y) && pcl_isfinite(pt.z))
            {
                if(pcl::euclideanDistance(pt, center_pt_3d_ori) < 0.010) //if pt distance less than 1 cm
                {               
                    center_pt_3d_avg.x += pt.x;
                    center_pt_3d_avg.y += pt.y;
                    center_pt_3d_avg.z += pt.z;
                    point_use_to_avg += 1;
                }
            }
        }

        center_pt_3d_avg.x /= point_use_to_avg;
        center_pt_3d_avg.y /= point_use_to_avg;
        center_pt_3d_avg.z /= point_use_to_avg;
        // cout<<center_pt_3d_avg.x <<", "<<center_pt_3d_avg.y <<", "<< center_pt_3d_avg.z<<", "<<point_use_to_avg<<endl;

        sauces_all[n].center_point.x = center_pt_3d_avg.x;
        sauces_all[n].center_point.y = center_pt_3d_avg.y;
        sauces_all[n].center_point.z = center_pt_3d_avg.z;         

        //=============//
        // prev method //
        //=============//
        // PointTRGB center_pt_3d = organized_cloud_ori->at(center_x, center_y);
        // // cout << "\tCenter_pt_3d = " << center_pt_3d.x << ", " << center_pt_3d.y << ", " << center_pt_3d.z << endl;

        // // if Center_pt_3d is NAN, use all cluster's points
        // if(!pcl_isfinite(center_pt_3d.x) || !pcl_isfinite(center_pt_3d.y) || !pcl_isfinite(center_pt_3d.z))
        // {
        //     int total_points = sauces_all[n].cloud->size();
        //     center_pt_3d.x = 0;
        //     center_pt_3d.y = 0;
        //     center_pt_3d.z = 0;

        //     for(int kk = 0; kk < total_points; ++kk)
        //     {
        //         PointTRGB pt = sauces_all[n].cloud->points[kk];
        //         center_pt_3d.x += pt.x;
        //         center_pt_3d.y += pt.y;
        //         center_pt_3d.z += pt.z;
        //     }

        //     center_pt_3d.x /= total_points;
        //     center_pt_3d.y /= total_points;
        //     center_pt_3d.z /= total_points;

        //     // cout << "\t**Center_pt_3d = " << center_pt_3d.x << ", " << center_pt_3d.y << ", " << center_pt_3d.z << endl;
        // }
        // sauces_all[n].center_point.x = center_pt_3d.x;
        // sauces_all[n].center_point.y = center_pt_3d.y;
        // sauces_all[n].center_point.z = center_pt_3d.z;         
    }
    
    //=================================================================//
    // Select highest sauce in CAMERA coordinate system
    // Highest sauce criteria: minimum z value (closest to the camera)
    //=================================================================//
    std::sort(sauces_all.begin(), sauces_all.end(), compare_highest_points);                


    //========================================//
    // Visualize highest sauce on Rviz
    //========================================//
    top3_sauce->clear();
    top1_sauce_center->clear();

    int top1_sauce_idx = 0;

    // for(int idx = 0; idx < sauces_all.size(); ++idx)
    // {   
    //     if(idx <3)
    //     {                    
    //         PointTRGB pt_target;
    //         pt_target.x = sauces_all[idx].center_point.x;
    //         pt_target.y = sauces_all[idx].center_point.y;
    //         pt_target.z = sauces_all[idx].center_point.z;

    //         *top3_sauce = *top3_sauce + *(sauces_all[idx].cloud);
        
    //         //Plot Center Point on Top1 sauce
    //         if(idx == 0)
    //         {
    //             top1_sauce_center->clear();
    //             top1_sauce_center->push_back(pt_target);
    //         }
    //     }
    // }

    int top3_cnt = 0;
    for(int idx = 0; idx < sauces_all.size(); ++idx)
    {   
        std::string obj_class = sauces_all[idx].sauce_class;
        if(!obj_class.compare("egg") |\
            !obj_class.compare("ketchup") |\
            !obj_class.compare("pepper"))
        {
            if(pcl_isfinite(sauces_all[idx].center_point.x) && pcl_isfinite(sauces_all[idx].center_point.y) && pcl_isfinite(sauces_all[idx].center_point.z))
            {
                if(top3_cnt == 0) //(top3_cnt < 3)
                {                    
                    PointTRGB pt_target;
                    pt_target.x = sauces_all[idx].center_point.x;
                    pt_target.y = sauces_all[idx].center_point.y;
                    pt_target.z = sauces_all[idx].center_point.z;

                    *top3_sauce = *top3_sauce + *(sauces_all[idx].cloud);
            
                    top1_sauce_center->clear();
                    top1_sauce_center->push_back(pt_target);
                    top1_sauce_idx = idx;

                    // //Plot Center Point on Top1 sauce
                    // if(top3_cnt == 0)
                    // {
                        // top1_sauce_center->clear();
                        // top1_sauce_center->push_back(pt_target);
                        // top1_sauce_idx = idx;
                    // }
                    top3_cnt++;
                }
            }
        }
        else
        {
            cout << "Detect UNWANTED class: " << sauces_all[idx].sauce_class << endl;
        }            
    }
    
    //Publish pcl::PointCloud to ROS sensor::PointCloud2, and to topic
    pcl::toROSMsg(*top3_sauce, top3_sauce_msg);
    pcl::toROSMsg(*top1_sauce_center, top1_sauce_center_msg);

    top3_sauce_msg.header.frame_id = "camera_depth_optical_frame";
    top1_sauce_center_msg.header.frame_id = "camera_depth_optical_frame";

    top3_sauce_pub.publish(top3_sauce_msg);
    top1_sauce_center_pub.publish(top1_sauce_center_msg);
    
    //==============================================//
    // Output highest sauce information to terminal
    //==============================================//
    if(sauces_all.size()!=0)
    {
        highest_sauce_class = sauces_all[top1_sauce_idx].sauce_class;
        highest_sauce_3D_point_x = sauces_all[top1_sauce_idx].center_point.x;
        highest_sauce_3D_point_y = sauces_all[top1_sauce_idx].center_point.y;
        highest_sauce_3D_point_z = sauces_all[top1_sauce_idx].center_point.z;    

        // cout << "\n\n"
        //     << "*****Find Highest Sauce, Done!*****\n"
        //     << "\thighest_sauce_class = " << highest_sauce_class << endl
        //     << "\thighest_sauce_3D_point_x = " << highest_sauce_3D_point_x << endl
        //     << "\thighest_sauce_3D_point_y = " << highest_sauce_3D_point_y << endl
        //     << "\thighest_sauce_3D_point_z = " << highest_sauce_3D_point_z << "\n\n\n";

        sauces_all = {};

        return true;
    }
    else
    {
        return false;
    }
}

bool response_highest(get_highest_sauce::HighestSauce::Request& req, get_highest_sauce::HighestSauce::Response& res)
{
    //當抵達拍照位置時(req.picture_pose_reached == true)
    //回傳最高醬料包資訊(res)

    cout << "\nService Node START" << endl;
    res.is_done = false;
    
    if(req.picture_pose_reached == true)
    {
        // cout << "req.picture_pose_reached = " << int(req.picture_pose_reached) << endl;

        bool highest_get = pick_highest_sauce();

        if(highest_get == true)
        {
            // cout << "highest_get == true" << endl;
            res.multiple_sauce_type = ((num_sauce_types == 0) || (num_sauce_types == 1))? false: true;
            res.sauce_class = highest_sauce_class;
            res.center_point_x = highest_sauce_3D_point_x;
            res.center_point_y = highest_sauce_3D_point_y;
            res.center_point_z = highest_sauce_3D_point_z;
            res.is_done = true;
            // cout << "[SERVER] sending back response (highest_sauce)" << res.sauce_class << endl;
            cout << "\033[1;33m=========Pick Sauce=========\033[0m\n";
            cout << "sauce type: " << res.sauce_class << endl;
            cout << "sauce position (x, y, z): " << res.center_point_x << ", "<< res.center_point_y << ", "<< res.center_point_z << endl;
            cout << "\033[1;33m============================\033[0m\n";
        }
        else
        {
            //NO SAUCE OBJECT; OR UNABLE TO DETECT SAUCES
            cout << "!!!!!!!!!!!!!!NO SAUCE OBJECT; OR UNABLE TO DETECT SAUCES!!!!!!!!!!!!!!" << endl;
            // cout << "!!!!!!!!!!!!!!NO SAUCE OBJECT; OR UNABLE TO DETECT SAUCES!!!!!!!!!!!!!!" << endl;
            // cout << "!!!!!!!!!!!!!!NO SAUCE OBJECT; OR UNABLE TO DETECT SAUCES!!!!!!!!!!!!!!" << endl;
        }            
    }
    else
    {
        cout << "req.picture_pose_reached = " << req.picture_pose_reached << endl;
    }
    cout << "Service Node END" << endl;

    return true;
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "get_highest_sauce_server");
    
    ros::NodeHandle nh;

    //Subscriber
    ros::Subscriber sub_yolov4 = nh.subscribe("/darknet_ros/bounding_boxes", 1, yolo_cb);                        //yolov4 detection results
    ros::Subscriber sub_depth_cloud = nh.subscribe("/camera/depth_registered/points", 1, organized_cloud_cb);    //Realsense organized point cloud
     
    //Publisher: Broadcast messages to visualize in Rviz
    top3_sauce_pub = nh.advertise<sensor_msgs::PointCloud2> ("/top3_sauce_pub", 1);
    top1_sauce_center_pub = nh.advertise<sensor_msgs::PointCloud2> ("/top1_sauce_center_pub", 1);

    //Service: When 'take pic pose' is reached, response 'hightest sauce' info to Client
    ros::ServiceServer service = nh.advertiseService("/get_highest_sauce", response_highest);

    ros::spin();

    return 0;    
}