#include "ros/ros.h"

#include "client/HighestSauce.h"

#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_highest_sauce_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<client::HighestSauce>("/get_highest_sauce");
    client::HighestSauce srv;
    
    srv.request.picture_pose_reached = true;
    //srv.request.picture_pose_reached = false;

    //_sleep(10*1000)；//延时5秒 
    while(ros::ok())
    {
        if (client.call(srv))
        {    
            cout<< "[CLIENT] receiving response (highest_sauce)" << srv.response.sauce_class <<endl;
            cout<< "multiple_sauce_type : "<< int(srv.response.multiple_sauce_type) <<endl;
        }
        else
        {
            ROS_ERROR("Failed to call service pick_highest");
            return 1;
        }
    }

    ros::spin();

    return 0;
}
