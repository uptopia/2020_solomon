#include <ros/ros.h>
#include "pcl_utils/snapshot.h"

using namespace std;

int main (int argc, char** argv)
{
  ros::init(argc, argv, "get_pointcloud_client");
  ros::NodeHandle nh;
  ros::ServiceClient savePointCloud_client = nh.serviceClient<pcl_utils::snapshot>("snapshot");
  ros::ServiceClient saveColorRegion_client = nh.serviceClient<pcl_utils::snapshot>("AlignPointCloud");

  pcl_utils::snapshot snapshot_srv, saveColorRegion_srv;

  // snapshot_srv.request.call = 1;
  // savePointCloud_client.call(snapshot_srv);
  
  saveColorRegion_srv.request.call = 1;
  saveColorRegion_client.call(saveColorRegion_srv);
  std::string aa = (saveColorRegion_srv.response.doit) ? "a" : "b";
  std::cout<<"========================="<<std::endl;
  std::cout<<saveColorRegion_srv.response.type<<std::endl;
  std::cout<<aa<<std::endl;
  // std::cout<<saveColorRegion_srv.response.trans<<std::endl;

  return 0;
}