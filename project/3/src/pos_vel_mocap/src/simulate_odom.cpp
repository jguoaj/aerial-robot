#include "ros/ros.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "pose_utils.h"

#include <iostream>
#include <Eigen/Eigen>
#include <cmath>
using namespace std;
using namespace Eigen;
ros::Publisher my_odom_pub;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "sim_odom");
  ros::NodeHandle n;
  
  cout << "jixin simulate odom starts" << endl;
  my_odom_pub = n.advertise< nav_msgs::Odometry >("/pos_vel_mocap/odom_TA", 100);

  // manually publish topic /pos_vel_mocap/odom_TA
  ros::Rate loop_rate(10);
  int my_cnt = 0;
  while (ros::ok()){
    nav_msgs::Odometry sim_odom_path;
    sim_odom_path.header.seq = my_cnt;
    sim_odom_path.header.stamp = ros::Time::now();
    sim_odom_path.header.frame_id = "world";

    sim_odom_path.pose.pose.position.x = 0.0 + my_cnt*0.001;
    sim_odom_path.pose.pose.position.y = 0.0 + my_cnt*0.001;
    sim_odom_path.pose.pose.position.z = 0.0;

    sim_odom_path.pose.pose.orientation.x = 0.0;
    sim_odom_path.pose.pose.orientation.y = 0.0;
    sim_odom_path.pose.pose.orientation.z = 0.0;
    sim_odom_path.pose.pose.orientation.w = 1.0;

    my_odom_pub.publish(sim_odom_path);

    loop_rate.sleep();
    my_cnt++;
  }
  
  ros::spin();
  return 0;
}

