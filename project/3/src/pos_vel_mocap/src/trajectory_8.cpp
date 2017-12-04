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
ros::Publisher traj_pub;
ros::Publisher my_path_pub;
ros::Publisher mocap_path_pub;

// pass time interval T0, T1, T2, T3, ...
//double time_interval[] = {0, 10.355339059327, 25.00000};
//double time_interval[] = {0, 3.125, 6.249999999999999, 9.374999999999998, 12.5, 15.625, 18.75, 21.875, 25};t_vec =
double time_interval[] = {0, 8.637287570313159, 1.727457514062632e+01, 25};
int time_length = sizeof(time_interval)/sizeof(*time_interval);
int index_t = 0;
int my_path_index = 0;
double total_t, base_t, delta_t;

// pass px, py, pz
//Vector3d ypr(0, 0, 0);
VectorXd px(8*(time_length-1));
VectorXd py(8*(time_length-1));
VectorXd pz(8*(time_length-1));
VectorXd px_t(8);
VectorXd py_t(8);
VectorXd pz_t(8);
VectorXd pos_poly(8);
VectorXd vel_poly(8);

// global varibles init
bool init_ok = false;
bool trigger_ok = true;     // change this variable when switching locally and on-board
int seq_position_cmd = 0;
Eigen::Vector3d init_P, last_P;
Eigen::Vector3d P_w, V_w, A_w;
ros::Time now_t, init_t, last_pub_t;
nav_msgs::Path run_path, run_path_1;


void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
  if( trigger_ok == true)
  {
    if( init_ok == false )
    { 
        init_ok     = true;
        init_P.x( ) = msg->pose.pose.position.x;
        init_P.y( ) = msg->pose.pose.position.y;
        init_P.z( ) = msg->pose.pose.position.z;
        last_P      = init_P;
        init_t      = msg->header.stamp;
    }
    else
    {
        // calculate now_t belongs to which time interval
        now_t = msg->header.stamp;
        total_t = (now_t - init_t).toSec();
        index_t = 0;
        if(total_t >= 25){
          index_t = time_length-1-1;
          delta_t = time_interval[time_length-1]-time_interval[time_length-2];
        }
        else{
          for(int i=0; i<(time_length-1); i++){
            if(total_t >= time_interval[i] && total_t <= time_interval[i+1]){
              index_t = i;
              base_t = time_interval[i];
              break;
            }
          }
          delta_t = total_t - base_t;
        }

        px_t = px.segment(index_t*8, 8);
        py_t = py.segment(index_t*8, 8);
        pz_t = pz.segment(index_t*8, 8);

        pos_poly << pow(delta_t,7), pow(delta_t,6), pow(delta_t,5), pow(delta_t,4), pow(delta_t,3), pow(delta_t,2), delta_t, 1;
        vel_poly << 7*pow(delta_t,6), 6*pow(delta_t,5), 5*pow(delta_t,4), 4*pow(delta_t,3), 3*pow(delta_t,2), 2*delta_t, 1, 0;

        P_w.x( ) = init_P.x( ) + px_t.dot(pos_poly);
        P_w.y( ) = init_P.y( ) + py_t.dot(pos_poly);
        //P_w.z( ) = pz_t.dot(pos_poly);
        P_w.z( ) = init_P.z( );

        cout << "P_w.x( ) is:  " << P_w.x( ) << endl;
        cout << "P_w.y( ) is:  " << P_w.y( ) << endl;
        cout << "P_w.z( ) is:  " << P_w.z( ) << endl;
        cout << endl;

        V_w.x( ) = px_t.dot(vel_poly);
        V_w.y( ) = py_t.dot(vel_poly);
        V_w.z( ) = pz_t.dot(vel_poly);
        
        /*********************/
        quadrotor_msgs::PositionCommand pos_cmd;
        pos_cmd.header.stamp      = now_t;
        pos_cmd.header.seq        = seq_position_cmd;
        pos_cmd.header.frame_id   = msg->header.frame_id;

        pos_cmd.trajectory_id     = seq_position_cmd + 1;
        pos_cmd.trajectory_flag   = 1;
        pos_cmd.kx[0] = 3.7;  pos_cmd.kx[1] = 3.7;  pos_cmd.kx[2] = 5.2;
        pos_cmd.kv[0] = 2.4;  pos_cmd.kv[1] = 2.4;  pos_cmd.kv[2] = 3.0;

        pos_cmd.position.x        = P_w.x( );
        pos_cmd.position.y        = P_w.y( );
        pos_cmd.position.z        = P_w.z( );

        pos_cmd.velocity.x        = V_w.x( );
        pos_cmd.velocity.y        = V_w.y( );
        pos_cmd.velocity.z        = V_w.z( );
        traj_pub.publish(pos_cmd);

        // cout << "index is: "  << index_t << endl;
        // cout << "delta_t is " << delta_t << endl;
        // cout << "position_cmd x is: " << P_w.x( ) << endl;
        // cout << "position_cmd y is: " << P_w.y( ) << endl;
        // cout << "position_cmd z is: " << P_w.z( ) << endl;

        // visualize path
        ros::Duration delta_t = now_t - last_pub_t;
        if ( delta_t.toSec( ) > 0.1 )
        {
            // position_cmd path
            geometry_msgs::PoseStamped pose;
            pose.header.stamp       = now_t;
            pose.header.frame_id    = "world";
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            pose.pose.orientation.w = 1;
            pose.pose.position.x    = pos_cmd.position.x;
            pose.pose.position.y    = pos_cmd.position.y;
            pose.pose.position.z    = pos_cmd.position.z;

            run_path.header.stamp    = now_t;
            run_path.header.frame_id = "world";
            run_path.poses.push_back( pose );
            my_path_pub.publish( run_path );

            // mocap path
            /*
            geometry_msgs::PoseStamped pose1;
            pose1.header.stamp       = now_t;
            pose1.header.frame_id    = "world";
            pose1.pose.orientation.x = msg->pose.pose.orientation.x;
            pose1.pose.orientation.y = msg->pose.pose.orientation.y;
            pose1.pose.orientation.z = msg->pose.pose.orientation.z;
            pose1.pose.orientation.w = msg->pose.pose.orientation.w;
            pose1.pose.position.x    = msg->pose.pose.position.x;
            pose1.pose.position.y    = msg->pose.pose.position.y;
            pose1.pose.position.z    = msg->pose.pose.position.z;

            run_path_1.header.stamp    = now_t;
            run_path_1.header.frame_id = "world";
            run_path_1.poses.push_back( pose1 );
            mocap_path_pub.publish( run_path_1 );

            last_pub_t = now_t;
            */
        }
    }
  }
}


void trigger_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ROS_INFO("enter trigger callback function");
  seq_position_cmd = msg->header.seq;
  trigger_ok = true;
}


int main(int argc, char **argv)
{
px <<-7.625628128948847e-08, 4.489367317117576e-06, -9.232265019742947e-05, 6.553760287965060e-04, 0, 0, 0, 0,
     -3.004184723209846e-08, -1.211624205410544e-07, 2.086560364777945e-05, -2.771550840585668e-05, -3.231023190321869e-03, -3.730526451995564e-03, 1.932927938617108e-01, 0.8,
      2.520565404173780e-07, -1.937518466110788e-06, -3.247878229939438e-05, 6.028268437663353e-05, 3.964285332632489e-03, -5.849866613679478e-03, -2.073326276666427e-01, 0.8;

py << 3.695308424811472e-07, -1.636273846505887e-05, 2.466902990752029e-04, -1.255948537203677e-03, 0, 0, 0, 0, 
     -2.043593156919599e-07, 5.979468390870935e-06, -2.235948426365653e-05, -5.788996895861320e-04, 1.756865123119789e-03, 3.442566541201664e-02, -1.775828563000625e-02, -0.6,
      4.759191576386712e-07, -6.376303523181193e-06, -3.264224970522811e-05, 5.378726583423754e-04, 2.326672632284366e-03, -3.036601841283293e-02, -1.348674457613441e-02, 0.4;

pz << -5.218048215738236e-14, 1.590394482775537e-12, -1.633404522749515e-11, 5.601430430601795e-11, 0, 0, 0, 1,
      -2.198241588757810e-14, 7.662759315962830e-13, -9.117373522826711e-12, 3.318012531394743e-11, 6.479794478764234e-11, -1.576916375256587e-11, -3.085105282352174e-09, 1,
      -2.664535259100376e-15, 1.760813717055498e-13, -3.200106846179551e-12, 2.163902390606154e-11, -1.866462540078828e-11, -3.088984623644819e-10, 6.344340608421817e-10, 1;
  // node name is trajectory_node
  ros::init(argc, argv, "trajectory_node");
  ros::NodeHandle n;
  
  cout << "jixin trajectory starts" << endl;
  // publisher and subscriber
  traj_pub = n.advertise< quadrotor_msgs::PositionCommand >("/position_cmd", 100);
  my_path_pub = n.advertise< nav_msgs::Path >("/my_path_sim", 10);
  mocap_path_pub = n.advertise< nav_msgs::Path >("/mocap_path_sim", 10);
  ros::Subscriber odem_sub = n.subscribe("/pos_vel_mocap/odom_TA", 100, odom_callback);
  ros::Subscriber trig_sub = n.subscribe("/traj_start_trigger", 100, trigger_callback);
  
  ros::spin();
  return 0;
}



