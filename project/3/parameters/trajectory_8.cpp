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
double time_interval[] = {0, 7.322330470336312, 1.464466094067262e+01, 25};
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
        P_w.z( ) = 1;

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
  // px << -4.7690097e-08, 2.8238086e-06, -5.9681666e-05, 4.5513965e-04, 0, 0, 0, 0, 
  //        1.2606972e-08, -6.3312154e-07, 8.37431556e-06, 5.3618481e-05, -1.6263453e-03, -2.0812056e-03, 0.196075746, 1;
  // py << 3.2589549e-08, -1.67534707e-06, 2.843484e-05, -1.509884e-04, 0, 0, 0, 0,
  //      -1.99827e-08, 6.8698773e-07, -2.26969e-06, -1.0691498e-04, 1.462e-04, 1.1127e-02, 4.854e-02, 0;
  // pz << 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1;
  px << -6.005898158800704e-07, 2.527501224802720e-05, -3.768322288464354e-04, 1.987780688203489e-03, 0, 0, 0, 0,
       1.423566017288991e-07, -5.509005488235630e-06, 5.736743372242170e-05, 2.660439065701459e-04, -5.793963769683330e-03, -1.557233032717287e-02, 2.493349271990163e-01, 1,
       -3.206211540707216e-08, 1.787668942654364e-06, -2.437910992458825e-05, -1.081338205877191e-04, 3.823521033663346e-03, -6.664583312393280e-03, -2.104857244496423e-01, 1;

  py <<  4.094863462755782e-07, -1.507657580135380e-05, 1.825686563627915e-04, -6.892379356284728e-04, 0, 0, 0, 0,
      -2.169257555806325e-07, 5.912185781165036e-06, -1.874544906055320e-05, -5.036795995811971e-04, 5.197304919535295e-04, 2.592799269335744e-02, 7.946625882003444e-02, 0,
       2.350674702666922e-07, -5.206623667763921e-06, -3.246529252320940e-06, 5.841206301820945e-04, 3.127570778392874e-04, -3.923423973874241e-02, -4.991986998756914e-03, 1;

  pz << -4.907185768843192e-14, 1.200817223434569e-12, -9.801714995205657e-12, 2.662958742405408e-11, 0, 0, 0, 1,
      -1.421085471520200e-14, 3.480549182199866e-13, -2.422506639732092e-12, -1.660005466419534e-12, 3.119793312578167e-11, 1.407177707690721e-10, -2.696540768454270e-10, 1,
      -4.041211809635570e-14, 1.524558257415265e-12, -2.016542488547657e-11, 9.723277738515890e-11, -2.901012763345534e-12, -5.029205940587644e-10, -1.953613049110459e-09, 1;

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


