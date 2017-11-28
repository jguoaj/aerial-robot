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
double time_interval[] = {0, 3.125, 6.249999999999999, 9.374999999999998, 12.5, 15.625, 18.75, 21.875, 25};
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
bool trigger_ok = false;     // change this variable when switching locally and on-board
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
  px << -1.060508383510239e-04, 1.898739783498860e-03, -1.210010272799145e-02, 2.774973507133871e-02, 0, 0, 0, 0,
        2.662660610686096e-05, -4.211223054124691e-04, 1.752561129859442e-03, 3.546824358822587e-03, -2.986303768996224e-02, -1.424608383581538e-02, 3.214787639667750e-01, 0.5,
       -9.816056296241271e-06, 1.613347036937274e-04, -6.829476442993254e-04, -2.316962068810158e-03, 1.746404384102540e-02, 1.267088633272528e-02, 4.675563710898345e-02, 1,
        3.277148595381974e-06, -5.339152644245893e-05, 3.290196291818592e-04, 1.603149608038201e-04, -1.248582214662686e-02, 1.575197501186842e-03, 2.535990157449196e-01, 1.5,
       -3.277148559632792e-06, 1.829609864023585e-05, 4.238387418808998e-12, 9.805946225865947e-04, -4.107825191113079e-14, -6.154430504911157e-02, 3.729017095110976e-12, 2, 
        9.816056213529656e-06, -5.339152687811044e-05, -3.290196332637052e-04, 1.603149584356034e-04, 1.248582214610283e-02, 1.575197501120562e-03, -2.535990157490600e-01, 1.5,
       -2.662660620833535e-05, 1.613347043342150e-04, 6.829476364705878e-04, -2.316962064745409e-03, -1.746404384155320e-02, 1.267088633028235e-02, -4.675563711544206e-02, 1, 
        1.060508383741166e-04, -4.211223056953539e-04, -1.752561127821073e-03, 3.546824357244849e-03, 2.986303769019871e-02, -1.424608383543724e-02, -3.214787639668113e-01, 0.5;

  py << 1.521939879495227e-04, -2.678366949441680e-03, 1.650204073874173e-02, -3.530042198533190e-02, 0, 0, 0, 0,
      -4.198079660189258e-05, 6.508765377680748e-04, -2.505681875035304e-03, -7.234226939240251e-03, 4.352892950162157e-02, 8.870862138291291e-02, -2.374487738189215e-01, -0.5, 
       1.646293517809649e-05, -2.674533873914253e-04, 1.088910152574063e-03, 4.117408201947725e-03, -3.445691728218225e-02, -2.341151643612616e-02, 4.045294482541035e-01, 0,
      -4.236494610898589e-06, 9.267331983686677e-05, -5.496529797721639e-04, -4.618010916066506e-04, 1.505994902149677e-02, -5.244378484538004e-02, -1.003442016322829e-01, 0.5,
      -4.236494695497584e-06, 6.834532939592464e-13, 3.191593924650737e-04, 3.567035555818165e-12, -1.967134221662725e-03, -1.156630347054488e-12, -1.672815735717268e-01, 0, 
       1.646293523238640e-05, -9.267332018780827e-05, -5.496529807558215e-04, 4.618010891199731e-04, 1.505994902144192e-02, 5.244378484724988e-02, -1.003442016341603e-01, -0.5,
      -4.198079664341492e-05, 2.674533883729735e-04, 1.088910153065781e-03, -4.117408195861927e-03, -3.445691728145706e-02, 2.341151643250738e-02, 4.045294482542393e-01, 0, 
       1.521939879505219e-04, -6.508765369512837e-04, -2.505681878458788e-03, 7.234226943629740e-03, 4.352892950096809e-02, -8.870862138247571e-02, -2.374487738177555e-01, 0.5;

  pz << 4.785061236134425e-14, -5.264677582772492e-13, 1.947220162890062e-12, -2.408739874226740e-12, 0, 0, 0, 1,
       3.486100297322992e-14, -3.641531520770514e-13, 1.225575196883710e-12, -9.885425811262394e-13, -1.435962460050178e-12, -5.297984273511247e-13, 4.935385433668671e-12, 1,
      -1.976196983832779e-14, 2.156053113822054e-13, -7.989164885202627e-13, 1.008970684779342e-12, 8.637535131583718e-14, -4.025668687290818e-13, -1.190159082398168e-13, 1,
      -2.930988785010413e-14, 3.193001418821950e-13, -1.187272502534142e-12, 1.519784298409377e-12, -6.905587213168474e-14, 5.603295605283165e-13, -1.999511667349907e-12, 1,
      -3.552713678800501e-14, 4.086730953645201e-13, -1.651345726827458e-12, 2.432609669256181e-12, -4.152234112098086e-14, -8.240075288767912e-13, -2.452260616792046e-12, 1,
       1.232347557333924e-14, -1.172395514004165e-13, 3.487210520347617e-13, -2.484679129111100e-13, -1.019184736605894e-13, -4.314326673693358e-13, 1.110223024625157e-15, 1,
      -4.596323321948148e-14, 5.155875726359227e-13, -2.001510068794232e-12, 2.778111074519529e-12, 5.107025913275720e-14, -1.598721155460225e-12, -2.653433028854124e-13, 1,
      -1.487698852997710e-14, 1.589839371263224e-13, -5.631051180898794e-13, 5.569988914544410e-13, 5.170308625679354e-13, 2.386979502944087e-14, -2.067679361061892e-12, 1;


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

