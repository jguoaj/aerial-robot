#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include "pose.h"
#define pi 3.1415926

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
MatrixXd Q = MatrixXd::Identity(12, 12);
MatrixXd Rt = MatrixXd::Identity(6,6);
MatrixXd x = MatrixXd::Zero(15,1);
MatrixXd my_cov = MatrixXd::Identity(15,15);
// double imu_mean = 0.0;
// double cam_mean = 0.0;
double theta0, theta1, theta2, lin_a0, lin_a1, lin_a2, ang_v0, ang_v1, ang_v2;
double cur_t = 0.0;
bool cam_ready = 0;

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    //your code for propagation
    double dt = msg->header.stamp.toSec() - cur_t;
    if(!cam_ready)
        dt = 0;

    // process model, Gyroscope and Accelerometer
    Vector3d ang_v(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    Vector3d lin_a(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    Vector3d x3(x(6,0), x(7,0), x(8,0));
    Vector3d x4(x(9,0), x(10,0), x(11,0));
    Vector3d x5(x(12,0), x(13,0), x(14,0));

    MatrixXd f = MatrixXd::Zero(15,1);
    MatrixXd Ft = MatrixXd::Zero(15,15);
    MatrixXd At = MatrixXd::Zero(15,15);
    MatrixXd Vt = MatrixXd::Zero(15,12);
    MatrixXd Ut = MatrixXd::Zero(15,12);
    Matrix3d G, R, G_inv_dot, R_dot;
    Vector3d g(0.0, 0.0, 9.81);

    // update G and R
    G <<    cos(x(4,0)), 0, -cos(x(3,0))*sin(x(4,0)),
            0, 1, sin(x(3,0)),
            sin(x(4,0)), 0, cos(x(3,0))*cos(x(4,0));
    R <<    cos(x(5,0))*cos(x(4,0))-sin(x(3,0))*sin(x(4,0))*sin(x(5,0)), -cos(x(3,0))*sin(x(5,0)), cos(x(5,0))*sin(x(4,0))+cos(x(4,0))*sin(x(3,0))*sin(x(5,0)),
            cos(x(4,0))*sin(x(5,0))+cos(x(5,0))*sin(x(4,0))*sin(x(3,0)),  cos(x(3,0))*cos(x(5,0)), sin(x(5,0))*sin(x(4,0))-cos(x(5,0))*cos(x(4,0))*sin(x(3,0)),
            -cos(x(3,0))*sin(x(4,0)), sin(x(3,0)), cos(x(3,0))*cos(x(4,0));

    // find close-form solution for At, Ut
    Ut.block<3,3>(3,0) = -G.inverse();
    Ut.block<3,3>(6,3) = -R;
    Ut.block<6,6>(9,6) = MatrixXd::Identity(6,6);

    theta0 = x(3,0); theta1 = x(4,0); theta2 = x(5,0);
    ang_v0 = ang_v(0); ang_v1 = ang_v(1); ang_v2 = ang_v(2); 
    lin_a0 = lin_a(0); lin_a1 = lin_a(1); lin_a2 = lin_a(2);

    G_inv_dot << 
        0, ang_v2*cos(theta1) - ang_v0*sin(theta1), 0,
        ang_v0*sin(theta1) - ang_v2*cos(theta1) - (ang_v2*cos(theta1)*sin(theta0)*sin(theta0))/(cos(theta0)*cos(theta0)) + (ang_v0*sin(theta0)*sin(theta0)*sin(theta1))/(cos(theta0)*cos(theta0)), (ang_v0*cos(theta1)*sin(theta0))/cos(theta0) + (ang_v2*sin(theta0)*sin(theta1))/cos(theta0), 0,
        (ang_v2*cos(theta1)*sin(theta0))/(cos(theta0)*cos(theta0)) - (ang_v0*sin(theta0)*sin(theta1))/(cos(theta0)*cos(theta0)), - (ang_v0*cos(theta1))/cos(theta0) - (ang_v2*sin(theta1))/cos(theta0), 0;

    R_dot <<
        lin_a1*sin(theta0)*sin(theta2) + lin_a2*cos(theta0)*cos(theta1)*sin(theta2) - lin_a0*cos(theta0)*sin(theta1)*sin(theta2), lin_a2*(cos(theta1)*cos(theta2) - sin(theta0)*sin(theta1)*sin(theta2)) - lin_a0*(cos(theta2)*sin(theta1) + cos(theta1)*sin(theta0)*sin(theta2)), - lin_a0*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta0)*sin(theta1)) - lin_a2*(sin(theta1)*sin(theta2) - cos(theta1)*cos(theta2)*sin(theta0)) - lin_a1*cos(theta0)*cos(theta2),
        lin_a0*cos(theta0)*cos(theta2)*sin(theta1) - lin_a2*cos(theta0)*cos(theta1)*cos(theta2) - lin_a1*cos(theta2)*sin(theta0), lin_a2*(cos(theta1)*sin(theta2) + cos(theta2)*sin(theta0)*sin(theta1)) - lin_a0*(sin(theta1)*sin(theta2) - cos(theta1)*cos(theta2)*sin(theta0)),   lin_a0*(cos(theta1)*cos(theta2) - sin(theta0)*sin(theta1)*sin(theta2)) + lin_a2*(cos(theta2)*sin(theta1) + cos(theta1)*sin(theta0)*sin(theta2)) - lin_a1*cos(theta0)*sin(theta2),
        lin_a1*cos(theta0) - lin_a2*cos(theta1)*sin(theta0) + lin_a0*sin(theta0)*sin(theta1), - lin_a0*cos(theta0)*cos(theta1) - lin_a2*cos(theta0)*sin(theta1), 0;

    At.block<3,3>(0,6) = MatrixXd::Identity(3,3);
    At.block<3,3>(3,3) = G_inv_dot;
    At.block<3,3>(6,3) = R_dot;
    At.block<3,3>(3,9) = -G.inverse();
    At.block<3,3>(6,12) = -R;

    // update Ft and Vt
    Ft = MatrixXd::Identity(15,15) + dt*At;
    Vt = dt*Ut;

    // update process model
    f.block<3,1>(0,0) = x3;
    f.block<3,1>(3,0) = G.inverse()*(ang_v - x4);
    f.block<3,1>(6,0) = g + R*(lin_a - x5);

    x += dt*f;                                                // update the state
    my_cov = Ft*my_cov*Ft.transpose() + Vt*Q*Vt.transpose();  // update the cov

    // cache timestamp
    cur_t = msg->header.stamp.toSec();

    // publish the information
    Vector3d x_rpy;
    x_rpy << x(3,0), x(4,0), x(5,0);

    cout << "value x is: " << x << endl;

    Quaterniond Q_yourwork;
    Q_yourwork = rpy_to_R(x_rpy);
    nav_msgs::Odometry ekf_odom;
    ekf_odom.header.stamp = msg->header.stamp;
    ekf_odom.header.frame_id = "world";
    ekf_odom.pose.pose.position.x = x(0,0);
    ekf_odom.pose.pose.position.y = x(1,0);
    ekf_odom.pose.pose.position.z = x(2,0);
    ekf_odom.pose.pose.orientation.w = Q_yourwork.w();
    ekf_odom.pose.pose.orientation.x = Q_yourwork.x();
    ekf_odom.pose.pose.orientation.y = Q_yourwork.y();
    ekf_odom.pose.pose.orientation.z = Q_yourwork.z();
    ekf_odom.twist.twist.linear.x = x(6,0);
    ekf_odom.twist.twist.linear.y = x(7,0);
    ekf_odom.twist.twist.linear.z = x(8,0);
    odom_pub.publish(ekf_odom);

}

//Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    //your code for update
    //camera position in the IMU frame = (0.05, 0.05, 0)
    //camera orientaion in the IMU frame = Quaternion(0, 1, 0, 0); w x y z, respectively
    //					   RotationMatrix << 1,  0,  0,
    //							             0, -1,  0,
    //                                       0,  0, -1;
    Quaterniond quat;
    Vector3d Tcw, Twi, Tic, rpy_wi;
    Matrix3d Riw, Rwi, Rcw;
    quat.w() = msg->pose.pose.orientation.w; 
    quat.x() = msg->pose.pose.orientation.x; 
    quat.y() = msg->pose.pose.orientation.y; 
    quat.z() = msg->pose.pose.orientation.z;
    Tcw(0) = msg->pose.pose.position.x;
    Tcw(1) = msg->pose.pose.position.y; 
    Tcw(2) = msg->pose.pose.position.z;

    Tic << 0.05, 0.05, 0;
    Rcw = quat.toRotationMatrix();                  // from world to camera, world in camera
    Rwi = Rcw.transpose() * Rcam.transpose();       // Rwi = Riw^-1 = (Ric*Rcw)^-1, from imu to world
    Twi = -Rcw.transpose() * (Tcw + Rcam.transpose()*Tic);
    rpy_wi = R_to_rpy(Rwi);

    // get zt, measurement model
    VectorXd zt = VectorXd::Zero(6);
    VectorXd zt_g = VectorXd::Zero(6);
    zt << Twi, rpy_wi;
    zt_g << zt(0,0)-x(0,0), zt(1,0)-x(1,0), zt(2,0)-x(2,0), zt(3,0)-x(3,0), zt(4,0)-x(4,0), zt(5,0)-x(5,0);

    // Be careful about the discontinuous nature in Euler angles 
    // when you obtain the Roll, Pitch and Yaw angles from rotation matrix.
    if(zt_g(3,0) > pi)
        zt_g(3,0) -= 2*pi;
    else if(zt_g(3,0) < -pi)
        zt_g(3,0) += 2*pi;

    if(zt_g(4,0) > pi)
        zt_g(4,0) -= 2*pi;
    else if(zt_g(4,0) < -pi)
        zt_g(4,0) += 2*pi;

    if(zt_g(5,0) > pi)
        zt_g(5,0) -= 2*pi;
    else if(zt_g(5,0) < -pi)
        zt_g(5,0) += 2*pi;
    // cout << "the value for zt_g is: " << zt_g << endl;

    MatrixXd Ct(6, 15);
    MatrixXd Kt(15, 6);
    MatrixXd Wt = MatrixXd::Identity(6,6);
    Ct <<      1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,
               0  ,  1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,
               0  ,  0  ,  1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,
               0  ,  0  ,  0  ,  1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,
               0  ,  0  ,  0  ,  0  ,  1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,
               0  ,  0  ,  0  ,  0  ,  0  ,  1  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ;

    Kt = my_cov*Ct.transpose()*( Ct*my_cov*Ct.transpose()+Wt*Rt*Wt.transpose() ).inverse();

    x += Kt*zt_g;             // state update
    my_cov -= Kt*Ct*my_cov;   // cov update

    // need camera odom as initial value
    cam_ready = 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 1000, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);

    Rcam = Quaterniond(0, 1, 0, 0).toRotationMatrix();
    cout << "R_cam" << endl << Rcam << endl;
    
    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    Q.topLeftCorner(6, 6)      = 0.01 * Q.topLeftCorner(6, 6);
    Q.bottomRightCorner(6, 6)  = 0.01 * Q.bottomRightCorner(6, 6);
    Rt.topLeftCorner(3, 3)     = 0.1 * Rt.topLeftCorner(3, 3);
    Rt.bottomRightCorner(3, 3) = 0.1 * Rt.bottomRightCorner(3, 3);
    Rt.bottomRightCorner(1, 1) = 0.1 * Rt.bottomRightCorner(1, 1);

    ros::spin();
}
