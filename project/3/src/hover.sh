#!/bin/bash

source /home/nvidia/ELEC5660_lab_code/src/app/demo_application/bashrc.sh;

roslaunch mocap_optitrack mocap.launch & sleep 1;

rosrun pos_vel_mocap pos_vel_mocap & sleep 1;
rosrun pos_vel_mocap simulate_odom & sleep 1;
rosrun pos_vel_mocap trajectory_8 & sleep 1;

#roslaunch djiros djiros.launch & sleep 3; 
roslaunch machine_defined ctrl_md.launch   
