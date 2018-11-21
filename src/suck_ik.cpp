#include "mikata_arm_toolbox/cli_core.h"
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include "mikata_arm_toolbox/kinematics.h"
#include "mikata_arm_toolbox/teach.h"
#include "mikata_arm_toolbox/basic_util.h"
#include <type_traits>
#include <termios.h>
#include <fcntl.h>
#include <cstdlib>
#include <time.h>
#include "mikata_arm_toolbox/arm.hpp"
using namespace std;


void print_table();

std::vector<double> q;
std::vector<double> link_x_pos;
std::vector<double> link_y_pos;
std::vector<double> link_z_pos;
std::vector<int> link_vel;
std::vector<int> link_acel;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Suck_ik");
    ros::NodeHandle nh;
    Arm arm(&nh);
    
    //basic setup
    //Opening port, Changing port baudrate
    dxl_setup(); //dxl_util.cpp
    //Checking connection with Dynamixels
    pingAll(); //dxl_util.cpp
  
    //If connection with Dynamixels has problem, it say error
    if (!dxl_read(GRIPPER_ID, ADDR_X_DRIVE_MODE, sizeof(int8_t)))
        throw std::runtime_error("Please run dxl_setup before execution.");
  
    //Set basic parameters for DYNAMIXEL Mikata Arm 4DOF
    setChain(); //kinematic.cpp
    //Turn fixed all motors
    enableAll(); //basic_util.h
    //It might set velosity
    setVelAll(20); //basic_util.h
    //It might set acceleration
    setAcelAll(2, false); //basic_util.h
    //Configuration of Gripper joint.
    dxl_write(GRIPPER_ID, 30, ADDR_X_PROFILE_VEL, sizeof(int32_t)); //dxl_util.cpp
    dxl_write(GRIPPER_ID, 250, ADDR_X_P_GAIN, sizeof(int16_t)); //dxl_util.cpp
  
    //Clear Parameters
    clear();
    q.clear();
    link_x_pos.clear();
    link_y_pos.clear();
    link_z_pos.clear();
    link_vel.clear();
    link_acel.clear();
    
    //Set Parameters
    //read motors' angle
    q = readAll(true); //basic_util.h
    //read motors' velosity
    link_vel = getVelAll(); //basic_util.h
    //read motors' aceleration
    link_acel = dxl_readAll(ADDR_X_PROFILE_ACEL, sizeof(int32_t)); //
    solveFK(q);
    //make motors' angle table
    for(int i=0; i<=LINK_NUM; i++) {
        int id = i+1;
        Vector3d val = chain[i].getPos();
        link_x_pos.push_back(val[0]);
        link_y_pos.push_back(val[1]);
        link_z_pos.push_back(val[2]);
    }
    print_table();
    ros::spin();
    return 0;
}

void print_table() {

  cout << setw(53) << "------------------------------------";
  cout << endl << endl;
  // ID
  cout << setw(13) << "ID:"
       << setw(10) << "1"
       << setw(10) << "2"
       << setw(10) << "3"
       << setw(10) << "4"
       << setw(10) << "5";
  cout << endl << endl;

  // Angle
  cout << fixed << setprecision(2);
  cout << setw(13) << "Angle [deg]:"
       << setw(12) << rad2deg(q[0])
       << setw(10) << rad2deg(q[1])
       << setw(10) << rad2deg(q[2])
       << setw(10) << rad2deg(q[3])
       << setw(10) << rad2deg(q[4]);  
  cout << endl << endl << defaultfloat;

  // Position
  cout << fixed << setprecision(3);
  cout << setw(13) << "Pos [mm]:" << " (x)"
       << defaultfloat
       << setw(6) << link_x_pos[0]
       << setw(10) << link_x_pos[1]
       << fixed
       << setw(12) << link_x_pos[2]
       << setw(10) << link_x_pos[3]
       << setw(10) << link_x_pos[4];
  cout << endl;
  cout << setw(13) << "" << " (y)"
       << defaultfloat
       << setw(6) << link_y_pos[0]
       << setw(10) << link_y_pos[1]
       << fixed
       << setw(12) << link_y_pos[2]
       << setw(10) << link_y_pos[3]
       << setw(10) << link_y_pos[4];
  cout << endl;
  cout << setw(13) << "" << " (z)"
       << defaultfloat
       << setw(6) << link_z_pos[0]
       << fixed
       << setw(12) << link_z_pos[1]
       << setw(10) << link_z_pos[2]
       << setw(10) << link_z_pos[3]
       << setw(10) << link_z_pos[4];
  cout << endl << endl << defaultfloat;

  // Velocity
  cout << setw(13) << "Vel :"
       << setw(11) << link_vel[0]
       << setw(10) << link_vel[1]
       << setw(10) << link_vel[2]
       << setw(10) << link_vel[3]
       << setw(10) << link_vel[4];
  cout << endl;

  cout << setw(13) << "Accel :"
       << setw(11) << link_acel[0]
       << setw(10) << link_acel[1]
       << setw(10) << link_acel[2]
       << setw(10) << link_acel[3]
       << setw(10) << link_acel[4];
  cout << endl << endl;

  cout << setw(53) << "------------------------------------";
  cout << endl;

}
