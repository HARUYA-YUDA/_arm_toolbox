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
    
    cout << setw(53) << "------------------------------------";
    cout << endl << endl;
    ros::spin();
    return 0;
}

