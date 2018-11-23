#ifndef ARM_HPP
#define ARM_HPP

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "/home/demulab/catkin_ws/devel/include/mikata_arm_msgs/dxl_double.h"

#include <vector>
#include <sstream>
#include <iostream>

using namespace std;

class Arm
{
public:
  Arm(ros::NodeHandle *n);
  void cycle();
  void armPos(std::vector<double> value);
  float *armPos();
  bool moveCheck();

private:
  bool armMove;
  float pos[5];
  std::vector<double> targetPos;
  mikata_arm_msgs::dxl_double dexArm;

  ros::Subscriber armsub;
  ros::Subscriber ik_sub;
  ros::Publisher pub;
  void armCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void ik_Callback(const geometry_msgs::Point& data);
};

Arm::Arm(ros::NodeHandle *n):
armsub(n->subscribe("dxl/joint_state", 1000, &Arm::armCallback,this)),
ik_sub(n->subscribe("goal_pose", 100, &Arm::ik_Callback, this)),
pub(n->advertise<mikata_arm_msgs::dxl_double>("dxl/goal_position", 1000))
{
  armMove = false;
  for(int i = 0;i <5;i++){
    pos[i] = 0.0;
  }
  std::vector<int32_t> id_init = {1,2,3,4,5};
  std::vector<double> data_init = {0.0,0.0,0.0,0.0,0.0};
  dexArm.id = id_init;
  dexArm.data = data_init;
  pub.publish(dexArm);
}

void Arm::armCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  pos[0] = msg->position[0];
  pos[1] = msg->position[1];
  pos[2] = msg->position[2];
  pos[3] = msg->position[3];
  pos[4] = msg->position[4];
}

void Arm::ik_Callback(const geometry_msgs::Point& data)
{
    std::vector<double> cur_q = readAll();     // Present Joint State
    std::vector<double> q;                     // Goal Joint State
    Vector3d goal;                             // Goal End Effector Position

    int time = 5; // robotic arm move for 5 seconds
  
    get_fk();
    q.resize(LINK_NUM);
  
    for(int i=0; i<LINK_NUM; i++)
        chain[i].setAngle(cur_q[i]);
  
    // Get goal position 
    goal[0] = data.x;
    goal[1] = data.y;
    goal[2] = data.z;

  
    // Solve Inverse Kinematics
    if (solve_suckIK(goal)) {
        for(int i=0; i<LINK_NUM; i++) q[i]=chain[i].getAngle();
        std::cout << std::setw(28) << "Current joint angle [deg]: ";
        print_rad2deg(cur_q);
        std::cout << std::setw(28) << "Target joint state [deg]: ";
        print_rad2deg(q);
  
//        armPos(q);
        writeAll(q, time);
    }
    else {
        std::cout << "IK Fail." << std::endl;
    }
}

void Arm::armPos(std::vector<double> value)
{
  targetPos = value;
  armMove = true;
  dexArm.data = targetPos;
  pub.publish(dexArm);
}

float *Arm::armPos(){return pos;}
bool Arm::moveCheck(){return armMove;}

void Arm::cycle(){
  if(armMove){
    bool flag = true;
    for(int i = 0;i<5;i++){
      if(fabs(targetPos[i] - pos[i]) >= 0.1)flag = false;
      //printf("[%f:]",fabs(targetPos[i] - pos[i]));
    }
    //printf("\n");
    if(flag)armMove = false;
  }
}

#endif // ARM_HPP
