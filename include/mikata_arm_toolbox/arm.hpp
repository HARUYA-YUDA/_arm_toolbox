#ifndef ARM_HPP
#define ARM_HPP

#include <cmath>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "/home/demulab/catkin_ws/devel/include/mikata_arm_msgs/dxl_double.h"
#include "mikata_arm_toolbox/kinematics.h"
//#include "../src/kinematics.cpp"

#include <stdlib.h>
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
//  void setChain();
//  Eigen::MatrixXd calcJacobian();

private:
  bool armMove;
  float pos[5];
  std::vector<double> targetPos;
  mikata_arm_msgs::dxl_double dexArm;
  Link chain[LINK_NUM_GRIPPER];
  const int X_AXIS = 1;
  const int Y_AXIS = 2;
  const int Z_AXIS = 3;
  const int IK_THRESHOLD = 1;    // [mm]
  const double IK_GAIN = 0.001;
  const double LIMIT_TIMES = 30000;
  const Matrix3d I = Matrix3d::Identity();

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
    Vector3d goal_pos; // Goal End Effector Position
    // Get goal position 
    goal_pos[0] = data.x;
    goal_pos[1] = data.y;
    goal_pos[2] = data.z;
    setChain();

    // Solve Inverse kinematic
    /* flow 
     ****************************
     * change goal_pos m to mm
     * Calculate relative coodinates from actual pose to goal pose and put into pos_diff
     * Calculate Jacobian and put into j  
     * Calculate Jacobian Pseudo Inverse 
     * Calculate inverse kinematics 
     * Check motors' angle limits
     * Calculate forward kinematics
     * Check the result pose is in threshold
     *
     * and return
     ****************************
    */
    VectorXd pos_diff(6);        // Difference of Actual Position and Goal Position
    pos_diff << 0,0,0,0,0,0;

    std::vector<double> q;       // Actual Joint State
    for(int i=0; i<LINK_NUM; i++) q.push_back(chain[i].getAngle());

    solveFK(q); 
  
    std::cout << "Calculating" << std::endl;

    int norm = 999;
    int count = 0;                 // Counter for executed times
    while( ((chain[LINK_NUM].getPos() - goal_pos).norm() > IK_THRESHOLD) && count < LIMIT_TIMES && ros::ok()) {
        
        pos_diff.head(3) = goal_pos - chain[LINK_NUM].getPos();

        MatrixXd J = calcJacobian();
        MatrixXd J_star = (J.transpose() * J).inverse() * J.transpose();
        Vector4d angle_diff;    // Joint State correction factor
  
        // Calculate by Jacobian Pseudo Inverse
        angle_diff= IK_GAIN * J_star * pos_diff;
  
        // Calculate by Jacobian Transpose
        //angle_diff= IK_GAIN * calcJacobian().transpose() * pos_diff;
        cout << setw(53) << "I have an idea";
        cout << endl << endl;
        int i = 0;
        while(i < LINK_NUM && ros::ok()){
          q[i] += angle_diff(i);
          // q%pi 
          double num = q[i];
          double a = fmod(num, M_PI);
          q[i] = a;
          
          while (q[i] >= M_PI && ros::ok()) {
              q[i]-=2*M_PI;
              cout << "?";
          }
          while (q[i] < -M_PI && ros::ok()) {
              q[i] += 2*M_PI;
              cout << "!";
          }
          if (q[i] > chain[i].getMaxAngle()) q[i] = chain[i].getMaxAngle();
          if (q[i] < chain[i].getMinAngle()) q[i] = chain[i].getMinAngle();
        cout << setw(3) << i+1;
        i ++;
        }

  
        solveFK(q);    // Set new Joint State and new End Effector Position
        if(count%1000==0) std::cout << "." << std::endl;;
        count++;
    }
    armPos(q);    // Set new Joint State and new End Effector Position
    std::cout << std::endl;
    if (count == LIMIT_TIMES) std::cout << "IK Fail." << std::endl; // Calculation fails to converge
    else armPos(q);

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

/*
void Arm::setChain()        // Set basic parameters for DYNAMIXEL Mikata Arm 4DOF
{
  chain[0].setChildPos(0.0, 0.0, 76.1);
  chain[1].setChildPos(24.0, 0.0, 148.0);
  chain[2].setChildPos(0.0, 0.0, 150.0);
  chain[3].setChildPos(0, 0.0, 150.0);
  
  chain[0].setChildAtt(I);
  chain[1].setChildAtt(Y_AXIS, M_PI/2);
  chain[2].setChildAtt(I);
  chain[3].setChildAtt(I);

  chain[0].setAxis(Z_AXIS);
  chain[1].setAxis(Y_AXIS);
  chain[2].setAxis(Y_AXIS);
  chain[3].setAxis(Y_AXIS);
		   
  chain[0].setMaxAngle(3.1241);
  chain[1].setMaxAngle(1.7976);
  chain[2].setMaxAngle(1.7976);
  chain[3].setMaxAngle(1.9198);

  chain[0].setMinAngle(-3.1416);
  chain[1].setMinAngle(-2.0595);
  chain[2].setMinAngle(-1.3963);
  chain[3].setMinAngle(-1.6406);
  
  chain[0].setPos(0.0, 0.0, 0.0);
}
*/

/*
Eigen::MatrixXd Arm::calcJacobian()        // Calculate base Jacobian
{
  MatrixXd J;

  for(int i=0; i<LINK_NUM; i++) {
    VectorXd col(6);
    Vector3d a = Vector3d::Zero();
    a(chain[i].getAxis()-1) = 1;
    col.tail(3) = a;
    col.head(3) = a.cross(chain[LINK_NUM].getPos() - chain[i].getPos());
    J.conservativeResize(6,J.cols() +1);
    J.col(J.cols()-1) = col;
  }

  return J;
}
*/
#endif // ARM_HPP
