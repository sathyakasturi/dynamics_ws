#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <math.h>
#include <iomanip>
#include <Eigen/Dense>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Empty.h>
using namespace Eigen;
using namespace std;
class ForwardKinematics{
private:
  VectorXd theta;
  VectorXd d;
  VectorXd a;
  VectorXd alpha;
  std_msgs:: Float64 orient_msg;
  geometry_msgs::Point tf;
  geometry_msgs::Point position_msg;
  ros::Publisher pos_pub1;
  ros::Publisher pos_pub2;
  ros::Publisher pos_pub3;
  ros::Publisher pos_pub4;
  ros::Publisher pos_pub5;
  ros::Publisher pos_pub6;
  ros::Subscriber pos1_sub;
  ros::Subscriber pos2_sub;
  ros::Subscriber orientation_sub;
  ros::Subscriber tf_sub;
  ros::ServiceClient client1;
  ros::ServiceClient client2;
  double x_new;
  double y_new;
  
public:
  ForwardKinematics(VectorXd theta_, VectorXd d_, VectorXd a_, VectorXd alpha_,ros::NodeHandle n)
  {
    theta = theta_;
    d = d_;
    a = a_;
    alpha = alpha_;
    theta(0) = theta(0)* (M_PI/180); theta(1) = (theta(1)+90)* (M_PI/180); theta(2) = theta(2)* (M_PI/180);
    theta(3) = theta(3)* (M_PI/180); theta(4) = theta(4)* (M_PI/180); theta(5) = theta(5)* (M_PI/180);
    pos_pub1 = n.advertise<std_msgs::Float64>("/irb120/joint_1_position_controller/command",1000);
    pos_pub2 = n.advertise<std_msgs::Float64>("/irb120/joint_2_position_controller/command",1000);
    pos_pub3 = n.advertise<std_msgs::Float64>("/irb120/joint_3_position_controller/command",1000);
    pos_pub4 = n.advertise<std_msgs::Float64>("/irb120/joint_4_position_controller/command",1000);
    pos_pub5 = n.advertise<std_msgs::Float64>("/irb120/joint_5_position_controller/command",1000);
    pos_pub6 = n.advertise<std_msgs::Float64>("/irb120/joint_6_position_controller/command",1000);
    pos1_sub  = n.subscribe("/detect/bga_pickup/xy", 100, &ForwardKinematics::PickupCallBack,this);
    pos2_sub  = n.subscribe("/detect/bga_place/xy", 100, &ForwardKinematics::PlaceCallBack,this);
    orientation_sub  = n.subscribe("/detect/bga/orientation", 10, &ForwardKinematics::OrientCallBack,this);
    client1 = n.serviceClient<std_srvs::Empty>("/irb120/on");
    client2 = n.serviceClient<std_srvs::Empty>("/irb120/off");
    tf_sub = n.subscribe("irb120/transform", 2, &ForwardKinematics::TFCB,this);
  }

  MatrixXd MatrixTransformation(double theta, double d, double a, double alpha);
  MatrixXd getR03(VectorXd theta_);
  MatrixXd getHomogeneous(MatrixXd theta, VectorXd d, VectorXd a, VectorXd alpha);
  MatrixXd InverseKinematics(MatrixXd H);
  void moveRobot(MatrixXd theta);
  MatrixXd Trajectory(MatrixXd i_pos, MatrixXd f_pos, double theta_deg);
  MatrixXd QuadGen(double theta_i, double theta_f, double ang_v, double t_start, double t_final);
  MatrixXd CubGen(double theta_i, double theta_f, double ang_vi, double ang_vf, double t_start, double t_final);
  MatrixXd Polynome(MatrixXd q, MatrixXd t, double v_i, double a_i, double v_f, double a_f);
  void PickupCallBack(const geometry_msgs::Point pos_msg);
  void PlaceCallBack(const geometry_msgs::Point pos_msg);
  void OrientCallBack(const std_msgs::Float64 msg);
  void TFCB(const geometry_msgs::Point pos); 
  MatrixXd FinalPos(const geometry_msgs::Point pos);
};
