#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <iostream>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "irb120_tf_calc_node");

  ros::NodeHandle node;

  ros::Publisher tf_pub = node.advertise<geometry_msgs:: Point>("irb120/transform", 10);

  tf::TransformListener listener;

  tf::Matrix3x3 transform_rot;

  ros::Rate rate(10.0);
   while (node.ok())
  {
    tf::StampedTransform transform;
    try{
      ros::Time now = ros::Time::now();
      listener.waitForTransform("/world", "/camera_link",
                              now, ros::Duration(10.0));
      listener.lookupTransform("/world", "/camera_link",  
                               ros::Time(0), transform);
      ROS_INFO("Got a transform! x = %f, y = %f",transform.getOrigin().x(),transform.getOrigin().y());
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    geometry_msgs:: Point pos_msg;

    pos_msg.x =  transform.getOrigin().x();
    pos_msg.y =  transform.getOrigin().y();
    pos_msg.z = transform.getOrigin().z();
    tf_pub.publish(pos_msg);

    rate.sleep();
  }
  return 0;
};