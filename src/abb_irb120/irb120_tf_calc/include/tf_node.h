#ifndef TF_NODE_H
#define TF_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <iostream>

class tf_node
{
public:

	ros::Publisher tf_pub;
  	ros::Publisher tf_pub1;
	ros::Subscriber tf_sub;
	ros::Subscriber tf_sub1;
	tf::TransformListener listener;
	tf::StampedTransform transform;
	tf::StampedTransform transform1;
	tf::StampedTransform transform2;
	tf::StampedTransform transform3;
	void tf_pick_broadcaster(const geometry_msgs::Point pos_msg); 
	void tf_place_broadcaster(const geometry_msgs::Point pos_msg);
	// void Pickup_Publish();
	// void Place_Publish();
	tf_node(ros::NodeHandle n);
};

#endif
