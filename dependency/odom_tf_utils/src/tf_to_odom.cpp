/*
* Copyright (c) 2016 Carnegie Mellon University, Author <sezal@andrew.cmu.edu, gdubey@andrew.cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/


#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_to_odom_converter");
  std::string source_frame_id, target_frame_id, odom_topic;
  int odom_freq;

  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");
  priv_node.param<std::string>("odom_topic", odom_topic, "odom");
  priv_node.param<std::string>("source_frame", source_frame_id, "source");
  priv_node.param<std::string>("target_frame", target_frame_id, "target");
  priv_node.param<int>("frequency", odom_freq, 10);

  ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>(odom_topic, 10);

  tf::TransformListener listener;
  ros::Duration(1.0).sleep();

  ros::Rate rate(odom_freq);
  while (node.ok()) {
    tf::StampedTransform transform;

    bool flag = true;
    while(flag) {
      flag = false;
      try {
        listener.lookupTransform(source_frame_id, target_frame_id,
                                 ros::Time(0), transform);
      }
      catch (tf::TransformException ex) {
        flag = true;
//         ROS_WARN_STREAM("tf_to_odom: cannot find transform from "<<source_frame_id<<" to "<<target_frame_id);
      }
    }

      nav_msgs::Odometry odom;
      // copy pose to odom msg
      odom.header.stamp = transform.stamp_;
      odom.header.frame_id = source_frame_id;
      odom.child_frame_id = target_frame_id;
      geometry_msgs::TransformStamped ts_msg;
      tf::transformStampedTFToMsg(transform, ts_msg);
      odom.pose.pose.position.x = ts_msg.transform.translation.x;
      odom.pose.pose.position.y = ts_msg.transform.translation.y;
      odom.pose.pose.position.z = ts_msg.transform.translation.z;
      odom.pose.pose.orientation = ts_msg.transform.rotation;

      odom_pub.publish(odom);

      rate.sleep();
    }

    return 0;
  }
