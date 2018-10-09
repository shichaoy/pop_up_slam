/*
* Copyright (c) 2016 Carnegie Mellon University, Author <sezal@andrew.cmu.edu, gdubey@andrew.cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
  static tf::TransformBroadcaster br;
  static tf::Transform odomTF;

  odomTF.setOrigin(tf::Vector3(odom_msg->pose.pose.position.x,
                               odom_msg->pose.pose.position.y,
                               odom_msg->pose.pose.position.z) );

  odomTF.setRotation( tf::Quaternion(odom_msg->pose.pose.orientation.x,
                                     odom_msg->pose.pose.orientation.y,
                                     odom_msg->pose.pose.orientation.z,
                                     odom_msg->pose.pose.orientation.w));

  br.sendTransform(tf::StampedTransform(odomTF,
                                        odom_msg->header.stamp,
                                        "/ned_origin",
                                        odom_msg->child_frame_id));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("odom", 100, odom_callback);
  ros::spin();

  return 0;
}
