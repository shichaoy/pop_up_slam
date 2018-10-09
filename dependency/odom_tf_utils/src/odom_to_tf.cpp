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
#include "iostream"

bool should_inverse;
bool stabilized;
bool footprint;

void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
  static tf::TransformBroadcaster br;
  static tf::Transform odomTF;
//   ROS_ERROR_STREAM("receive image");
  odomTF.setOrigin(tf::Vector3(odom_msg->pose.pose.position.x,
                               odom_msg->pose.pose.position.y,
                               odom_msg->pose.pose.position.z) );

  std::string child_frame_id = odom_msg->child_frame_id;

  if(stabilized)  // only use yaw, don't use roll, pitch
  {
      odomTF.setRotation(
                  tf::createQuaternionFromYaw(
                      tf::getYaw(odom_msg->pose.pose.orientation)));
      child_frame_id = child_frame_id+"_stabilized";
  }
  else
  {
//       std::cout<<"not using stabilized version"<<std::endl;
      odomTF.setRotation(tf::Quaternion(odom_msg->pose.pose.orientation.x,
                                        odom_msg->pose.pose.orientation.y,
                                        odom_msg->pose.pose.orientation.z,
                                        odom_msg->pose.pose.orientation.w));
  }
  
  if (footprint)  // desired footprint frame, parallel to ground, yaw is same, depends to use roll, pitch
  {
//       std::cout<<"using roll and quaternion"<<std::endl;
      tf::Quaternion bt_q;
      tf::quaternionMsgToTF(odom_msg->pose.pose.orientation,bt_q);
      tfScalar roll, pitch, yaw;
      tf::Matrix3x3(bt_q).getRPY(roll, pitch, yaw);
      odomTF.setRotation(tf::createQuaternionFromRPY(roll,0,yaw));
      child_frame_id = child_frame_id+"_footprint";
  }
  
  if (should_inverse) {
    br.sendTransform(tf::StampedTransform(odomTF.inverse(),
                                          odom_msg->header.stamp,
                                          child_frame_id,
                                          odom_msg->header.frame_id));
  } else {
    br.sendTransform(tf::StampedTransform(odomTF, odom_msg->header.stamp,
                                          odom_msg->header.frame_id,
                                          child_frame_id));
  }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "odom_listener");
	ros::NodeHandle n;
	ros::NodeHandle np("~");
	ros::Subscriber sub = np.subscribe("/odom", 100, OdomCallback);

	np.param<bool>("inverse", should_inverse, false);
	np.param<bool>("stabilize", stabilized, false);
	np.param<bool>("footprint", footprint, false);

	ros::spin();

	return 0;
}
