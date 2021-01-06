#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <fake_ar_publisher/ARMarker.h>
#include <myworkcell_core/LocalizePart.h>

class Localizer
{
public:
  fake_ar_publisher::ARMarkerConstPtr last_msg_;
  ros::Subscriber ar_sub_;
  ros::ServiceServer server_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  geometry_msgs::PoseStamped target_pose_from_cam;
  geometry_msgs::PoseStamped target_pose_from_req;

  Localizer(ros::NodeHandle& nh) //passing node handle as reference
    : tf_listener_(tf_buffer_)
  {
    //create a subscriber
    ar_sub_ = nh.subscribe<fake_ar_publisher::ARMarker>("ar_pose_marker", 1,
                                                         &Localizer::visionCallback, this);
    server_ = nh.advertiseService("localize_part", &Localizer::localizePart, this);
  }

  bool localizePart(myworkcell_core::LocalizePart::Request& req,
                    myworkcell_core::LocalizePart::Response& res)
  {
    //TODO: Check for the request type ?
    fake_ar_publisher::ARMarkerConstPtr p = last_msg_;

    // Check for the validity of the last message
    if (!p)
    {
      ROS_WARN("Last ARMarker message in invalid");
      return false;
    }

    // Send the pose as the response to client
    target_pose_from_cam.header = p->header;
    target_pose_from_cam.pose = p->pose.pose;

    // Transform the pose with respect to the base frame
    target_pose_from_req = tf_buffer_.transform(target_pose_from_cam, req.base_frame);

    res.pose = target_pose_from_req.pose;

    return true;
  }

  void visionCallback(const fake_ar_publisher::ARMarkerConstPtr& msg)
  {
    last_msg_ = msg;
    //ROS_INFO("Streaming last message");
    //ROS_INFO_STREAM(last_msg_->pose.pose);
  }
};

int main(int argc, char* argv[])
{
    // ROS Initialization
    ros::init(argc, argv, "vision_node");

    // ROS node handle
    ros::NodeHandle nh;

    // Initialize localizer instance
    Localizer localizer(nh);

    ROS_INFO("Hello World, this is vision_node");

    ros::spin();
}
