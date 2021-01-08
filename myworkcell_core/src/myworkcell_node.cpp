#include <memory>

#include <ros/ros.h>
#include <tf/tf.h>
#include <myworkcell_core/LocalizePart.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <myworkcell_core/PlanCartesianPath.h>

class ScanNPlan
{
private:
  // ROS Client for the vision node server
  ros::ServiceClient vision_client_;

  // ROS client for descartes Cartesian planner
  ros::ServiceClient cartesian_client_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> cartesian_action_;

public:

  // Unique pointer to move group instance
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;

  ScanNPlan(ros::NodeHandle& nh)
    : cartesian_action_("joint_trajectory_action", true)
  {
    // Initialize moveit move group
    move_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(moveit::planning_interface::MoveGroupInterface("manipulator"));

    vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");

    cartesian_client_ = nh.serviceClient<myworkcell_core::PlanCartesianPath>("plan_path");
  }

  void start(const std::string& base_frame)
  {
    ROS_INFO_STREAM("Attempting to localize part");

    // Initialzie localize part service instance
    myworkcell_core::LocalizePart srv;

    srv.request.base_frame = base_frame;
    ROS_INFO_STREAM("Requesting pose with " << base_frame << " as the base frame");

    if(!vision_client_.call(srv))
    {
      ROS_ERROR("Failed to localize part");
      return;
    }

    ROS_INFO_STREAM("Part localized with " << srv.response);

    geometry_msgs::Pose move_target = srv.response.pose;

    // Plan for robot to move to localize part location
    move_group->setPoseReferenceFrame(base_frame);

    // Get current end-effector pose using default end-effector link
    geometry_msgs::PoseStamped old_pose = move_group->getCurrentPose();

    // Set target pose
    move_group->setPoseTarget(move_target);

    // Move to target pose
    ROS_INFO_STREAM("Moving to target pose ...");
    move_group->move();

    // Move back to old pose after a small delay
    ros::Duration(2).sleep();
    ROS_INFO_STREAM("Moving back to home pose ... ");
    move_group->setPoseTarget(old_pose);
    move_group->move();

    // Request descartes Cartesian trajectory
    myworkcell_core::PlanCartesianPath cartesian_srv;
    cartesian_srv.request.pose = move_target;
    if (!cartesian_client_.call(cartesian_srv))
    {
      ROS_ERROR("Could not plan the cartesian path");
      return;
    }

    ROS_INFO_STREAM("Cartesian path obtained from descartes planner");
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = cartesian_srv.response.trajectory;
    cartesian_action_.sendGoal(goal);
    cartesian_action_.waitForResult();

    ROS_INFO_STREAM("Cartesian path execution done");

  }
};

int main(int argc, char* argv[])
{
  // Initialzie ros
  ros::init(argc, argv, "myworkcell_node");
  ros::NodeHandle nh;

  // Private node handle for ros parameters
  ros::NodeHandle private_node_handle("~");

  // NOTE: ros parameters are always string data type ?
  std::string base_frame_request = private_node_handle.param<std::string>("base_frame", "world");

  ROS_INFO_STREAM("ScanNPlan application node initialized");

  ScanNPlan app(nh);

  ros::Duration(0.5).sleep();

  // Adding asynchronous spinner for enabling move group planning
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  // Call ScanNPlan application start method
  // This start is called from the main thread of this Ros node
  app.start(base_frame_request);

  ros::waitForShutdown();

}
