#include <ros/ros.h>
#include <myworkcell_core/LocalizePart.h>

class ScanNPlan
{
private:
  // Ros Client for the vision node server
  ros::ServiceClient vision_client_;

public:
  ScanNPlan(ros::NodeHandle& nh)
  {
    vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
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

  // Call ScanNPlan application start method
  // This start is called from the main thread of this Ros node
  app.start(base_frame_request);

  ros::spin();
}
