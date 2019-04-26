
#include <ft_sensor_client.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ft_sensor_client");
  ros::NodeHandle nodeHandle("~");

  std::string wrenchTopic = "";
  
  double idleRate = 30;
  int bufferSize = 200;

  if (!nodeHandle.getParam("wrenchTopic", wrenchTopic))
  {
    ROS_ERROR("Failed to get parameter 'wrenchTopic'");
    return 1;
  }

  if (!nodeHandle.getParam("bufferSize", bufferSize))
  {
    ROS_WARN("Failed to get parameter 'bufferSize'");
  }
  if (!nodeHandle.getParam("idleRate", idleRate))
  {
    ROS_WARN("Failed to get parameter 'idleRate'");
  }
  SensorClient sensor_client(nodeHandle, wrenchTopic, idleRate, bufferSize);
  sensor_client.run();
  return 0;
}
