#ifndef FT_SENSOR_CLIENT_H
#define FT_SENSOR_CLIENT_H

#include <chrono>
#include <array>
#include <boost/circular_buffer.hpp>
#include <string>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>

#include "std_srvs/Trigger.h"

#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Transform.h"

#include "ft_sensor_client/Nullify.h"
#include "ft_sensor_client/BeginDataCapture.h"
#include "ft_sensor_client/GetCapturedData.h"

class SensorClient
{
public:
  SensorClient(ros::NodeHandle nodeHandle, std::string wrenchTopic, double idleRate,
               int bufferSize);
  ~SensorClient() = default;

  void run();

private:
  ros::NodeHandle nodeHandle;
  std::string wrenchTopic;
  std::string wrenchFrame;
  std::string referenceFrame;

  bool isReading;

  // True when capturing data has no timeout
  bool hasTimeout;

  // True, when not all wrenched data is
  bool throttling;

  // When > 0, defines max amount of data captured per reading process
  // else, reads indefinitely data
  int maxWrenchCount;

  // Keeps track of the amount of
  int currentWrenchCount;

  ros::Rate idleRate;

  // When reading, the time the reading process should end
  ros::Time endTime;
  ros::Time currentTimeSlot;
  ros::Duration pubDuration;

  geometry_msgs::WrenchStamped currentState;
  geometry_msgs::WrenchStamped nulledWrench;
  geometry_msgs::WrenchStamped nulledReferenceWrench;

  int maxBuffer;
  boost::circular_buffer<tf2::Vector3> forces;
  boost::circular_buffer<tf2::Vector3> torques;

  // Save the captured data into a vecotr
  std::vector<geometry_msgs::WrenchStamped> capturedWrenchData;

  tf2::Vector3 currentForce;
  tf2::Vector3 currentTorque;

  tf2::Vector3 biasForce;
  tf2::Vector3 biasTorque;

  tf2::Transform referenceTransform;

  // Publishes the current sensor data on some topics
  void publishSensorData();

  bool nullify(uint sampleSize);
  // Checks if a transformation is valid
  bool checkTransform();

  bool setFrequency(double frequency);
  bool setReferenceFrame(std::string newFrame);
  void setTimeout(ros::Duration timeout);

  void updateReferenceTransform();

  void captureCurrentWrenchDataPoint();

  void checkTimeout();
  void checkMaxWrenchCount();

  tf2_ros::TransformListener tfListener;
  tf2_ros::Buffer tfBuffer;

  // Subscriber
  ros::Subscriber wrenchData_sub;
  // Publisher
  ros::Publisher wrenchData_pub;
  ros::Publisher wrenchNulled_pub;
  ros::Publisher wrenchNulledReference_pub;

  // services:
  ros::ServiceServer beginDataCollection_srv;
  ros::ServiceServer getCapturedData_srv;
  ros::ServiceServer abortReading_srv;
  ros::ServiceServer nullify_srv;

  // This function is subscribed to the topic producing the wrenched data
  void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench);

  bool beginDataCollection(ft_sensor_client::BeginDataCapture::Request& request,
                           ft_sensor_client::BeginDataCapture::Response& response);
  bool getCapturedData(ft_sensor_client::GetCapturedData::Request& request,
                       ft_sensor_client::GetCapturedData::Response& response);
  bool abortReading(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);
  bool nullify(ft_sensor_client::Nullify::Request& request, ft_sensor_client::Nullify::Response& response);
};

#endif