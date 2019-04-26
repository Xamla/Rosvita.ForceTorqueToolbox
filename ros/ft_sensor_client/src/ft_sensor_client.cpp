#include <ft_sensor_client.h>

SensorClient::SensorClient(ros::NodeHandle nodeHandle, std::string wrenchTopic, 
                           double idleRate, int bufferSize)
  : nodeHandle(nodeHandle)
  , isReading(false)
  , hasTimeout(true)
  , throttling(false)
  , pubDuration(1)
  , idleRate(idleRate)
  , biasForce(0, 0, 0)
  , biasTorque(0, 0, 0)
  , maxBuffer(bufferSize)
  , forces(bufferSize+1)
  , torques(bufferSize+1)
  , tfListener(tfBuffer)
  , wrenchTopic(wrenchTopic)
  , wrenchFrame("world")
  , referenceFrame("world")
{
  const std::string WRENCH_TOPIC = "ft_wrench";
  const std::string WRENCH_NULLED_TOPIC = "ft_nulled_wrench";
  const std::string WRENCH_NULLED_REF_TOPIC = "ft_nulled_reference_wrench";

  wrenchData_pub = nodeHandle.advertise<geometry_msgs::WrenchStamped>(WRENCH_TOPIC, 1, this);
  wrenchNulled_pub = nodeHandle.advertise<geometry_msgs::WrenchStamped>(WRENCH_NULLED_TOPIC, 1, this);
  wrenchNulledReference_pub = nodeHandle.advertise<geometry_msgs::WrenchStamped>(WRENCH_NULLED_REF_TOPIC, 1, this);

  wrenchData_sub = nodeHandle.subscribe(wrenchTopic, 1024, &SensorClient::wrenchCallback, this);

  beginDataCollection_srv =
      nodeHandle.advertiseService("beginDataCollection", &SensorClient::beginDataCollection, this);
  getCapturedData_srv = nodeHandle.advertiseService("getCapturedData", &SensorClient::getCapturedData, this);
  abortReading_srv = nodeHandle.advertiseService("abortReadingSensor", &SensorClient::abortReading, this);
  nullify_srv = nodeHandle.advertiseService("nullify", &SensorClient::nullify, this);
  ROS_INFO("ft_client_started ...");
}

void SensorClient::run()
{
  ROS_INFO("... and running");
  while (ros::ok())
  {
    // ros::spinOnce();
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(idleRate));
    if (isReading)
    {
      publishSensorData();
    }
  }
}

void SensorClient::publishSensorData()
{
  wrenchData_pub.publish(currentState);
  wrenchNulled_pub.publish(nulledWrench);
  wrenchNulledReference_pub.publish(nulledReferenceWrench);
}

bool SensorClient::nullify(uint sample_size)
{
  ROS_INFO("Nullify with the average of a sample size of %d ", sample_size);
  tf2::Vector3 sum_force(0, 0, 0);
  tf2::Vector3 sum_torque(0, 0, 0);

  if (maxBuffer > forces.size())
  {
    ROS_ERROR("Not enough data sampled to nullify. Only %d samples", (int)forces.size());
    return false;
  }

  if (sample_size > forces.size())
  {
    ROS_WARN("Sample size too big. Changed to %d", (int)forces.size());
    sample_size = forces.size();
  }
  for (uint i = 0; i < sample_size; ++i)
  {
    sum_force += forces[i];
    sum_torque += torques[i];
  }

  biasForce = sum_force / (double)sample_size;
  biasTorque = sum_torque / (double)sample_size;

  ROS_INFO("Nulled  forces  %f %f  %f", biasForce.getX(), biasForce.getY(), biasForce.getZ());
  ROS_INFO("Nulled  torque  %f %f  %f", biasTorque.getX(), biasTorque.getY(), biasTorque.getZ());
  return true;
}

bool SensorClient::setFrequency(double frequency)
{
  if (frequency > 0)
  {
    pubDuration = ros::Duration(ros::Rate(frequency));
    ROS_INFO("Set publishing frequency to %f Hz", frequency);
    throttling = true;
    return true;
  }
  else
  {
    // Assuming no throttling of messages
    ROS_INFO("No frequency set. Pass data through");
    throttling = false;
    return true;
  }
}

bool SensorClient::setReferenceFrame(std::string newFrame)
{
  if (tfBuffer._frameExists(newFrame)) 
  {
    referenceFrame = newFrame;
    ROS_INFO("Set reference frame to %s", newFrame.c_str());
    return true;
  }
  else
  {
    ROS_WARN("Could not set reference frame to %s", newFrame.c_str());
    return false;
  }
}

void SensorClient::setTimeout(ros::Duration timeout)
{
  if (timeout.sec == 0 && timeout.nsec == 0)
  {
    hasTimeout = false;
    ROS_INFO("No timeout.");
  }
  else
  {
    endTime = ros::Time::now() + timeout;
    ROS_INFO("Set timeout.");
    hasTimeout = true;
  }
}

void SensorClient::updateReferenceTransform()
{
  try
  {
    geometry_msgs::TransformStamped transform_stamped =
        tfBuffer.lookupTransform(wrenchFrame, referenceFrame, ros::Time(0));
    geometry_msgs::Transform transform = transform_stamped.transform;
    // tf2::fromMsg(transform.transform, referenceTransform);
    referenceTransform = tf2::Transform::getIdentity();
    auto quat = transform.rotation;
    referenceTransform.setRotation(tf2::Quaternion(quat.x, quat.y, quat.z, quat.w));
    referenceTransform.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
    ROS_WARN("Could not transform to reference frame");
    referenceTransform = tf2::Transform::getIdentity();
    return;
  }
}

void SensorClient::checkTimeout()
{
  if (hasTimeout && ros::Time::now() > endTime )
  {
    isReading = false;
    ROS_INFO("Timout reached, stop collecting data");
    return;
  }
}
void SensorClient::checkMaxWrenchCount()
{
  if (maxWrenchCount > 0 && (maxWrenchCount <= currentWrenchCount))
  {
    isReading = false;
    ROS_INFO("Max count of wrenches reached, stop collecting data");
    return;
  }
}

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Services Definitions //////////////////////////
///////////////////////////////////////////////////////////////////////////////

bool SensorClient::beginDataCollection(ft_sensor_client::BeginDataCapture::Request& request,
                                       ft_sensor_client::BeginDataCapture::Response& response)
{
  ROS_INFO("Begin data collection");
  double frequency = (double)request.frequency;
  std::string referenceFrame = (std::string)request.referenceFrame;
  ros::Duration timeout = (ros::Duration)request.timeout;
  maxWrenchCount = (int)request.maxWrenchCount;
  bool nullifyData = (bool)request.nullifyData;
  std::ostringstream stringStream;
  if (isReading)
  {
    ROS_WARN("Already reading");
    response.success = false;
    response.message = "Already reading data.";
    return true;
  }
  if (!setFrequency(frequency))
  {
    response.success = false;
    stringStream << "Could not set frequency to "  << frequency << ".";
    response.message = stringStream.str();
    ROS_ERROR("Could not set frequency");
    return true;
  }
  if (!tfBuffer._frameExists(wrenchFrame))
  {
    response.success = false;
    stringStream << "The current wrench frame '" << wrenchFrame << "' does not exists. Please enter a valid frame for the wrench in the configuration.";
    ROS_ERROR("The current wrench frame '%s' does not exists.", wrenchFrame.c_str());
    response.message = stringStream.str();
    return true;
  }
  if (!setReferenceFrame(referenceFrame))
  {
    response.success = false;
    stringStream <<  "Could not set reference frame to '" << referenceFrame << "'.";

    response.message = stringStream.str();
    ROS_ERROR("Could not set reference frame");
    return true;
  }
  if (!checkTransform())
  {
    response.success = false;
    std::ostringstream stringStream;
    stringStream << "Could not transform " << wrenchFrame << "to reference frame " << referenceFrame << ". ";
    response.message = stringStream.str();

    ROS_ERROR("Could not transform to reference frame ");
    return true;
  }

  if (nullifyData)
  {
    bool success = nullify(maxBuffer);
    if (!success)
    {
      response.success = false;
      response.message = "Could not nullify the data";
      return true;
    }
  }

  setTimeout(timeout);
  isReading = true;

  currentTimeSlot = ros::Time::now();
  currentWrenchCount = 0;
  capturedWrenchData = std::vector<geometry_msgs::WrenchStamped>();

  ROS_INFO("Begin collecting data.");
  response.success = true;
  response.message = "Begin reading success";
  return true;
}

bool SensorClient::getCapturedData(ft_sensor_client::GetCapturedData::Request& request,
                                   ft_sensor_client::GetCapturedData::Response& response)
{
  response.success = true;
  response.wrenchData = capturedWrenchData;
  if (isReading)
  {
    ROS_INFO("Stopping the data capture.");
    isReading = false;
  }
  response.message = "Success";
  return true;
}

bool SensorClient::abortReading(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response)
{
  if (isReading)
  {
    isReading = false;
    ROS_INFO("Abort reading");
    response.success = true;
    response.message = "End reading success";
  }
  else
  {
    ROS_INFO("Not reading");
    response.success = true;
    response.message = "Not reading";
  }
  return true;
}

bool SensorClient::nullify(ft_sensor_client::Nullify::Request& request, ft_sensor_client::Nullify::Response& response)
{
  ROS_INFO("Nullify");
  uint sampleSize = (uint)request.sampleSize;

  bool result = nullify(sampleSize);
  if (result)
  {
    response.success = true;
    response.message = "Data nullified";
  }
  else
  {
    response.success = false;
    response.message = "Could not nullify data";
  }
  return true;
}

bool SensorClient::checkTransform()
{
  // Tests if the transformation to reference frame is successfull.
  try
  {
    geometry_msgs::TransformStamped transform_stamped =
        tfBuffer.lookupTransform(wrenchFrame, referenceFrame, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    std::ostringstream stringStream;
    stringStream << "Could not transform " << wrenchFrame << "to reference frame " << referenceFrame << ". "
                 << ex.what();
    std::string message = stringStream.str();
    //  ROS_WARN(message);
    return false;
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Callbacks /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void SensorClient::wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench)
{
  // update wrench frame given the header
  wrenchFrame = wrench->header.frame_id;
  updateReferenceTransform();

  currentForce.setX(wrench->wrench.force.x);
  currentForce.setY(wrench->wrench.force.y);
  currentForce.setZ(wrench->wrench.force.z);
  currentTorque.setX(wrench->wrench.torque.x);
  currentTorque.setY(wrench->wrench.torque.y);
  currentTorque.setZ(wrench->wrench.torque.z);

  forces.push_back(currentForce);
  torques.push_back(currentTorque);
  if (forces.size() > maxBuffer)
    forces.pop_front();
  if (torques.size() > maxBuffer)
    torques.pop_front();

  currentState.header.stamp = wrench->header.stamp;
  currentState.header.frame_id = wrench->header.frame_id;
  currentState.wrench.force.x = wrench->wrench.force.x;
  currentState.wrench.force.y = wrench->wrench.force.y;
  currentState.wrench.force.z = wrench->wrench.force.z;
  currentState.wrench.torque.x = wrench->wrench.torque.x;
  currentState.wrench.torque.y = wrench->wrench.torque.y;
  currentState.wrench.torque.z = wrench->wrench.torque.z;

  tf2::Vector3 nulledForce = currentForce - biasForce;
  tf2::Vector3 nulledTorque = currentTorque - biasTorque;

  nulledWrench = currentState;
  nulledWrench.wrench.force.x = nulledForce.getX();
  nulledWrench.wrench.force.y = nulledForce.getY();
  nulledWrench.wrench.force.z = nulledForce.getZ();
  nulledWrench.wrench.torque.x = nulledTorque.getX();
  nulledWrench.wrench.torque.y = nulledTorque.getY();
  nulledWrench.wrench.torque.z = nulledTorque.getZ();

  tf2::Vector3 nulledReferenceForce = referenceTransform * nulledForce;
  tf2::Vector3 nulledReferenceTorque = referenceTransform * nulledTorque;
  nulledReferenceWrench = currentState;
  nulledReferenceWrench.header.frame_id = referenceFrame;
  nulledReferenceWrench.wrench.force.x = nulledReferenceForce.getX();
  nulledReferenceWrench.wrench.force.y = nulledReferenceForce.getY();
  nulledReferenceWrench.wrench.force.z = nulledReferenceForce.getZ();
  nulledReferenceWrench.wrench.torque.x = nulledReferenceTorque.getX();
  nulledReferenceWrench.wrench.torque.y = nulledReferenceTorque.getY();
  nulledReferenceWrench.wrench.torque.z = nulledReferenceTorque.getZ();

  if (isReading)
  {
    if (throttling)
    {
      ros::Time wrenchTime = wrench->header.stamp;
      if (wrenchTime > currentTimeSlot)
      {
        captureCurrentWrenchDataPoint();
        while (currentTimeSlot < wrenchTime)
        {
          currentTimeSlot += pubDuration;
        }
      }
      else
      {
        // ignore this datapoint, since it lies not in current slot
      }
    }
    else
    {
      // When not throttling, pass everything through
      captureCurrentWrenchDataPoint();
    }
    checkTimeout();
    checkMaxWrenchCount();
  }
}

void SensorClient::captureCurrentWrenchDataPoint()
{
  capturedWrenchData.push_back(nulledReferenceWrench);
  ++currentWrenchCount;
}


