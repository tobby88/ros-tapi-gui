#ifndef API_H
#define API_H

// Intervals in ms
#define CHECK_INTERVAL 1000L

#include <map>
#include <string>
#include <vector>
#include "connection.hpp"
#include "device.hpp"
#include "ros/ros.h"
#include "std_msgs/Time.h"

namespace Tapi
{
class Api
{
public:
  // Constructor/Destructor
  explicit Api(ros::NodeHandle* nh);
  ~Api();

  // Public member functions
  void Run();

  void changed();
  static bool compareDeviceNames(const Tapi::Device* first, const Tapi::Device* second);
  Tapi::Device* getDeviceByFeatureUUID(std::string uuid);
  void timer(const ros::TimerEvent& e);
  void updateAvailable(const std_msgs::Time::ConstPtr& time);
  void updateData();

  // Public member variables
  ros::Publisher clearPub;
  std::map<std::string, Tapi::Connection> connections;
  ros::ServiceClient conListClient;
  ros::Publisher conPub;
  ros::Publisher delPub;
  std::map<std::string, Tapi::Device> devices;
  ros::ServiceClient devListClient;
  ros::ServiceClient helloClient;
  ros::Time lastUpdated;
  ros::Subscriber lastUpdatedSub;
  ros::NodeHandle* nh;
  bool pendingChanges;
  ros::AsyncSpinner* spinner;
  ros::Timer updateTimer;

};
}

#endif  // API_H
