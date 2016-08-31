#ifndef API_H
#define API_H

// Intervals in ms
#define HEARTBEAT_CHECK_INTERVAL 100L
#define STANDARD_HEARTBEAT_INTERVAL 2000L

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
  void AddDeviceWithoutHello(uint8_t type, std::string name, std::string uuid, unsigned long heartbeat,
                             std::map<std::string, Tapi::Feature> features);
  bool CheckPending();
  void Clear();
  bool ConnectFeatures(std::string feature1UUID, std::string feature2UUID, double coefficient);
  void DebugOutput();
  bool DeleteConnection(std::string receiverFeatureUUID);
  void Done();
  std::vector<Tapi::Connection*> GetConnections();
  std::vector<Tapi::Device*> GetDevicesSorted();
  void Run();

private:
  // Private member variables
  std::map<std::string, Tapi::Connection> connections;
  ros::Publisher conPub;
  ros::Publisher delPub;
  std::map<std::string, Tapi::Device> devices;
  ros::ServiceClient devListClient;
  ros::Timer heartbeatCheckTimer;
  ros::Time lastUpdated;
  ros::Subscriber lastUpdatedSub;
  ros::NodeHandle* nh;
  bool pendingChanges;
  ros::AsyncSpinner* spinner;

  // Private member functions
  void changed();
  static bool compareDeviceNames(const Tapi::Device* first, const Tapi::Device* second);
  Tapi::Device* getDeviceByFeatureUUID(std::string uuid);
  void heartbeatCheck(const ros::TimerEvent& e);
  void sendAllConnections();
  void updateData(const std_msgs::Time::ConstPtr& time);
};
}

#endif  // API_H
