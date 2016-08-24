#ifndef API_H
#define API_H

#define STANDARD_HEARTBEAT_INTERVAL 2000L

#include "assignment.hpp"
#include "device.hpp"
#include "ros/ros.h"
#include "tobbyapi_msgs/Hello.h"
#include <map>
#include <string>
#include <vector>

using namespace ros;
using namespace std;

class Api
{
public:
  // Constructor/Destructor
  Api(NodeHandle* nh);
  ~Api();

  // Public member functions
  bool CheckPending();
  bool ConnectFeatures(string feature1UUID, string feature2UUID,
                       double coefficient);
  void DebugOutput();
  bool DeleteConnection(string receiverFeatureUUID);
  void Done();
  vector<Assignment*> GetConnections();
  vector<Device*> GetDevicesSorted();
  void Run();

private:
  // Private member variables
  Publisher configPub;
  map<string, Assignment> connections;
  map<string, Device> devices;
  ServiceServer helloServ;
  NodeHandle* nh;
  bool pendingChanges;
  AsyncSpinner* spinner;

  // Private member functions
  void changed();
  static bool compareDeviceNames(const Device* first, const Device* second);
  Device* getDeviceByFeatureUUID(string uuid);
  bool hello(tobbyapi_msgs::Hello::Request& helloReq,
             tobbyapi_msgs::Hello::Response& helloResp);
  void sendAllConnections();
};

#endif // API_H
