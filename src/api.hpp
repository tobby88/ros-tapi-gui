#ifndef API_H
#define API_H

#include "assignment.hpp"
#include "device.hpp"
#include "ros/ros.h"
#include "tobby/Hello.h"
#include <map>
#include <string>
#include <vector>

using namespace ros;
using namespace std;

class Api
{
private:
  NodeHandle* nh;
  map<string, Device> devices;
  map<string, Assignment> connections;
  ServiceServer helloServ;
  Publisher configPub;
  bool hello(tobby::Hello::Request& helloReq,
             tobby::Hello::Response& helloResp);
  bool pendingChanges;
  AsyncSpinner* spinner;
  void changed();
  static bool compareDeviceNames(const Device* first, const Device* second);
  Device* getDeviceByFeatureUUID(string uuid);
  void sendAllConnections();

public:
  Api(NodeHandle* nh);
  ~Api();
  bool CheckPending();
  void DebugOutput();
  vector<Device*> GetDevicesSorted();
  void Run();
  void Done();
  bool ConnectFeatures(string feature1uuid, string feature2uuid);
  vector<Assignment*> GetConnections();
  bool DeleteConnection(string receiverFeatureUUID);
};

#endif // API_H
