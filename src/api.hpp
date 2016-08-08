#ifndef API_H
#define API_H

#include "assignment.hpp"
#include "device.hpp"
#include "ros/ros.h"
#include "tobby/Hello.h"
#include <map>

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

public:
  Api(NodeHandle* nh);
  ~Api();
  bool CheckPending();
  void DebugOutput();
  void Run();
};

#endif // API_H
