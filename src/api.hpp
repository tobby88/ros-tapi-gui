#ifndef API_H
#define API_H

#include "assignment.hpp"
#include "device.hpp"
#include "ros/ros.h"
#include "tobby/Hello.h"
#include <unordered_map>

using namespace ros;
using namespace std;

class Api
{
private:
  NodeHandle* nh;
  unordered_map<string, Device> devices;
  unordered_map<string, Assignment> connections;
  ServiceServer helloServ;
  Publisher configPub;
  bool hello(tobby::Hello::Request& helloReq,
             tobby::Hello::Response& helloResp);
  bool changes;
  AsyncSpinner* spinner;

public:
  Api(NodeHandle* nh);
  ~Api();
  void DebugOutput();
  void Run();
};

#endif // API_H
