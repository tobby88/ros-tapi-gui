#include "api.hpp"
#include "feature.hpp"
#include "std_msgs/Bool.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "tapi_msgs/Connect.h"
#include "tapi_msgs/Connection.h"
#include "tapi_msgs/Device.h"
#include "tapi_msgs/Feature.h"
#include "tapi_msgs/GetConnectionList.h"
#include "tapi_msgs/GetDeviceList.h"
#include "tapi_msgs/Hello.h"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

Api::Api(ros::NodeHandle* nh) : nh(nh)
{
  spinner = new ros::AsyncSpinner(1);
  pendingChanges = false;
  devListClient = nh->serviceClient<tapi_msgs::GetDeviceList>("Tapi/GetDeviceList");
  delPub = nh->advertise<std_msgs::String>("Tapi/DeleteConnection", 1000);
  conPub = nh->advertise<tapi_msgs::Connect>("Tapi/ConnectFeatures", 1000);
  conListClient = nh->serviceClient<tapi_msgs::GetConnectionList>("Tapi/GetConnectionList");
  clearPub = nh->advertise<std_msgs::Bool>("Tapi/Clear", 2);
  helloClient = nh->serviceClient<tapi_msgs::Hello>("Tapi/HelloServ");
  updateTimer.start();
}

Api::~Api()
{
  updateTimer.stop();
  spinner->stop();
  delete spinner;
  devListClient.shutdown();
  lastUpdatedSub.shutdown();
  delPub.shutdown();
  conPub.shutdown();
  clearPub.shutdown();
  helloClient.shutdown();
}
}
