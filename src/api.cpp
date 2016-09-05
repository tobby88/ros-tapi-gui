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
}
}
