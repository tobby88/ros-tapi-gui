#include "device.hpp"

using namespace ros;
using namespace std;

Device::Device(Device_Type type, string name, string uuid, Time last_seen_timestamp, unsigned long heartbeat_interval)
{
  this->type = type;
  this->name = name;
  this->uuid = uuid;
  this->last_seen_timestamp = last_seen_timestamp;
  this->heartbeat_interval = heartbeat_interval;
}

Device::~Device()
{ }

void Device::addFeature(Feature feature)
{
  if (features.count(feature.getID()) == 0)
    features.emplace(feature.getID(), feature);
}
