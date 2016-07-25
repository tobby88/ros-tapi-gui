#include "device.hpp"

using namespace ros;
using namespace std;

Device::Device(Device_Type type, string name, string uuid, unsigned long last_seq, Time last_seen_timestamp, unsigned long heartbeat_interval)
{
  this->type = type;
  this->name = name;
  this->uuid = uuid;
  this->last_seq = last_seq;
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

Device_Type Device::getType()
{
  return type;
}
map<unsigned long, Feature> Device::getFeatureMap()
{
  return features;
}

string Device::getName()
{
  return name;
}

string Device::getUUID()
{
  return uuid;
}

unsigned long Device::getLastSeq()
{
  return last_seq;
}

Time Device::getLastSeenTimestamp()
{
  return last_seen_timestamp;
}

unsigned long Device::getHeartbeatInterval()
{
  return heartbeat_interval;
}
