#include "device.hpp"

using namespace ros;
using namespace std;

// Constructor/Destructor

Device::Device(DeviceType type, string name, string uuid, unsigned long lastSeq,
               Time lastSeen, unsigned long heartbeat)
{
  this->type = type;
  this->name = name;
  this->uuid = uuid;
  this->lastSeq = lastSeq;
  this->lastSeen = lastSeen;
  this->heartbeat = heartbeat;
}

Device::~Device() {}

// Public member functions

void Device::addFeature(Feature feature)
{
  if (features.count(feature.getUUID()) == 0)
    features.emplace(feature.getUUID(), feature);
}

bool Device::compareFeatureNames(const Feature* first, const Feature* second)
{
  return first->getName() < second->getName();
}

Feature* Device::getFeatureByUUID(string uuid)
{
  if (features.count(uuid) > 0)
    return &features.at(uuid);
  else
    return 0;
}

map<string, Feature> Device::getFeatureMap() { return features; }

unsigned long Device::getHeartbeat() { return heartbeat; }

Time Device::getLastSeen() { return lastSeen; }

unsigned long Device::getLastSeq() { return lastSeq; }

string Device::getName() const
{
  if (name.empty())
    return uuid;
  else
    return name;
}

vector<Feature*> Device::GetSortedFeatures()
{
  vector<Feature*> featureList;
  for (map<string, Feature>::iterator it = features.begin();
       it != features.end(); it++)
    featureList.push_back(&it->second);
  if (featureList.size() > 1)
    sort(featureList.begin(), featureList.end(), compareFeatureNames);
  return featureList;
}

DeviceType Device::getType() { return type; }

string Device::getUUID() { return uuid; }

void Device::Update(DeviceType type, string name, unsigned long lastSeq,
                    Time lastSeen, unsigned long heartbeat)
{
  this->type = type;
  this->name = name;
  this->lastSeq = lastSeq;
  this->lastSeen = lastSeen;
  this->heartbeat = heartbeat;
}
