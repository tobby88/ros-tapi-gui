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

void Device::AddFeature(Feature feature)
{
  if (features.count(feature.GetUUID()) == 0)
    features.emplace(feature.GetUUID(), feature);
}

Feature* Device::GetFeatureByUUID(string uuid)
{
  if (features.count(uuid) > 0)
    return &features.at(uuid);
  else
    return 0;
}

unsigned long Device::GetHeartbeat() { return heartbeat; }

Time Device::GetLastSeen() { return lastSeen; }

unsigned long Device::GetLastSeq() { return lastSeq; }

string Device::GetName() const
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

DeviceType Device::GetType() { return type; }

string Device::GetUUID() { return uuid; }

void Device::Update(DeviceType type, string name, unsigned long lastSeq,
                    Time lastSeen, unsigned long heartbeat)
{
  this->type = type;
  this->name = name;
  this->lastSeq = lastSeq;
  this->lastSeen = lastSeen;
  this->heartbeat = heartbeat;
}

// Private member functions

bool Device::compareFeatureNames(const Feature* first, const Feature* second)
{
  return first->GetName() < second->GetName();
}
