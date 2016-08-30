#include "device.hpp"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

Device::Device(uint8_t type, string name, string uuid, unsigned long lastSeq, ros::Time lastSeen,
               unsigned long heartbeat, map<string, Feature> features)
  : type(type), name(name), uuid(uuid), lastSeq(lastSeq), lastSeen(lastSeen), heartbeat(heartbeat), features(features)
{
  active = true;
}

Device::~Device()
{
}

// Public member functions

bool Device::Active()
{
  return active;
}

void Device::Deactivate()
{
  active = false;
}

Feature* Device::GetFeatureByUUID(string uuid)
{
  if (features.count(uuid) > 0)
    return &features.at(uuid);
  else
    return 0;
}

unsigned long Device::GetHeartbeat()
{
  return heartbeat;
}

ros::Time Device::GetLastSeen()
{
  return lastSeen;
}

unsigned long Device::GetLastSeq()
{
  return lastSeq;
}

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
  for (auto it = features.begin(); it != features.end(); ++it)
    featureList.push_back(&it->second);
  if (featureList.size() > 1)
    sort(featureList.begin(), featureList.end(), compareFeatureNames);
  return featureList;
}

uint8_t Device::GetType()
{
  return type;
}

string Device::GetUUID()
{
  return uuid;
}

void Device::Update(uint8_t type, string name, unsigned long lastSeq, ros::Time lastSeen, unsigned long heartbeat,
                    map<string, Feature> featureUpdate)
{
  this->type = type;
  this->name = name;
  this->lastSeq = lastSeq;
  this->lastSeen = lastSeen;
  this->heartbeat = heartbeat;
  if (features.size() == 0)
  {
    features = featureUpdate;
    return;
  }

  bool equ = true;
  if (features.size() == featureUpdate.size())
  {
    for (auto it = features.begin(); it != features.end(); ++it)
    {
      if (!featureUpdate.count(it->second.GetUUID()) == 1)
        equ = false;
      else if (!(featureUpdate.at(it->second.GetUUID()) == it->second))
        equ = false;
    }
  }
  else
    equ = false;

  if (!equ)
  {
    for (auto it = featureUpdate.begin(); it != featureUpdate.end(); ++it)
    {
      if (features.count(it->second.GetUUID()) == 0)
        features.emplace(it->second.GetUUID(), it->second);
      else
        features.at(it->second.GetUUID()).Update(it->second.GetType(), it->second.GetName());
    }
    for (auto it = features.begin(); it != features.end(); ++it)
    {
      if (featureUpdate.count(it->second.GetUUID()) == 0)
        features.erase(it->second.GetUUID());
    }
  }
  active = true;
}

// Private member functions

bool Device::compareFeatureNames(const Feature* first, const Feature* second)
{
  return first->GetName() < second->GetName();
}
}
