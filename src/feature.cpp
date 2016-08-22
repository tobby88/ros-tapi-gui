#include "feature.hpp"

using namespace std;

// Constructor/Destructor

Feature::Feature(FeatureType type, string name, string description, string uuid)
{
  this->type = type;
  this->name = name;
  this->description = description;
  this->uuid = uuid;
  connections = 0;
}

Feature::~Feature() {}

// Public member functions

void Feature::decrementConnections() { connections--; }

int Feature::getConnectionCount() { return connections; }

string Feature::getDescription() { return description; }

string Feature::getName() const
{
  if (name.empty())
    return uuid;
  else
    return name;
}

FeatureType Feature::getType() { return type; }

string Feature::getUUID() { return uuid; }

void Feature::incrementConnections() { connections++; }
