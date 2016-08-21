#include "feature.hpp"

Feature::Feature(FeatureType type, string name, string description, string uuid)
{
  this->type = type;
  this->name = name;
  this->description = description;
  this->uuid = uuid;
  connections = 0;
}

Feature::~Feature() {}

string Feature::getName() const
{
  if (name.empty())
    return uuid;
  else
    return name;
}

string Feature::getDescription() { return description; }

string Feature::getUUID() { return uuid; }

FeatureType Feature::getType() { return type; }

void Feature::incrementConnections() { connections++; }

void Feature::decrementConnections() { connections--; }

int Feature::getConnectionCount() { return connections; }
