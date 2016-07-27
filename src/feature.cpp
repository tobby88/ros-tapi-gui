#include "feature.hpp"

Feature::Feature(FeatureType type, string name, string description, string uuid)
{
  this->type = type;
  this->name = name;
  this->description = description;
  this->uuid = uuid;
}

Feature::~Feature()
{ }

string Feature::getName()
{
  return name;
}

string Feature::getDescription()
{
  return description;
}

string Feature::getUUID()
{
  return uuid;
}

FeatureType Feature::getType()
{
  return type;
}
