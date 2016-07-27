#include "feature.hpp"

Feature::Feature(FeatureType type, string name, string description, unsigned long id)
{
  this->type = type;
  this->name = name;
  this->description = description;
  this->id = id;
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

unsigned long Feature::getID()
{
  return id;
}

FeatureType Feature::getType()
{
  return type;
}
