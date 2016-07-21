#include "feature.hpp"

Feature::Feature(Feature_Type type, string feature_name, unsigned long id)
{
  this->type = type;
  this->feature_name = feature_name;
  this->id = id;
}

Feature::~Feature()
{ }

unsigned long Feature::getID()
{
  return id;
}
