#include "feature.hpp"

using namespace std;

// Constructor/Destructor

Feature::Feature(uint8_t type, string name, string description, string uuid)
{
  this->type = type;
  this->name = name;
  this->description = description;
  this->uuid = uuid;
  connections = 0;
}

Feature::~Feature()
{
}

// Public member functions

void Feature::DecrementConnections()
{
  connections--;
}

int Feature::GetConnectionCount()
{
  return connections;
}

string Feature::GetDescription()
{
  return description;
}

string Feature::GetName() const
{
  if (name.empty())
    return uuid;
  else
    return name;
}

uint8_t Feature::GetType()
{
  return type;
}

string Feature::GetUUID()
{
  return uuid;
}

void Feature::IncrementConnections()
{
  connections++;
}
