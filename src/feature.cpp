#include "feature.hpp"

using namespace std;

// Constructor/Destructor

Feature::Feature(uint8_t type, string name, string description, string uuid)
  : type(type), name(name), description(description), uuid(uuid)
{
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

string Feature::GetDescription() const
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

uint8_t Feature::GetType() const
{
  return type;
}

string Feature::GetUUID() const
{
  return uuid;
}

void Feature::IncrementConnections()
{
  connections++;
}

bool Feature::operator==(const Feature &other) const
{
  if (GetUUID() == other.GetUUID() && GetType() == other.GetType() && GetName() == other.GetName() &&
      GetDescription() == other.GetDescription())
    return true;
  else
    return false;
}

void Feature::Update(uint8_t type, std::string name, std::string description)
{
  this->type = type;
  this->name = name;
  this->description = description;
}
