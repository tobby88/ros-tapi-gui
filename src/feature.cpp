#include "feature.hpp"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

Feature::Feature(string type, string name, string uuid) : type(type), name(name), uuid(uuid)
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

string Feature::GetName() const
{
  if (name.empty())
    return uuid;
  else
    return name;
}

string Feature::GetType() const
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
  if (GetUUID() == other.GetUUID() && GetType() == other.GetType() && GetName() == other.GetName())
    return true;
  else
    return false;
}

void Feature::Update(string type, std::string name)
{
  this->type = type;
  this->name = name;
}
}
