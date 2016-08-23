#ifndef FEATURE_H
#define FEATURE_H

#include <string>

using namespace std;

class Feature
{
public:
  // Constructor/Destructor
  Feature(uint8_t type, string name, string description, string uuid);
  ~Feature();

  // Public member functions
  void DecrementConnections();
  int GetConnectionCount();
  string GetDescription();
  string GetName() const;
  uint8_t GetType();
  string GetUUID();
  void IncrementConnections();

private:
  // Private member variables
  uint8_t type;
  string name;
  string description;
  string uuid;
  int connections;
};

#endif // FEATURE_H
