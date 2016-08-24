#ifndef FEATURE_H
#define FEATURE_H

#include <string>

class Feature
{
public:
  // Constructor/Destructor
  Feature(uint8_t type, std::string name, std::string description, std::string uuid);
  ~Feature();

  // Public member functions
  void DecrementConnections();
  int GetConnectionCount();
  std::string GetDescription();
  std::string GetName() const;
  uint8_t GetType();
  std::string GetUUID();
  void IncrementConnections();

private:
  // Private member variables
  uint8_t type;
  std::string name;
  std::string description;
  std::string uuid;
  int connections;
};

#endif  // FEATURE_H
