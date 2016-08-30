#ifndef FEATURE_H
#define FEATURE_H

#include <string>

namespace Tapi
{
class Feature
{
public:
  // Constructor/Destructor
  Feature(uint8_t type, std::string name, std::string description, std::string uuid);
  ~Feature();

  // Public member functions
  void DecrementConnections();
  int GetConnectionCount();
  std::string GetDescription() const;
  std::string GetName() const;
  uint8_t GetType() const;
  std::string GetUUID() const;
  void IncrementConnections();
  bool operator==(const Feature &other) const;
  void Update(uint8_t type, std::string name, std::string description);

private:
  // Private member variables
  uint8_t type;
  std::string name;
  std::string description;
  std::string uuid;
  int connections;
};
}

#endif  // FEATURE_H
