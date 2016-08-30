#ifndef ASSIGNMENT_H
#define ASSIGNMENT_H

#include <string>

namespace Tapi
{
class Assignment
{
public:
  // Constructor/Destructor
  Assignment(std::string senderUUID, std::string senderFeatureUUID, std::string receiverUUID,
             std::string receiverFeatureUUID, double coefficient);
  ~Assignment();

  // Public member functions
  double GetCoefficient();
  std::string GetReceiverFeatureUUID();
  std::string GetReceiverUUID();
  std::string GetSenderFeatureUUID();
  std::string GetSenderUUID();

private:
  // Private member variables
  double coefficient;
  std::string receiverFeatureUUID;
  std::string receiverUUID;
  std::string senderFeatureUUID;
  std::string senderUUID;
};
}

#endif  // ASSIGNMENT_H
