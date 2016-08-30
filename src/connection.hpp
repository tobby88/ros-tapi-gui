#ifndef CONNECTION_H
#define CONNECTION_H

#include <string>

namespace Tapi
{
class Connection
{
public:
  // Constructor/Destructor
  Connection(std::string senderUUID, std::string senderFeatureUUID, std::string receiverUUID,
             std::string receiverFeatureUUID, double coefficient);
  ~Connection();

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

#endif  // CONNECTION_H
