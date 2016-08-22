#ifndef ASSIGNMENT_H
#define ASSIGNMENT_H

#include <string>

using namespace std;

class Assignment
{
public:
  // Constructor/Destructor
  Assignment(string senderUUID, string senderFeatureUUID, string receiverUUID,
             string receiverFeatureUUID, double coefficient);
  ~Assignment();

  // Public member functions
  double GetCoefficient();
  string GetReceiverFeatureUUID();
  string GetReceiverUUID();
  string GetSenderFeatureUUID();
  string GetSenderUUID();

private:
  // Private member variables
  double coefficient;
  string receiverFeatureUUID;
  string receiverUUID;
  string senderFeatureUUID;
  string senderUUID;
};

#endif // ASSIGNMENT_H
