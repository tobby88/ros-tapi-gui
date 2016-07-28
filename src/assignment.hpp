#ifndef ASSIGNMENT_H
#define ASSIGNMENT_H

#include <string>

using namespace std;

class Assignment
{
private:
  string receiverUUID;
  string publisherUUID;
  string publisherFeatureUUID;
  string receiverFeatureUUID;
  double coefficient;

public:
  Assignment(string receiverUUID, string publisherUUID, string publisherFeatureUUID, string receiverFeatureUUID, double coefficient);
  ~Assignment();

};

#endif // ASSIGNMENT_H
