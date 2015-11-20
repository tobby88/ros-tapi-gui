#ifndef HELLO_H
#define HELLO_H

#include <ctime>
#include <string>

using namespace std;

class Hello
{
private:
  unsigned long seq;
  time_t timestamp;
  string product;
  string name;
  string uuid;
  unsigned short type;
  //Input features
  unsigned long button_count;
  string *button_names;
  unsigned long axis_count;
  string *axis_names;

public:
  Hello();
  ~Hello();
};

#endif // HELLO_H
