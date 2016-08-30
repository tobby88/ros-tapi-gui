#include <QApplication>
#include "api.hpp"
#include "maingui.hpp"
#include "ros/ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Tapi");
  ros::NodeHandle nh;
  Tapi::Api api(&nh);
  QApplication a(argc, argv);
  Tapi::MainGui w(&api);
  w.show();
  api.Run();
  return a.exec();
}
