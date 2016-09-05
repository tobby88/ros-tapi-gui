#include <QApplication>
#include "maingui.hpp"
#include "ros/init.h"
#include "ros/node_handle.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Tapi_GUI");
  ros::NodeHandle nh;
  QApplication a(argc, argv);
  Tapi::MainGui w(&nh);
  w.show();
  return a.exec();
}
