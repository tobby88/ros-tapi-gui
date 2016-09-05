#ifndef MAINGUI_HPP
#define MAINGUI_HPP

#include <QMainWindow>
#include "apigui.hpp"
#include "ros/node_handle.h"

namespace Ui
{
class MainGui;
}

namespace Tapi
{
class MainGui : public QMainWindow
{
  Q_OBJECT

public:
  // Constructor/Destructor
  MainGui(ros::NodeHandle* nh, QWidget* parent = 0);
  ~MainGui();

private:
  // Private member variables
  Tapi::ApiGui* apiui;
  ros::NodeHandle* nh;
  Ui::MainGui* ui;
};
}

#endif  // MAINGUI_HPP
