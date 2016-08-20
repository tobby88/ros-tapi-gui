#ifndef APIGUI_HPP
#define APIGUI_HPP

#include "api.hpp"
#include "device.hpp"
#include "guidevice.hpp"
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <vector>

namespace Ui
{
class ApiGui;
}

class ApiGui : public QWidget
{
  Q_OBJECT

public:
  ApiGui(Api* api, QWidget* parent = 0);
  ~ApiGui();

private:
  Ui::ApiGui* ui;
  Api* api;
  QTimer* timer;
  void addDevice(Device* device);
  unsigned int temp2;
  QVBoxLayout* layoutSender;
  QVBoxLayout* layoutReceiver;
  vector<GuiDevice*> receiverGuiDevices;
  vector<GuiDevice*> senderGuiDevices;

private slots:
  void checkApiForUpdate();
};

#endif // APIGUI_HPP
