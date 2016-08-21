#ifndef APIGUI_HPP
#define APIGUI_HPP

#include "api.hpp"
#include "device.hpp"
#include "guidevice.hpp"
#include <QPoint>
#include <QTimer>
#include <QVBoxLayout>
#include <vector>

using namespace std;

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

protected:
  void paintEvent(QPaintEvent*) Q_DECL_OVERRIDE;

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
  Feature* selectedFeature;
  GuiDevice* selectedGuiDevice;
  QPoint mousePosition;
  int timerInterval;

private slots:
  void checkApiForUpdate();
  void featureClicked(GuiDevice* guidevice, Feature* feature);
};

#endif // APIGUI_HPP
