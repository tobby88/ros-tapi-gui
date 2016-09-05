#ifndef APIGUI_HPP
#define APIGUI_HPP

#include <QPoint>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <map>
#include <vector>
#include "api.hpp"
#include "device.hpp"
#include "guidevice.hpp"

namespace Ui
{
class ApiGui;
}

namespace Tapi
{
class ApiGui : public QWidget
{
  Q_OBJECT

public:
  // Constructor/Destructor
  ApiGui(Tapi::Api* api, QWidget* parent = 0);
  ~ApiGui();

protected:
  // Protected member functions
  void paintEvent(QPaintEvent*) Q_DECL_OVERRIDE;

private:
  // Private member variables
  Tapi::Api* api;
  std::map<std::string, QColor> colorKeys;
  QVBoxLayout* layoutReceiver;
  QVBoxLayout* layoutSender;
  QPoint mousePosition;
  std::vector<Tapi::GuiDevice*> receiverGuiDevices;
  Tapi::Feature* selectedFeature;
  Tapi::GuiDevice* selectedGuiDevice;
  std::vector<Tapi::GuiDevice*> senderGuiDevices;
  QTimer* timer;
  int timerInterval;
  Ui::ApiGui* ui;

  // Private member functions
  void addDevice(Tapi::Device* device);
  void addDevice(uint8_t type, std::string name, std::string uuid, std::map<std::string, Tapi::Feature> features);
  bool checkPending();
  void clear();

private slots:
  // Slot functions
  void checkApiForUpdate();
  void clearButtonClicked();
  void featureClicked(Tapi::GuiDevice* guidevice, Tapi::Feature* feature);
  void loadButtonClicked();
  void saveButtonClicked();
};
}

#endif  // APIGUI_HPP
