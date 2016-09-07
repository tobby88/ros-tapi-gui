#ifndef GUIDEVICE_H
#define GUIDEVICE_H

#include <QColor>
#include <QMouseEvent>
#include <QWidget>
#include <vector>
#include "tapi_lib/device.hpp"
#include "tapi_lib/feature.hpp"

namespace Tapi
{
class GuiDevice : public QWidget, public Device
{
  Q_OBJECT

public:
  // Constructor/Destructor
  GuiDevice(QWidget* parent, Tapi::Device* device);
  ~GuiDevice();

  // Public member functions
  QPoint FeatureBoxPosition(Tapi::Feature* feature);
  std::vector<Tapi::Feature*> GetFeatures();
  static QColor stringToColor(std::string messagetype);

protected:
  // Protected member functions
  void mouseReleaseEvent(QMouseEvent* event) Q_DECL_OVERRIDE;
  void paintEvent(QPaintEvent*) Q_DECL_OVERRIDE;

private:
  // Private member variables
  int connectbox_size;
  std::vector<Tapi::Feature*> features;
  int footer_height;
  int footer_start;
  int header_end;
  int items;
  int line_end;
  int line_height;
  int line_start;
  int line_width;

signals:
  // Signals without implementation
  void featureClicked(Tapi::GuiDevice* guidevice, Tapi::Feature* feature);
};
}

#endif  // GuiDevice_H
