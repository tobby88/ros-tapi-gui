#ifndef GUIDEVICE_H
#define GUIDEVICE_H

#include "device.hpp"
#include "feature.hpp"
#include <QMouseEvent>
#include <QWidget>
#include <vector>

using namespace std;

class GuiDevice : public QWidget
{
  Q_OBJECT

public:
  // Constructor/Destructor
  GuiDevice(QWidget* parent, Device* device);

  // Public member variables
  Device* DevicePointer;            // TODO: getDevice() & private
  vector<Feature*> Features; // TODO: private

  // Public member functions
  QPoint FeatureBoxPosition(Feature* feature);

protected:
  // Protected member functions
  void mouseReleaseEvent(QMouseEvent* event) Q_DECL_OVERRIDE;
  void paintEvent(QPaintEvent*) Q_DECL_OVERRIDE;

private:
  // Private member variables
  int connectbox_size;
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
  void featureClicked(GuiDevice* guidevice, Feature* feature);
};

#endif // GuiDevice_H
