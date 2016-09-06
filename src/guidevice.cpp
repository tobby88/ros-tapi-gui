#include "guidevice.hpp"
#include <QFont>
#include <QPainter>
#include <QPoint>
#include <QRect>
#include <QSize>
#include <QString>
#include "tapi_lib/Device.h"
#include "tapi_lib/HelloRequest.h"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

GuiDevice::GuiDevice(QWidget* parent, Tapi::Device* device)
  : QWidget(parent), device(device), features(device->GetSortedFeatures())
{
  connectbox_size = 10;
  header_end = 30;
  footer_height = 10;
  line_height = 20;
  line_start = connectbox_size;
  items = features.size();

  // min height
  this->setMinimumHeight(header_end + line_height * items + footer_height);
  this->setMaximumHeight(header_end + line_height * items + footer_height);

  // Prevent uninitialized usage
  footer_start = 0;
  line_end = 0;
  line_width = 0;
}

GuiDevice::~GuiDevice()
{
}

// Public member functions

QPoint GuiDevice::FeatureBoxPosition(Tapi::Feature* feature)
{
  int x, y, i;
  y = 0;
  if (device->GetType() == tapi_lib::Device::Type_Subscriber)
    x = 0;
  else
    x = this->width() - 1;
  i = 0;
  for (auto it = features.begin(); it != features.end(); ++it)
  {
    if (*it == feature)
      y = header_end + i * line_height + line_height / 2;
    i++;
  }
  return QPoint(x, y);
}

Tapi::Device* GuiDevice::GetDevice()
{
  return device;
}

vector<Tapi::Feature*> GuiDevice::GetFeatures()
{
  return features;
}

// Public member functions
QColor GuiDevice::stringToColor(string messagetype)
{
  char array[1024];
  strncpy(array, messagetype.c_str(), sizeof(array));
  array[sizeof(array) - 1] = 0;
  unsigned long sum = 0;
  for (int i = 0; i < messagetype.length(); i++)
    sum += array[i] * i;
  sum *= 7;
  uint8_t red, green, blue;
  red = sum % 255;
  green = sum / 255;
  blue = (sum >> 6) % 255;
  QColor color;
  color = QColor(red, green, blue);
  return color;
}

// Protected member functions

void GuiDevice::mouseReleaseEvent(QMouseEvent* event)
{
  int i = 0;
  QPoint p = event->pos();
  for (auto it = features.begin(); it != features.end(); ++it)
  {
    int y = header_end + i * line_height;

    if (p.y() >= y && p.y() < y + line_height)
    {
      emit featureClicked(this, *it);
    }
    i++;
  }
}

void GuiDevice::paintEvent(QPaintEvent*)
{
  QPainter painter(this);
  footer_start = this->height() - footer_height;
  line_width = this->width() - 2 * connectbox_size - 1;
  line_end = line_start + line_width;

  // draw main box
  if (device->Active())
    painter.setBrush(Qt::white);
  else
    painter.setBrush(Qt::gray);
  painter.setPen(Qt::black);

  QSize a(line_width, this->height() - 1);
  painter.drawRoundedRect(QRect(QPoint(line_start, 0), a), 10, 10);

  // print heading
  painter.setFont(QFont("Arial", 11));
  painter.drawText(QRect(QPoint(line_start, 0), QPoint(line_end, header_end)), Qt::AlignCenter,
                   QString::fromStdString(device->GetName()));

  // Draw Features
  int i = 0;
  for (auto it = features.begin(); it != features.end(); ++it)
  {
    int line_y = header_end + i * line_height;

    QColor color;

    color = stringToColor((*it)->GetType());

    if (!device->Active())
      color = color.darker();
    painter.setBrush(color);

    painter.drawLine(QPoint(line_start, line_y), QPoint(line_end, line_y));

    // draw connect box
    QRect connect_box(QPoint(0, line_y + line_height / 2 - connectbox_size / 2),
                      QSize(connectbox_size, connectbox_size));

    if (device->GetType() == tapi_lib::Device::Type_Publisher)
    {
      connect_box.moveLeft(line_end);
    }
    painter.drawRect(connect_box);

    painter.setFont(QFont("Arial", 10));
    painter.drawText(QRect(QPoint(line_start, line_y), QPoint(line_end, line_y + line_height)), Qt::AlignCenter,
                     QString::fromStdString((*it)->GetName()));
    i++;
  }

  // footer line
  painter.drawLine(QPoint(line_start, footer_start), QPoint(line_end, footer_start));
}
}
