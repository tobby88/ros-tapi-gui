#include "guidevice.hpp"
#include <QCryptographicHash>
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

GuiDevice::GuiDevice(QWidget* parent, uint8_t type, string name, string uuid, unsigned long lastSeq, ros::Time lastSeen,
                     unsigned long heartbeat, map<string, Feature> features)
  : QWidget(parent), Device(type, name, uuid, lastSeq, lastSeen, heartbeat, features)
{
  connectbox_size = 10;
  header_end = 30;
  footer_height = 10;
  line_height = 20;
  line_start = connectbox_size;
  this->features = GetSortedFeatures();
  items = this->features.size();

  // height
  this->setMinimumHeight(header_end + line_height * items + footer_height);
  this->setMaximumHeight(header_end + line_height * items + footer_height);

  // Prevent uninitialized usage of some member variables
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
  if (GetType() == tapi_lib::Device::Type_Subscriber)
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

// Public member functions
QColor GuiDevice::stringToColor(string messagetype)
{
  QCryptographicHash hasher(QCryptographicHash::Md5);
  hasher.addData(messagetype.c_str());
  QByteArray hash = hasher.result();
  uint8_t red, green, blue;
  red = hash.at(0);
  green = hash.at(1);
  blue = hash.at(2);
  QColor color;
  color = QColor(red, green, blue);
  return color;
}

void GuiDevice::Update(uint8_t type, string name, unsigned long lastSeq, ros::Time lastSeen, unsigned long heartbeat,
                       map<string, Feature> features)
{
  this->features.clear();
  Device::Update(type, name, lastSeq, lastSeen, heartbeat, features);
  this->features = GetSortedFeatures();
  items = this->features.size();
  // height
  this->setMinimumHeight(header_end + line_height * items + footer_height);
  this->setMaximumHeight(header_end + line_height * items + footer_height);
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
  if (Active())
    painter.setBrush(Qt::white);
  else
    painter.setBrush(Qt::gray);
  painter.setPen(Qt::black);

  QSize a(line_width, this->height() - 1);
  painter.drawRoundedRect(QRect(QPoint(line_start, 0), a), 10, 10);

  // print heading
  painter.setFont(QFont("Arial", 11));
  painter.drawText(QRect(QPoint(line_start, 0), QPoint(line_end, header_end)), Qt::AlignCenter,
                   QString::fromStdString(GetName()));

  // Draw Features
  int i = 0;
  for (auto it = features.begin(); it != features.end(); ++it)
  {
    int line_y = header_end + i * line_height;

    QColor color;

    color = stringToColor((*it)->GetType());

    if (!Active())
      color = color.darker();
    painter.setBrush(color);

    painter.drawLine(QPoint(line_start, line_y), QPoint(line_end, line_y));

    // draw connect box
    QRect connect_box(QPoint(0, line_y + line_height / 2 - connectbox_size / 2),
                      QSize(connectbox_size, connectbox_size));

    if (GetType() == tapi_lib::Device::Type_Publisher)
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
