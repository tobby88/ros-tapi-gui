#ifndef ENUMS_H
#define ENUMS_H

enum class DeviceStatusResponse
{
  OK,
  Error
};

enum class DeviceType
{
  SenderDevice,
  ReceiverDevice,
};

enum class FeatureType
{
  Images,
  Switch,
  Tristate,
  Axis
};

#endif // ENUMS_H
