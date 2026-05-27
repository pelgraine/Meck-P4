#pragma once

#include <Arduino.h>

#define MAX_FRAME_SIZE  172

class BaseSerialInterface {
protected:
  BaseSerialInterface() { }

public:
  virtual void enable() = 0;
  virtual void disable() = 0;
  virtual bool isEnabled() const = 0;

  virtual bool isConnected() const = 0;

  virtual bool isWriteBusy() const = 0;
  virtual size_t writeFrame(const uint8_t src[], size_t len) = 0;
  virtual size_t checkRecvFrame(uint8_t dest[]) = 0;

  // Optional: dynamically switch connection speed.
  // Fast mode (true)  = tight interval for bulk sync (contact/channel streaming).
  // Slow mode (false) = relaxed interval for idle/messaging — lower power, more reliable.
  // Default is a no-op; override in transport implementations that support it.
  virtual void setFastMode(bool fast) {}
};
