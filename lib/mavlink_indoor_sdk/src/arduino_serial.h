#include "lowlevel.h"
#include <Arduino.h>

namespace mavlink_indoor_sdk
{
class ArduinoSerial : public LowLevel
{
public:
    ArduinoSerial();
    ArduinoSerial(HardwareSerial *Serial__);

    // void stop();

    int read_message(mavlink_message_t &message);
    int write_message(mavlink_message_t &message);
    int status;
    // void start();
    // void stop();

private:
    HardwareSerial *Serial_;

};
} // namespace mavlink_indoor_sdk