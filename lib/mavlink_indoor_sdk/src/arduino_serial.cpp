#include "arduino_serial.h"

namespace mavlink_indoor_sdk
{
ArduinoSerial::ArduinoSerial()
{
    Serial_ = &Serial1;
}
ArduinoSerial::ArduinoSerial(HardwareSerial *Serial__)
{
    Serial_ = Serial__;
}
int ArduinoSerial::read_message(mavlink_message_t &message)
{
    mavlink_message_t msg;

    mavlink_status_t status;
    uint8_t buf[1024];
    while (Serial_->available() < 0)
        ;
    delay(30);
    int n = Serial_->available();
    // Serial1.readBytes()
    // for (int i = 0; (i < 1024) && (Serial1.available() > 0); i++){
    //     buf[i] =
    // }
    Serial_->readBytes(buf, n);

    bool received = false;
    for (int i = 0; i < n; ++i)
    {
        // temp = buf_r[i];
        // printf("%02x ", (unsigned char)temp);
        if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
        {
            // Packet received
            received = true;
            // printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", message.sysid, msg.compid, msg.len, msg.msgid);
        }
    }
    message = msg;
    return n;
}
int ArduinoSerial::write_message(mavlink_message_t &message)
{
    uint8_t buf[300];

    // Translate message to buffer
    unsigned len = mavlink_msg_to_send_buffer(buf, &message);

    // Write buffer to serial port, locks port while writing
    int bytesWritten = Serial_->write(buf, len);
    return len;
}

}; // namespace mavlink_indoor_sdk