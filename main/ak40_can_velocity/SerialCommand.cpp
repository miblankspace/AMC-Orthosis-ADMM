#include "serial_command.h"
#include <string.h>
#include <stdlib.h>
#include "driver/uart.h"
#include "MotionController.h"

SerialCommand::SerialCommand(MotionController& motion)
    : motion_(motion),
      statusRequested_(false),
      reconnectRequested_(false),
      emergencyStopRequested_(false) {}

void SerialCommand::update()
{
    uint8_t byte;
    // UART0 is the default console UART; assumes it's already installed
    // by ESP-IDF's stdio/console setup (the default in most projects).
    while (uart_read_bytes(UART_NUM_0, &byte, 1, 0) > 0)
    {
        if (byte == '\n' || byte == '\r')
        {
            if (lineLen_ > 0)
            {
                lineBuf_[lineLen_] = '\0';
                handleLine(lineBuf_);
                lineLen_ = 0;
            }
            continue;
        }

        if (lineLen_ < kLineBufSize - 1)
        {
            lineBuf_[lineLen_++] = (char)byte;
        }
        // else: silently drop overflow chars, same effect as truncating
    }
}

bool SerialCommand::consumeStatusRequest()
{
    const bool requested = statusRequested_;
    statusRequested_ = false;
    return requested;
}

bool SerialCommand::consumeReconnectRequest()
{
    const bool requested = reconnectRequested_;
    reconnectRequested_ = false;
    return requested;
}

bool SerialCommand::consumeEmergencyStopRequest()
{
    const bool requested = emergencyStopRequested_;
    emergencyStopRequested_ = false;
    return requested;
}

void SerialCommand::handleLine(const char* line)
{
    if (strcmp(line, "s") == 0)
    {
        motion_.stop();
        return;
    }

    if (strcmp(line, "i") == 0)
    {
        statusRequested_ = true;
        return;
    }

    if (strcmp(line, "h") == 0)
    {
        reconnectRequested_ = true;
        return;
    }

    if (strcmp(line, "e") == 0)
    {
        emergencyStopRequested_ = true;
        return;
    }

    if (line[0] != 'v')
    {
        printf("Use: v <erpm> | s | e | i | h\n");
        return;
    }

    const int32_t erpm = parseValue(line);
    if (erpm > 0)
    {
        motion_.moveUp(erpm);
        return;
    }
    if (erpm < 0)
    {
        motion_.moveDown(-erpm);
        return;
    }

    motion_.stop();
}

int32_t SerialCommand::parseValue(const char* line) const
{
    return (int32_t)strtol(line + 1, nullptr, 10); // skip 'v' prefix
}