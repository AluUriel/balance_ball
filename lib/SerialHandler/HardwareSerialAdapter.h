#pragma once
#include <Arduino.h>
#include "ICommunicationInterface.h"

class HardwareSerialAdapter : public ICommunicationInterface {
public:
    void begin(int baudRate) override {
        Serial.begin(baudRate);
    }

    size_t write(const String &s) override {
        return Serial.println(s);
    }

    int available() override {
        return Serial.available();
    }

    int read() override {
        return Serial.read();
    }

    String readStringUntil(char terminator) override {
        return Serial.readStringUntil(terminator);
    }
};
