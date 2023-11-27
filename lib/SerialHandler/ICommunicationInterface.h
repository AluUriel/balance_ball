#pragma once

class ICommunicationInterface {
public:
    virtual void begin(int baudRate) = 0;
    virtual size_t write(const String &s) = 0;
    virtual int available() = 0;
    virtual int read() = 0;
    virtual String readStringUntil(char terminator) = 0;
    virtual ~ICommunicationInterface() {}
};
