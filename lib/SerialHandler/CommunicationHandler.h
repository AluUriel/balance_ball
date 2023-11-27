#pragma once
#include <Arduino.h>
#include <map>
#include <functional>
#include "ICommunicationInterface.h"

class CommunicationHandler {
public:
    using CallbackFunction = std::function<void(String)>;

    CommunicationHandler(ICommunicationInterface &comm) : comm(comm) {}

    void attachCallback(String key, CallbackFunction callback) {
        callbacks[key] = callback;
    }

    void start() {
        xTaskCreatePinnedToCore(
            CommunicationHandler::taskRead,
            "TaskRead",
            10000,
            this,
            1,
            NULL,
            0);
    }

    void emit(String key, String payload) {
        comm.write(key + "," + payload + "\n");
    }

private:
    ICommunicationInterface &comm;
    std::map<String, CallbackFunction> callbacks;

    static void taskRead(void *parameter) {
        CommunicationHandler *handler = static_cast<CommunicationHandler *>(parameter);
        for (;;) {
            handler->handleInput();
            delay(10);
        }
    }

    void handleInput() {
        if (comm.available() > 0) {
            String line = comm.readStringUntil('\n');
            int separatorIndex = line.indexOf(',');
            if (separatorIndex != -1) {
                String key = line.substring(0, separatorIndex);
                String payload = line.substring(separatorIndex + 1);
                if (callbacks.find(key) != callbacks.end()) {
                    callbacks[key](payload);
                }
            }
        }
    }
};
