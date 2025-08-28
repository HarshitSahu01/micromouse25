#ifndef WIFILOGGER_H
#define WIFILOGGER_H

#include <Arduino.h>

// Initialize WiFi logger with credentials and target
void initWiFiLogger(const char* ssid, const char* password, const char* targetHost = "py", int port = 12345);

// Log a message via UDP
void log(String message);

// Check if logger is ready
bool isLoggerReady();

#endif