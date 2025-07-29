#include <Arduino.h>
#include <WebSocketsServer.h>
#include "FS.h"

#define SD_PIN  5
#define PWM_PIN  14
#define ZC_PIN  27
#define ONE_WIRE_BUS 15
#define ONE_WIRE_BUS2 2
#define HEAT_PIN 22
#define PUMP_PIN 22

boolean initSdCard();
void initWifi();
void initSettings();
void initState();
void initHttpServer();
void initWebSocket();

void onEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void notifyClients();

void sendError(int code, const char* message);
void sendResponse(int code, const char* message);
void sendOptions();

void getInfo();
void getSettings();

void getRecipes();
void getRecipe();
void addRecipe();
void updateRecipe();
void deleteRecipe();

void setHeatLimit();
void setPumpLimit();
void setTargetTemperature();  
void setConfirme();
void setRecipe();
void setStart();
void setStop();
void setPidSettings();
void setSensorDiff();
void setBoilingPoint();
void setWifiSettings();

void togglePaused();
void togglePump();

boolean saveSettings();
void saveState();

String readFile(fs::FS &fs, const char * path);
boolean writeFile(fs::FS &fs, const char * path, const char * message);
void readTemperature();
void autoProgramm();
void pidControl();
void pumpControl();

//void IRAM_ATTR isr();
//void IRAM_ATTR timerInterrupt();

