#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RBDdimmer.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define SD_PIN  5
#define PWM_PIN  14
#define ZC_PIN  27
#define ONE_WIRE_BUS 15
#define ONE_WIRE_BUS2 2
#define HEAT_PIN 22

#define PUMP_PIN 22

const int SENSOR_PERIOD = 2000;
const int PID_DT = 1000;
const int PUMP_DT = 100;
const int SAVE_DT = 1000;

WebServer server(80);
StaticJsonDocument<1024> jsonDocument;
dimmerLamp dimmer(PWM_PIN, ZC_PIN);
OneWire oneWire(ONE_WIRE_BUS);
OneWire oneWire2(ONE_WIRE_BUS2);
DallasTemperature sensors(&oneWire);
DallasTemperature sensors2(&oneWire2);

String ssid = "";//"Keenetic-6768";
String password = "";//"factory86!ttl";
const String MODE_IDLE = "idle";
const String MODE_MANUAL = "manual";
const String MODE_AUTO = "auto";

char buffer[1024];
String mode = MODE_IDLE;
float temperature = 0;
float heatTemperature = 0;
float targetTemperature = 0;
int heatLimit = 100;
int pumpLimit = 100;
long timeToEnd = -1;
bool isPumpEnabled = false;
bool isPaused = false;
bool isNeedConfirm = false;
bool isHeatBlock = false;
String confirmMessage = "";

// 0 - Нагрев до температуы внесение солода
// 1 - Ожидание внесение солода
// 2 - Нагрев до следующей температурной паузы
// 3 - Удержание температурной паузы
// 4 - Ожидание фильтрации сусла
// 5 - Нагрев до кипичение
// 6 - Кипичение
// 7 - Завршение программы
int stage = 0;
int step = 0;
unsigned long lastTime;
unsigned long sensorTimer;
unsigned long pidTimer;
unsigned long pumpTimer;
unsigned long saveTimer;

float kp = 0;
float ki = 0;
float kd = 0;
int sensorDiff = 0;
int boilingPoint = 0;
//int dim;

StaticJsonDocument<1024> recipe;
JsonArray hops;
int indexHops = 0;
String message = "";

//hw_timer_t *timer = NULL;

boolean initSdCard();
void initWifi();
void initSettings();
void initState();
void initHttpServer();

void sendError(int code, const char* message);
void sendResponse(int code, const char* message);
void sendOptions();

void getInfo();
void getSettings();

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

void setup() {
  Serial.begin(115200);
  
  dimmer.begin(NORMAL_MODE, OFF);
  // pinMode(ZC_PIN, INPUT_PULLUP);
  // pinMode(PWM_PIN, OUTPUT);
  // attachInterrupt(ZC_PIN, isr, FALLING);
  // timer = timerBegin(0, 80, true);                 //timer 1Mhz resolution
  // timerAttachInterrupt(timer, &timerInterrupt, true);       //attach callback

  if (!initSdCard()) {
    return;
  }

  initSettings();
  initState();
  initWifi();
  initHttpServer();
  
  sensors.begin();
  sensorTimer = millis();
  pidTimer = millis();
  pumpTimer = millis();
  saveTimer = millis();
}

void sendError(int code, const char* message) {
  StaticJsonDocument<128> doc;
  doc["status"] = "error";
  doc["error"] = message;
  char response[128];
  serializeJson(doc, response);
  server.send(code, "application/json", response);
}

void sendResponse(int code, const char* message) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Credentials", "true");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "*");
  server.send(code, "application/json", message);
}

void sendOptions() {
  sendResponse(200, "{}");
}

void initWifi() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("SmartBrewery", "123456789");
  IPAddress Ip(192, 168, 123, 123);    //setto IP Access Point same as gateway
  IPAddress NMask(255, 255, 255, 0);
  WiFi.softAPConfig(Ip, Ip, NMask);

  if (ssid == "") {
    return;
  }

  Serial.println("Connection to ");
  Serial.println(ssid);
  int connectionCount = 5;
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED){
    delay(1000);
    Serial.println(".");
    connectionCount--;
    return;
  } 
  Serial.println("");
  Serial.println("WiFi connected..!");
  Serial.print("Got IP: ");  Serial.println(WiFi.localIP());
}

void initHttpServer() {
  server.on("/v1/info", HTTP_GET, getInfo);
  server.on("/v1/info", HTTP_OPTIONS, sendOptions);
  server.on("/v1/wifi-settings", HTTP_OPTIONS, sendOptions);
  server.on("/v1/wifi-settings", HTTP_POST, setWifiSettings);
  server.on("/v1/heat-limit", HTTP_OPTIONS, sendOptions);
  server.on("/v1/heat-limit", HTTP_POST, setHeatLimit);
  server.on("/v1/pump-limit", HTTP_OPTIONS, sendOptions);
  server.on("/v1/pump-limit", HTTP_POST, setPumpLimit);
  server.on("/v1/target-temperature", HTTP_OPTIONS, sendOptions);
  server.on("/v1/target-temperature", HTTP_POST, setTargetTemperature);
  server.on("/v1/confirme", HTTP_OPTIONS, sendOptions);
  server.on("/v1/confirme", HTTP_GET, setConfirme);
  server.on("/v1/paused", HTTP_OPTIONS, sendOptions);
  server.on("/v1/paused", HTTP_GET, togglePaused);
  server.on("/v1/pump", HTTP_OPTIONS, sendOptions);
  server.on("/v1/pump", HTTP_GET, togglePump);
  server.on("/v1/recipe", HTTP_OPTIONS, sendOptions);
  server.on("/v1/recipe", HTTP_POST, setRecipe);
  server.on("/v1/start", HTTP_OPTIONS, sendOptions);
  server.on("/v1/start", HTTP_POST, setStart);
  server.on("/v1/stop", HTTP_OPTIONS, sendOptions);
  server.on("/v1/stop", HTTP_GET, setStop);
  server.on("/v1/pid", HTTP_POST, setPidSettings);
  server.on("/v1/pid", HTTP_OPTIONS, sendOptions);
  server.on("/v1/sensor-diff", HTTP_POST, setSensorDiff);
  server.on("/v1/sensor-diff", HTTP_OPTIONS, sendOptions);
  server.on("/v1/boiling-point", HTTP_POST, setBoilingPoint);
  server.on("/v1/boiling-point", HTTP_OPTIONS, sendOptions);
  server.on("/v1/settings", HTTP_GET, getSettings);
  server.begin();
  Serial.println("HTTP server started");
}

void initSettings() {
  String json = readFile(SD, "/settings.txt");
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, json);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }

  kp = doc['kp'];
  ki = doc['ki'];
  kd = doc['kd'];
  sensorDiff = doc['sensorDiff'];
  boilingPoint = doc['boilingPoint'];
  ssid = doc['ssid'].as<String>();
  password = doc['password'].as<String>();
}

void initState() {
  String json = readFile(SD, "/state.txt");
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, json);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }

  mode = doc["mode"].as<String>();
  targetTemperature = doc["targetTemperature"];
  heatLimit = doc["heatLimit"];
  pumpLimit = doc["pumpLimit"];
  isPumpEnabled = doc["isPumpEnabled"];
  isPaused = doc["isPaused"];
  isNeedConfirm = doc["isNeedConfirm"];
  confirmMessage = doc["confirmMessage"].as<String>();
  stage = doc["stage"];
  step = doc["step"];
  timeToEnd = doc["timeToEnd"];
  recipe = doc["recipe"];
}

boolean initSdCard() {
  if(!SD.begin(SD_PIN)) {
    Serial.println("Card Mount Failed");
    return false;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return false;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  return true;
}

String readFile(fs::FS &fs, const char * path) {
  String data;
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if(!file){
    Serial.println("Failed to open file for reading");
    return data;
  }

  Serial.print("Read from file: ");
  data = file.readString();
  file.close();
  return data;
}

boolean writeFile(fs::FS &fs, const char * path, const char * message) {
  boolean result = false;
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file){
    Serial.println("Failed to open file for writing");
    return result;
  }
  if (file.print(message)) {
    result = true;
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
  return result;
}

void getInfo() {
  jsonDocument.clear();
  jsonDocument["mode"] = mode;
  jsonDocument["temparature"] = temperature;
  jsonDocument["heatTemperature"] = heatTemperature;
  jsonDocument["targetTemperature"] = targetTemperature;
  jsonDocument["heatLimit"] = heatLimit;
  jsonDocument["pumpLimit"] = pumpLimit;
  jsonDocument["isPumpEnabled"] = isPumpEnabled;
  jsonDocument["isPaused"] = isPaused;
  jsonDocument["isNeedConfirm"] = isNeedConfirm;
  jsonDocument["confirmMessage"] = confirmMessage;
  jsonDocument["stage"] = stage;
  jsonDocument["step"] = step;
  jsonDocument["timeToEnd"] = timeToEnd;
  jsonDocument["recipe"] = recipe;

  serializeJson(jsonDocument, buffer);
  sendResponse(200, buffer);
}

void setWifiSettings() {
  if (server.hasArg("plain") == false) {
    sendError(500, "Bad request");
    return;
  }

  String body = server.arg("plain");
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, body);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    sendError(500, error.c_str());
    return;
  }

  ssid = doc["ssid"].as<String>();
  password = doc["password"].as<String>();
  int connectionCount = 5;
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED){
    delay(1000);
    Serial.println(".");
    connectionCount--;
    if (connectionCount == 0) {
      sendError(500, "Connection failed");
      break;
    }
  } 
  
  doc.clear();
  doc['ip'] = WiFi.localIP();

  serializeJson(doc, buffer);
  sendResponse(200, buffer);
}

void getSettings() {
  StaticJsonDocument<128> doc;
  doc["kp"] = kp;
  doc["ki"] = ki;
  doc["kd"] = kd;
  doc["sensorDiff"] = sensorDiff;
  doc["boilingPoint"] = boilingPoint;

  serializeJson(doc, buffer);
  sendResponse(200, buffer);
}

boolean saveSettings() {
  StaticJsonDocument<128> doc;
  doc["kp"] = kp;
  doc["ki"] = ki;
  doc["kd"] = kd;
  doc["sensorDiff"] = sensorDiff;
  doc["boilingPoint"] = boilingPoint;
  doc["ssid"] = ssid;
  doc["password"] = password;

  serializeJson(doc, buffer);
  return writeFile(SD, "/settings.txt", buffer);
}

void saveState() {
  if (millis() - saveTimer < SAVE_DT) {
    return;
  }
  StaticJsonDocument<1500> doc;
  doc["mode"] = mode;
  doc["targetTemperature"] = targetTemperature;
  doc["heatLimit"] = heatLimit;
  doc["pumpLimit"] = pumpLimit;
  doc["isPumpEnabled"] = isPumpEnabled;
  doc["isPaused"] = isPaused;
  doc["isNeedConfirm"] = isNeedConfirm;
  doc["confirmMessage"] = confirmMessage;
  doc["stage"] = stage;
  doc["step"] = step;
  doc["timeToEnd"] = timeToEnd;
  doc["recipe"] = recipe;
  saveTimer = millis();

  serializeJson(doc, buffer);
  writeFile(SD, "/state.txt", buffer);
}

void setHeatLimit() {
  if (server.hasArg("plain") == false) {
    sendError(500, "Bad request");
    return;
  }
  String body = server.arg("plain");
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, body);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    sendError(502, "Bad request");
    return;
  }
  heatLimit = doc["heatLimit"];
  sendResponse(200, "{}");
}

void setPumpLimit() {
  if (server.hasArg("plain") == false) {
    sendError(400, "Bad request");
    return;
  }
  String body = server.arg("plain");
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, body);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    sendError(400, "Bad request");
    return;
  }

  pumpLimit = doc["pumpLimit"];
  sendResponse(200, "{}");
}

void setPidSettings() 
{
  if (server.hasArg("plain") == false) {
    sendError(500, "Bad request");
    return;
  }

  String body = server.arg("plain");
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, body);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    sendError(400, "Bad request");
    return;
  }

  kp = doc["kp"];
  ki = doc["ki"];
  kd = doc["kd"];

  if (saveSettings()) {
    serializeJson(doc, buffer);
    sendResponse(200, buffer);
  } else {
    sendError(500, "Internal error");
  }
}

void setSensorDiff() 
{
  if (server.hasArg("plain") == false) {
    sendError(500, "Bad request");
    return;
  }

  String body = server.arg("plain");
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, body);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    sendError(400, "Bad request");
    return;
  }

  sensorDiff = doc["sensorDiff"].as<int>();

  if (saveSettings()) {
    serializeJson(doc, buffer);
    sendResponse(200, buffer);
  } else {
    sendError(500, "Internal error");
  }
}

void setBoilingPoint() 
{
  if (server.hasArg("plain") == false) {
    sendError(500, "Bad request");
    return;
  }

  String body = server.arg("plain");
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, body);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    sendError(400, "Bad request");
    return;
  }

  boilingPoint = doc["boilingPoint"];

  if (saveSettings()) {
    serializeJson(doc, buffer);
    sendResponse(200, buffer);
  } else {
    sendError(500, "Internal error");
  }
}

void setTargetTemperature() 
{
  if (server.hasArg("plain") == false) {
    sendError(500, "Bad request");
    return;
  }
  if (mode == MODE_AUTO) {
    sendError(400, "Bad request");
    return;
  }
  String body = server.arg("plain");
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, body);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    sendError(400, "Bad request");
    return;
  }
  if (!doc["targetTemperature"].is<float>()) {
    sendError(400, "Bad request");
    return;
  }
  targetTemperature = doc["targetTemperature"];
  sendResponse(200, "{}");
}

void setConfirme() {
  isNeedConfirm = !isNeedConfirm;
  StaticJsonDocument<64> doc;
  char response[64];
  doc["result"] = isNeedConfirm;
  serializeJson(doc, response);
  sendResponse(200, response);
}

void togglePaused() {
  isPaused = !isPaused;
  StaticJsonDocument<64> doc;
  char response[64];
  doc["result"] = isPaused;
  serializeJson(doc, response);
  sendResponse(200, response);
}

void togglePump() {
  isPumpEnabled = !isPumpEnabled;
  StaticJsonDocument<64> doc;
  char response[64];
  doc["result"] = isPumpEnabled;
  serializeJson(doc, response);
  sendResponse(200, response);
}

void setRecipe() {
  if (server.hasArg("plain") == false) {
    sendError(400, "Bad request");
    return;
  }

  String body = server.arg("plain");
  recipe.clear();
  DeserializationError error = deserializeJson(recipe, body);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    sendError(400, "Неверный формат данных");
    return;
  }

  sendResponse(200, "{}");
}

void setStart() {
  if (server.hasArg("plain") == false) {
    sendError(400, "Bad request");
    return;
  }

  String body = server.arg("plain");
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, body);
  if (!doc["mode"] || (doc["mode"] != MODE_AUTO && doc["mode"] != MODE_MANUAL)) {
    sendError(400, "Bad request");
    return;
  }

  mode = doc["mode"].as<String>();
  sensorDiff = doc["sensorDiff"].as<int>();
  
  if (mode == MODE_AUTO) {
    if (recipe.isNull()) {
      sendError(400, "Bad request");
      return;
    }
    
    step = 0;
    stage = 0;
  }

  sendResponse(200, "{}");
}

void setStop() 
{
  mode = MODE_IDLE;
  step = 0;
  stage = 0;
  timeToEnd = -1;
  recipe.clear();
  sendResponse(200, "{}");
}

void autoProgramm() {
  if (isPaused || mode != MODE_AUTO) {
    return;
  }
  switch (stage) { 
    case 0:
    case 2:
    case 5: 
      targetTemperature = recipe[step]["temperature"].as<float>();//60
      heatLimit = recipe[step]["power"].as<int>();
      if (temperature >= constrain(targetTemperature, 0, boilingPoint)) {//60 60
        if (stage == 0) { 
          isNeedConfirm = true;
          confirmMessage = "Внесите солод";
        }
        timeToEnd = recipe[step]["time"].as<long>() * 1000 * 60;
        lastTime = millis();
        stage++;                                                            
      }
    break;                                                        
    
    case 1:
    case 4:
      if (!isNeedConfirm) {
        confirmMessage = "";
        if (timeToEnd <= 0 && stage != 4) step++;
        lastTime = millis();
        stage++;
      }
    break;

    case 3:
    case 6:
      if (timeToEnd > 0) {
        const int delta = millis() - lastTime;
        timeToEnd -= delta; 
        lastTime = millis();
        if (hops.size() > 0 
          && hops.size() > indexHops 
          && (recipe[step]["time"].as<int>() * 60 * 1000) - (hops[indexHops].as<int>() * 60 * 1000) >= timeToEnd) {
          indexHops++;
          message = "Внесите хмель №" + indexHops;
        }
      } else if (recipe.size() > step + 1) {
        step++;
        targetTemperature = recipe[step]["temperature"].as<float>();
        heatLimit = recipe[step]["power"].as<int>();
        stage = targetTemperature == 100 ? 4 : 2;
        if (stage == 4) {
          isNeedConfirm = true;
          confirmMessage = "Отфильтруйте сусло";
        }
        if (recipe[step]["hops"]) {
          hops = recipe[step]["hops"].as<JsonArray>();
          indexHops = 0;
        }
      } else if (recipe.size() == step + 1) {
        hops.clear();
        indexHops = 0;
        isNeedConfirm = true;
        confirmMessage = "Завершение программы";
        timeToEnd = -1;
        stage++;
      } 
    break;

    case 7:
      if (!isNeedConfirm) {
        mode = MODE_IDLE;
        targetTemperature = 0;
        confirmMessage = "";
        timeToEnd = -1;
        step = 0;
        stage = 0;
      }
    break;

  default:
    mode = MODE_IDLE;
    targetTemperature = 0;
    timeToEnd = -1;
    break;
  }
}

void pidControl() {
  //isHeatBlock = (heatTemperature - targetTemperature > 5) || ()
  if (isHeatBlock || isPaused || mode == MODE_IDLE) {
    if (isHeatBlock && heatTemperature - targetTemperature < 1) {
      isHeatBlock = false;
    }
    analogWrite(HEAT_PIN, 0);
    return;
  }
  if (millis() - pidTimer >= PID_DT) {
    if (heatTemperature - targetTemperature >= sensorDiff) {
      isHeatBlock = true;
      return;
    }
    int maxOut = map(heatLimit, 0, 100, 0, 255);
    float err = targetTemperature - temperature;
    static float integral = 0, prevErr = 0;
    integral += err * PID_DT;
    integral = constrain(integral + (float)err * PID_DT * ki, 0, maxOut);
    float D = (err - prevErr) / PID_DT;
    prevErr = err;
    pidTimer = millis();
    analogWrite(HEAT_PIN, constrain(err * kp + integral + D * kd, 0, maxOut));
  }
}

 void pumpControl() {
  if (!isPumpEnabled || isPaused || mode == MODE_IDLE || pumpLimit == 0) {
    dimmer.setState(OFF);
  } else {
    dimmer.setState(ON);
    dimmer.setPower(map(pumpLimit, 0, 100, 29, 94));
  }
}

// void pumpControl() {
//   if (!isPumpEnabled || isPaused || mode == MODE_IDLE || pumpLimit == 0) {
//     dim = 0;
//   } else {
//     dim = map(pumpLimit, 0, 100, 500, 9300);
//   }
// }

// void IRAM_ATTR timerInterrupt() {
//   digitalWrite(PWM_PIN, 1);
//   timerStop(timer);
// }

// void IRAM_ATTR isr() {
//   static int lastDim;
//   digitalWrite(PWM_PIN, 0);  // выключаем симистор
//   // если значение изменилось, устанавливаем новый период
//   // если нет, то просто перезапускаем со старым
//   if (lastDim != dim) {
//     lastDim = dim;
//     timerAlarmWrite(timer, dim, false);
//     timerAlarmEnable(timer);
//   } else timerRestart(timer);
// }

void readTemperature() {
  if (millis() - sensorTimer >= SENSOR_PERIOD) {
    sensors.requestTemperatures(); 
    heatTemperature = sensors.getTempCByIndex(0);

    sensors2.requestTemperatures(); 
    temperature = sensors2.getTempCByIndex(0);
  }
}

void loop() {
  server.handleClient();
  readTemperature();
  autoProgramm();
  pidControl();
  pumpControl();
  saveState();
}