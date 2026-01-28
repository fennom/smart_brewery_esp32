#include <WiFi.h>
#include <ArduinoJson.h>
#include <WebServer.h>

#include <RBDdimmer.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "main.h"
#include <uri/UriRegex.h>
#include <WebSocketsServer.h>

const int SENSOR_PERIOD = 1250;
const int SENSOR_DELAY = 2000;
const int PID_DT = 1000;
const int PUMP_DT = 100;
const int SAVE_DT = 60000;
const int WS_NOTIFY_DT = 1000;

WebServer server(80);
WebSocketsServer ws = WebSocketsServer(81);

dimmerLamp dimmer(PWM_PIN, ZC_PIN);
OneWire oneWire(ONE_WIRE_BUS);
OneWire oneWire2(ONE_WIRE_BUS2);
DallasTemperature sensors(&oneWire);
DallasTemperature sensors2(&oneWire2);
boolean sensorsRequested = false;

String ssid = "";
String password = "";
const String MODE_IDLE = "idle";
const String MODE_MANUAL = "manual";
const String MODE_AUTO = "auto";

const char *pathSettings = "/settings.txt";
const char *pathState = "/state.txt";

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
bool isNeedSave = false;
String confirmMessage = "";

// 0 - Нагрев до температуры внесение солода
// 1 - Ожидание внесение солода
// 2 - Нагрев до следующей температурной паузы
// 3 - Удержание температурной паузы
// 4 - Ожидание фильтрации сусла
// 5 - Нагрев до кипячение
// 6 - Кипячение
// 7 - Завершение программы
int stage = 0;
int step = 0;
unsigned long lastTime;
unsigned long sensorTimer;
unsigned long pidTimer;
unsigned long pumpTimer;
unsigned long saveTimer;
unsigned long wsNotifyTimer;

float kp = 0;
float ki = 0;
float kd = 0;
float integral = 0;
float prevErr = 0;
int sensorDiff = 0;
int boilingPoint = 0;

uint16_t test;
//int dim;

StaticJsonDocument<1024> recipe;
JsonArray hops;
int indexHops = 0;
String message = "";

//hw_timer_t *timer = NULL;

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
  initWebSocket();
  
  sensors.begin();
  sensors2.begin();
  sensorTimer = millis();
  pidTimer = millis();
  pumpTimer = millis();
  saveTimer = millis();
  wsNotifyTimer = millis();
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
  int connectionCount = 150;
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && connectionCount > 0){
    delay(1000);
    Serial.println(".");
    connectionCount--;
  } 

  if (connectionCount < 1) {
    ssid = "";
    initWifi();
    return;
  }

  Serial.println("");
  Serial.println("WiFi connected..!");
  Serial.print("Got IP: ");  Serial.println(WiFi.localIP());
}

void initHttpServer() {
  server.on("/v1/info", HTTP_GET, getInfo);
  
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

  server.on("/v1/recipes", HTTP_GET, getRecipes);
  server.on(UriRegex("^\\/v1\\/recipes\\/([0-9]+)$"), HTTP_GET, getRecipe);
  server.on("/v1/recipes", HTTP_POST, addRecipe);
  server.on(UriRegex("^\\/v1\\/recipes\\/([0-9]+)$"), HTTP_PUT, updateRecipe);
  server.on(UriRegex("^\\/v1\\/recipes\\/([0-9]+)$"), HTTP_DELETE, deleteRecipe);

  server.begin();
  Serial.println("HTTP server started");
}

void initWebSocket() {
  ws.begin();
  ws.onEvent(onEvent);

}

void initSettings() {
  File file = SD.open(pathSettings);
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, file);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
  }

  kp = doc["kp"];
  ki = doc["ki"];
  kd = doc["kd"]; 
  sensorDiff = doc["sensorDiff"] | 0;
  boilingPoint = doc["boilingPoint"] | 100;
  ssid = doc["ssid"] | "";
  password = doc["password"] | "";

  file.close();
}

void initState() {
  File file = SD.open(pathState);
  StaticJsonDocument<1500> doc;
  DeserializationError error = deserializeJson(doc, file);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
  }

  mode = doc["mode"] | MODE_IDLE;
  targetTemperature = doc["targetTemperature"] | 0;
  heatLimit = doc["heatLimit"] | 100;
  pumpLimit = doc["pumpLimit"] | 100;
  isPumpEnabled = doc["isPumpEnabled"] | false;
  isPaused = doc["isPaused"] | false;
  isNeedConfirm = doc["isNeedConfirm"] | false;
  confirmMessage = doc["confirmMessage"] | "";
  stage = doc["stage"] | 0;
  step = doc["step"] | 0;
  timeToEnd = doc["timeToEnd"] | -1;
  indexHops = doc["indexHops"] | 0;
  recipe = doc["recipe"];

  file.close();
}

boolean initSdCard() {
  SPI.begin(18, 19, 21, SD_PIN);

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

  if (!SD.mkdir("/recipes")) {
    Serial.print(F("mkdir failed"));
  }
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

void onEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
}

void getInfo() {
  StaticJsonDocument<1024> doc;
  doc["mode"] = mode;
  doc["temperature"] = temperature;
  doc["heatTemperature"] = heatTemperature;
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
  doc["test"] = test;

  char response[1024];
  serializeJson(doc, response);
  sendResponse(200, response);
}

void notifyClients() {
  StaticJsonDocument<1024> doc;
  
  doc["mode"] = mode;
  doc["temperature"] = temperature;
  doc["heatTemperature"] = heatTemperature;
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
  doc["test"] = test;

  String response;
  serializeJson(doc, response);
  ws.broadcastTXT(response);
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
  doc["ip"] = WiFi.localIP();

  if (saveSettings()) {
    char response[128];
    serializeJson(doc, response);
    sendResponse(200, response);
  } else {
    sendError(500, "Internal error");
  }
}

void getSettings() {
  StaticJsonDocument<128> doc;

  doc["kp"] = kp;
  doc["ki"] = ki;
  doc["kd"] = kd;
  doc["sensorDiff"] = sensorDiff;
  doc["boilingPoint"] = boilingPoint;

  char response[128];
  serializeJson(doc, response);
  sendResponse(200, response);
}

void getRecipes() {
  File file;
  if (!SD.exists("/recipes/recipes.txt")) {
    file = SD.open("/recipes/recipes.txt", FILE_WRITE, true);
    if(file.print("[]")) {
      Serial.println("Message write");
    } else {
      Serial.println("Write failed");
    }
    file.close();
  } 

  file = SD.open("/recipes/recipes.txt", FILE_READ);
  StaticJsonDocument<1500> doc;
  DeserializationError error = deserializeJson(doc, file);
  file.close();
  
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    SD.remove("/recipes/recipes.txt");
    sendError(500, error.c_str());
    return;
  }
  
  char response[1500];
  serializeJson(doc, response);
  sendResponse(200, response);
}

void getRecipe() {
  Serial.println("Start getRecipe");
  String id = server.pathArg(0);
  File fileRecipes = SD.open("/recipes/recipes.txt", FILE_READ);
  StaticJsonDocument<1024> recipesDoc;
  DeserializationError error = deserializeJson(recipesDoc, fileRecipes);
  fileRecipes.close();
  Serial.println("recipes ready");

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    sendError(500, error.c_str());
    return;
  }
  Serial.println("find index");
  JsonArray recipes = recipesDoc.as<JsonArray>();
  int index = 0;
  for (JsonVariant value : recipes) {
      JsonObject doc = value.as<JsonObject>();
      if (doc["id"].as<String>() == id) {
        break;
      }
      index++;
  }

  Serial.println("find recipe by id");
  File file = SD.open("/recipes/"+id+".txt");
  StaticJsonDocument<1024> doc;
  error = deserializeJson(doc, file);
  file.close();

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    sendError(500, error.c_str());
    return;
  }

  Serial.println("result");
  StaticJsonDocument<1024> result;
  result["id"] = recipes[index]["id"];
  result["name"] = recipes[index]["name"];
  result["temperaturePauses"] = doc;

  char response[1024];
  serializeJson(result, response);
  sendResponse(200, response);
}

void addRecipe() {
  if (server.hasArg("plain") == false) {
    sendError(500, "Bad request");
    return;
  }

  String body = server.arg("plain");
  StaticJsonDocument<1024> json;
  DeserializationError error = deserializeJson(json, body);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    sendError(500, error.c_str());
    return;
  }

  File fileRecipes = SD.open("/recipes/recipes.txt", FILE_READ);
  StaticJsonDocument<1024> recipesDoc;
  error = deserializeJson(recipesDoc, fileRecipes);
  fileRecipes.close();

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    sendError(500, error.c_str());
    return;
  }
  
  JsonArray recipes = recipesDoc.as<JsonArray>();
  int size = recipes.size();
  String id = String(size + 1);

  StaticJsonDocument<128> recipeInfo;
  recipeInfo["id"] = size + 1;
  recipeInfo["name"] = json["name"];
  recipes.add(recipeInfo);

  File file = SD.open("/recipes/"+id+".txt", FILE_WRITE, true);
  if (serializeJson(json["temperaturePauses"], file) == 0) {
    Serial.println(F("Failed to write to file"));
    file.close();
    sendError(500, "Failed to write to file");
    return;
  }
  file.close();

  fileRecipes = SD.open("/recipes/recipes.txt", FILE_WRITE);
  if (serializeJson(recipes, fileRecipes) == 0) {
    Serial.println(F("Failed to write to file"));
    fileRecipes.close();
    sendError(500, "Failed to write to file");
    return;
  }
  fileRecipes.close();

  char response[128];
  serializeJson(recipeInfo, response);
  sendResponse(200, response);
}

void updateRecipe() {
  if (server.hasArg("plain") == false) {
    sendError(500, "Bad request");
    return;
  }

  String body = server.arg("plain");
  StaticJsonDocument<1024> json;
  DeserializationError error = deserializeJson(json, body);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    sendError(500, error.c_str());
    return;
  }

  File fileRecipes = SD.open("/recipes/recipes.txt", FILE_READ, true);
  StaticJsonDocument<1024> recipesDoc;
  error = deserializeJson(recipesDoc, fileRecipes);
  fileRecipes.close();
  JsonArray recipes = recipesDoc.as<JsonArray>();
  String id = server.pathArg(0);

  for (JsonVariant value : recipes) {
      JsonObject d = value.as<JsonObject>();
      if (d["id"].as<String>() == id) {
        json["id"] = id;
        d["name"] = json["name"];
        break;
      }
  }

  File file = SD.open("/recipes/"+id+".txt", FILE_WRITE);
  if (serializeJson(json["temperaturePauses"], file) == 0) {
    Serial.println(F("Failed to write to file"));
    file.close();
    sendError(500, "Failed to write to file");
    return;
  }
  file.close();

  fileRecipes = SD.open("/recipes/recipes.txt", FILE_WRITE, true);
  if (serializeJson(recipes, fileRecipes) == 0) {
    Serial.println(F("Failed to write to file"));
    fileRecipes.close();
    sendError(500, "Failed to write to file");
    return;
  }
  fileRecipes.close();

  char response[1024];
  serializeJson(json, response);
  sendResponse(200, response);
}

void deleteRecipe() {
  StaticJsonDocument<1024> doc;

  File fileRecipes = SD.open("/recipes/recipes.txt", FILE_READ);
  StaticJsonDocument<1500> recipesDoc;
  DeserializationError error = deserializeJson(recipesDoc, fileRecipes);
  fileRecipes.close();

  JsonArray recipes = recipesDoc.as<JsonArray>();
  String id = server.pathArg(0);

  int index = 0;
  for (JsonVariant value : recipes) {
      JsonObject doc = value.as<JsonObject>();
      if (doc["id"].as<String>() == id) {
        recipes.remove(index);
        break;
      }
      index++;
  }

  bool isDeleted = SD.remove("/recipes/"+id+".txt");
  fileRecipes = SD.open("/recipes/recipes.txt", FILE_WRITE);
  if (!isDeleted || serializeJson(recipes, fileRecipes) == 0) {
    Serial.println(F("Failed to write to file"));
    fileRecipes.close();
    sendError(500, "Failed to write to file");
    return;
  }
  fileRecipes.close();

  char response[1024];
  serializeJson(doc, response);
  sendResponse(200, response);
}

boolean saveSettings() {
  SD.remove(pathSettings);

  File file = SD.open(pathSettings, FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to create file"));
    return false;
  }

  StaticJsonDocument<256> doc;
  doc["kp"] = kp;
  doc["ki"] = ki;
  doc["kd"] = kd;
  doc["sensorDiff"] = sensorDiff;
  doc["boilingPoint"] = boilingPoint;
  doc["ssid"] = ssid;
  doc["password"] = password;

  if (serializeJson(doc, file) == 0) {
    Serial.println(F("Failed to write to file"));
    file.close();
    return false;
  }
  
  file.close();
  return true;
}

void saveState() {
  if (!isNeedSave && millis() - saveTimer < SAVE_DT) {
    return;
  }
  SD.remove(pathState);

  File file = SD.open(pathState, FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to create file"));
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
  doc["indexHops"] = indexHops;
  doc["recipe"] = recipe;
  saveTimer = millis();

  if (serializeJson(doc, file) == 0) {
    Serial.println(F("Failed to write to file"));
    file.close();
  }
  
  file.close();
  isNeedSave = false;
}

void setHeatLimit() {
  if (server.hasArg("plain") == false) {
    sendError(500, "Bad request");
    return;
  }

  String body = server.arg("plain");
  StaticJsonDocument<128> json;
  DeserializationError error = deserializeJson(json, body);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    sendError(500, error.c_str());
    return;
  }

  isNeedSave = true;
  heatLimit = json["heatLimit"];
  sendResponse(200, "{}");
}

void setPumpLimit() {
  if (server.hasArg("plain") == false) {
    sendError(500, "Bad request");
    return;
  }

  String body = server.arg("plain");
  StaticJsonDocument<128> json;
  DeserializationError error = deserializeJson(json, body);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    sendError(500, error.c_str());
    return;
  }

  isNeedSave = true;
  pumpLimit = json["pumpLimit"];
  sendResponse(200, "{}");
}

void setPidSettings() {
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
    char response[128];
    serializeJson(doc, response);
    sendResponse(200, response);
  } else {
    sendError(500, "Internal error");
  }
}

void setSensorDiff() {
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
    char response[128];
    serializeJson(doc, response);
    sendResponse(200, response);
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
    char response[128];
    serializeJson(doc, response);
    sendResponse(200, response);
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

  isNeedSave = true;
  targetTemperature = doc["targetTemperature"];
  sendResponse(200, "{}");
}

void setConfirme() {
  isNeedConfirm = !isNeedConfirm;
  StaticJsonDocument<64> doc;
  char response[64];
  isNeedSave = true;
  doc["result"] = isNeedConfirm;
  serializeJson(doc, response);
  sendResponse(200, response);
}

void togglePaused() {
  isPaused = !isPaused;
  StaticJsonDocument<64> doc;
  char response[64];
  isNeedSave = true;
  doc["result"] = isPaused;
  serializeJson(doc, response);
  sendResponse(200, response);
}

void togglePump() {
  isPumpEnabled = !isPumpEnabled;
  StaticJsonDocument<64> doc;
  char response[64];
  isNeedSave = true;
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

  isNeedSave = true;
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
  
  if (mode == MODE_AUTO) {
    if (recipe.isNull()) {
      sendError(400, "Bad request");
      return;
    }
    
    step = 0;
    stage = 0;
  }

  isNeedSave = true;
  sendResponse(200, "{}");
}

void setStop() 
{
  mode = MODE_IDLE;
  step = 0;
  stage = 0;
  timeToEnd = -1;
  recipe.clear();

  isNeedSave = true;
  sendResponse(200, "{}");
}

void autoProgramm() {
  if (isPaused || mode != MODE_AUTO) {
    return;
  }
  float currentTemperature;

  switch (stage) { 
    case 0:
    case 2:
    case 5: 
      targetTemperature = recipe[step]["temperature"].as<float>();//60
      heatLimit = recipe[step]["power"].as<int>();
      if (temperature >= constrain(targetTemperature, 0, boilingPoint)) {//60 60
        if (stage == 0) { 
          isNeedConfirm = true;
          confirmMessage = "confirm.addMalt";
        }
        timeToEnd = recipe[step]["time"].as<long>() * 1000 * 60;
        lastTime = millis();
        stage++;     
        isNeedSave = true;                                                       
      }
    break;                                                        
    
    case 1:
    case 4:
      if (!isNeedConfirm) {
        confirmMessage = "";
        if (timeToEnd <= 0 && stage != 4) step++;
        lastTime = millis();
        stage++;
        isNeedSave = true;
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
          && ((recipe[step]["time"].as<int>() - hops[indexHops].as<int>()) * 60 * 1000) >= timeToEnd) {
          message = "confirm.addHops";
          indexHops++;
          isNeedConfirm = true;
          isNeedSave = true;
        }
      } else if (recipe.size() > step + 1) {
        step++;
        stage = recipe[step]["temperature"].as<float>() == 100 ? 4 : 2;
        if (stage == 4) {
          isNeedConfirm = true;
          confirmMessage = "confirm.extractSpentGrains";
        }
        if (recipe[step]["hops"]) {
          hops = recipe[step]["hops"].as<JsonArray>();
          indexHops = 0;
        }
        isNeedSave = true;
      } else if (recipe.size() == step + 1) {
        hops.clear();
        targetTemperature = 0;
        indexHops = 0;
        isNeedConfirm = true;
        confirmMessage = "confirm.finish";
        timeToEnd = -1;
        stage++;
        isNeedSave = true;
      } 
    break;

    case 7:
      if (!isNeedConfirm) {
        mode = MODE_IDLE;
        confirmMessage = "";
        step = 0;
        stage = 0;
        isNeedSave = true;
      }
    break;

  default:
    mode = MODE_IDLE;
    targetTemperature = 0;
    timeToEnd = -1;
    isNeedSave = true;
    break;
  }
}

void pidControl() {
  if (isPaused || mode == MODE_IDLE || (sensorDiff > 0 && heatTemperature - temperature >= sensorDiff)) {
    analogWrite(HEAT_PIN, 0);
    return;
  }

  if (millis() - pidTimer >= PID_DT) {
    int maxOut = map(heatLimit, 0, 100, 0, 255);
    float err = targetTemperature - temperature;
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
    if (!sensorsRequested) {
      sensors.requestTemperatures(); 
      sensors2.requestTemperatures(); 
      sensorsRequested = true;
    }
    
    if (millis() - sensorTimer >= SENSOR_DELAY) {
      heatTemperature = sensors.getTempCByIndex(0);
      temperature = sensors2.getTempCByIndex(0);
      sensorTimer = millis();
      sensorsRequested = false;
    }
  }
}

void loop() {
  server.handleClient();
  ws.loop();
  if (millis() - wsNotifyTimer >= WS_NOTIFY_DT) {
    notifyClients();
    wsNotifyTimer = millis();
  }
  readTemperature();
  autoProgramm();
  pidControl();
  pumpControl();
  saveState();
}
