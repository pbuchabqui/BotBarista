/*
 * BOT BARISTA v1.7.2 - PID Time-Proportioning + Alexa + UI Sync
 * * Baseado na v1.7.1
 * Novidade: A interface Web agora exibe e permite editar diretamente os 
 * parâmetros de PID aprendidos pelo Autotune na aba "CONFIG".
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <max6675.h>
#include <QuickPID.h>

// ==================== SINRIC PRO ====================
#include <SinricPro.h>
#include <SinricProSwitch.h>

#define FIRMWARE_VERSION "1.7.2"
#define DEBUG_STATS true      

// ==================== CONFIGURAÇÃO SINRIC PRO ====================
namespace SinricConfig {
  const char* APP_KEY    = "c440728d-ad27-4c86-8858-7e9d3a9efbfb";
  const char* APP_SECRET = "0b4e9a78-334d-4233-bbb7-ccfc320f6e62-2867f28c-fb01-4a5c-9d38-7671dfc57c2a";    
  const char* DEVICE_ID  = "696a5a5fdc90b2c9e07744eb";       
}

namespace Pins {
  constexpr uint8_t TEMP_CLK=18, TEMP_CS=19, TEMP_SO=23;
  constexpr uint8_t HEATER=25, PUMP=16, BUTTON=21;
  constexpr uint8_t BUZZER=13, LED_R=32, LED_G=33, LED_B=15;
}

namespace NetConfig {
  const char* WIFI_SSID     = "VIVOFIBRA-414";             
  const char* WIFI_PASSWORD = "juliapedro";
  const char* HOSTNAME      = "botbarista";
  const char* OTA_PASSWORD  = "cafe1234";
  const char* AP_SSID       = "BotBarista";
  const char* AP_PASSWORD   = "cafe1234";
}

// Timing e limites
constexpr uint16_t TEMP_READ_MS    = 250;
constexpr uint16_t DEBOUNCE_MS     = 50;
constexpr uint16_t WIFI_TIMEOUT_MS = 10000;
constexpr uint16_t WIFI_CHECK_MS   = 5000;
constexpr float TEMP_MIN=0, TEMP_MAX=120;
constexpr float TEMP_IDLE_MIN=50, TEMP_IDLE_MAX=85;
constexpr float TEMP_TARGET_MIN=70, TEMP_TARGET_MAX=110;
constexpr bool RGB_COMMON_CATHODE = true;
constexpr uint8_t LED_BRIGHTNESS = 20;

// Segurança
constexpr float TEMP_CRITICAL = 115.0f;
constexpr uint32_t HEATING_TIMEOUT_MS = 10 * 60 * 1000;

enum class State : uint8_t { IDLE, HEATING, BREWING, ERROR };
State currentState = State::IDLE;

struct Config {
  float targetTemp = 90.0f;
  float idleTemp = 75.0f;
  uint32_t brewTimeMs = 30000;
  float kp = 3.0f;
  float ki = 0.01f;
  float kd = 55.0f;
  uint32_t pidWindowMs = 1000;
  void validate() {
    targetTemp   = constrain(targetTemp,   TEMP_TARGET_MIN, TEMP_TARGET_MAX);
    idleTemp     = constrain(idleTemp,     TEMP_IDLE_MIN,   TEMP_IDLE_MAX);
    brewTimeMs   = constrain(brewTimeMs,   10000UL,         60000UL);
    kp           = constrain(kp,           1.0f,            10.0f);
    ki           = constrain(ki,           0.0f,            1.0f);
    kd           = constrain(kd,           30.0f,            100.0f);
    pidWindowMs  = constrain(pidWindowMs,  1000UL,          30000UL);
  }
} cfg;

struct Runtime {
  float currentTemp = 0.0f;
  float filteredTemp = 0.0f;
  float activeSetpoint = 0.0f;
  uint8_t heaterPower = 0;
  bool heaterOn = false;
  bool pumpOn = false;
  bool brewRequested = false;
  bool wifiConnected = false;
  bool apMode = false;
  uint32_t brewStart = 0;
  uint32_t lastTemp = 0;
  uint32_t lastWifi = 0;
  uint32_t heatingStart = 0;
  uint32_t lastStats = 0;
} rt;

struct AutotuneData {
  float maxOvershoot = 0.0f;
  float minTemperature = 100.0f;
  float maxTemperature = 0.0f;
  float avgError = 0.0f;
  uint32_t stableTime = 0;
  uint32_t brewCount = 0;
  bool tuningComplete = false;
  // Parâmetros otimizados
  float optimizedKp = 2.0f;
  float optimizedKi = 0.01f;
  float optimizedKd = 55.0f;
  float optimizedCompensation = 0.5f;
} at;

// ==================== UTILITÁRIOS ====================
bool timeElapsed(uint32_t startTime, uint32_t interval) {
  return (millis() - startTime) >= interval;
}

// ==================== KALMAN FILTER ====================
class KalmanFilter {
  float estimate = 25.0f;
  float errorEst = 8.0f;
public:
  float update(float z) {
    errorEst += 0.015f;
    float K = errorEst / (errorEst + 0.4f);
    estimate += K * (z - estimate);
    errorEst *= (1.0f - K);
    return estimate;
  }
  void reset(float v = 25.0f) { estimate = v; errorEst = 8.0f; }
} kalman;

// ==================== PID ====================
float pidInput = 0.0f;
float pidOutput = 0.0f;
float pidSetpoint = 0.0f;

QuickPID myPID(&pidInput, &pidOutput, &pidSetpoint);
uint32_t windowStartTime = 0;

// ==================== PERIFÉRICOS ====================
class Beeper {
  uint32_t start = 0;
  uint16_t duration = 0;
  bool active = false;
public:
  void play(uint16_t ms) {
    if (!active) {
      digitalWrite(Pins::BUZZER, HIGH);
      start = millis();
      duration = ms;
      active = true;
    }
  }
  void update() {
    if (active && timeElapsed(start, duration)) {
      digitalWrite(Pins::BUZZER, LOW);
      active = false;
    }
  }
  void ok() { play(50); }
  void warn() { play(200); }
} beep;

class Button {
  uint32_t lastChange = 0;
  bool lastRead = HIGH;
  bool stableState = HIGH;
public:
  bool pressed() {
    bool r = digitalRead(Pins::BUTTON);
    if (r != lastRead) lastChange = millis();
    lastRead = r;
    if (timeElapsed(lastChange, DEBOUNCE_MS) && r != stableState) {
      stableState = r;
      return stableState == LOW;
    }
    return false;
  }
} button;

// ==================== GLOBAIS ====================
MAX6675 thermocouple(Pins::TEMP_CLK, Pins::TEMP_CS, Pins::TEMP_SO);
WebServer server(80);
Preferences prefs;

// ==================== DECLARAÇÕES FORWARD ====================
void emergencyStop();
void collectBrewingData();
void optimizeParameters();
void finalizeBrewingData();
void saveAutotuneParams();

// ==================== SINRIC PRO CALLBACKS ====================
bool onPowerState(const String &deviceId, bool &state) {
  Serial.printf("[ALEXA] Comando: %s\n", state ? "LIGAR" : "DESLIGAR");
  if (state) {
    if (currentState == State::IDLE) {
      rt.brewRequested = true;
      beep.ok();
      Serial.println("[ALEXA] Brew iniciado");
    } else if (currentState == State::ERROR) {
      currentState = State::IDLE;
      rt.brewRequested = true;
      beep.ok();
    }
  } else {
    if (currentState == State::HEATING || currentState == State::BREWING) {
      emergencyStop();
      beep.warn();
      Serial.println("[ALEXA] Processo parado");
    }
  }
  return true;
}

void setupSinricPro() {
  if (String(SinricConfig::APP_KEY) == "SUA_APP_KEY_AQUI") {
    Serial.println("[SINRIC] Credenciais não configuradas!");
    return;
  }
  
  SinricProSwitch &mySwitch = SinricPro[SinricConfig::DEVICE_ID];
  mySwitch.onPowerState(onPowerState);
  
  SinricPro.onConnected([]() { Serial.println("[SINRIC] Conectado!"); });
  SinricPro.onDisconnected([]() { Serial.println("[SINRIC] Desconectado"); });
  SinricPro.begin(SinricConfig::APP_KEY, SinricConfig::APP_SECRET);
  
  Serial.println("[SINRIC] Inicializado");
}

void updateSinricState() {
  static State lastReportedState = State::IDLE;
  static uint32_t lastUpdate = 0;
  if (!timeElapsed(lastUpdate, 2000)) return;
  lastUpdate = millis();
  
  bool isActive = (currentState == State::HEATING || currentState == State::BREWING);
  bool wasActive = (lastReportedState == State::HEATING || lastReportedState == State::BREWING);
  if (isActive != wasActive) {
    SinricProSwitch &mySwitch = SinricPro[SinricConfig::DEVICE_ID];
    mySwitch.sendPowerStateEvent(isActive);
    lastReportedState = currentState;
  }
}

// ==================== LED ====================
void setLED(uint8_t r, uint8_t g, uint8_t b) {
  r = (r * LED_BRIGHTNESS) >> 8;
  g = (g * LED_BRIGHTNESS) >> 8;
  b = (b * LED_BRIGHTNESS) >> 8;
  if constexpr (!RGB_COMMON_CATHODE) { r = 255-r; g = 255-g; b = 255-b; }
  analogWrite(Pins::LED_R, r);
  analogWrite(Pins::LED_G, g);
  analogWrite(Pins::LED_B, b);
}

void updateLED() {
  static uint32_t lastBlink = 0;
  static bool blinkState = false;
  switch (currentState) {
    case State::IDLE: setLED(0, 255, 0); break;
    case State::HEATING:
      if (timeElapsed(lastBlink, 300)) { blinkState = !blinkState; lastBlink = millis(); }
      setLED(blinkState ? 255 : 0, 0, 0); break;
    case State::BREWING: setLED(255, 0, 0); break;
    case State::ERROR:
      if (timeElapsed(lastBlink, 100)) { blinkState = !blinkState; lastBlink = millis(); }
      setLED(blinkState ? 255 : 0, 0, 0); break;
  }
}

// ==================== ATUADORES ====================
void setHeater(bool on) {
  if (on != rt.heaterOn) {
    rt.heaterOn = on;
    digitalWrite(Pins::HEATER, on ? HIGH : LOW);
    #if DEBUG_STATS
    Serial.printf("[SSR] %s\n", on ? "ON" : "OFF");
    #endif
  }
}

void setPump(bool on) {
  if (on != rt.pumpOn) {
    rt.pumpOn = on;
    digitalWrite(Pins::PUMP, on ? HIGH : LOW);
    #if DEBUG_STATS
    Serial.printf("[PUMP] %s\n", on ? "ON" : "OFF");
    #endif
  }
}

void emergencyStop() {
  setHeater(false);
  setPump(false);
  rt.brewRequested = false;
  rt.brewStart = 0;
  rt.heatingStart = 0;
  currentState = State::IDLE;
  windowStartTime = millis();
  Serial.println("[EMERGENCY] Sistema parado");
}

// ==================== SENSOR + SEGURANÇA ====================
bool readTemperature() {
  static float lastValid = 25.0f;
  static int errorCount = 0;
  
  float raw = thermocouple.readCelsius();
  
  if (isnan(raw) || raw < TEMP_MIN || raw > TEMP_MAX) {
    errorCount++;
    if (errorCount > 5) { Serial.println("[ERROR] Sensor falhou múltiplas vezes!"); return false; }
    rt.currentTemp = lastValid;
  } else {
    errorCount = 0; lastValid = raw; rt.currentTemp = raw;
  }
  
  rt.filteredTemp = kalman.update(rt.currentTemp);
  if (rt.filteredTemp > TEMP_CRITICAL) {
    if (currentState != State::ERROR) {
      Serial.printf("[SAFETY] Sobre-temperatura! (%.1f°C)\n", rt.filteredTemp);
      currentState = State::ERROR;
      emergencyStop();
    }
    return false;
  }
  return true;
}

// ==================== CONTROLE PID ====================
void updatePIDWindow() {
  uint32_t now = millis();
  uint32_t currentWindow = cfg.pidWindowMs;
  if (timeElapsed(windowStartTime, currentWindow)) { windowStartTime = now; }
}

void controlTemperature(float target) {
  rt.activeSetpoint = target;
  pidSetpoint = target;
  pidInput = rt.filteredTemp;

  if (currentState == State::BREWING) {
    myPID.SetTunings(cfg.kp * 2.0f, cfg.ki * 1.2f, cfg.kd * 1.5f);
  } else {
    myPID.SetTunings(cfg.kp, cfg.ki, cfg.kd);
  }

  myPID.Compute();
  updatePIDWindow();

  uint32_t now = millis();
  uint32_t onTime = (uint32_t)((pidOutput * cfg.pidWindowMs) / 100.0f);
  uint32_t elapsed = now - windowStartTime;
  bool shouldBeOn = (elapsed < onTime);
  rt.heaterPower = (uint8_t)round(pidOutput);
  setHeater(shouldBeOn);
}

// ==================== HANDLERS DE ESTADO ====================
void startHeating() {
  Serial.printf("[STATE] IDLE->HEATING (%.0f->%.0f)\n", cfg.idleTemp, cfg.targetTemp);
  currentState = State::HEATING;
  rt.heatingStart = millis();
  windowStartTime = millis();
  beep.ok();
}

void handleIdle() {
  controlTemperature(cfg.idleTemp);
  if (rt.brewRequested) startHeating();
}

void handleHeating() {
  controlTemperature(cfg.targetTemp);
  if (timeElapsed(rt.heatingStart, HEATING_TIMEOUT_MS)) {
    Serial.println("[SAFETY] Heating timeout!");
    currentState = State::ERROR; emergencyStop(); return;
  }
  
  if (!rt.brewRequested) {
    Serial.println(F("[STATE] HEATING->IDLE (cancel)"));
    currentState = State::IDLE; return;
  }
  
  if (rt.filteredTemp >= (cfg.targetTemp - 1.5f)) {
    float heatingTime = (millis() - rt.heatingStart) / 1000.0f;
    Serial.printf("[STATE] HEATING->BREWING (%.1fs)\n", heatingTime);
    currentState = State::BREWING;
    rt.brewStart = millis();
    setPump(true); windowStartTime = millis(); beep.ok();
  }
}

void handleBrewing() {
  collectBrewingData();
  float timeInBrew = (millis() - rt.brewStart) / 1000.0f;
  float compensation = 0.0f;
  
  if (at.tuningComplete) {
    compensation = at.optimizedCompensation;
    if (timeInBrew < 15.0f) compensation *= 1.5f; 
  } else {
    compensation = (timeInBrew < 15.0f) ? 3.5f : 1.5f; 
  }
  
  static float lastTemp = 0.0f;
  static uint32_t lastTempTime = 0;
  float tempRate = 0.0f;
  
  if (lastTempTime > 0) {
    uint32_t timeDiff = millis() - lastTempTime;
    if (timeDiff > 0) tempRate = (rt.filteredTemp - lastTemp) * 1000.0f / timeDiff;
  }
  
  if (tempRate < -0.1f) compensation += (fabs(tempRate) * 2.0f);
  
  float compensatedTarget = cfg.targetTemp + compensation;
  lastTemp = rt.filteredTemp; lastTempTime = millis();

  if (at.tuningComplete) myPID.SetTunings(at.optimizedKp, at.optimizedKi, at.optimizedKd);
  controlTemperature(compensatedTarget);
  
  if (timeElapsed(rt.brewStart, cfg.brewTimeMs)) {
    finalizeBrewingData(); optimizeParameters(); at.brewCount++;
    if (at.brewCount >= 5) {
      at.tuningComplete = true; Serial.println("[AUTOTUNE] Otimização concluída!");
      saveAutotuneParams();
    }
    
    Serial.println(F("[STATE] BREWING->IDLE (done)"));
    setPump(false); rt.brewRequested = false; rt.brewStart = 0;
    currentState = State::IDLE; windowStartTime = millis();
    beep.ok(); delay(100); beep.ok();
    
    SinricProSwitch &mySwitch = SinricPro[SinricConfig::DEVICE_ID];
    mySwitch.sendPowerStateEvent(false);
  }
}

void handleButton() {
  if (!button.pressed()) return;
  switch (currentState) {
    case State::IDLE: rt.brewRequested = true; beep.ok(); break;
    case State::HEATING:
    case State::BREWING: emergencyStop(); beep.warn(); break;
    case State::ERROR: currentState = State::IDLE; kalman.reset(rt.filteredTemp); beep.ok(); break;
  }
}

// ==================== PERSISTÊNCIA ====================
void loadConfig() {
  prefs.begin("bb", true);
  cfg.targetTemp   = prefs.getFloat("tgt", 83.0f);
  cfg.idleTemp     = prefs.getFloat("idle", 50.0f);
  cfg.brewTimeMs   = prefs.getULong("brew", 30000);
  cfg.kp           = prefs.getFloat("kp", 2.0f);
  cfg.ki           = prefs.getFloat("ki", 0.01f);
  cfg.kd           = prefs.getFloat("kd", 55.0f);
  cfg.pidWindowMs  = prefs.getULong("win", 2000);
  
  at.optimizedKp = prefs.getFloat("at_kp", 2.0f);
  at.optimizedKi = prefs.getFloat("at_ki", 0.01f);
  at.optimizedKd = prefs.getFloat("at_kd", 55.0f);
  at.optimizedCompensation = prefs.getFloat("at_comp", 0.5f);
  at.brewCount = prefs.getULong("at_count", 0);
  at.tuningComplete = prefs.getBool("at_complete", false);
  
  prefs.end(); cfg.validate();
}

void saveConfig() {
  cfg.validate();
  prefs.begin("bb", false);
  prefs.putFloat("tgt", cfg.targetTemp);
  prefs.putFloat("idle", cfg.idleTemp);
  prefs.putULong("brew", cfg.brewTimeMs);
  prefs.putFloat("kp", cfg.kp);
  prefs.putFloat("ki", cfg.ki);
  prefs.putFloat("kd", cfg.kd);
  prefs.putULong("win", cfg.pidWindowMs);
  prefs.end();
}

void saveAutotuneParams() {
  prefs.begin("bb", false);
  prefs.putFloat("at_kp", at.optimizedKp);
  prefs.putFloat("at_ki", at.optimizedKi);
  prefs.putFloat("at_kd", at.optimizedKd);
  prefs.putFloat("at_comp", at.optimizedCompensation);
  prefs.putULong("at_count", at.brewCount);
  prefs.putBool("at_complete", at.tuningComplete);
  prefs.end();
}

// ==================== REDE ====================
void setupWiFi() {
  WiFi.mode(WIFI_STA); WiFi.setHostname(NetConfig::HOSTNAME); WiFi.setAutoReconnect(true);
  WiFi.begin(NetConfig::WIFI_SSID, NetConfig::WIFI_PASSWORD);
  Serial.print(F("[WIFI] Connecting"));
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && !timeElapsed(start, WIFI_TIMEOUT_MS)) { delay(250); Serial.print("."); }
  if (WiFi.status() == WL_CONNECTED) {
    rt.wifiConnected = true; Serial.printf("\n[WIFI] %s\n", WiFi.localIP().toString().c_str());
    MDNS.begin(NetConfig::HOSTNAME);
  } else {
    Serial.println(F("\n[WIFI] AP Mode"));
    WiFi.mode(WIFI_AP); WiFi.softAP(NetConfig::AP_SSID, NetConfig::AP_PASSWORD); rt.apMode = true;
  }
}

void checkWiFi() {
  if (rt.apMode || !timeElapsed(rt.lastWifi, WIFI_CHECK_MS)) return;
  rt.lastWifi = millis();
  if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();
}

void setupOTA() {
  ArduinoOTA.setHostname(NetConfig::HOSTNAME); ArduinoOTA.setPassword(NetConfig::OTA_PASSWORD);
  ArduinoOTA.onStart([]() { emergencyStop(); setLED(0, 0, 255); }); ArduinoOTA.begin();
}

// ==================== API ====================
String statusJSON() {
  StaticJsonDocument<512> doc;
  doc["v"]    = FIRMWARE_VERSION;
  doc["s"]    = currentState == State::IDLE ? "IDLE" : 
                currentState == State::HEATING ? "HEATING" : 
                currentState == State::BREWING ? "BREWING" : "ERROR";
  doc["t"]    = round(rt.filteredTemp * 10) / 10.0f;
  doc["sp"]   = rt.activeSetpoint;
  doc["idle"] = cfg.idleTemp;
  doc["tgt"]  = cfg.targetTemp;
  doc["pwr"]  = rt.heaterPower;
  doc["h"]    = rt.heaterOn;
  doc["p"]    = rt.pumpOn;
  if (currentState == State::BREWING && rt.brewStart > 0) {
    uint32_t el = millis() - rt.brewStart;
    doc["prog"] = min(100UL, el * 100 / cfg.brewTimeMs);
    doc["rem"]  = el < cfg.brewTimeMs ? (cfg.brewTimeMs - el) / 1000 : 0;
  }
  String out; serializeJson(doc, out); return out;
}

String configJSON() {
  StaticJsonDocument<512> doc;
  doc["targetTemp"] = cfg.targetTemp;
  doc["idleTemp"]   = cfg.idleTemp;
  doc["brewTime"]   = cfg.brewTimeMs / 1000;
  
  // Sincroniza a interface Web com os dados do Autotune se aplicável
  if (at.brewCount > 0 || at.tuningComplete) {
    doc["kp"] = at.optimizedKp;
    doc["ki"] = at.optimizedKi;
    doc["kd"] = at.optimizedKd;
  } else {
    doc["kp"] = cfg.kp;
    doc["ki"] = cfg.ki;
    doc["kd"] = cfg.kd;
  }
  
  doc["pidWindow"]  = cfg.pidWindowMs / 1000;
  String out; serializeJson(doc, out); return out;
}

void sendJSON(int code, const String& json) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(code, "application/json", json);
}

void handleAutotune() {
  StaticJsonDocument<512> doc;
  doc["brewCount"] = at.brewCount; doc["maxOvershoot"] = at.maxOvershoot;
  doc["totalVariation"] = at.maxTemperature - at.minTemperature; doc["stabilityTime"] = at.stableTime;
  doc["optimizedKp"] = at.optimizedKp; doc["optimizedKi"] = at.optimizedKi;
  doc["optimizedKd"] = at.optimizedKd; doc["optimizedComp"] = at.optimizedCompensation;
  doc["tuningComplete"] = at.tuningComplete;
  String out; serializeJson(doc, out); sendJSON(200, out);
}

void resetAutotune() {
  at = AutotuneData{}; Serial.println("[AUTOTUNE] Sistema resetado"); sendJSON(200, "{\"status\":\"reset\"}");
}

void handlePostConfig() {
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, server.arg("plain"))) { sendJSON(400, "{\"error\":\"Invalid JSON\"}"); return; }
  
  if (doc.containsKey("targetTemp")) cfg.targetTemp = doc["targetTemp"];
  if (doc.containsKey("idleTemp")) cfg.idleTemp = doc["idleTemp"];
  if (doc.containsKey("brewTime")) cfg.brewTimeMs = doc["brewTime"].as<uint32_t>() * 1000;
  if (doc.containsKey("pidWindow")) cfg.pidWindowMs = doc["pidWindow"].as<uint32_t>() * 1000;
  
  // Se o usuário editar na interface, salva em ambos (base e autotune) para não ser sobrescrito
  if (doc.containsKey("kp")) { cfg.kp = doc["kp"]; at.optimizedKp = doc["kp"]; }
  if (doc.containsKey("ki")) { cfg.ki = doc["ki"]; at.optimizedKi = doc["ki"]; }
  if (doc.containsKey("kd")) { cfg.kd = doc["kd"]; at.optimizedKd = doc["kd"]; }
  
  saveConfig();
  saveAutotuneParams(); 
  myPID.SetTunings(cfg.kp, cfg.ki, cfg.kd);
  sendJSON(200, "{\"status\":\"ok\"}");
}

void handleBrew() {
  if (currentState == State::IDLE) { rt.brewRequested = true; beep.ok(); sendJSON(200, "{\"status\":\"started\"}"); } 
  else { sendJSON(400, "{\"error\":\"Not in IDLE state\"}"); }
}

void handleStop() { emergencyStop(); beep.warn(); sendJSON(200, "{\"status\":\"stopped\"}"); }

// ==================== FUNÇÕES AUTOTUNE ====================
void collectBrewingData() {
  static uint32_t lastCollect = 0; if (!timeElapsed(lastCollect, 1000)) return; lastCollect = millis();
  if (rt.filteredTemp > at.maxTemperature) at.maxTemperature = rt.filteredTemp;
  if (rt.filteredTemp < at.minTemperature) at.minTemperature = rt.filteredTemp;
  float error = cfg.targetTemp - rt.filteredTemp; at.avgError = (at.avgError * 0.9f) + (error * 0.1f);
  static float stableTemp = 0.0f; static uint32_t stableStart = 0;
  if (fabs(rt.filteredTemp - stableTemp) < 0.2f) { if (stableStart == 0) stableStart = millis(); } 
  else { at.stableTime = millis() - stableStart; stableStart = 0; }
  stableTemp = rt.filteredTemp;
}

void finalizeBrewingData() {
  float overshoot = at.maxTemperature - cfg.targetTemp;
  if (overshoot > at.maxOvershoot) at.maxOvershoot = overshoot;
  Serial.printf("[AUTOTUNE] Cycle %d: Overshoot=%.2f Var=%.2f Stable=%dms\n", at.brewCount, overshoot, at.maxTemperature - at.minTemperature, at.stableTime);
}

void optimizeParameters() {
  if (at.brewCount < 2) return; 
  float dropError = cfg.targetTemp - at.minTemperature;
  if (dropError > 2.0f) at.optimizedCompensation += 0.5f; 
  else if (at.maxOvershoot > 1.5f) at.optimizedCompensation -= 0.3f;
  float totalVariation = at.maxTemperature - at.minTemperature;
  if (totalVariation > 3.0f) { at.optimizedKp *= 1.15f; at.optimizedKi *= 1.10f; } 
  else { at.optimizedKd *= 1.10f; }
  at.optimizedKp = constrain(at.optimizedKp, 1.5f, 8.0f); at.optimizedKi = constrain(at.optimizedKi, 0.005f, 0.08f);
  at.optimizedKd = constrain(at.optimizedKd, 30.0f, 150.0f); at.optimizedCompensation = constrain(at.optimizedCompensation, 0.5f, 5.0f);
}

// ==================== WEB INTERFACE ====================
const char HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover">
<title>Bot Barista</title>
<style>
*{margin:0;padding:0;box-sizing:border-box;user-select:none;-webkit-tap-highlight-color:transparent}
:root{--g:#0f0;--gd:rgba(0,255,0,.3);--r:#f44}
html,body{height:100%;overflow:hidden;position:fixed;width:100%}
body{font-family:monospace;background:#000;color:var(--g);font-size:14px}
.app{height:100%;display:flex;flex-direction:column}
.hdr{display:flex;justify-content:space-between;align-items:center;padding:8px 10px;border-bottom:1px solid var(--gd);flex-shrink:0}
.logo{font-weight:700;font-size:.85em}
.st{font-size:.55em;opacity:.6}
.cfg-btn{padding:5px 10px;border:1px solid var(--gd);background:0;color:var(--g);font-size:.65em;border-radius:4px}
.main{flex:1;display:flex;flex-direction:column;padding:6px;gap:5px;min-height:0}
.temp{text-align:center;padding:8px 0;flex-shrink:0}
.temp-val{font-size:2.5em;font-weight:700;text-shadow:0 0 20px var(--g)}
.temp-info{font-size:.65em;opacity:.6;margin-top:2px}
.state{display:inline-block;margin-top:3px;padding:2px 8px;border:1px solid var(--g);border-radius:8px;font-size:.55em}
.state.heating{border-color:#fa0;color:#fa0}
.state.brewing{border-color:var(--g);background:rgba(0,255,0,.15)}
.state.error{border-color:var(--r);color:var(--r)}
.metrics{display:grid;grid-template-columns:repeat(4,1fr);gap:3px;flex-shrink:0}
.m{text-align:center;padding:4px 2px;background:rgba(0,255,0,.03);border:1px solid var(--gd);border-radius:3px}
.m-l{font-size:.45em;opacity:.5}
.m-v{font-size:.7em;font-weight:700}
.m-v.on{color:var(--g);text-shadow:0 0 5px var(--g)}
.m-v.off{color:#333}
.brew-bar{height:14px;background:rgba(0,255,0,.1);border:1px solid var(--g);border-radius:7px;overflow:hidden;position:relative;display:none;flex-shrink:0}
.brew-bar.show{display:block}
.brew-fill{height:100%;background:linear-gradient(90deg,#040,var(--g));width:0%;transition:width .3s}
.brew-txt{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;font-size:.55em;font-weight:700}
.chart{flex:1;min-height:0;border:1px solid var(--gd);border-radius:3px;overflow:hidden}
#chart{width:100%;height:100%}
.ctrl{padding:8px;border-top:1px solid var(--gd);flex-shrink:0}
.btn-main{width:100%;padding:14px;border-radius:25px;border:3px solid var(--g);background:linear-gradient(135deg,rgba(0,255,0,.1),transparent);color:var(--g);font-size:1em;font-weight:700;letter-spacing:1px;text-shadow:0 0 10px var(--g)}
.btn-main:active{transform:scale(.98)}
.btn-main.stop{border-color:var(--r);color:var(--r);text-shadow:0 0 10px var(--r)}
.modal{display:none;position:fixed;inset:0;z-index:99;background:rgba(0,0,0,.97)}
.modal.show{display:flex;flex-direction:column}
.modal-hdr{display:flex;justify-content:space-between;align-items:center;padding:8px 10px;border-bottom:1px solid var(--gd);flex-shrink:0}
.modal-title{font-size:.8em;font-weight:700}
.close-btn{padding:5px 10px;border:1px solid var(--r);background:0;color:var(--r);font-size:.65em;border-radius:4px}
.modal-body{flex:1;overflow-y:auto;padding:8px}
.section{font-size:.5em;opacity:.4;margin:6px 0 3px;padding-top:5px;border-top:1px solid var(--gd)}
.section:first-child{border-top:none;margin-top:0;padding-top:0}
.row{display:flex;gap:3px;margin-bottom:4px}
.row-label{font-size:.5em;opacity:.6;margin-bottom:1px}
.col{flex:1;min-width:0}
.inp-row{display:flex;gap:2px}
.inp{flex:1;min-width:0;height:32px;background:rgba(0,255,0,.03);border:1px solid var(--gd);border-radius:3px;color:var(--g);font-size:.8em;text-align:center;font-weight:600}
.adj{width:28px;height:32px;flex-shrink:0;background:rgba(0,255,0,.05);border:1px solid var(--gd);border-radius:3px;color:var(--g);font-size:.9em;font-weight:700}
.adj:active{background:rgba(0,255,0,.15)}
.modal-foot{padding:8px;border-top:1px solid var(--gd);flex-shrink:0}
.save-btn{width:100%;padding:10px;border:2px solid var(--g);border-radius:5px;background:0;color:var(--g);font-size:.75em;font-weight:700}
.toast{position:fixed;top:8px;left:50%;transform:translateX(-50%);padding:5px 12px;border-radius:5px;font-size:.6em;z-index:200;opacity:0;transition:opacity .3s}
.toast.show{opacity:1}
.toast.ok{background:rgba(0,255,0,.15);border:1px solid var(--g)}
.toast.err{background:rgba(255,0,0,.15);border:1px solid var(--r);color:var(--r)}
</style>
</head>
<body>
<div class="app">
<div class="hdr">
<div><div class="logo">BOT BARISTA</div><div class="st" id="st">...</div></div>
<button class="cfg-btn" onclick="openCfg()">CONFIG</button>
</div>
<div class="main">
<div class="temp">
<div class="temp-val" id="temp">--.-</div>
<div class="temp-info">Target <span id="tgt">--</span>°C · Idle <span id="idl">--</span>°C</div>
<div class="state" id="state">IDLE</div>
</div>
<div class="metrics">
<div class="m"><div class="m-l">POWER</div><div class="m-v" id="pwr">0%</div></div>
<div class="m"><div class="m-l">HEAT</div><div class="m-v off" id="ht">OFF</div></div>
<div class="m"><div class="m-l">PUMP</div><div class="m-v off" id="pm">OFF</div></div>
<div class="m"><div class="m-l">SETPOINT</div><div class="m-v" id="spv">--</div></div>
</div>
<div class="brew-bar" id="bb"><div class="brew-fill" id="bf"></div><div class="brew-txt" id="bt">0%</div></div>
<div class="chart"><canvas id="chart"></canvas></div>
</div>
<div class="ctrl">
<button class="btn-main" id="btn" onclick="act()">START BREW</button>
</div>
</div>
<div class="modal" id="modal">
<div class="modal-hdr">
<div class="modal-title">SETTINGS</div>
<button class="close-btn" onclick="closeCfg()">CLOSE</button>
</div>
<div class="modal-body">
<div class="section">TEMPERATURES</div>
<div class="row">
<div class="col"><div class="row-label">Idle °C</div><div class="inp-row"><button class="adj" onclick="adj('idle',-1)">-</button><input class="inp" id="idle" readonly><button class="adj" onclick="adj('idle',1)">+</button></div></div>
<div class="col"><div class="row-label">Target °C</div><div class="inp-row"><button class="adj" onclick="adj('target',-1)">-</button><input class="inp" id="target" readonly><button class="adj" onclick="adj('target',1)">+</button></div></div>
</div>
<div class="row">
<div class="col"><div class="row-label">Brew Time s</div><div class="inp-row"><button class="adj" onclick="adj('brewTime',-5)">-</button><input class="inp" id="brewTime" readonly><button class="adj" onclick="adj('brewTime',5)">+</button></div></div>
<div class="col"><div class="row-label">PID Window s</div><div class="inp-row"><button class="adj" onclick="adj('pidWindow',-2)">-</button><input class="inp" id="pidWindow" readonly><button class="adj" onclick="adj('pidWindow',2)">+</button></div></div>
</div>
<div class="section">PID TUNING</div>
<div class="row">
<div class="col"><div class="row-label">Kp</div><div class="inp-row"><button class="adj" onclick="adj('kp',-1)">-</button><input class="inp" id="kp" readonly><button class="adj" onclick="adj('kp',1)">+</button></div></div>
<div class="col"><div class="row-label">Ki</div><div class="inp-row"><button class="adj" onclick="adj('ki',-.01)">-</button><input class="inp" id="ki" readonly><button class="adj" onclick="adj('ki',.01)">+</button></div></div>
<div class="col"><div class="row-label">Kd</div><div class="inp-row"><button class="adj" onclick="adj('kd',-1)">-</button><input class="inp" id="kd" readonly><button class="adj" onclick="adj('kd',1)">+</button></div></div>
</div>
</div>
<div class="modal-foot">
<button class="save-btn" onclick="save()">SAVE</button>
</div>
</div>
<script>
var hist=[],ctx,st='IDLE';
var lim={idle:[20,80,0],target:[70,110,0],brewTime:[10,60,0],pidWindow:[1,60,0],kp:[1,100,0],ki:[0,5,3],kd:[0,100,0]};
window.onload=function(){for(var i=0;i<60;i++)hist.push(null);var c=document.getElementById('chart');c.width=c.offsetWidth;c.height=c.offsetHeight;ctx=c.getContext('2d');poll();setInterval(poll,1000);};
function poll(){fetch('/api/status').then(r=>r.json()).then(d=>{document.getElementById('st').textContent='v'+d.v+' Online';document.getElementById('temp').textContent=d.t.toFixed(1)+'°C';document.getElementById('tgt').textContent=Math.round(d.tgt);document.getElementById('idl').textContent=Math.round(d.idle);document.getElementById('pwr').textContent=d.pwr+'%';document.getElementById('spv').textContent=Math.round(d.sp)+'°C';var h=document.getElementById('ht');h.textContent=d.h?'ON':'OFF';h.className='m-v '+(d.h?'on':'off');var p=document.getElementById('pm');p.textContent=d.p?'ON':'OFF';p.className='m-v '+(d.p?'on':'off');st=d.s;var se=document.getElementById('state');se.textContent=d.s;se.className='state '+d.s.toLowerCase();var btn=document.getElementById('btn');if(d.s==='HEATING'||d.s==='BREWING'){btn.textContent='STOP';btn.className='btn-main stop';}else{btn.textContent='START BREW';btn.className='btn-main';}var bb=document.getElementById('bb');if(d.s==='BREWING'&&d.prog!==undefined){bb.className='brew-bar show';document.getElementById('bf').style.width=d.prog+'%';document.getElementById('bt').textContent=d.prog+'% ('+d.rem+'s)';}else bb.className='brew-bar';hist.shift();hist.push(d.t);draw(d.sp);}).catch(()=>document.getElementById('st').textContent='Offline');}
function draw(sp){if(!ctx)return;var w=ctx.canvas.width,h=ctx.canvas.height,pad=3;ctx.clearRect(0,0,w,h);var vals=hist.filter(v=>v!==null);if(vals.length<2)return;var min=Math.min(...vals)-2,max=Math.max(...vals)+2;if(max-min<5){var c=(min+max)/2;min=c-2.5;max=c+2.5;}var rng=max-min;function y(v){return h-pad-((v-min)/rng)*(h-pad*2)}function x(i){return pad+(w-pad*2)*i/(hist.length-1)}ctx.strokeStyle='rgba(0,255,0,.2)';ctx.setLineDash([2,2]);ctx.beginPath();ctx.moveTo(pad,y(sp));ctx.lineTo(w-pad,y(sp));ctx.stroke();ctx.setLineDash([]);ctx.strokeStyle='#0f0';ctx.lineWidth=1.5;ctx.beginPath();var first=true;for(var i=0;i<hist.length;i++)if(hist[i]!==null){if(first){ctx.moveTo(x(i),y(hist[i]));first=false;}else ctx.lineTo(x(i),y(hist[i]));}ctx.stroke();}
function act(){if(st==='HEATING'||st==='BREWING')fetch('/api/stop',{method:'POST'}).then(()=>poll());else fetch('/api/brew',{method:'POST'}).then(()=>poll());}
function toast(msg,ok){var t=document.createElement('div');t.className='toast '+(ok?'ok':'err');t.textContent=msg;document.body.appendChild(t);setTimeout(()=>t.classList.add('show'),10);setTimeout(()=>{t.classList.remove('show');setTimeout(()=>t.remove(),300)},1500);}
function openCfg(){document.getElementById('modal').classList.add('show');fetch('/api/config').then(r=>r.json()).then(d=>{document.getElementById('idle').value=Math.round(d.idleTemp);document.getElementById('target').value=Math.round(d.targetTemp);document.getElementById('brewTime').value=d.brewTime;document.getElementById('kp').value=d.kp.toFixed(0);document.getElementById('ki').value=d.ki.toFixed(2);document.getElementById('kd').value=d.kd.toFixed(0);document.getElementById('pidWindow').value=d.pidWindow;});}
function closeCfg(){document.getElementById('modal').classList.remove('show');}
function adj(id,d){var el=document.getElementById(id),l=lim[id];if(!l)return;var v=parseFloat(el.value)||l[0];el.value=(Math.max(l[0],Math.min(l[1],v+d))).toFixed(l[2]);}
function save(){var d={idleTemp:parseFloat(document.getElementById('idle').value),targetTemp:parseFloat(document.getElementById('target').value),brewTime:parseInt(document.getElementById('brewTime').value),kp:parseFloat(document.getElementById('kp').value),ki:parseFloat(document.getElementById('ki').value),kd:parseFloat(document.getElementById('kd').value),pidWindow:parseInt(document.getElementById('pidWindow').value)};fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)}).then(r=>{if(!r.ok)throw 0;toast('Saved',true);closeCfg();}).catch(()=>toast('Error',false));}
window.onresize=function(){var c=document.getElementById('chart');c.width=c.offsetWidth;c.height=c.offsetHeight;draw(93);};
</script>
</body></html>
)rawliteral";

void handleRoot() {
  size_t len = strlen_P(HTML); server.setContentLength(len); server.send(200, "text/html", "");
  char buf[1025];
  for (size_t i = 0; i < len; i += 1024) {
    size_t chunk = min((size_t)1024, len - i); memcpy_P(buf, HTML + i, chunk); buf[chunk] = 0;
    server.sendContent(buf); yield();
  }
}

void setupServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, []() { sendJSON(200, statusJSON()); });
  server.on("/api/config", HTTP_GET, []() { sendJSON(200, configJSON()); });
  server.on("/api/config", HTTP_POST, handlePostConfig);
  server.on("/api/brew", HTTP_POST, handleBrew);
  server.on("/api/stop", HTTP_POST, handleStop);
  server.on("/api/autotune", HTTP_GET, handleAutotune);
  server.on("/api/autotune/reset", HTTP_POST, resetAutotune);
  server.begin();
}

// ==================== SETUP & LOOP ====================
void setup() {
  Serial.begin(115200); Serial.println(F("\n=== BOT BARISTA v1.7.2 - PID + Alexa ===\n"));
  pinMode(Pins::BUTTON, INPUT_PULLUP); pinMode(Pins::BUZZER, OUTPUT);
  pinMode(Pins::LED_R, OUTPUT); pinMode(Pins::LED_G, OUTPUT); pinMode(Pins::LED_B, OUTPUT);
  pinMode(Pins::PUMP, OUTPUT); pinMode(Pins::HEATER, OUTPUT);
  digitalWrite(Pins::HEATER, LOW); digitalWrite(Pins::PUMP, LOW); setLED(255, 255, 255);
  
  delay(500); for(int i = 0; i < 3; i++) { thermocouple.readCelsius(); delay(100); }
  loadConfig();
  
  myPID.SetOutputLimits(0, 100); myPID.SetTunings(cfg.kp, cfg.ki, cfg.kd);
  myPID.SetAntiWindupMode(QuickPID::iAwMode::iAwClamp); myPID.SetSampleTimeUs(TEMP_READ_MS * 1000UL);
  myPID.SetMode(QuickPID::Control::automatic); myPID.SetProportionalMode(QuickPID::pMode::pOnError);
  windowStartTime = millis();
  
  if (!readTemperature()) Serial.println("[WARN] Sensor com leitura inicial problemática");
  setupWiFi(); setupOTA(); setupServer();
  
  if (rt.wifiConnected && !rt.apMode) setupSinricPro();
  setLED(0, 255, 0); beep.ok(); delay(200); setLED(0, 0, 0);
  
  Serial.printf("Idle=%.0f Target=%.0f Brew=%ds\n", cfg.idleTemp, cfg.targetTemp, cfg.brewTimeMs/1000);
  if (at.tuningComplete) Serial.printf("AUTOTUNE: Otimizado Kp=%.2f Ki=%.3f Kd=%.1f Comp=%.2f\n", at.optimizedKp, at.optimizedKi, at.optimizedKd, at.optimizedCompensation);
  else Serial.printf("AUTOTUNE: Aprendendo (%d/%d ciclos)\n", at.brewCount, 5);
}

void loop() {
  ArduinoOTA.handle(); server.handleClient(); checkWiFi(); beep.update(); handleButton();
  SinricPro.handle();
  
  if (timeElapsed(rt.lastTemp, TEMP_READ_MS)) {
    rt.lastTemp = millis();
    if (!readTemperature() && currentState != State::ERROR) {
      currentState = State::ERROR; emergencyStop();
    }
  }
  
  if (currentState != State::ERROR) {
    switch (currentState) {
      case State::IDLE: handleIdle(); break;
      case State::HEATING: handleHeating(); break;
      case State::BREWING: handleBrewing(); break;
      default: break;
    }
  }
  
  updateSinricState(); updateLED(); yield();
}