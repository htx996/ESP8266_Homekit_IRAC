#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include <Updater.h>

#include <ArduinoJson.h>

#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <IRac.h>
#include <IRutils.h>

#include <arduino_homekit_server.h>
#include <homekit/homekit.h>
#include <homekit/characteristics.h>

/*
  GPIO 映射：

  IR send     D5 / GPIO14
  IR receive  D6 / GPIO12
  LED WiFi    D4 / GPIO2
  LED IR      D7 / GPIO13
  KEY         D3 / GPIO0

  全局 GPIO 禁用规则：
  任意 GPIO 配置栏填 255 = 禁用该 GPIO
  任意 GPIO 配置栏留空保存 = 255

  LED 有效电平：
  false = LOW 点亮，HIGH 熄灭
  true  = HIGH 点亮，LOW 熄灭
*/

#define GPIO_DISABLED 255

#define DEFAULT_IR_SEND_PIN   14
#define DEFAULT_IR_RECV_PIN   12
#define DEFAULT_BUTTON_PIN    0
#define DEFAULT_LED_WIFI_PIN  2
#define DEFAULT_LED_IR_PIN    13
#define DEFAULT_RESERVED_BUTTON_PIN GPIO_DISABLED

#define CONFIG_FILE "/config.json"
#define DNS_PORT 53

#define IR_RECV_BUFFER 512
#define IR_RECV_TIMEOUT 15

#define AC_MIN_TEMP 16.0
#define AC_MAX_TEMP 30.0

bool gpioEnabled(uint8_t pin) {
  return pin != GPIO_DISABLED;
}

extern "C" {
  extern homekit_server_config_t config;

  extern homekit_characteristic_t cha_name;
  extern homekit_characteristic_t cha_active;
  extern homekit_characteristic_t cha_current_temp;
  extern homekit_characteristic_t cha_current_state;
  extern homekit_characteristic_t cha_target_state;
  extern homekit_characteristic_t cha_cooling_threshold;
  extern homekit_characteristic_t cha_heating_threshold;
  extern homekit_characteristic_t cha_temp_units;
  extern homekit_characteristic_t cha_rotation_speed;
  extern homekit_characteristic_t cha_swing_mode;

  extern homekit_characteristic_t cha_display_light_name;
  extern homekit_characteristic_t cha_display_light_on;

  extern homekit_characteristic_t cha_firmware_revision;
}

ESP8266WebServer server(80);
DNSServer dnsServer;

uint8_t parseGpioArg(const String &name, uint8_t fallback) {
  if (!server.hasArg(name)) return fallback;

  String v = server.arg(name);
  v.trim();

  if (v.length() == 0) return GPIO_DISABLED;

  int n = v.toInt();

  if (n < 0) return GPIO_DISABLED;
  if (n > 255) return GPIO_DISABLED;

  return (uint8_t)n;
}

IRrecv *irrecv = nullptr;
decode_results irResults;

char firmwareDate[9] = "00000000";

struct DeviceConfig {
  String ssid = "";
  String pass = "";

  String protocol = "MIDEA";

  /*
    -1 = Default
     0 = Unknown
     1~6 = IRremoteESP8266 里部分协议的 model 变体
  */
  int16_t model = -1;

  /*
    AUTO = 接收所有可识别红外协议
    具体协议名 = 只接收该协议的实体遥控器反馈
  */
  String decodeProtocol = "AUTO";

  bool irFeedback = true;

  String hkPin = "147-25-836";

  uint8_t pinIrSend = DEFAULT_IR_SEND_PIN;
  uint8_t pinIrRecv = DEFAULT_IR_RECV_PIN;
  uint8_t pinButton = DEFAULT_BUTTON_PIN;
  uint8_t pinLedWifi = DEFAULT_LED_WIFI_PIN;
  uint8_t pinLedIr = DEFAULT_LED_IR_PIN;
  uint8_t pinReservedButton = DEFAULT_RESERVED_BUTTON_PIN;

  bool ledWifiActiveHigh = false;
  bool ledIrActiveHigh = false;

  bool power = false;

  float targetTemp = 26.0;
  float currentTemp = 26.0;

  /*
    mode:
    0 = auto
    1 = heat
    2 = cool
    3 = dry
    4 = fan
  */
  uint8_t mode = 2;

  /*
    fan:
    0 = auto
    1 = low
    2 = medium
    3 = high
  */
  uint8_t fan = 0;

  bool swingV = false;
  bool swingH = false;

  bool displayLight = false;

  /*
    0 = Celsius
    1 = Fahrenheit
  */
  uint8_t tempUnit = 0;
};

DeviceConfig cfg;

bool wifiConnected = false;
bool apMode = false;
bool homekitStarted = false;
bool apStarted = false;

unsigned long wifiConnectStartMs = 0;
unsigned long lastWifiRetryMs = 0;
unsigned long lastWifiCheckMs = 0;
unsigned long lastWifiLogMs = 0;

const unsigned long WIFI_AP_FALLBACK_MS = 10UL * 60UL * 1000UL;  // 10 分钟后开启 AP
const unsigned long WIFI_RETRY_INTERVAL_MS = 10000;              // 每 10 秒重试 Wi-Fi
const unsigned long WIFI_CHECK_INTERVAL_MS = 1000;               // 每 1 秒检查 Wi-Fi

enum WifiLedState {
  WIFI_LED_FAST_BLINK,   // 未配网 / AP 模式 / 未联网
  WIFI_LED_SLOW_BLINK,   // 正在连接 / 重连
  WIFI_LED_OFF           // 已联网
};

WifiLedState wifiLedState = WIFI_LED_FAST_BLINK;
unsigned long lastWifiLedMs = 0;
bool wifiLedOn = false;

unsigned long lastButtonMs = 0;
unsigned long lastIrMs = 0;

String strongestSSID = "";
String scannedSSIDs[20];
uint8_t scannedSSIDCount = 0;
unsigned long lastWifiScanMs = 0;

bool pendingSend = false;
unsigned long pendingSendAt = 0;

bool pendingSave = false;
unsigned long pendingSaveAt = 0;

bool learningProtocol = false;
unsigned long learningProtocolUntil = 0;

unsigned long lastFanSpeedChangeMs = 0;
const unsigned long IGNORE_ACTIVE_OFF_AFTER_FAN_MS = 3000;

void setWifiLed(bool on) {
  if (!gpioEnabled(cfg.pinLedWifi)) return;

  if (cfg.ledWifiActiveHigh) {
    digitalWrite(cfg.pinLedWifi, on ? HIGH : LOW);
  } else {
    digitalWrite(cfg.pinLedWifi, on ? LOW : HIGH);
  }
}

void setIrLed(bool on) {
  if (!gpioEnabled(cfg.pinLedIr)) return;

  if (cfg.ledIrActiveHigh) {
    digitalWrite(cfg.pinLedIr, on ? HIGH : LOW);
  } else {
    digitalWrite(cfg.pinLedIr, on ? LOW : HIGH);
  }
}

void handleWifiLed() {
  unsigned long now = millis();

  if (wifiLedState == WIFI_LED_OFF) {
    setWifiLed(false);
    wifiLedOn = false;
    return;
  }

  unsigned long intervalMs = 200;

  if (wifiLedState == WIFI_LED_FAST_BLINK) {
    intervalMs = 180;
  } else if (wifiLedState == WIFI_LED_SLOW_BLINK) {
    intervalMs = 700;
  }

  if (now - lastWifiLedMs >= intervalMs) {
    lastWifiLedMs = now;
    wifiLedOn = !wifiLedOn;
    setWifiLed(wifiLedOn);
  }
}

int monthNumberFromName(const char *m) {
  if (!strncmp(m, "Jan", 3)) return 1;
  if (!strncmp(m, "Feb", 3)) return 2;
  if (!strncmp(m, "Mar", 3)) return 3;
  if (!strncmp(m, "Apr", 3)) return 4;
  if (!strncmp(m, "May", 3)) return 5;
  if (!strncmp(m, "Jun", 3)) return 6;
  if (!strncmp(m, "Jul", 3)) return 7;
  if (!strncmp(m, "Aug", 3)) return 8;
  if (!strncmp(m, "Sep", 3)) return 9;
  if (!strncmp(m, "Oct", 3)) return 10;
  if (!strncmp(m, "Nov", 3)) return 11;
  if (!strncmp(m, "Dec", 3)) return 12;
  return 1;
}

void setupFirmwareVersion() {
  const char *d = __DATE__;

  char mon[4] = {0};
  int day = 1;
  int year = 2026;

  sscanf(d, "%3s %d %d", mon, &day, &year);

  int month = monthNumberFromName(mon);

  snprintf(firmwareDate, sizeof(firmwareDate), "%04d%02d%02d", year, month, day);

  cha_firmware_revision.value.string_value = firmwareDate;

  Serial.print("Firmware revision: ");
  Serial.println(firmwareDate);
}

decode_type_t protocolFromString(const String &p) {
  if (p == "GREE") return GREE;
  if (p == "MIDEA") return MIDEA;
  if (p == "COOLIX") return COOLIX;

  if (p == "DAIKIN") return DAIKIN;
  if (p == "DAIKIN2") return DAIKIN2;
  if (p == "DAIKIN64") return DAIKIN64;
  if (p == "DAIKIN128") return DAIKIN128;
  if (p == "DAIKIN152") return DAIKIN152;
  if (p == "DAIKIN160") return DAIKIN160;
  if (p == "DAIKIN176") return DAIKIN176;
  if (p == "DAIKIN216") return DAIKIN216;
  if (p == "DAIKIN312") return DAIKIN312;

  if (p == "HAIER_AC") return HAIER_AC;
  if (p == "HAIER_AC_YRW02") return HAIER_AC_YRW02;
  if (p == "HAIER_AC176") return HAIER_AC176;

  if (p == "PANASONIC_AC") return PANASONIC_AC;
  if (p == "PANASONIC_AC32") return PANASONIC_AC32;

  if (p == "MITSUBISHI_AC") return MITSUBISHI_AC;
  if (p == "MITSUBISHI112") return MITSUBISHI112;
  if (p == "MITSUBISHI136") return MITSUBISHI136;
  if (p == "MITSUBISHI_HEAVY_88") return MITSUBISHI_HEAVY_88;
  if (p == "MITSUBISHI_HEAVY_152") return MITSUBISHI_HEAVY_152;

  if (p == "FUJITSU_AC") return FUJITSU_AC;
  if (p == "TOSHIBA_AC") return TOSHIBA_AC;
  if (p == "HITACHI_AC") return HITACHI_AC;
  if (p == "HITACHI_AC1") return HITACHI_AC1;
  if (p == "SAMSUNG_AC") return SAMSUNG_AC;

  if (p == "LG") return LG;
  if (p == "LG2") return LG2;

  if (p == "KELVINATOR") return KELVINATOR;
  if (p == "CARRIER_AC64") return CARRIER_AC64;
  if (p == "SHARP_AC") return SHARP_AC;
  if (p == "TCL112AC") return TCL112AC;
  if (p == "VESTEL_AC") return VESTEL_AC;
  if (p == "VOLTAS") return VOLTAS;

  if (p == "ARGO") return ARGO;
  if (p == "TROTEC") return TROTEC;
  if (p == "WHIRLPOOL_AC") return WHIRLPOOL_AC;
  if (p == "ELECTRA_AC") return ELECTRA_AC;
  if (p == "TECO") return TECO;
  if (p == "GOODWEATHER") return GOODWEATHER;
  if (p == "NEOCLIMA") return NEOCLIMA;
  if (p == "AMCOR") return AMCOR;
  if (p == "DELONGHI_AC") return DELONGHI_AC;
  if (p == "CORONA_AC") return CORONA_AC;

  return MIDEA;
}

String protocolToString(decode_type_t t) {
  switch (t) {
    case GREE: return "GREE";
    case MIDEA: return "MIDEA";
    case COOLIX: return "COOLIX";

    case DAIKIN: return "DAIKIN";
    case DAIKIN2: return "DAIKIN2";
    case DAIKIN64: return "DAIKIN64";
    case DAIKIN128: return "DAIKIN128";
    case DAIKIN152: return "DAIKIN152";
    case DAIKIN160: return "DAIKIN160";
    case DAIKIN176: return "DAIKIN176";
    case DAIKIN216: return "DAIKIN216";
    case DAIKIN312: return "DAIKIN312";

    case HAIER_AC: return "HAIER_AC";
    case HAIER_AC_YRW02: return "HAIER_AC_YRW02";
    case HAIER_AC176: return "HAIER_AC176";

    case PANASONIC_AC: return "PANASONIC_AC";
    case PANASONIC_AC32: return "PANASONIC_AC32";

    case MITSUBISHI_AC: return "MITSUBISHI_AC";
    case MITSUBISHI112: return "MITSUBISHI112";
    case MITSUBISHI136: return "MITSUBISHI136";
    case MITSUBISHI_HEAVY_88: return "MITSUBISHI_HEAVY_88";
    case MITSUBISHI_HEAVY_152: return "MITSUBISHI_HEAVY_152";

    case FUJITSU_AC: return "FUJITSU_AC";
    case TOSHIBA_AC: return "TOSHIBA_AC";
    case HITACHI_AC: return "HITACHI_AC";
    case HITACHI_AC1: return "HITACHI_AC1";
    case SAMSUNG_AC: return "SAMSUNG_AC";

    case LG: return "LG";
    case LG2: return "LG2";

    case KELVINATOR: return "KELVINATOR";
    case CARRIER_AC64: return "CARRIER_AC64";
    case SHARP_AC: return "SHARP_AC";
    case TCL112AC: return "TCL112AC";
    case VESTEL_AC: return "VESTEL_AC";
    case VOLTAS: return "VOLTAS";

    case ARGO: return "ARGO";
    case TROTEC: return "TROTEC";
    case WHIRLPOOL_AC: return "WHIRLPOOL_AC";
    case ELECTRA_AC: return "ELECTRA_AC";
    case TECO: return "TECO";
    case GOODWEATHER: return "GOODWEATHER";
    case NEOCLIMA: return "NEOCLIMA";
    case AMCOR: return "AMCOR";
    case DELONGHI_AC: return "DELONGHI_AC";
    case CORONA_AC: return "CORONA_AC";

    default: return "";
  }
}

String normalizePin(String pin) {
  pin.trim();
  pin.replace("-", "");

  if (pin.length() != 8) {
    return "147-25-836";
  }

  for (uint8_t i = 0; i < pin.length(); i++) {
    if (!isDigit(pin[i])) return "147-25-836";
  }

  return pin.substring(0, 3) + "-" + pin.substring(3, 5) + "-" + pin.substring(5, 8);
}

float clampTemp(float t) {
  if (t < AC_MIN_TEMP) return AC_MIN_TEMP;
  if (t > AC_MAX_TEMP) return AC_MAX_TEMP;
  return t;
}

float fanPercentFromCfg() {
  if (cfg.fan == 0) return 25;   // Auto / 空调默认自动风；HomeKit 不上报 0%，避免被家庭 App 当成关闭
  if (cfg.fan == 1) return 33;   // 低风
  if (cfg.fan == 2) return 66;   // 中风
  return 100;                    // 高风
}

void scheduleSendAcState() {
  pendingSend = true;
  pendingSendAt = millis() + 500;
}

void scheduleSaveConfig() {
  pendingSave = true;
  pendingSaveAt = millis() + 10000;
}

void scanStrongestWifi() {
  strongestSSID = "";
  scannedSSIDCount = 0;

  int bestRssi = -1000;

  WiFi.mode(WIFI_AP_STA);
  int n = WiFi.scanNetworks();

  for (int i = 0; i < n && scannedSSIDCount < 20; i++) {
    String ssid = WiFi.SSID(i);
    if (ssid.length() == 0) continue;

    bool exists = false;
    for (uint8_t j = 0; j < scannedSSIDCount; j++) {
      if (scannedSSIDs[j] == ssid) {
        exists = true;
        break;
      }
    }

    if (exists) continue;

    scannedSSIDs[scannedSSIDCount++] = ssid;

    if (WiFi.RSSI(i) > bestRssi) {
      bestRssi = WiFi.RSSI(i);
      strongestSSID = ssid;
    }
  }

  WiFi.scanDelete();
}

void loadConfig() {
  if (!LittleFS.begin()) return;
  if (!LittleFS.exists(CONFIG_FILE)) return;

  File f = LittleFS.open(CONFIG_FILE, "r");
  if (!f) return;

  StaticJsonDocument<1280> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();

  if (err) return;

  cfg.ssid = doc["ssid"] | cfg.ssid;
  cfg.pass = doc["pass"] | cfg.pass;

  cfg.protocol = doc["protocol"] | cfg.protocol;
  cfg.model = doc["model"] | cfg.model;
  cfg.decodeProtocol = doc["decodeProtocol"] | cfg.decodeProtocol;
  cfg.irFeedback = doc["irFeedback"] | cfg.irFeedback;

  cfg.hkPin = normalizePin(String((const char *)(doc["hkPin"] | cfg.hkPin.c_str())));

  cfg.pinIrSend = doc["pinIrSend"] | cfg.pinIrSend;
  cfg.pinIrRecv = doc["pinIrRecv"] | cfg.pinIrRecv;
  cfg.pinButton = doc["pinButton"] | cfg.pinButton;
  cfg.pinLedWifi = doc["pinLedWifi"] | cfg.pinLedWifi;
  cfg.pinLedIr = doc["pinLedIr"] | cfg.pinLedIr;
  cfg.pinReservedButton = doc["pinReservedButton"] | cfg.pinReservedButton;

  cfg.ledWifiActiveHigh = doc["ledWifiActiveHigh"] | cfg.ledWifiActiveHigh;
  cfg.ledIrActiveHigh = doc["ledIrActiveHigh"] | cfg.ledIrActiveHigh;

  cfg.power = doc["power"] | cfg.power;
  cfg.targetTemp = doc["targetTemp"] | cfg.targetTemp;
  cfg.currentTemp = doc["currentTemp"] | cfg.currentTemp;
  cfg.mode = doc["mode"] | cfg.mode;
  cfg.fan = doc["fan"] | cfg.fan;
  cfg.swingV = doc["swingV"] | cfg.swingV;
  cfg.swingH = doc["swingH"] | cfg.swingH;
  cfg.displayLight = doc["displayLight"] | cfg.displayLight;
  cfg.tempUnit = doc["tempUnit"] | cfg.tempUnit;

  cfg.targetTemp = clampTemp(cfg.targetTemp);
  cfg.currentTemp = clampTemp(cfg.currentTemp);
}

void resetRuntimeAcStateAfterBoot() {
  cfg.power = false;
  cfg.mode = 2;
  cfg.targetTemp = 26.0;
  cfg.currentTemp = 26.0;
  cfg.fan = 0;
  cfg.swingV = false;
  cfg.swingH = false;
}

void saveConfig() {
  StaticJsonDocument<1280> doc;

  doc["ssid"] = cfg.ssid;
  doc["pass"] = cfg.pass;

  doc["protocol"] = cfg.protocol;
  doc["model"] = cfg.model;
  doc["decodeProtocol"] = cfg.decodeProtocol;
  doc["irFeedback"] = cfg.irFeedback;

  doc["hkPin"] = cfg.hkPin;

  doc["pinIrSend"] = cfg.pinIrSend;
  doc["pinIrRecv"] = cfg.pinIrRecv;
  doc["pinButton"] = cfg.pinButton;
  doc["pinLedWifi"] = cfg.pinLedWifi;
  doc["pinLedIr"] = cfg.pinLedIr;
  doc["pinReservedButton"] = cfg.pinReservedButton;

  doc["ledWifiActiveHigh"] = cfg.ledWifiActiveHigh;
  doc["ledIrActiveHigh"] = cfg.ledIrActiveHigh;

  doc["power"] = cfg.power;
  doc["targetTemp"] = cfg.targetTemp;
  doc["currentTemp"] = cfg.currentTemp;
  doc["mode"] = cfg.mode;
  doc["fan"] = cfg.fan;
  doc["swingV"] = cfg.swingV;
  doc["swingH"] = cfg.swingH;
  doc["displayLight"] = cfg.displayLight;
  doc["tempUnit"] = cfg.tempUnit;

  File f = LittleFS.open(CONFIG_FILE, "w");
  if (!f) return;

  serializeJson(doc, f);
  f.close();

  pendingSave = false;
}

String htmlEscape(String s) {
  s.replace("&", "&amp;");
  s.replace("<", "&lt;");
  s.replace(">", "&gt;");
  s.replace("\"", "&quot;");
  return s;
}

String wifiOptions() {
  String out;

  String current = cfg.ssid.length() ? cfg.ssid : strongestSSID;

  out += "<option value=''>手动输入 / Manual</option>";

  for (uint8_t i = 0; i < scannedSSIDCount; i++) {
    out += "<option value='";
    out += htmlEscape(scannedSSIDs[i]);
    out += "'";

    if (scannedSSIDs[i] == current) {
      out += " selected";
    }

    out += ">";
    out += htmlEscape(scannedSSIDs[i]);

    if (scannedSSIDs[i] == strongestSSID) {
      out += " · 最强";
    }

    out += "</option>";
  }

  return out;
}

String protocolOptions() {
  const char *items[] = {
    "MIDEA",
    "COOLIX",
    "GREE",
    "DAIKIN",
    "DAIKIN2",
    "DAIKIN64",
    "DAIKIN128",
    "DAIKIN152",
    "DAIKIN160",
    "DAIKIN176",
    "DAIKIN216",
    "DAIKIN312",
    "HAIER_AC",
    "HAIER_AC_YRW02",
    "HAIER_AC176",
    "PANASONIC_AC",
    "PANASONIC_AC32",
    "MITSUBISHI_AC",
    "MITSUBISHI112",
    "MITSUBISHI136",
    "MITSUBISHI_HEAVY_88",
    "MITSUBISHI_HEAVY_152",
    "FUJITSU_AC",
    "TOSHIBA_AC",
    "HITACHI_AC",
    "HITACHI_AC1",
    "SAMSUNG_AC",
    "LG",
    "LG2",
    "KELVINATOR",
    "CARRIER_AC64",
    "SHARP_AC",
    "TCL112AC",
    "VESTEL_AC",
    "VOLTAS",
    "ARGO",
    "TROTEC",
    "WHIRLPOOL_AC",
    "ELECTRA_AC",
    "TECO",
    "GOODWEATHER",
    "NEOCLIMA",
    "AMCOR",
    "DELONGHI_AC",
    "CORONA_AC"
  };

  String out;
  for (uint8_t i = 0; i < sizeof(items) / sizeof(items[0]); i++) {
    out += "<option value='";
    out += items[i];
    out += "'";
    if (cfg.protocol == items[i]) out += " selected";
    out += ">";
    out += items[i];
    out += "</option>";
  }

  return out;
}

String modelOptions() {
  struct ModelOption {
    int16_t value;
    const char *label;
  };

  const ModelOption items[] = {
    {-1, "Default"},
    {0, "Unknown"},
    {1, "(1) Fujitsu/Gree/Hitachi/Panasonic/Whirlpool/LG"},
    {2, "(2) Fujitsu/Gree/Hitachi/Panasonic/Whirlpool/LG"},
    {3, "(3) Fujitsu/Panasonic"},
    {4, "(4) Fujitsu/Panasonic"},
    {5, "(5) Fujitsu/Panasonic"},
    {6, "(6) Panasonic"}
  };

  String out;
  for (uint8_t i = 0; i < sizeof(items) / sizeof(items[0]); i++) {
    out += "<option value='";
    out += String(items[i].value);
    out += "'";
    if (cfg.model == items[i].value) out += " selected";
    out += ">";
    out += items[i].label;
    out += "</option>";
  }

  return out;
}

String decodeProtocolOptions() {
  const char *items[] = {
    "AUTO",
    "MIDEA",
    "COOLIX",
    "GREE",
    "DAIKIN",
    "DAIKIN2",
    "DAIKIN64",
    "DAIKIN128",
    "DAIKIN152",
    "DAIKIN160",
    "DAIKIN176",
    "DAIKIN216",
    "HAIER_AC",
    "HAIER_AC_YRW02",
    "PANASONIC_AC",
    "MITSUBISHI_AC",
    "MITSUBISHI112",
    "MITSUBISHI136",
    "MITSUBISHI_HEAVY_88",
    "MITSUBISHI_HEAVY_152",
    "FUJITSU_AC",
    "TOSHIBA_AC",
    "HITACHI_AC",
    "SAMSUNG_AC",
    "LG",
    "LG2",
    "KELVINATOR",
    "SHARP_AC",
    "TCL112AC",
    "VESTEL_AC"
  };

  String out;
  for (uint8_t i = 0; i < sizeof(items) / sizeof(items[0]); i++) {
    out += "<option value='";
    out += items[i];
    out += "'";
    if (cfg.decodeProtocol == items[i]) out += " selected";
    out += ">";
    out += items[i];
    out += "</option>";
  }

  return out;
}

String pageHtml() {
  if (millis() - lastWifiScanMs > 30000 || scannedSSIDCount == 0) {
    lastWifiScanMs = millis();
    scanStrongestWifi();
  }

  String h;

  h += "<!doctype html><html><head><meta charset='utf-8'>";
  h += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>IR AC Settings</title>";

  h += "<style>";
  h += ":root{color-scheme:light dark;--bg:#f5f5f7;--card:#ffffff;--text:#1d1d1f;--sub:#6e6e73;--border:#d2d2d7;--shadow:rgba(0,0,0,.12);--input:#ffffff;--button:#1d1d1f;--buttonText:#ffffff;--warn:#9a5a00;--green:#34c759;--red:#ff453a;--pink:#ff7ab6;}";
  h += "@media(prefers-color-scheme:dark){:root{--bg:#000000;--card:#1c1c1e;--text:#f5f5f7;--sub:#a1a1a6;--border:#3a3a3c;--shadow:rgba(0,0,0,.55);--input:#2c2c2e;--button:#f5f5f7;--buttonText:#000000;--warn:#ffd60a;}}";
  h += "*{box-sizing:border-box}";
  h += "body{font-family:-apple-system,BlinkMacSystemFont,'SF Pro Text','Segoe UI',Arial,sans-serif;margin:0;padding:18px;background:var(--bg);color:var(--text);}";
  h += "h2{margin:6px 0 14px;font-size:24px}";
  h += "h3{margin:0 0 12px;font-size:18px}";
  h += "h4{margin:18px 0 8px;font-size:15px;color:var(--text)}";
  h += ".card{background:var(--card);border:1px solid var(--border);border-radius:14px;padding:16px;margin:12px 0;box-shadow:0 4px 18px var(--shadow);}";
  h += "label{font-weight:600;margin-top:10px;display:block;font-size:14px}";
  h += "input,select,button{font-size:16px;padding:10px 11px;margin:5px 0 8px;width:100%;border-radius:10px;}";
  h += "input,select{background:var(--input);color:var(--text);border:1px solid var(--border);}";
  h += "button{background:var(--button);color:var(--buttonText);border:0;font-weight:700;}";
  h += "button:active{opacity:.75}";
  h += ".row{display:grid;grid-template-columns:1fr 1fr;gap:8px}";
  h += ".small{font-size:13px;color:var(--sub);line-height:1.55}";
  h += ".warn{font-size:13px;color:var(--warn);line-height:1.55;margin-top:12px}";
  h += ".checkrow{display:flex;align-items:center;gap:8px;font-weight:600}";
  h += ".checkrow input{width:auto;margin:0}";
  h += ".actions{display:grid;grid-template-columns:repeat(4,1fr);gap:12px}";
  h += ".actions form{margin:0}";
  h += ".btn{height:48px;border-radius:14px;font-weight:800;font-size:16px}";
  h += ".secondary{background:transparent;color:var(--text);border:2px solid var(--text)}";
  h += ".danger{background:transparent;color:var(--red);border:2px solid var(--red)}";
  h += ".progressBox{width:100%;height:14px;background:var(--input);border:1px solid var(--border);border-radius:999px;overflow:hidden;margin:10px 0}";
  h += "#otaBar{height:100%;width:0%;background:var(--green);transition:width .15s}";
  h += ".fileBox{display:flex;align-items:center;gap:12px;background:var(--input);border:1px solid var(--border);border-radius:14px;padding:12px;margin:8px 0 12px}";
  h += ".fileIcon{width:32px;height:32px;display:flex;align-items:center;justify-content:center;color:var(--pink);font-size:28px;font-weight:700;flex:0 0 auto}";
  h += ".fileInfo{flex:1;min-width:0}";
  h += ".fileName{font-size:16px;font-weight:700;color:var(--text);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}";
  h += ".fileSize{font-size:13px;color:var(--sub);margin-top:2px}";
  h += ".fileRemove{width:40px;height:40px;margin:0;padding:0;border:0;background:transparent;color:var(--text);font-size:32px;line-height:1;font-weight:400;flex:0 0 auto}";
  h += ".fileRemove:active{opacity:.55}";
  h += "@media(max-width:720px){.actions{grid-template-columns:1fr 1fr}}";
  h += "@media(max-width:520px){body{padding:12px}.row{grid-template-columns:1fr}.card{padding:14px;border-radius:12px}h2{font-size:22px}}";
  h += "@media(max-width:420px){.actions{grid-template-columns:1fr}}";
  h += "</style>";

  h += "</head><body>";

  h += "<h2>配置中心</h2>";

  h += "<div class='card'><div class='small'>";

  h += "设备 IP: ";
  if (WiFi.status() == WL_CONNECTED) {
    h += WiFi.localIP().toString();
  } else {
    h += "未连接";
  }

  h += "<br>Wi-Fi 状态: ";
  if (WiFi.status() == WL_CONNECTED) {
    h += "已连接";
  } else {
    h += "未连接 / 正在重试";
  }

  h += "<br>当前 Wi-Fi: ";
  if (WiFi.status() == WL_CONNECTED) {
    h += htmlEscape(WiFi.SSID());
  } else if (cfg.ssid.length() > 0) {
    h += htmlEscape(cfg.ssid);
    h += "（正在尝试连接）";
  } else {
    h += "未配置";
  }

  h += "<br>Wi-Fi 信号: ";
  if (WiFi.status() == WL_CONNECTED) {
    h += String(WiFi.RSSI());
    h += " dBm";
  } else {
    h += "-";
  }

  h += "<br>设备 MAC: ";
  h += WiFi.macAddress();

  h += "<br>AP 模式: ";
  if (apMode) {
    h += "ON";
  } else {
    h += "OFF";
  }

  h += "<br>AP IP: ";
  if (apMode) {
    h += WiFi.softAPIP().toString();
  } else {
    h += "-";
  }

  h += "<br>固件版本: ";
  h += firmwareDate;

  h += "<br>HomeKit 控制请在 Apple 家庭 App 中完成。";
  h += "</div></div>";

  h += "<div class='card'><h3>参数设置</h3>";
  h += "<form method='POST' action='/save'>";

  h += "<h4>Wi-Fi</h4>";
  h += "<label>附近 Wi-Fi</label>";
  h += "<select id='ssidSelect' onchange='selectSsidFromList()'>";
  h += wifiOptions();
  h += "</select>";

  h += "<label>SSID</label><input id='ssidInput' name='ssid' value='";
  h += htmlEscape(cfg.ssid.length() ? cfg.ssid : strongestSSID);
  h += "' placeholder='请输入 Wi-Fi 名称'>";

  h += "<div class='small'>可以从附近 Wi-Fi 列表选择，也可以手动输入隐藏 SSID。列表每 30 秒最多刷新一次。</div>";

  h += "<label>Wi-Fi 密码</label><input name='pass' type='password' value='' placeholder='留空则保留原密码'>";
  h += "<div class='small'>只修改其他参数时，Wi-Fi 密码留空即可；只有手动输入新密码时才会覆盖旧密码。</div>";

  h += "<h4>HomeKit</h4>";
  h += "<label>HomeKit 配对码</label><input name='hkPin' value='";
  h += htmlEscape(cfg.hkPin);
  h += "'>";
  h += "<div class='small'>格式支持 111-11-111 或 11111111。</div>";

  h += "<h4>空调型号 / 协议</h4>";
  h += "<label>协议 Protocol</label><select name='protocol' id='protocol'>";
  h += protocolOptions();
  h += "</select>";

  h += "<label>子型号 Model</label><select name='model'>";
  h += modelOptions();
  h += "</select>";

  h += "<label>协议识别 Decode Protocol</label><select name='decodeProtocol'>";
  h += decodeProtocolOptions();
  h += "</select>";

  h += "<label class='checkrow'>";
  h += "<input type='checkbox' name='irFeedback' value='1'";
  if (cfg.irFeedback) h += " checked";
  h += "> 红外反馈 IR Feedback";
  h += "</label>";

  h += "<div class='small'>";
  h += "美的空调优先选 MIDEA；不响应再选 COOLIX。";
  h += "<br>Model 默认选 Default。";
  h += "<br>Decode Protocol 选 AUTO 表示接收所有可识别协议；选具体协议表示只接受该协议的实体遥控器反馈。";
  h += "</div>";

  h += "<h4>GPIO</h4>";
  h += "<div class='row'>";
  h += "<div><label>IR 发射 GPIO</label><input name='pinIrSend' type='number' min='0' max='255' value='" + String(cfg.pinIrSend) + "'></div>";
  h += "<div><label>IR 接收 GPIO</label><input name='pinIrRecv' type='number' min='0' max='255' value='" + String(cfg.pinIrRecv) + "'></div>";

  h += "<div><label>KEY1 GPIO</label><input name='pinButton' type='number' min='0' max='255' value='" + String(cfg.pinButton) + "'></div>";
  h += "<div><label>LED WiFi GPIO</label><input name='pinLedWifi' type='number' min='0' max='255' value='" + String(cfg.pinLedWifi) + "'></div>";

  h += "<div><label>LED IR GPIO</label><input name='pinLedIr' type='number' min='0' max='255' value='" + String(cfg.pinLedIr) + "'></div>";
  h += "<div><label>KEY2 GPIO</label><input name='pinReservedButton' type='number' min='0' max='255' value='" + String(cfg.pinReservedButton) + "'></div>";

  h += "<div><label>LED WiFi 有效电平</label><select name='ledWifiActiveHigh'>";
  h += "<option value='0'";
  if (!cfg.ledWifiActiveHigh) h += " selected";
  h += ">LOW / HIGH</option>";
  h += "<option value='1'";
  if (cfg.ledWifiActiveHigh) h += " selected";
  h += ">HIGH / LOW</option>";
  h += "</select></div>";

  h += "<div><label>LED IR 有效电平</label><select name='ledIrActiveHigh'>";
  h += "<option value='0'";
  if (!cfg.ledIrActiveHigh) h += " selected";
  h += ">LOW / HIGH</option>";
  h += "<option value='1'";
  if (cfg.ledIrActiveHigh) h += " selected";
  h += ">HIGH / LOW</option>";
  h += "</select></div>";

  h += "</div>";

  h += "<div class='small'>";
  h += "GPIO：IR 发射 14，IR 接收 12，KEY1 默认 0，LED WiFi 2，LED IR 13。";
  h += "<br>KEY1：长按 5 秒后松开，只清除 Wi-Fi；长按 10 秒，清除 Wi-Fi 和 HomeKit 配对数据，恢复到初始化待配网状态。";
  h += "<br>任意 GPIO 配置栏填 255 表示禁用该 GPIO。";
  h += "<br>留空保存也会按 255 处理，避免误变为 GPIO0。";
  h += "<br>LED WiFi：未配网/未联网快闪，联网中慢闪，联网成功关闭。";
  h += "<br>若外接 LED 是 HIGH 有效、LOW 关闭，请把对应有效电平设为 HIGH / LOW。";
  h += "</div>";

  h += "<div class='warn'>保存后设备会自动重启。协议、GPIO、HomeKit 配对码变更后，建议在家庭 App 中确认配件状态。</div>";
  h += "<button type='submit'>保存参数并重启</button>";
  h += "</form></div>";

  h += "<div class='card'><h3>学习协议</h3>";
  h += "<div class='small'>点击开始学习后，请在 10 秒内用原装空调遥控器对准设备按一次开机、制冷或温度键。";
  h += "<br>识别成功后会自动保存 Protocol / Model / Decode Protocol 并重启。</div>";
  h += "<form method='POST' action='/learn_protocol'><button type='submit'>开始学习协议</button></form>";
  h += "</div>";

  h += "<div class='card'><h3>OTA 固件更新</h3>";
  h += "<div class='small'>请选择编译的固件二进制文件（.bin）。上传完成后设备会自动重启。</div>";
  h += "<label>Firmware File (.bin)</label>";
  h += "<input id='otaFile' type='file' accept='.bin' onchange='onOtaFileSelected()'>";
  h += "<div id='otaSelectedBox' class='fileBox' style='display:none'>";
  h += "<div class='fileIcon'>▯</div>";
  h += "<div class='fileInfo'><div id='otaFileName' class='fileName'></div><div id='otaFileSize' class='fileSize'></div></div>";
  h += "<button type='button' class='fileRemove' onclick='clearOtaFile()'>×</button>";
  h += "</div>";
  h += "<button type='button' onclick='uploadOTA()'>上传固件</button>";
  h += "<div class='progressBox'><div id='otaBar'></div></div>";
  h += "<div id='otaText' class='small'>等待选择固件</div>";
  h += "</div>";

  h += "<div class='card'><h3>设备管理</h3>";
  h += "<div class='actions'>";
  h += "<form method='POST' action='/reboot'><button class='btn secondary' type='submit'>↻ 重启</button></form>";
  h += "<form method='POST' action='/start_ap'><button class='btn secondary' type='submit'>Wi-Fi 启动AP</button></form>";
  h += "<form method='POST' action='/reset_homekit'><button class='btn danger' type='submit'>重置HomeKit</button></form>";
  h += "<form method='POST' action='/reset_wifi'><button class='btn danger' type='submit'>Wi-Fi 重置WiFi</button></form>";
  h += "</div>";
  h += "<div class='small'>重置 HomeKit 后，需要在家庭 App 删除旧配件并重新添加。";
  h += "<br>重置 WiFi 只清除 SSID / Password，不清除协议、GPIO、HomeKit 配对码等参数。</div>";
  h += "</div>";

  h += "<script>";

  h += "function selectSsidFromList(){";
  h += "var sel=document.getElementById('ssidSelect');";
  h += "var input=document.getElementById('ssidInput');";
  h += "if(sel.value){input.value=sel.value;}";
  h += "}";

  h += "function formatFileSize(bytes){";
  h += "if(bytes>=1024*1024){return (bytes/1024/1024).toFixed(2)+' MB';}";
  h += "if(bytes>=1024){return Math.round(bytes/1024)+' KB';}";
  h += "return bytes+' B';";
  h += "}";

  h += "function onOtaFileSelected(){";
  h += "var input=document.getElementById('otaFile');";
  h += "var box=document.getElementById('otaSelectedBox');";
  h += "var name=document.getElementById('otaFileName');";
  h += "var size=document.getElementById('otaFileSize');";
  h += "var text=document.getElementById('otaText');";
  h += "var bar=document.getElementById('otaBar');";
  h += "var f=input.files[0];";
  h += "bar.style.width='0%';";
  h += "if(!f){box.style.display='none';name.innerText='';size.innerText='';text.innerText='等待选择固件';return;}";
  h += "name.innerText=f.name;";
  h += "size.innerText=formatFileSize(f.size);";
  h += "box.style.display='flex';";
  h += "text.innerText='已选择固件，点击上传。';";
  h += "}";

  h += "function clearOtaFile(){";
  h += "var input=document.getElementById('otaFile');";
  h += "var box=document.getElementById('otaSelectedBox');";
  h += "var name=document.getElementById('otaFileName');";
  h += "var size=document.getElementById('otaFileSize');";
  h += "var text=document.getElementById('otaText');";
  h += "var bar=document.getElementById('otaBar');";
  h += "input.value='';";
  h += "box.style.display='none';";
  h += "name.innerText='';";
  h += "size.innerText='';";
  h += "bar.style.width='0%';";
  h += "text.innerText='已取消选择固件';";
  h += "}";

  h += "function uploadOTA(){";
  h += "var input=document.getElementById('otaFile');";
  h += "var f=input.files[0];";
  h += "var t=document.getElementById('otaText');";
  h += "var b=document.getElementById('otaBar');";
  h += "if(!f){t.innerText='请先选择 .bin 文件';return;}";
  h += "if(!f.name.toLowerCase().endsWith('.bin')){t.innerText='文件格式错误，请选择 .bin 固件';return;}";
  h += "var xhr=new XMLHttpRequest();";
  h += "xhr.open('POST','/ota_update',true);";
  h += "xhr.upload.onprogress=function(e){if(e.lengthComputable){var p=Math.round((e.loaded/e.total)*100);b.style.width=p+'%';t.innerText='上传进度：'+p+'%';}};";
  h += "xhr.onload=function(){if(xhr.status==200){b.style.width='100%';t.innerText='上传完成，设备正在重启...';}else{t.innerText='上传失败：HTTP '+xhr.status;}};";
  h += "xhr.onerror=function(){t.innerText='上传失败：网络错误';};";
  h += "var data=new FormData();";
  h += "data.append('firmware',f);";
  h += "xhr.send(data);";
  h += "}";

  h += "</script>";

  h += "</body></html>";
  return h;
}

stdAc::opmode_t stdModeFromCfg() {
  switch (cfg.mode) {
    case 1: return stdAc::opmode_t::kHeat;
    case 2: return stdAc::opmode_t::kCool;
    case 3: return stdAc::opmode_t::kDry;
    case 4: return stdAc::opmode_t::kFan;
    default: return stdAc::opmode_t::kAuto;
  }
}

stdAc::fanspeed_t stdFanFromCfg() {
  switch (cfg.fan) {
    case 1: return stdAc::fanspeed_t::kLow;
    case 2: return stdAc::fanspeed_t::kMedium;
    case 3: return stdAc::fanspeed_t::kHigh;
    default: return stdAc::fanspeed_t::kAuto;
  }
}

void syncThresholdsForHomeKit() {
  float heatTemp;
  float coolTemp;

  if (cfg.mode == 1) {
    heatTemp = cfg.targetTemp;
    coolTemp = cfg.targetTemp + 1.0;
  } else if (cfg.mode == 2) {
    heatTemp = cfg.targetTemp - 1.0;
    coolTemp = cfg.targetTemp;
  } else {
    heatTemp = cfg.targetTemp - 1.0;
    coolTemp = cfg.targetTemp + 1.0;
  }

  heatTemp = clampTemp(heatTemp);
  coolTemp = clampTemp(coolTemp);

  if (heatTemp >= coolTemp) {
    if (cfg.targetTemp <= 16.5) {
      heatTemp = 16.0;
      coolTemp = 17.0;
    } else if (cfg.targetTemp >= 29.5) {
      heatTemp = 29.0;
      coolTemp = 30.0;
    } else {
      heatTemp = cfg.targetTemp - 0.5;
      coolTemp = cfg.targetTemp + 0.5;
    }
  }

  cha_heating_threshold.value.float_value = heatTemp;
  cha_cooling_threshold.value.float_value = coolTemp;
}

void updateHomeKitNotify() {
  cha_active.value.int_value = cfg.power ? 1 : 0;
  homekit_characteristic_notify(&cha_active, cha_active.value);

  cha_current_temp.value.float_value = cfg.currentTemp;
  homekit_characteristic_notify(&cha_current_temp, cha_current_temp.value);

  int currentState = 0;
  if (cfg.power) {
    if (cfg.mode == 1) {
      currentState = 2;
    } else if (cfg.mode == 2) {
      currentState = 3;
    } else {
      currentState = 1;
    }
  }

  cha_current_state.value.int_value = currentState;
  homekit_characteristic_notify(&cha_current_state, cha_current_state.value);

  int targetState = 0;
  if (cfg.mode == 1) {
    targetState = 1;
  } else if (cfg.mode == 2) {
    targetState = 2;
  }

  cha_target_state.value.int_value = targetState;
  homekit_characteristic_notify(&cha_target_state, cha_target_state.value);

  syncThresholdsForHomeKit();

  homekit_characteristic_notify(&cha_heating_threshold, cha_heating_threshold.value);
  homekit_characteristic_notify(&cha_cooling_threshold, cha_cooling_threshold.value);

  cha_temp_units.value.int_value = cfg.tempUnit ? 1 : 0;
  homekit_characteristic_notify(&cha_temp_units, cha_temp_units.value);

  cha_rotation_speed.value.float_value = fanPercentFromCfg();
  homekit_characteristic_notify(&cha_rotation_speed, cha_rotation_speed.value);

  cha_swing_mode.value.int_value = cfg.swingV ? 1 : 0;
  homekit_characteristic_notify(&cha_swing_mode, cha_swing_mode.value);

  cha_display_light_on.value.bool_value = cfg.displayLight;
  homekit_characteristic_notify(&cha_display_light_on, cha_display_light_on.value);
}

void sendAcState() {
  if (!gpioEnabled(cfg.pinIrSend)) {
    Serial.println("IR send disabled. HomeKit state updated only.");
    updateHomeKitNotify();
    scheduleSaveConfig();
    return;
  }

  Serial.println();
  Serial.println("==== SEND AC STATE ====");
  Serial.print("Protocol: ");
  Serial.println(cfg.protocol);
  Serial.print("Model: ");
  Serial.println(cfg.model);
  Serial.print("IR Send GPIO: ");
  Serial.println(cfg.pinIrSend);
  Serial.print("Power: ");
  Serial.println(cfg.power ? "ON" : "OFF");
  Serial.print("Mode: ");
  Serial.println(cfg.mode);
  Serial.print("Temp: ");
  Serial.println(cfg.targetTemp);
  Serial.print("Fan: ");
  Serial.println(cfg.fan);
  Serial.print("SwingV: ");
  Serial.println(cfg.swingV ? "ON" : "OFF");
  Serial.print("Light: ");
  Serial.println(cfg.displayLight ? "ON" : "OFF");
  Serial.println("=======================");

  IRac ac(cfg.pinIrSend);

  stdAc::state_t state;
  state.protocol = protocolFromString(cfg.protocol);
  state.model = cfg.model;
  state.power = cfg.power;
  state.mode = stdModeFromCfg();
  state.degrees = cfg.targetTemp;
  state.celsius = cfg.tempUnit == 0;
  state.fanspeed = stdFanFromCfg();
  state.swingv = cfg.swingV ? stdAc::swingv_t::kAuto : stdAc::swingv_t::kOff;
  state.swingh = cfg.swingH ? stdAc::swingh_t::kAuto : stdAc::swingh_t::kOff;

  state.quiet = false;
  state.turbo = false;
  state.econo = false;
  state.light = cfg.displayLight;
  state.filter = false;
  state.clean = false;
  state.beep = false;
  state.sleep = -1;
  state.clock = -1;

  setIrLed(true);
  ac.sendAc(state);
  setIrLed(false);

  updateHomeKitNotify();
  scheduleSaveConfig();
}

void applyDecodedState(const decode_results &results) {
  stdAc::state_t state;
  bool ok = IRAcUtils::decodeToState(&results, &state, nullptr);

  if (!ok) {
    Serial.println("IR received, but cannot decode to AC state.");
    Serial.println(resultToHumanReadableBasic(&results));
    return;
  }

  cfg.power = state.power;
  cfg.targetTemp = clampTemp(state.degrees);
  cfg.currentTemp = cfg.targetTemp;
  cfg.tempUnit = state.celsius ? 0 : 1;
  cfg.displayLight = state.light;

  switch (state.mode) {
    case stdAc::opmode_t::kHeat: cfg.mode = 1; break;
    case stdAc::opmode_t::kCool: cfg.mode = 2; break;
    case stdAc::opmode_t::kDry:  cfg.mode = 3; break;
    case stdAc::opmode_t::kFan:  cfg.mode = 4; break;
    default: cfg.mode = 0; break;
  }

  switch (state.fanspeed) {
    case stdAc::fanspeed_t::kLow: cfg.fan = 1; break;
    case stdAc::fanspeed_t::kMedium: cfg.fan = 2; break;
    case stdAc::fanspeed_t::kHigh:
    case stdAc::fanspeed_t::kMax: cfg.fan = 3; break;
    default: cfg.fan = 0; break;
  }

  cfg.swingV = state.swingv != stdAc::swingv_t::kOff;
  cfg.swingH = state.swingh != stdAc::swingh_t::kOff;

  Serial.println("IR AC state decoded and synced to HomeKit.");
  Serial.println(resultToHumanReadableBasic(&results));

  updateHomeKitNotify();
  scheduleSaveConfig();
}

bool learnProtocolFromIr(const decode_results &results) {
  String learnedProtocol = protocolToString(results.decode_type);

  if (learnedProtocol.length() == 0) {
    Serial.println();
    Serial.println("==== LEARN PROTOCOL FAILED ====");
    Serial.print("Unsupported protocol: ");
    Serial.println(typeToString(results.decode_type));
    Serial.println(resultToHumanReadableBasic(&results));
    Serial.println("===============================");
    return false;
  }

  stdAc::state_t state;
  bool ok = IRAcUtils::decodeToState(&results, &state, nullptr);

  cfg.protocol = learnedProtocol;
  cfg.decodeProtocol = learnedProtocol;

  if (ok) {
    cfg.model = state.model;
    cfg.power = state.power;
    cfg.targetTemp = clampTemp(state.degrees);
    cfg.currentTemp = cfg.targetTemp;
    cfg.tempUnit = state.celsius ? 0 : 1;
    cfg.displayLight = state.light;

    switch (state.mode) {
      case stdAc::opmode_t::kHeat: cfg.mode = 1; break;
      case stdAc::opmode_t::kCool: cfg.mode = 2; break;
      case stdAc::opmode_t::kDry:  cfg.mode = 3; break;
      case stdAc::opmode_t::kFan:  cfg.mode = 4; break;
      default: cfg.mode = 0; break;
    }

    switch (state.fanspeed) {
      case stdAc::fanspeed_t::kLow: cfg.fan = 1; break;
      case stdAc::fanspeed_t::kMedium: cfg.fan = 2; break;
      case stdAc::fanspeed_t::kHigh:
      case stdAc::fanspeed_t::kMax: cfg.fan = 3; break;
      default: cfg.fan = 0; break;
    }

    cfg.swingV = state.swingv != stdAc::swingv_t::kOff;
    cfg.swingH = state.swingh != stdAc::swingh_t::kOff;
  } else {
    cfg.model = -1;
  }

  Serial.println();
  Serial.println("==== LEARN PROTOCOL OK ====");
  Serial.print("Protocol: ");
  Serial.println(cfg.protocol);
  Serial.print("Model: ");
  Serial.println(cfg.model);
  Serial.print("Decode Protocol: ");
  Serial.println(cfg.decodeProtocol);
  Serial.println(resultToHumanReadableBasic(&results));
  Serial.println("===========================");

  updateHomeKitNotify();
  saveConfig();

  return true;
}

void handleIrReceive() {
  if (!irrecv) return;

  if (learningProtocol && millis() > learningProtocolUntil) {
    learningProtocol = false;
    Serial.println("Learn protocol timeout.");
  }

  if (irrecv->decode(&irResults)) {
    if (millis() - lastIrMs > 1000) {
      lastIrMs = millis();

      if (learningProtocol) {
        setIrLed(true);

        bool ok = learnProtocolFromIr(irResults);

        setIrLed(false);

        learningProtocol = false;

        if (ok) {
          Serial.println("Protocol learned and saved. Restarting...");
          delay(800);
          ESP.restart();
        }
      } else {
        if (!cfg.irFeedback) {
          irrecv->resume();
          return;
        }

        bool accept = true;

        if (cfg.decodeProtocol != "AUTO") {
          decode_type_t expected = protocolFromString(cfg.decodeProtocol);
          if (irResults.decode_type != expected) {
            accept = false;

            Serial.println();
            Serial.println("IR received but ignored by Decode Protocol filter.");
            Serial.print("Expected: ");
            Serial.println(cfg.decodeProtocol);
            Serial.print("Actual: ");
            Serial.println(typeToString(irResults.decode_type));
          }
        }

        if (accept) {
          setIrLed(true);
          applyDecodedState(irResults);
          setIrLed(false);
        }
      }
    }

    irrecv->resume();
  }
}

void setupHomeKitCallbacks();

void startHomeKitIfNeeded() {
  if (homekitStarted) return;
  if (WiFi.status() != WL_CONNECTED) return;

  setupHomeKitCallbacks();
  arduino_homekit_setup(&config);

  homekitStarted = true;
  wifiConnected = true;

  Serial.println("HomeKit started.");
}

void startAP() {
  if (apStarted) return;

  apStarted = true;
  apMode = true;

  scanStrongestWifi();

  String apName = "AC_";
  apName += String(ESP.getChipId(), HEX);

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(apName.c_str());

  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

  wifiLedState = WIFI_LED_FAST_BLINK;

  Serial.println();
  Serial.println("==== AP FALLBACK STARTED ====");
  Serial.print("AP SSID: ");
  Serial.println(apName);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("STA will keep retrying saved Wi-Fi in background.");
  Serial.println("=============================");
}

void stopAPIfRunning() {
  if (!apStarted) return;

  dnsServer.stop();
  WiFi.softAPdisconnect(true);

  apStarted = false;
  apMode = false;

  Serial.println();
  Serial.println("AP stopped because saved Wi-Fi is connected.");
}

void beginWifiConnect() {
  if (cfg.ssid.length() == 0) {
    Serial.println();
    Serial.println("No saved Wi-Fi. Starting AP setup mode.");
    wifiConnected = false;
    wifiLedState = WIFI_LED_FAST_BLINK;
    startAP();
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);

  WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());

  wifiConnectStartMs = millis();
  lastWifiRetryMs = millis();
  lastWifiCheckMs = 0;
  lastWifiLogMs = 0;

  wifiConnected = false;
  wifiLedState = WIFI_LED_SLOW_BLINK;

  Serial.println();
  Serial.println("==== WIFI CONNECT START ====");
  Serial.print("SSID: ");
  Serial.println(cfg.ssid);
  Serial.println("AP fallback will start if Wi-Fi is not connected in 10 minutes.");
  Serial.println("============================");
}

void handleWifiConnection() {
  unsigned long now = millis();

  if (now - lastWifiCheckMs < WIFI_CHECK_INTERVAL_MS) return;
  lastWifiCheckMs = now;

  if (cfg.ssid.length() == 0) {
    wifiConnected = false;
    wifiLedState = WIFI_LED_FAST_BLINK;
    startAP();
    return;
  }

  wl_status_t status = WiFi.status();

  if (status == WL_CONNECTED) {
    if (!wifiConnected) {
      wifiConnected = true;

      Serial.println();
      Serial.println("==== WIFI CONNECTED ====");
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());
      Serial.print("SSID: ");
      Serial.println(WiFi.SSID());
      Serial.print("RSSI: ");
      Serial.println(WiFi.RSSI());
      Serial.println("========================");

      wifiLedState = WIFI_LED_OFF;
      setWifiLed(false);

      stopAPIfRunning();
      startHomeKitIfNeeded();
    }

    return;
  }

  if (wifiConnected) {
    Serial.println();
    Serial.println("Wi-Fi disconnected. Keep retrying...");
  }

  wifiConnected = false;
  wifiLedState = WIFI_LED_SLOW_BLINK;

  if (!apStarted && now - wifiConnectStartMs > WIFI_AP_FALLBACK_MS) {
    Serial.println();
    Serial.println("Wi-Fi not connected after 10 minutes. Starting AP fallback.");
    startAP();
  }

  if (now - lastWifiRetryMs > WIFI_RETRY_INTERVAL_MS) {
    lastWifiRetryMs = now;

    Serial.print("Retry Wi-Fi: ");
    Serial.println(cfg.ssid);

    WiFi.disconnect();
    delay(20);
    WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());
  }

  if (now - lastWifiLogMs > 5000) {
    lastWifiLogMs = now;

    Serial.print("Wi-Fi status: ");
    Serial.print(status);
    Serial.print(" | AP fallback: ");
    Serial.println(apStarted ? "ON" : "OFF");
  }
}

void startWebServer() {
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", pageHtml());
  });

  server.on("/save", HTTP_POST, []() {
    String newSsid = server.arg("ssid");
    String newPass = server.arg("pass");

    newSsid.trim();
    newPass.trim();

    if (newSsid.length() > 0) {
      cfg.ssid = newSsid;
    }

    if (newPass.length() > 0) {
      cfg.pass = newPass;
    }

    cfg.protocol = server.arg("protocol");
    cfg.model = server.arg("model").toInt();
    cfg.decodeProtocol = server.arg("decodeProtocol");
    cfg.irFeedback = server.hasArg("irFeedback");

    cfg.hkPin = normalizePin(server.arg("hkPin"));

    cfg.pinIrSend = parseGpioArg("pinIrSend", cfg.pinIrSend);
    cfg.pinIrRecv = parseGpioArg("pinIrRecv", cfg.pinIrRecv);
    cfg.pinButton = parseGpioArg("pinButton", cfg.pinButton);
    cfg.pinLedWifi = parseGpioArg("pinLedWifi", cfg.pinLedWifi);
    cfg.pinLedIr = parseGpioArg("pinLedIr", cfg.pinLedIr);
    cfg.pinReservedButton = parseGpioArg("pinReservedButton", cfg.pinReservedButton);

    cfg.ledWifiActiveHigh = server.arg("ledWifiActiveHigh").toInt() == 1;
    cfg.ledIrActiveHigh = server.arg("ledIrActiveHigh").toInt() == 1;

    Serial.println();
    Serial.println("==== SAVE CONFIG ====");
    Serial.print("SSID: ");
    Serial.println(cfg.ssid);
    Serial.print("Protocol: ");
    Serial.println(cfg.protocol);
    Serial.print("Model: ");
    Serial.println(cfg.model);
    Serial.print("Decode Protocol: ");
    Serial.println(cfg.decodeProtocol);
    Serial.print("IR Feedback: ");
    Serial.println(cfg.irFeedback ? "ON" : "OFF");
    Serial.print("IR Send GPIO: ");
    Serial.println(cfg.pinIrSend);
    Serial.print("IR Recv GPIO: ");
    Serial.println(cfg.pinIrRecv);
    Serial.print("KEY1 GPIO: ");
    Serial.println(cfg.pinButton);
    Serial.print("LED WiFi GPIO: ");
    Serial.println(cfg.pinLedWifi);
    Serial.print("LED WiFi Active Level: ");
    Serial.println(cfg.ledWifiActiveHigh ? "HIGH" : "LOW");
    Serial.print("LED IR GPIO: ");
    Serial.println(cfg.pinLedIr);
    Serial.print("LED IR Active Level: ");
    Serial.println(cfg.ledIrActiveHigh ? "HIGH" : "LOW");
    Serial.print("KEY2 GPIO: ");
    Serial.println(cfg.pinReservedButton);
    Serial.println("=====================");

    saveConfig();

    server.send(200, "text/html", "<meta charset='utf-8'>已保存，设备即将重启。");
    delay(800);
    ESP.restart();
  });

  server.on("/learn_protocol", HTTP_POST, []() {
    if (!gpioEnabled(cfg.pinIrRecv)) {
      server.send(
        200,
        "text/html",
        "<meta charset='utf-8'>"
        "<h3>IR 接收 GPIO 已禁用</h3>"
        "<p>请先在 GPIO 设置中启用 IR 接收 GPIO。</p>"
      );
      return;
    }

    learningProtocol = true;
    learningProtocolUntil = millis() + 10000;

    Serial.println();
    Serial.println("==== LEARN PROTOCOL START ====");
    Serial.println("Press original AC remote button within 10 seconds.");
    Serial.println("==============================");

    server.send(
      200,
      "text/html",
      "<meta charset='utf-8'>"
      "<h3>已进入协议学习模式</h3>"
      "<p>请在 10 秒内用原装空调遥控器对准设备按一次开机、制冷或温度键。</p>"
      "<p>识别成功后设备会自动保存并重启。</p>"
    );
  });

  server.on(
    "/ota_update",
    HTTP_POST,
    []() {
      bool ok = !Update.hasError();

      server.send(
        ok ? 200 : 500,
        "text/plain",
        ok ? "OTA OK. Rebooting..." : "OTA FAILED"
      );

      delay(800);

      if (ok) {
        ESP.restart();
      }
    },
    []() {
      HTTPUpload &upload = server.upload();

      if (upload.status == UPLOAD_FILE_START) {
        Serial.println();
        Serial.println("==== OTA START ====");
        Serial.print("Filename: ");
        Serial.println(upload.filename);

        uint32_t maxSketchSpace =
            (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;

        if (!Update.begin(maxSketchSpace)) {
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
        }

        Serial.print("OTA received: ");
        Serial.println(upload.totalSize);
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
          Serial.println("==== OTA OK ====");
          Serial.print("Size: ");
          Serial.println(upload.totalSize);
        } else {
          Serial.println("==== OTA FAILED ====");
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_ABORTED) {
        Update.end();
        Serial.println("OTA aborted.");
      }
    }
  );

  server.on("/reboot", HTTP_POST, []() {
    server.send(200, "text/html", "<meta charset='utf-8'><h3>正在重启...</h3>");
    delay(500);
    ESP.restart();
  });

  server.on("/start_ap", HTTP_POST, []() {
    startAP();

    server.send(
      200,
      "text/html",
      "<meta charset='utf-8'>"
      "<h3>AP 已启动</h3>"
      "<p>请连接 AC_xxxxxx 热点后访问 192.168.4.1。</p>"
      "<p>设备仍会在后台继续尝试连接已保存的 Wi-Fi。</p>"
    );
  });

  server.on("/reset_homekit", HTTP_POST, []() {
    server.send(
      200,
      "text/html",
      "<meta charset='utf-8'>"
      "<h3>正在重置 HomeKit...</h3>"
      "<p>请稍后在家庭 App 删除旧配件并重新添加。</p>"
    );

    delay(500);

    homekit_storage_reset();

    delay(500);
    ESP.restart();
  });

  server.on("/reset_wifi", HTTP_POST, []() {
    cfg.ssid = "";
    cfg.pass = "";
    saveConfig();

    server.send(
      200,
      "text/html",
      "<meta charset='utf-8'>"
      "<h3>Wi-Fi 已重置</h3>"
      "<p>设备即将重启并进入 AP 配网模式。</p>"
    );

    delay(800);
    ESP.restart();
  });

  server.onNotFound([]() {
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });

  server.begin();
}

extern "C" void identify(homekit_value_t value) {
  if (!gpioEnabled(cfg.pinLedIr)) return;

  for (uint8_t i = 0; i < 3; i++) {
    setIrLed(true);
    delay(150);
    setIrLed(false);
    delay(150);
  }
}

void activeSetter(const homekit_value_t value) {
  bool requestedPower = value.int_value == 1;

  Serial.print("HomeKit Active Setter: ");
  Serial.println(requestedPower ? "ON" : "OFF");

  if (!requestedPower && cfg.power && millis() - lastFanSpeedChangeMs < IGNORE_ACTIVE_OFF_AFTER_FAN_MS) {
    Serial.println("Ignore Active OFF because it happened immediately after fan speed change.");
    cfg.power = true;
    updateHomeKitNotify();
    return;
  }

  cfg.power = requestedPower;

  if (cfg.power) {
    if (cfg.mode != 1 && cfg.mode != 2 && cfg.mode != 3 && cfg.mode != 4) {
      cfg.mode = 2;  // 默认制冷
    }

    if (cfg.fan > 3) {
      cfg.fan = 0;  // 默认自动风
    }

    cfg.targetTemp = clampTemp(cfg.targetTemp);
    cfg.currentTemp = cfg.targetTemp;
  }

  updateHomeKitNotify();
  scheduleSendAcState();
}

void targetStateSetter(const homekit_value_t value) {
  int v = value.int_value;

  bool wasOff = !cfg.power;

  cfg.power = true;

  if (v == 1) {
    cfg.mode = 1;  // Heat
  } else if (v == 2) {
    cfg.mode = 2;  // Cool
  } else {
    cfg.mode = 0;  // Auto
  }

  /*
    从关闭状态直接切到制冷 / 制热时，补全一次完整运行状态。
    不重复发送红外，只让本次红外包含完整 power/mode/temp/fan/swing 状态。
  */
  if (wasOff) {
    cfg.targetTemp = clampTemp(cfg.targetTemp);

    if (cfg.targetTemp < AC_MIN_TEMP || cfg.targetTemp > AC_MAX_TEMP) {
      cfg.targetTemp = 26.0;
    }

    cfg.currentTemp = cfg.targetTemp;

    if (cfg.fan > 3) {
      cfg.fan = 0;  // Auto / 空调默认自动风
    }

    cfg.swingV = false;
    cfg.swingH = false;
  } else {
    if (cfg.fan > 3) {
      cfg.fan = 0;
    }

    cfg.targetTemp = clampTemp(cfg.targetTemp);
    cfg.currentTemp = cfg.targetTemp;
  }

  updateHomeKitNotify();
  scheduleSendAcState();
}

void coolingThresholdSetter(const homekit_value_t value) {
  cfg.targetTemp = clampTemp(value.float_value);
  cfg.currentTemp = cfg.targetTemp;
  cfg.power = true;
  cfg.mode = 2;

  scheduleSendAcState();
}

void heatingThresholdSetter(const homekit_value_t value) {
  cfg.targetTemp = clampTemp(value.float_value);
  cfg.currentTemp = cfg.targetTemp;
  cfg.power = true;
  cfg.mode = 1;

  scheduleSendAcState();
}

void tempUnitSetter(const homekit_value_t value) {
  cfg.tempUnit = value.int_value ? 1 : 0;
  updateHomeKitNotify();
  scheduleSaveConfig();
}

void rotationSpeedSetter(const homekit_value_t value) {
  float speed = value.float_value;

  Serial.print("HomeKit RotationSpeed Setter: ");
  Serial.println(speed);

  lastFanSpeedChangeMs = millis();

  if (speed <= 25) {
    cfg.fan = 0;       // Auto / 空调默认自动风
  } else if (speed <= 50) {
    cfg.fan = 1;       // 低风
  } else if (speed <= 75) {
    cfg.fan = 2;       // 中风
  } else {
    cfg.fan = 3;       // 高风
  }

  cfg.power = true;

  if (cfg.mode != 1 && cfg.mode != 2 && cfg.mode != 3 && cfg.mode != 4) {
    cfg.mode = 2;      // 默认制冷
  }

  cfg.targetTemp = clampTemp(cfg.targetTemp);
  cfg.currentTemp = cfg.targetTemp;

  updateHomeKitNotify();
  scheduleSendAcState();
}

void swingModeSetter(const homekit_value_t value) {
  cfg.swingV = value.int_value == 1;
  cfg.power = true;
  scheduleSendAcState();
}

void displayLightSetter(const homekit_value_t value) {
  cfg.displayLight = value.bool_value;
  scheduleSendAcState();
}

void setupHomeKitCallbacks() {
  setupFirmwareVersion();

  cha_active.setter = activeSetter;
  cha_target_state.setter = targetStateSetter;
  cha_cooling_threshold.setter = coolingThresholdSetter;
  cha_heating_threshold.setter = heatingThresholdSetter;
  cha_temp_units.setter = tempUnitSetter;
  cha_rotation_speed.setter = rotationSpeedSetter;
  cha_swing_mode.setter = swingModeSetter;
  cha_display_light_on.setter = displayLightSetter;

  cha_active.value.int_value = cfg.power ? 1 : 0;
  cha_current_temp.value.float_value = cfg.currentTemp;

  int currentState = 0;
  if (cfg.power) {
    if (cfg.mode == 1) {
      currentState = 2;
    } else if (cfg.mode == 2) {
      currentState = 3;
    } else {
      currentState = 1;
    }
  }

  int targetState = 0;
  if (cfg.mode == 1) {
    targetState = 1;
  } else if (cfg.mode == 2) {
    targetState = 2;
  }

  cha_current_state.value.int_value = currentState;
  cha_target_state.value.int_value = targetState;

  syncThresholdsForHomeKit();

  cha_temp_units.value.int_value = cfg.tempUnit ? 1 : 0;
  cha_rotation_speed.value.float_value = fanPercentFromCfg();
  cha_swing_mode.value.int_value = cfg.swingV ? 1 : 0;
  cha_display_light_on.value.bool_value = cfg.displayLight;

  config.password = (char *)cfg.hkPin.c_str();
}

void setupPins() {
  if (gpioEnabled(cfg.pinLedWifi)) {
    pinMode(cfg.pinLedWifi, OUTPUT);
    setWifiLed(false);
  }

  if (gpioEnabled(cfg.pinLedIr)) {
    pinMode(cfg.pinLedIr, OUTPUT);
    setIrLed(false);
  }

  if (gpioEnabled(cfg.pinButton)) {
    pinMode(cfg.pinButton, INPUT_PULLUP);
  }

  if (gpioEnabled(cfg.pinReservedButton)) {
    pinMode(cfg.pinReservedButton, INPUT_PULLUP);
  }
}

void setupIrRecv() {
  if (irrecv) {
    delete irrecv;
    irrecv = nullptr;
  }

  if (!gpioEnabled(cfg.pinIrRecv)) {
    Serial.println("IR recv disabled.");
    return;
  }

  irrecv = new IRrecv(cfg.pinIrRecv, IR_RECV_BUFFER, IR_RECV_TIMEOUT, true);
  irrecv->enableIRIn();

  Serial.print("IR recv enabled on GPIO");
  Serial.println(cfg.pinIrRecv);
}

void handleButton() {
  if (!gpioEnabled(cfg.pinButton)) return;

  static bool keyPressed = false;
  static bool wifiResetHandled = false;
  static bool factoryResetHandled = false;
  static unsigned long keyPressedAt = 0;

  if (digitalRead(cfg.pinButton) == LOW) {
    if (!keyPressed) {
      keyPressed = true;
      wifiResetHandled = false;
      factoryResetHandled = false;
      keyPressedAt = millis();

      Serial.println("KEY1 pressed. Hold 5 seconds to reset Wi-Fi, 10 seconds to reset Wi-Fi and HomeKit.");
    }

    unsigned long heldMs = millis() - keyPressedAt;

    if (!wifiResetHandled && heldMs >= 5000) {
      wifiResetHandled = true;

      Serial.println();
      Serial.println("==== KEY1 WIFI RESET READY ====");
      Serial.println("Release before 10 seconds to clear Wi-Fi only.");
      Serial.println("Keep holding to 10 seconds to clear Wi-Fi and HomeKit.");
      Serial.println("===============================");
    }

    if (!factoryResetHandled && heldMs >= 10000) {
      factoryResetHandled = true;

      Serial.println();
      Serial.println("==== KEY1 FACTORY RESET ====");
      Serial.println("Clear Wi-Fi config.");
      Serial.println("Reset HomeKit pairing data.");
      Serial.println("Device will reboot to AP setup mode.");
      Serial.println("============================");

      cfg.ssid = "";
      cfg.pass = "";
      saveConfig();

      homekit_storage_reset();

      delay(800);
      ESP.restart();
    }
  } else {
    if (keyPressed && wifiResetHandled && !factoryResetHandled) {
      Serial.println();
      Serial.println("==== KEY1 WIFI RESET ====");
      Serial.println("Clear Wi-Fi config only.");
      Serial.println("HomeKit pairing data is kept.");
      Serial.println("Device will reboot to AP setup mode.");
      Serial.println("=========================");

      cfg.ssid = "";
      cfg.pass = "";
      saveConfig();

      delay(800);
      ESP.restart();
    }

    keyPressed = false;
    wifiResetHandled = false;
    factoryResetHandled = false;
  }
}

void handleReservedButton() {
  if (!gpioEnabled(cfg.pinReservedButton)) return;

  static unsigned long lastReservedButtonMs = 0;

  if (digitalRead(cfg.pinReservedButton) == LOW && millis() - lastReservedButtonMs > 1000) {
    lastReservedButtonMs = millis();

    Serial.print("KEY2 button pressed on GPIO");
    Serial.println(cfg.pinReservedButton);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("OW8266 HomeKit IR AC final build starting...");

  LittleFS.begin();
  loadConfig();

  resetRuntimeAcStateAfterBoot();

  setupPins();
  setupFirmwareVersion();
  setupIrRecv();

  beginWifiConnect();

  startWebServer();

  Serial.println("Web server started.");
}

void loop() {
  handleWifiLed();
  handleWifiConnection();

  if (apMode) {
    dnsServer.processNextRequest();
  }

  server.handleClient();

  if (homekitStarted) {
    arduino_homekit_loop();
  }

  if (pendingSend && millis() > pendingSendAt) {
    pendingSend = false;
    sendAcState();
  }

  if (pendingSave && millis() > pendingSaveAt) {
    saveConfig();
  }

  static unsigned long lastIrPoll = 0;
  if (millis() - lastIrPoll > 120) {
    lastIrPoll = millis();
    handleIrReceive();
  }

  handleButton();
  handleReservedButton();

  delay(10);
}
