#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>

// ================= Wi-Fi =================
const char *SSID = "Lalyn_2.4G";
const char *PASS = "Panda8133";

// ================= Tesira TCP =================
const char *TESIRA_IP = "192.168.0.50";
const uint16_t TESIRA_PORT = 23;

WiFiClient tesira;
unsigned long lastTConnTry = 0;
const unsigned long RECONN_MS = 3000;

// ================= WebSocket (สำหรับแอพ) =================
WebSocketsServer ws(81);

// ================= App State (เล็ก ๆ สำหรับตอบกลับ) =================
String curBlock = "Analog_Level";
int curCh = 1;
float levelDb = -60.0;
bool isMute = false;

// ================= Smart Logger =================
bool LOG_EVENTS = true;
bool LOG_ERRORS = true;
bool LOG_WELCOME = true;
bool LOG_OK_SUMMARY = false;
bool LOG_RAW = false;

const unsigned long REPEAT_WINDOW_MS = 300;
unsigned long _logLastMs = 0;
String _logLastLine;

// ===== ตัวแปรมิเตอร์ล่าสุด =====
float analogLeft = -100.0f, analogRight = -100.0f;
float usbLeft = -100.0f, usbRight = -100.0f;

// pace การส่ง VU
unsigned long lastSendAnalogVU = 0, lastSendUsbVU = 0;
const unsigned long vuInterval = 12;

// ===== Control state cache (แทน levelDb/isMute เดิม) =====
struct CtrlState {
  float level;   bool hasLevel;
  bool  mute;    bool hasMute;
};
CtrlState ctrlAnalog[3] = {{0,false},{0,false},{0,false}}; // ch1..2 ใช้ index 1..2
CtrlState ctrlUsb[3]    = {{0,false},{0,false},{0,false}};


CtrlState* pickCtrl(const String& alias) {
  if (alias=="Analog_Level") return ctrlAnalog;
  if (alias=="USB_Level")    return ctrlUsb;
  
  return nullptr;
}

// ส่งอัปเดต control ไปแอพ (เฉพาะฟิลด์ที่มี)
void wsSendCtrl(const String& alias, int ch, bool sendLevel, float lv, bool sendMute, bool m) {
  StaticJsonDocument<160> d;
  d["block"] = alias;
  d["ch"]    = ch;
  if (sendLevel) d["level_db"] = lv;
  if (sendMute)  d["mute"]     = m;
  String s; serializeJson(d, s);
  ws.broadcastTXT(s);
}

// ===== map publishToken -> (alias,ch) =====
struct SubMap
{
  String token;
  String alias;
  int ch;
};
static SubMap gSubs[16];
static int gSubsN = 0;

void addSubMap(const String &token, const String &alias, int ch)
{
  if (!token.length())
    return;
  for (int i = 0; i < gSubsN; i++)
  {
    if (gSubs[i].token == token)
    {
      gSubs[i].alias = alias;
      gSubs[i].ch = ch;
      return;
    }
  }
  if (gSubsN < (int)(sizeof(gSubs) / sizeof(gSubs[0])))
  {
    gSubs[gSubsN++] = SubMap{token, alias, ch};
  }
}

bool findSubMap(const String &token, String &alias, int &ch)
{
  for (int i = 0; i < gSubsN; i++)
  {
    if (gSubs[i].token == token)
    {
      alias = gSubs[i].alias;
      ch = gSubs[i].ch;
      return true;
    }
  }
  return false;
}

// ===== boot subscribe ถูกยิงครั้งเดียว =====
bool gBootDone = false;

// ================= Logger =================
void logTTP(const String &l)
{
  if (l.isEmpty())
    return;

  const unsigned long now = millis();
  if (l == _logLastLine && (now - _logLastMs) < REPEAT_WINDOW_MS)
    return;
  _logLastLine = l;
  _logLastMs = now;

  if (LOG_RAW)
  {
    Serial.print("[TTP <<] ");
    Serial.println(l);
    return;
  }

  if (l.startsWith("Welcome"))
  {
    if (LOG_WELCOME)
      Serial.println("[TTP] Welcome");
    return;
  }
  if (l.startsWith("-ERR"))
  {
    if (LOG_ERRORS)
    {
      Serial.print("[TTP ERR] ");
      Serial.println(l);
    }
    return;
  }

  if (l.startsWith("EVENT"))
  {
    if (!LOG_EVENTS)
      return;
    int p1 = l.indexOf(' '), p2 = l.indexOf(' ', p1 + 1), p3 = l.indexOf(' ', p2 + 1);
    if (p1 > 0 && p2 > p1 && p3 > p2)
    {
      String alias = l.substring(p1 + 1, p2);
      String attr = l.substring(p2 + 1, p3);
      String rest = l.substring(p3 + 1);
      Serial.printf("[EVT] %s %s %s\n", alias.c_str(), attr.c_str(), rest.c_str());
    }
    else
    {
      Serial.print("[EVT] ");
      Serial.println(l);
    }
    return;
  }

  if (l.startsWith("+OK"))
  {
    if (!LOG_OK_SUMMARY)
      return;
    if (l.indexOf("level") >= 0 || l.indexOf("mute") >= 0)
    {
      Serial.print("[OK] ");
      Serial.println(l);
    }
    return;
  }
}

// ================= ส่งสถานะไปแอพ =================
void sendStateTo(uint8_t num = 0xFF)
{
  DynamicJsonDocument doc(256);
  doc["block"] = curBlock;
  doc["ch"] = curCh;
  doc["level_db"] = levelDb;
  doc["mute"] = isMute;
  String s;
  serializeJson(doc, s);
  if (num == 0xFF)
    ws.broadcastTXT(s);
  else
    ws.sendTXT(num, s);
}

// ================= TTP ส่งคำสั่ง (ต่อท้าย CRLF) =================
void ttpSend(const String &line)
{
  if (!tesira.connected())
    return;
  String out = line;
  if (!out.endsWith("\r\n"))
    out += "\r\n";
  tesira.print(out);
  Serial.print("[TTP >>] ");
  Serial.println(line);
}

// ================= จำบริบท +OK =================
String gLastAlias, gLastAttr;
int gLastCh = 0;
inline void rememberCmd(const String &a, const String &at, int ch)
{
  gLastAlias = a;
  gLastAttr = at;
  gLastCh = ch;
}

// <alias> get/set/subscribe level|mute <ch>
void ttpGetLevel(const String &alias, int ch)
{
  rememberCmd(alias, "level", ch);
  ttpSend(alias + " get level " + String(ch));
}
void ttpSetLevel(const String &alias, int ch, float dB)
{
  rememberCmd(alias, "level", ch);
  char buf[16];
  dtostrf(dB, 0, 1, buf);
  ttpSend(alias + " set level " + String(ch) + " " + String(buf));
}
void ttpSetMute(const String &alias, int ch, bool on)
{
  rememberCmd(alias, "mute", ch);
  ttpSend(alias + " set mute " + String(ch) + " " + (on ? "true" : "false"));
}

// ================= Telnet negotiate (IAC) =================
inline void telnetSend(uint8_t a, uint8_t b, uint8_t c)
{
  uint8_t buf[3] = {a, b, c};
  tesira.write(buf, 3);
}

int ttpReadFilteredByte()
{
  if (!tesira.available())
    return -1;
  int b = tesira.read();
  if (b != 255)
    return b;

  while (!tesira.available())
    delay(1);
  int cmd = tesira.read();

  if (cmd == 251 || cmd == 252 || cmd == 253 || cmd == 254)
  {
    while (!tesira.available())
      delay(1);
    int opt = tesira.read();
    if (cmd == 251 || cmd == 252)
      telnetSend(255, 254, opt); // DONT
    else
      telnetSend(255, 252, opt); // WONT
    return -2;
  }

  if (cmd == 250)
  { // SB ... SE
    int prev = -1, cur = -1;
    while (true)
    {
      while (!tesira.available())
        delay(1);
      cur = tesira.read();
      if (prev == 255 && cur == 240)
        break;
      prev = cur;
    }
    return -2;
  }

  if (cmd == 240)
    return -2; // SE
  return -2;
}

bool ttpReadLine(String &outLine)
{
  static String buf;
  while (tesira.connected() && tesira.available())
  {
    int r = ttpReadFilteredByte();
    if (r == -1)
      break;
    if (r == -2)
      continue;
    char c = (char)r;
    if (c == '\r')
      continue;
    buf += c;
    if (c == '\n')
    {
      outLine = buf;
      buf = "";
      return true;
    }
  }
  return false;
}

// ================= subscribe de-dup (สำหรับ Level/Mute แบบปกติ) =================
struct SubKey
{
  String alias, attr;
  int ch;
};
static SubKey _subs[64];
static int _subsN = 0;
bool _same(const SubKey &a, const SubKey &b) { return a.alias == b.alias && a.attr == b.attr && a.ch == b.ch; }
bool _hasSub(const String &alias, const String &attr, int ch)
{
  for (int i = 0; i < _subsN; i++)
    if (_same(_subs[i], SubKey{alias, attr, ch}))
      return true;
  return false;
}
void _markSub(const String &alias, const String &attr, int ch)
{
  if (_subsN < (int)(sizeof(_subs) / sizeof(_subs[0])))
    _subs[_subsN++] = SubKey{alias, attr, ch};
}
void subscribeLevelOnce(const String &alias, int ch)
{
  if (!_hasSub(alias, "level", ch))
  {
    ttpSend(alias + " subscribe level " + String(ch));
    _markSub(alias, "level", ch);
  }
}
void subscribeMuteOnce(const String &alias, int ch)
{
  if (!_hasSub(alias, "mute", ch))
  {
    ttpSend(alias + " subscribe mute " + String(ch));
    _markSub(alias, "mute", ch);
  }
}

// ================= Boot Subscribe แบบมี token =================
void bootSubscribeWithTokens()
{
  if (gBootDone)
    return;
  gBootDone = true;

  ttpSend("SESSION set verbose true");

  // USB meters
  ttpSend("USB_Meter subscribe level 1 USB1");
  addSubMap("USB1", "USB_Meter", 1);
  ttpSend("USB_Meter subscribe level 2 USB2");
  addSubMap("USB2", "USB_Meter", 2);

  // Analog meters (ถ้ามี)
  ttpSend("Analog_Meter subscribe level 1 AN1");
  addSubMap("AN1", "Analog_Meter", 1);
  ttpSend("Analog_Meter subscribe level 2 AN2");
  addSubMap("AN2", "Analog_Meter", 2);

  // snapshot (ออปชั่น)
  ttpSend("USB_Meter get level 1");
  ttpSend("USB_Meter get level 2");
  ttpSend("Analog_Meter get level 1");
  ttpSend("Analog_Meter get level 2");
}

// ===== ส่ง L/R พร้อมกัน เพื่อลดโอเวอร์เฮด JSON/WS =====
void sendVU(const char *alias, float left, float right, unsigned long &lastSendTime)
{
  const unsigned long now = millis();
  if (now - lastSendTime < vuInterval)
    return;

  // ปัดทศนิยมเล็กน้อย ช่วย serialize/ส่งไวขึ้น
  float l = roundf(left * 10.0f) / 10.0f;
  float r = roundf(right * 10.0f) / 10.0f;

  StaticJsonDocument<128> doc;
  doc["alias"] = alias;
  doc["block"] = alias;
  doc["left"] = l;
  doc["right"] = r;

  String s;
  serializeJson(doc, s);
  ws.broadcastTXT(s);
  lastSendTime = now;

  // ถ้าจะ debug:
  // Serial.printf("[WS TX] %s  L=%.1f R=%.1f\n", alias, l, r);
}

// ================= parse ตอบกลับ TTP -> อัปเดต + ส่งต่อแอพ =================
void handleTtpLine(const String &line)
{
  String l = line;
  l.trim();
  if (l.isEmpty())
    return;

  // ดู raw ได้ถ้าต้องการ
  // Serial.print("[TTP <<] "); Serial.println(l);

  // 1) ค่ามิเตอร์แบบมี publishToken:  ! "publishToken":"USB1" "value":-53.42
  // 1) ค่ามิเตอร์แบบมี publishToken
  if (l.startsWith("!"))
  {
    int t1 = l.indexOf("\"publishToken\"");
    int v1 = l.indexOf("\"value\"");
    if (t1 >= 0 && v1 >= 0)
    {
      int q1 = l.indexOf('"', t1 + 15);
      int q2 = (q1 >= 0) ? l.indexOf('"', q1 + 1) : -1;
      String token = (q1 >= 0 && q2 > q1) ? l.substring(q1 + 1, q2) : "";
      int colon = l.indexOf(':', v1);
      float val = (colon > 0) ? l.substring(colon + 1).toFloat() : -100.0f;

      if (token == "USB1")
      {
        usbLeft = val;
        sendVU("USB_Meter", usbLeft, usbRight, lastSendUsbVU);
      }
      else if (token == "USB2")
      {
        usbRight = val;
        sendVU("USB_Meter", usbLeft, usbRight, lastSendUsbVU);
      }
      else if (token == "AN1")
      {
        analogLeft = val;
        sendVU("Analog_Meter", analogLeft, analogRight, lastSendAnalogVU);
      }
      else if (token == "AN2")
      {
        analogRight = val;
        sendVU("Analog_Meter", analogLeft, analogRight, lastSendAnalogVU);
      }

      return;
    }
  }

  // 2) EVENT (ถ้าเผื่อมีใช้ในบล็อกอื่น)
 if (l.startsWith("EVENT")) {
  // EVENT <alias> <attr> <ch> <value>
  auto parts = String(l).substring(6); parts.trim();
  int p1=parts.indexOf(' '), p2=parts.indexOf(' ',p1+1), p3=parts.indexOf(' ',p2+1);
  if (p1>0 && p2>p1 && p3>p2) {
    String alias = parts.substring(0,p1);
    String attr  = parts.substring(p1+1,p2);
    int    ch    = parts.substring(p2+1,p3).toInt();
    float  v     = parts.substring(p3+1).toFloat();

    // 1) meter blocks (เดิม)
    if (alias=="USB_Meter" || alias=="Analog_Meter") {
      if (alias=="USB_Meter") {
        if (ch==1) usbLeft=v; else if (ch==2) usbRight=v;
        sendVU("USB_Meter", usbLeft, usbRight, lastSendUsbVU);
      } else {
        if (ch==1) analogLeft=v; else if (ch==2) analogRight=v;
        sendVU("Analog_Meter", analogLeft, analogRight, lastSendAnalogVU);
      }
      return;
    }

    // 2) control blocks (Analog_Level / USB_Level
    if (attr=="level" || attr=="mute") {
      if (CtrlState* cs = pickCtrl(alias)) {
        if (attr=="level") { cs[ch].level=v; cs[ch].hasLevel=true; wsSendCtrl(alias, ch, true, v, false, false); }
        else { bool m = (v!=0.0f); cs[ch].mute=m; cs[ch].hasMute=true; wsSendCtrl(alias, ch, false, 0, true, m); }
      }
    }
  }
  return;
}

  // 3) +OK (snapshot จาก get level ...)
  if (l.startsWith("+OK")) {
  // ดึงค่าตัวเลขจากท้ายบรรทัด หรือหลัง "value":
  float v = NAN;
  int iv = l.indexOf("\"value\":");
  if (iv >= 0) {
    v = l.substring(iv + 8).toFloat();
  } else {
    int colon = l.lastIndexOf(':');
    if (colon > 0) v = l.substring(colon+1).toFloat();
  }

  // ถ้าเป็น meter (เดิม)
  if (gLastAlias=="USB_Meter" || gLastAlias=="Analog_Meter") {
    if (!isnan(v)) {
      if (gLastAlias=="USB_Meter") {
        if (gLastCh==1) usbLeft=v; else if (gLastCh==2) usbRight=v;
        sendVU("USB_Meter", usbLeft, usbRight, lastSendUsbVU);
      } else {
        if (gLastCh==1) analogLeft=v; else if (gLastCh==2) analogRight=v;
        sendVU("Analog_Meter", analogLeft, analogRight, lastSendAnalogVU);
      }
    }
    return;
  }

  // ถ้าเป็น control blocks
  if (CtrlState* cs = pickCtrl(gLastAlias)) {
    if (gLastAttr=="level" && !isnan(v)) {
      cs[gLastCh].level = v; cs[gLastCh].hasLevel = true;
      wsSendCtrl(gLastAlias, gLastCh, true, v, false, false);
    } else if (gLastAttr=="mute") {
      // บางเฟิร์มแวร์ +OK ของ mute ไม่มี "value:" → ใช้ข้อความลงท้าย true/false
      bool m = (l.endsWith("true") || l.endsWith("1"));
      cs[gLastCh].mute = m; cs[gLastCh].hasMute = true;
      wsSendCtrl(gLastAlias, gLastCh, false, 0, true, m);
    }
    return;
  }

  return;
}

  // 4) อื่น ๆ ไม่สนใจ
}

// ===== เรียกส่งทุกแชนเนลที่สนใจ =====
void updateMeters()
{
  sendVU("Analog_Meter", analogLeft, analogRight, lastSendAnalogVU);
  sendVU("USB_Meter", usbLeft, usbRight, lastSendUsbVU);
}

// ===== ดูดข้อมูลจาก Tesira แบบ time-slice (หัวใจลดดีเลย์) =====
// budgetMicros = งบเวลาอ่านต่อเฟรม (ปรับได้ 4000–10000 µs)
void pumpTesira(unsigned long budgetMicros = 6000)
{
  unsigned long t0 = micros();
  String line;
  while (tesira.connected() && ttpReadLine(line))
  {
    handleTtpLine(line); // ← อัปเดต analog*/usb* ที่นี่
    if (micros() - t0 >= budgetMicros)
      break;
  }
}

// ================= เชื่อมต่อ Tesira =================
void ensureTesiraConnected()
{
  if (tesira.connected())
    return;
  unsigned long now = millis();
  if (now - lastTConnTry < RECONN_MS)
    return;
  lastTConnTry = now;

  Serial.printf("[TTP] Connecting %s:%u ...\n", TESIRA_IP, TESIRA_PORT);
  if (tesira.connect(TESIRA_IP, TESIRA_PORT))
  {
    tesira.setNoDelay(true);
    Serial.println("[TTP] Connected.");

    // ไม่ต้องรอ Welcome ก็ยิง subscribe ได้เลย (กันบางครั้ง Welcome มาช้า)
    bootSubscribeWithTokens();
  }
  else
  {
    Serial.println("[TTP] Connect failed.");
  }
}

// ================= WebSocket handlers =================
void sendStateTo(uint8_t num);
void handleMsg(uint8_t num, const String &msg)
{
  DynamicJsonDocument doc(256);
  if (deserializeJson(doc, msg))
    return;

  const String cmd = doc["cmd"] | "";
  const String block = doc["block"] | curBlock;
  const int ch = doc["ch"] | curCh;
if (cmd == "subscribe") {
  curBlock = block; curCh = ch;

  const bool isMeter = (block == "Analog_Meter" || block == "USB_Meter");

  subscribeLevelOnce(block, ch);
  if (!isMeter) subscribeMuteOnce(block, ch);

  if (tesira.connected()) ttpGetLevel(block, ch);

  // >>> อย่าส่งค่า default -60 ออกไปก่อน
  // ส่งเฉพาะถ้า cache มีค่าแล้ว
  if (CtrlState* cs = pickCtrl(block)) {
    if (cs[ch].hasLevel || cs[ch].hasMute) {
      wsSendCtrl(block, ch, cs[ch].hasLevel, cs[ch].level,
                          cs[ch].hasMute,  cs[ch].mute);
    }
  }
  return;
}
  else if (cmd == "setLevel")
  {
    curBlock = block;
    curCh = ch;
    levelDb = doc["level_db"] | levelDb;
    if (tesira.connected())
      ttpSetLevel(block, ch, levelDb);
    sendStateTo();
  }
  else if (cmd == "setMute")
  {
    curBlock = block;
    curCh = ch;
    isMute = doc["mute"] | isMute;
    if (block != "Analog_Meter" && block != "USB_Meter")
    {
      if (tesira.connected())
        ttpSetMute(block, ch, isMute);
    }
    sendStateTo();
  }
  else if (cmd == "getLevel") {
  // ดึงค่าปัจจุบันของบล็อก/ช่องนั้น ๆ
  const String block = doc["block"] | "";
  const int    ch    = doc["ch"]    | 1;
  if (block.length()) {
    ttpGetLevel(block, ch);   // <-- มีอยู่แล้วในโค้ดคุณ
  }
}

// ดึง snapshot ทั้งชุดที่เราต้องใช้บนแอพ
else if (cmd == "snapshot") {
  for (auto alias : {String("Analog_Level"), String("USB_Level")}) {
    ttpGetLevel(alias, 1);
    ttpGetLevel(alias, 2);
  }
  
}
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_CONNECTED:
  {
    Serial.printf("[WS] Client %u connected from %s\n", num, ws.remoteIP(num).toString().c_str());
    DynamicJsonDocument d(256);
    d["cmd"] = "hello";
    d["host"] = "dw821.local";
    d["esp_ip"] = WiFi.localIP().toString();
    String out;
    serializeJson(d, out);
    ws.sendTXT(num, out);
    break;
  }
  case WStype_TEXT:
    handleMsg(num, String((char *)payload, length));
    break;
  case WStype_DISCONNECTED:
    Serial.printf("[WS] Client %u disconnected\n", num);
    break;
  default:
    break;
  }
}

// ================= Arduino Core =================
void setup()
{
  Serial.begin(115200);
  Serial.println("Booting...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(300);
    Serial.print(".");
  }
  WiFi.setSleep(false); // <<< ลด latency Wi-Fi
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  WiFi.setHostname("dw821");
  if (MDNS.begin("dw821"))
  {
    Serial.println("[mDNS] Started as dw821.local");
    MDNS.addService("ws", "tcp", 81);
  }
  else
  {
    Serial.println("[mDNS] Failed to start");
  }

  ws.begin();
  ws.onEvent(onWsEvent);

  ensureTesiraConnected();
}

void loop()
{
  // 1) ให้ WS ทำงาน
  ws.loop();

  // 2) ดูแลการเชื่อมต่อกับ Tesira
  ensureTesiraConnected();

  pumpTesira(6000);

  // 3) อ่าน Tesira non-blocking
  String line;
  // จำกัดรอบนี้ไม่ให้กินเวลานานเกินไป
  unsigned long t0 = micros();
  while (tesira.connected() && ttpReadLine(line))
  {
    handleTtpLine(line);
    if (micros() - t0 > 2000)
      break; // ~2ms/loop
  }
  updateMeters();
}


