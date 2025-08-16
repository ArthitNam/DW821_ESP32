// dw821_tesira_bridge.ino
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

// ================= App State (ไว้ใช้เล็ก ๆ น้อย ๆ) =================
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

// เวลาล่าสุดที่ได้รับ +OK ของ EQ (ใช้ debounce กันชน LEVEL)
static unsigned long gLastEqOkMs = 0;
static const unsigned long EQ_QUIET_MS = 220; // 150–300 ตาม latency เครื่องจริง

// ===== Boot settling window (กันค่าเพี้ยนช่วงบูต) =====
static unsigned long gLastConnectedMs = 0;
static inline bool _isBootSettling(unsigned long winMs = 5000) {
  return (gLastConnectedMs != 0) && (millis() - gLastConnectedMs < winMs);
}

// ===== Control state cache (เพิ่ม timestamp + last-broadcast + last mute sent) =====
struct CtrlState {
  float level; bool hasLevel; unsigned long levelMs;
  float lastSentLevel; unsigned long lastSentLevelMs;

  bool mute; bool hasMute; unsigned long muteMs;
  bool lastSentMute; unsigned long lastSentMuteMs;   // กัน spam/เด้ง
};

CtrlState ctrlAnalog[3] = {
  {0,false,0, 123456789.0f,0,  false,false,0,  false,0},
  {0,false,0, 123456789.0f,0,  false,false,0,  false,0},
  {0,false,0, 123456789.0f,0,  false,false,0,  false,0}
};
CtrlState ctrlUsb[3] = {
  {0,false,0, 123456789.0f,0,  false,false,0,  false,0},
  {0,false,0, 123456789.0f,0,  false,false,0,  false,0},
  {0,false,0, 123456789.0f,0,  false,false,0,  false,0}
};

CtrlState *pickCtrl(const String &alias) {
  if (alias == "Analog_Level") return ctrlAnalog;
  if (alias == "USB_Level") return ctrlUsb;
  return nullptr;
}

// ----- Pending desired mute (ใช้เป็น fallback ถ้า +OK ไม่ชัด) -----
struct PendingMute {
  bool active = false;
  bool value = false;
  unsigned long ms = 0;
};
static PendingMute pendAnalog[3], pendUsb[3];   // index 1..2
static inline PendingMute* pickPend(const String& alias){
  if (alias=="Analog_Level") return pendAnalog;
  if (alias=="USB_Level")    return pendUsb;
  return nullptr;
}
static const unsigned long PENDING_MUTE_WIN_MS = 1500; // 1.5s หน้าต่างเชื่อค่าที่เพิ่งสั่ง

static unsigned long lastRetryMs = 0;
const unsigned long RETRY_MS = 400;
static int retryLeft = 12; // ~4.8s

// ===== EQ request queue (one in-flight) =====
struct EqPending {
  String alias;  // "USB_EQ" | "A_EQ"
  String attr;   // "frequency" | "gain" | "bandwidth" | "bypass"
  int band;      // 1..5
};
static EqPending EQ_Q[32];
static int EQ_Q_N = 0;
static bool EQ_INFLIGHT = false;

// ==== EQ cache ต่อบล็อก (5 แบนด์) ====
struct EqBand {
  float f = 0;   bool hasF = false; // frequency (Hz)
  float g = 0;   bool hasG = false; // gain (dB)
  float bw = 0;  bool hasBW = false;// bandwidth (oct)
  bool by = false; bool hasBy = false; // bypass
};
EqBand usbEq[6]; // index 1..5
EqBand aEq[6];

static inline EqBand *_eqTable(const String &alias) {
  return (alias == "USB_EQ") ? usbEq : aEq;
}

// จำค่าสุดท้าย ๆ ของ EQ (+OK) เอาไว้ช่วยเดาทับซ้อน
struct EqRecentVal { float v; unsigned long ms; };
static EqRecentVal gEqVals[8];
static int gEqValsN = 0;

static inline void _eqRemember(float v) {
  if (isnan(v)) return;
  if (gEqValsN < (int)(sizeof(gEqVals)/sizeof(gEqVals[0]))) {
    gEqVals[gEqValsN++] = EqRecentVal{v, millis()};
  } else {
    for (int i=1;i<gEqValsN;i++) gEqVals[i-1]=gEqVals[i];
    gEqVals[gEqValsN-1] = EqRecentVal{v, millis()};
  }
}

static inline bool _looksLikeRecentEqValue(float v, unsigned long windowMs = 450) {
  unsigned long now = millis();
  for (int i = gEqValsN - 1; i >= 0; --i) {
    if (now - gEqVals[i].ms > windowMs) break;
    if (fabsf(gEqVals[i].v - v) < 1e-3f) return true;
  }
  return false;
}

// ===== ส่ง CTRL ไปแอพ (เลือก field ที่มี + throttle ส่งซ้ำ) =====
static const unsigned long CTRL_RESEND_MS = 250; // เวลาขั้นต่ำส่งซ้ำค่าเดิม
static const float CTRL_EPS = 0.05f;             // เปลี่ยน <= 0.05 dB ไม่ถือว่าต่าง

void _broadcastLevelThrottled(const String &alias, int ch, float lv) {
  CtrlState *cs = pickCtrl(alias);
  if (!cs) return;
  unsigned long now = millis();

  if (cs[ch].lastSentLevelMs != 0 &&
      now - cs[ch].lastSentLevelMs < CTRL_RESEND_MS &&
      fabsf(cs[ch].lastSentLevel - lv) <= CTRL_EPS) {
    return; // ไม่ส่งซ้ำถ้ายังเหมือนเดิมในช่วงสั้นๆ
  }

  StaticJsonDocument<160> d;
  d["block"] = alias;
  d["ch"] = ch;
  d["level_db"] = lv;
  String s; serializeJson(d, s);
  Serial.printf("[WS TX][CTRL] %s\n", s.c_str());
  ws.broadcastTXT(s);

  cs[ch].lastSentLevel = lv;
  cs[ch].lastSentLevelMs = now;
}

void wsSendCtrl(const String &alias, int ch, bool sendLevel, float lv, bool sendMute, bool m) {
  if (sendLevel) {
    _broadcastLevelThrottled(alias, ch, lv);
  }
  if (sendMute) {
    CtrlState* cs = pickCtrl(alias);
    unsigned long now = millis();
    if (cs) {
      if (cs[ch].lastSentMuteMs != 0 && cs[ch].lastSentMute == m && (now - cs[ch].lastSentMuteMs) < 120) {
        return;
      }
      cs[ch].lastSentMute = m;
      cs[ch].lastSentMuteMs = now;
    }
    StaticJsonDocument<96> d;
    d["block"] = alias;
    d["ch"] = ch;
    d["mute"] = m;
    String s; serializeJson(d, s);
    Serial.printf("[WS TX][CTRL] %s\n", s.c_str());
    ws.broadcastTXT(s);
  }
}

// ===== map publishToken -> (alias,ch) =====
struct SubMap { String token; String alias; int ch; };
static SubMap gSubs[16]; static int gSubsN = 0;
void addSubMap(const String &token, const String &alias, int ch) {
  if (!token.length()) return;
  for (int i=0;i<gSubsN;i++) if (gSubs[i].token == token) { gSubs[i].alias=alias; gSubs[i].ch=ch; return; }
  if (gSubsN < (int)(sizeof(gSubs)/sizeof(gSubs[0]))) gSubs[gSubsN++] = SubMap{token, alias, ch};
}
bool findSubMap(const String &token, String &alias, int &ch) {
  for (int i=0;i<gSubsN;i++) if (gSubs[i].token == token) { alias=gSubs[i].alias; ch=gSubs[i].ch; return true; }
  return false;
}

// ===== boot subscribe ยิงครั้งเดียว =====
bool gBootDone = false;

// ================= Logger =================
void logTTP(const String &l) {
  if (l.isEmpty()) return;
  const unsigned long now = millis();
  if (l == _logLastLine && (now - _logLastMs) < REPEAT_WINDOW_MS) return;
  _logLastLine = l; _logLastMs = now;

  if (LOG_RAW) { Serial.print("[TTP <<] "); Serial.println(l); return; }
  if (l.startsWith("Welcome")) { if (LOG_WELCOME) Serial.println("[TTP] Welcome"); return; }
  if (l.startsWith("-ERR")) { if (LOG_ERRORS) { Serial.print("[TTP ERR] "); Serial.println(l); } return; }
  if (l.startsWith("EVENT")) {
    if (!LOG_EVENTS) return;
    int p1=l.indexOf(' '), p2=l.indexOf(' ', p1+1), p3=l.indexOf(' ', p2+1);
    if (p1>0 && p2>p1 && p3>p2) {
      String alias=l.substring(p1+1,p2), attr=l.substring(p2+1,p3), rest=l.substring(p3+1);
      Serial.printf("[EVT] %s %s %s\n", alias.c_str(), attr.c_str(), rest.c_str());
    } else { Serial.print("[EVT] "); Serial.println(l); }
    return;
  }
  if (l.startsWith("+OK")) {
    if (!LOG_OK_SUMMARY) return;
    if (l.indexOf("level")>=0 || l.indexOf("mute")>=0) { Serial.print("[OK] "); Serial.println(l); }
    return;
  }
}

// ================= ส่งสถานะไปแอพ (legacy) =================
void sendStateTo(uint8_t num = 0xFF) {
  DynamicJsonDocument doc(256);
  doc["block"] = curBlock; doc["ch"] = curCh;
  doc["level_db"] = levelDb; doc["mute"] = isMute;
  String s; serializeJson(doc, s);
  if (num == 0xFF) ws.broadcastTXT(s); else ws.sendTXT(num, s);
}

// ================= TTP ส่งคำสั่ง (ต่อท้าย CRLF) =================
void ttpSend(const String &line) {
  if (!tesira.connected()) return;
  String out = line; if (!out.endsWith("\r\n")) out += "\r\n";
  tesira.print(out);
  Serial.print("[TTP >>] "); Serial.println(line);
}

static void eqq_push(const String &alias, const char *attr, int band) {
  if (EQ_Q_N >= (int)(sizeof(EQ_Q)/sizeof(EQ_Q[0]))) return;
  EQ_Q[EQ_Q_N++] = EqPending{alias, String(attr), band};
}
static bool eqq_empty() { return EQ_Q_N == 0; }
static EqPending eqq_pop_front() {
  EqPending p = EQ_Q[0];
  for (int i=1;i<EQ_Q_N;++i) EQ_Q[i-1]=EQ_Q[i];
  if (EQ_Q_N>0) --EQ_Q_N;
  return p;
}
static void eq_kick() {
  if (EQ_INFLIGHT || eqq_empty() || !tesira.connected()) return;
  const EqPending &p = EQ_Q[0];
  String line = p.alias + " get " + p.attr + " " + String(p.band);
  EQ_INFLIGHT = true;
  Serial.printf("[EQ KICK] inflight=1 size=%d  >> %s\n", EQ_Q_N, line.c_str());
  ttpSend(line);
}

// ยิงครบทั้งแบนด์
void eqGetBandAll(const char *alias, int band) {
  eqq_push(alias, "frequency", band);
  eqq_push(alias, "gain", band);
  eqq_push(alias, "bandwidth", band);
  eqq_push(alias, "bypass", band);
  Serial.printf("[EQ] Request %s band %d (all attrs)\n", alias, band);
  eq_kick();
}

// ================= จำบริบท +OK =================
String gLastAlias, gLastAttr; int gLastCh = 0;
inline void rememberCmd(const String &a, const String &at, int ch) {
  gLastAlias = a; gLastAttr = at; gLastCh = ch;
}
void ttpGetLevel(const String &alias, int ch) { rememberCmd(alias,"level",ch); ttpSend(alias + " get level " + String(ch)); }
void ttpSetLevel(const String &alias, int ch, float dB) {
  rememberCmd(alias,"level",ch);
  char buf[16]; dtostrf(dB, 0, 1, buf);
  ttpSend(alias + " set level " + String(ch) + " " + String(buf));
}
void ttpSetMute(const String &alias, int ch, bool on) {
  // จำคำสั่งล่าสุด + ตั้ง pending
  rememberCmd(alias, "mute", ch);
  if (PendingMute* pm = pickPend(alias)) {
    pm[ch].active = true;
    pm[ch].value = on;
    pm[ch].ms = millis();
  }
  // optimistic update ทันที
  if (CtrlState* cs = pickCtrl(alias)) {
    cs[ch].mute = on; cs[ch].hasMute = true; cs[ch].muteMs = millis();
    wsSendCtrl(alias, ch, false, 0, true, on);
  }
  // ส่งจริงไป Tesira
  ttpSend(alias + " set mute " + String(ch) + " " + (on ? "true" : "false"));
}

// ================= Telnet negotiate (IAC) =================
inline void telnetSend(uint8_t a, uint8_t b, uint8_t c) { uint8_t buf[3]={a,b,c}; tesira.write(buf,3); }
int ttpReadFilteredByte() {
  if (!tesira.available()) return -1;
  int b = tesira.read(); if (b != 255) return b;
  while (!tesira.available()) delay(1);
  int cmd = tesira.read();
  if (cmd==251||cmd==252||cmd==253||cmd==254) {
    while (!tesira.available()) delay(1);
    int opt = tesira.read();
    (cmd==251||cmd==252) ? telnetSend(255,254,opt) : telnetSend(255,252,opt);
    return -2;
  }
  if (cmd==250) { // SB ... SE
    int prev=-1,cur=-1;
    while (true) {
      while (!tesira.available()) delay(1);
      cur = tesira.read();
      if (prev==255 && cur==240) break;
      prev = cur;
    }
    return -2;
  }
  if (cmd==240) return -2; // SE
  return -2;
}
bool ttpReadLine(String &outLine) {
  static String buf;
  while (tesira.connected() && tesira.available()) {
    int r = ttpReadFilteredByte();
    if (r==-1) break; if (r==-2) continue;
    char c = (char)r; if (c=='\r') continue;
    buf += c;
    if (c=='\n') { outLine = buf; buf = ""; return true; }
  }
  return false;
}

// ================= subscribe de-dup (สำหรับ Level/Mute) =================
struct SubKey { String alias, attr; int ch; };
static SubKey _subs[64]; static int _subsN = 0;
bool _same(const SubKey &a, const SubKey &b) { return a.alias==b.alias && a.attr==b.attr && a.ch==b.ch; }
bool _hasSub(const String &alias, const String &attr, int ch) {
  for (int i=0;i<_subsN;i++) if (_same(_subs[i], SubKey{alias,attr,ch})) return true;
  return false;
}
void _markSub(const String &alias, const String &attr, int ch) {
  if (_subsN < (int)(sizeof(_subs)/sizeof(_subs[0]))) _subs[_subsN++] = SubKey{alias,attr,ch};
}
void subscribeLevelOnce(const String &alias, int ch) { if (!_hasSub(alias,"level",ch)) { ttpSend(alias + " subscribe level " + String(ch)); _markSub(alias,"level",ch);} }
void subscribeMuteOnce(const String &alias, int ch)  { if (!_hasSub(alias,"mute",ch))  { ttpSend(alias + " subscribe mute " + String(ch));  _markSub(alias,"mute",ch);} }

// ================= Boot Subscribe แบบมี token =================
void bootSubscribeWithTokens() {
  if (gBootDone) return; gBootDone = true;
  ttpSend("SESSION set verbose true");

  // USB meters
  ttpSend("USB_Meter subscribe level 1 USB1"); addSubMap("USB1","USB_Meter",1);
  ttpSend("USB_Meter subscribe level 2 USB2"); addSubMap("USB2","USB_Meter",2);

  // Analog meters
  ttpSend("Analog_Meter subscribe level 1 AN1"); addSubMap("AN1","Analog_Meter",1);
  ttpSend("Analog_Meter subscribe level 2 AN2"); addSubMap("AN2","Analog_Meter",2);

  // snapshot (ออปชั่น)
  ttpSend("USB_Meter get level 1"); ttpSend("USB_Meter get level 2");
  ttpSend("Analog_Meter get level 1"); ttpSend("Analog_Meter get level 2");
}

// ===== ส่ง L/R พร้อมกัน ลดโอเวอร์เฮด JSON/WS =====
void sendVU(const char *alias, float left, float right, unsigned long &lastSendTime) {
  const unsigned long now = millis();
  if (now - lastSendTime < vuInterval) return;
  float l = roundf(left*10.0f)/10.0f, r = roundf(right*10.0f)/10.0f;
  StaticJsonDocument<128> doc;
  doc["alias"]=alias; doc["block"]=alias; doc["left"]=l; doc["right"]=r;
  String s; serializeJson(doc, s); ws.broadcastTXT(s); lastSendTime = now;
}

// ===== helper: parse mute true/false อย่างระวัง =====
inline bool parse_mute_from_event(const String &line, float numeric, bool fallbackPrev){
  int t = line.lastIndexOf("true");
  int f = line.lastIndexOf("false");
  if (t>=0 && (f<0 || t>f)) return true;
  if (f>=0 && (t<0 || f>t)) return false;
  if (!isnan(numeric)) return (numeric != 0.0f);  // EVENT อนุญาตเลข
  return fallbackPrev;
}
inline bool parse_mute_from_ok(const String &line, bool fallbackPrev){
  int t = line.lastIndexOf("true");
  int f = line.lastIndexOf("false");
  if (t>=0 && (f<0 || t>f)) return true;
  if (f>=0 && (t<0 || f>t)) return false;
  // +OK ไม่เจอ true/false → อย่าเชื่อเลข ให้คงตาม fallbackPrev
  return fallbackPrev;
}

// ส่ง JSON band กลับแอพเมื่อครบ 4 ค่า
void eqEmitBand(const char *alias, int band) {
  EqBand *table = (strcmp(alias,"USB_EQ")==0) ? usbEq : aEq;
  EqBand &b = table[band];
  if (!(b.hasF && b.hasG && b.hasBW && b.hasBy)) return;

  StaticJsonDocument<256> j;
  j["block"] = alias; j["band"] = band;
  j["frequency"] = b.f; j["gain"] = b.g; j["bandwidth"] = b.bw; j["bypass"] = b.by;
  String out; serializeJson(j, out);
  Serial.printf("[WS TX][EQ] %s\n", out.c_str());
  ws.broadcastTXT(out);
}

void eqUpdateAttr(const char *alias, const char *attr, int band, float numValue, bool boolValue, bool isBoolAttr) {
  if (band < 1 || band > 5) return;
  EqBand *table = (strcmp(alias,"USB_EQ")==0) ? usbEq : aEq;
  EqBand &b = table[band];

  if (!isBoolAttr) {
    if (strcmp(attr,"frequency")==0) { b.f = numValue; b.hasF = true; }
    else if (strcmp(attr,"gain")==0) { b.g = numValue; b.hasG = true; }
    else if (strcmp(attr,"bandwidth")==0) { b.bw = numValue; b.hasBW = true; }
  } else { b.by = boolValue; b.hasBy = true; }

  eqEmitBand(alias, band);
}

void ttpEqGet(const char *alias, const char *attr, int band) {
  rememberCmd(String(alias), String(attr), band);
  ttpSend(String(alias) + " get " + attr + " " + String(band));
  Serial.printf("[EQ >>] %s %s %d\n", alias, attr, band);
}

// ================= parse ตอบกลับ TTP =================
void handleTtpLine(const String &line) {
  String l = line; l.trim(); if (l.isEmpty()) return;

  // 1) publishToken meters
  if (l.startsWith("!")) {
    int t1=l.indexOf("\"publishToken\""), v1=l.indexOf("\"value\"");
    if (t1>=0 && v1>=0) {
      int q1=l.indexOf('"', t1+15), q2=(q1>=0)?l.indexOf('"', q1+1):-1;
      String token = (q1>=0 && q2>q1)?l.substring(q1+1,q2):"";
      int colon=l.indexOf(':', v1);
      float val = (colon>0)?l.substring(colon+1).toFloat():-100.0f;

      if (token=="USB1") { usbLeft=val; sendVU("USB_Meter", usbLeft, usbRight, lastSendUsbVU); }
      else if (token=="USB2") { usbRight=val; sendVU("USB_Meter", usbLeft, usbRight, lastSendUsbVU); }
      else if (token=="AN1") { analogLeft=val; sendVU("Analog_Meter", analogLeft, analogRight, lastSendAnalogVU); }
      else if (token=="AN2") { analogRight=val; sendVU("Analog_Meter", analogLeft, analogRight, lastSendAnalogVU); }
      return;
    }
  }

  // 2) EVENT
  if (l.startsWith("EVENT")) {
    auto parts = String(l).substring(6); parts.trim();
    int p1=parts.indexOf(' '), p2=parts.indexOf(' ', p1+1), p3=parts.indexOf(' ', p2+1);
    if (p1>0 && p2>p1 && p3>p2) {
      String alias=parts.substring(0,p1);
      String attr =parts.substring(p1+1,p2);
      int ch=parts.substring(p2+1,p3).toInt();
      float v=parts.substring(p3+1).toFloat();

      // meters
      if (alias=="USB_Meter" || alias=="Analog_Meter") {
        if (alias=="USB_Meter") { if (ch==1) usbLeft=v; else if (ch==2) usbRight=v; sendVU("USB_Meter", usbLeft, usbRight, lastSendUsbVU); }
        else { if (ch==1) analogLeft=v; else if (ch==2) analogRight=v; sendVU("Analog_Meter", analogLeft, analogRight, lastSendAnalogVU); }
        return;
      }

      // control (Level/Mute)
      if (attr=="level" || attr=="mute") {
        if (CtrlState *cs = pickCtrl(alias)) {
          if (attr=="level") {
            const bool firstTime = !cs[ch].hasLevel;

            // Quiet window (ถ้าไม่ใช่ครั้งแรก)
            if (!firstTime && (millis() - gLastEqOkMs < EQ_QUIET_MS)) {
              Serial.printf("[DEBOUNCE EVT] Skip LEVEL %.3f for %s ch=%d (EQ quiet window)\n", v, alias.c_str(), ch);
              return;
            }

            // Guard ค่าเพี้ยนช่วงบูต: USB_Level ch=2 อาจหลุด 0/0.5/160/500
            if (alias=="USB_Level" && ch==2 && _isBootSettling() && (v<=0.5f || v>=160.0f)) {
              Serial.printf("[BOOT-GUARD EVT] Ignore USB R bogus v=%.3f\n", v);
              return;
            }

            // Guard ทั่วไป (ยกเว้น first time)
            if (!firstTime && (v < -100.0f || v > 20.0f || _looksLikeRecentEqValue(v))) {
              Serial.printf("[GUARD EVT] Ignore out-of-range/eq-like LEVEL for %s ch=%d v=%.3f\n", alias.c_str(), ch, v);
              return;
            }

            cs[ch].level = v; cs[ch].hasLevel = true; cs[ch].levelMs = millis();
            _broadcastLevelThrottled(alias, ch, v);
          } else { // mute (EVENT)
            bool m = parse_mute_from_event(l, v, (cs[ch].hasMute ? cs[ch].mute : false));
            cs[ch].mute = m; cs[ch].hasMute = true; cs[ch].muteMs = millis();
            wsSendCtrl(alias, ch, false, 0, true, m);
          }
          Serial.printf("[TTP EVT][CTRL] %s %s ch=%d v=%.2f\n", alias.c_str(), attr.c_str(), ch, v);
        }
        return;
      }
    }
    return;
  }

  // 3) +OK (snapshot / responses)
  if (l.startsWith("+OK")) {
    // parse value
    float v = NAN;
    int iv=l.indexOf("\"value\":");
    if (iv>=0) v = l.substring(iv+8).toFloat();
    else { int colon=l.lastIndexOf(':'); if (colon>0) v = l.substring(colon+1).toFloat(); }

    // ===== EQ queue in-flight =====
    if (EQ_INFLIGHT && !eqq_empty()) {
      EqPending p = eqq_pop_front(); EQ_INFLIGHT = false;

      const bool isBypass = (p.attr=="bypass");
      bool bv = false;
      if (isBypass) {
        const bool hasTrue = (l.indexOf("true")>=0);
        const bool hasFalse= (l.indexOf("false")>=0);
        bv = hasTrue ? true : (hasFalse ? false : (!isnan(v) && v!=0.0f));
      }
      if (!isnan(v)) _eqRemember(v);

      eqUpdateAttr(p.alias.c_str(), p.attr.c_str(), p.band,
                   isnan(v)?0.0f:v, bv, isBypass);
      Serial.printf("[EQ +OK] %s %s %d = %s\n", p.alias.c_str(), p.attr.c_str(), p.band, l.c_str());
      gLastEqOkMs = millis();
      eq_kick();
      return;
    }

    // ===== meter snapshot =====
    if (gLastAlias=="USB_Meter" || gLastAlias=="Analog_Meter") {
      if (!isnan(v)) {
        if (gLastAlias=="USB_Meter") { if (gLastCh==1) usbLeft=v; else if (gLastCh==2) usbRight=v; sendVU("USB_Meter", usbLeft, usbRight, lastSendUsbVU); }
        else { if (gLastCh==1) analogLeft=v; else if (gLastCh==2) analogRight=v; sendVU("Analog_Meter", analogLeft, analogRight, lastSendAnalogVU); }
      }
      return;
    }

    // ===== control +OK (Level/Mute) =====
    if (CtrlState *cs = pickCtrl(gLastAlias)) {
      if (gLastAttr=="level" && !isnan(v)) {
        if (millis() - gLastEqOkMs < EQ_QUIET_MS) {
          Serial.printf("[DEBOUNCE] Skip LEVEL %.3f for %s ch=%d (EQ quiet window)\n", v, gLastAlias.c_str(), gLastCh);
          return;
        }
        // Guard ค่าเพี้ยนช่วงบูต (USB R)
        if (gLastAlias=="USB_Level" && gLastCh==2 && _isBootSettling() && (v<=0.5f || v>=160.0f)) {
          Serial.printf("[BOOT-GUARD OK] Ignore USB R bogus v=%.3f\n", v);
          return;
        }
        if (v < -100.0f || v > 20.0f || _looksLikeRecentEqValue(v)) {
          Serial.printf("[GUARD] Ignore out-of-range/eq-like LEVEL for %s ch=%d v=%.3f\n", gLastAlias.c_str(), gLastCh, v);
          return;
        }
        cs[gLastCh].level = v; cs[gLastCh].hasLevel = true; cs[gLastCh].levelMs = millis();
        _broadcastLevelThrottled(gLastAlias, gLastCh, v);
      } else if (gLastAttr=="mute") {
        bool m;
        bool hasExplicitTF = (l.indexOf("true")>=0) || (l.indexOf("false")>=0);

        if (hasExplicitTF) {
          m = parse_mute_from_ok(l, cs[gLastCh].hasMute ? cs[gLastCh].mute : false);
        } else {
          PendingMute* pm = pickPend(gLastAlias);
          bool fallback = cs[gLastCh].hasMute ? cs[gLastCh].mute : false;
          if (pm && pm[gLastCh].active && (millis() - pm[gLastCh].ms <= PENDING_MUTE_WIN_MS)) {
            m = pm[gLastCh].value;
          } else {
            m = fallback;
          }
          if (pm) pm[gLastCh].active = false; // เคลียร์ pending
        }

        if (!cs[gLastCh].hasMute || cs[gLastCh].mute != m) {
          cs[gLastCh].mute = m; cs[gLastCh].hasMute = true; cs[gLastCh].muteMs = millis();
          wsSendCtrl(gLastAlias, gLastCh, false, 0, true, m);
        }
      }
      return;
    }
    return;
  }

  // 4) อื่น ๆ ไม่สนใจ
}

// ===== เรียกส่งทุกแชนเนลที่สนใจ =====
void updateMeters() {
  sendVU("Analog_Meter", analogLeft, analogRight, lastSendAnalogVU);
  sendVU("USB_Meter", usbLeft, usbRight, lastSendUsbVU);
}

// ===== ดูดข้อมูลจาก Tesira แบบ time-slice =====
void pumpTesira(unsigned long budgetMicros = 6000) {
  unsigned long t0 = micros();
  String line;
  while (tesira.connected() && ttpReadLine(line)) {
    handleTtpLine(line);
    if (micros() - t0 >= budgetMicros) break;
  }
}

void bootSubscribeControls() {
  for (auto alias : {String("Analog_Level"), String("USB_Level")}) {
    for (int ch = 1; ch <= 2; ++ch) {
      subscribeLevelOnce(alias, ch);
      subscribeMuteOnce(alias, ch);
    }
  }
}

void bootSnapshotControls() {
  for (auto alias : {String("Analog_Level"), String("USB_Level")}) {
    for (int ch = 1; ch <= 2; ++ch) {
      ttpGetLevel(alias, ch); // ขอค่าจริง
    }
  }
}

// ================= เชื่อมต่อ Tesira =================
void ensureTesiraConnected() {
  if (tesira.connected()) return;
  unsigned long now = millis();
  if (now - lastTConnTry < RECONN_MS) return;
  lastTConnTry = now;

  Serial.printf("[TTP] Connecting %s:%u ...\n", TESIRA_IP, TESIRA_PORT);
  if (tesira.connect(TESIRA_IP, TESIRA_PORT)) {
    tesira.setNoDelay(true);
    Serial.println("[TTP] Connected.");
    gLastConnectedMs = millis(); // เริ่มนับหน้าต่างบูต

    bootSubscribeWithTokens(); // meters + verbose
    bootSubscribeControls();   // subscribe level/mute
    bootSnapshotControls();    // snapshot รอบแรก

    // หน่วงสั้น ๆ ให้ control ได้ค่าจริงก่อน แล้วค่อยเริ่ม EQ
    static const unsigned long EQ_START_DELAY_MS = 700;
    unsigned long tStart = millis();
    while (millis() - tStart < EQ_START_DELAY_MS) {
      String line;
      if (ttpReadLine(line)) handleTtpLine(line);
      ws.loop();
      delay(1);
    }
    eq_kick(); // เริ่มยิงคิว EQ หลัง control seed แล้ว
  } else {
    Serial.println("[TTP] Connect failed.");
  }
}

// ================= WebSocket handlers =================
void sendStateTo(uint8_t num);
void handleMsg(uint8_t num, const String &msg) {
  DynamicJsonDocument doc(256);
  if (deserializeJson(doc, msg)) return;

  const String cmd = doc["cmd"] | "";
  const String block = doc["block"] | curBlock;
  const int ch = doc["ch"] | curCh;

  if (cmd == "subscribe") {
    curBlock = block; curCh = ch;

    const bool isMeter = (block=="Analog_Meter" || block=="USB_Meter");
    subscribeLevelOnce(block, ch);
    if (!isMeter) subscribeMuteOnce(block, ch);
    if (tesira.connected()) ttpGetLevel(block, ch);

    // ถ้า cache ยังไม่มี level → seed จาก meters เพื่อให้แอพ arm ได้ก่อน
    if (CtrlState *cs = pickCtrl(block)) {
      if (!cs[ch].hasLevel) {
        float seed = -60.0f;
        if (block == "Analog_Level") seed = (ch == 1 ? analogLeft : analogRight);
        else if (block == "USB_Level") seed = (ch == 1 ? usbLeft : usbRight);
        if (seed <= -99.0f) seed = -60.0f;
        seed = constrain(seed, -100.0f, 12.0f);

        cs[ch].level = seed; cs[ch].hasLevel = true; cs[ch].levelMs = millis();
        wsSendCtrl(block, ch, true, seed, false, false);
        Serial.printf("[SEED] %s ch=%d level_db=%.1f (on subscribe)\n", block.c_str(), ch, seed);
      }
    }

    // ส่งเฉพาะค่า "สด"
    if (CtrlState *cs = pickCtrl(block)) {
      const unsigned long now = millis();
      const bool freshLevel = cs[ch].hasLevel && (now - cs[ch].levelMs <= 2000);
      const bool freshMute  = cs[ch].hasMute  && (now - cs[ch].muteMs  <= 2000);
      if (freshLevel) _broadcastLevelThrottled(block, ch, cs[ch].level);
      if (freshMute)  wsSendCtrl(block, ch, false, 0, true, cs[ch].mute);
    }
    return;
  }
  else if (cmd == "setLevel") {
    curBlock = block; curCh = ch;
    levelDb = doc["level_db"] | levelDb;
    if (tesira.connected()) ttpSetLevel(block, ch, levelDb);
    Serial.printf("[APP >>] setLevel %s ch=%d db=%.1f (echo off)\n", block.c_str(), ch, levelDb);
    return;
  }
  else if (cmd == "setMute") {
    curBlock = block; curCh = ch;
    isMute = doc["mute"] | isMute;
    if (block!="Analog_Meter" && block!="USB_Meter") {
      if (tesira.connected()) ttpSetMute(block, ch, isMute);
    }
    Serial.printf("[APP >>] setMute %s ch=%d %s (echo off)\n", block.c_str(), ch, isMute?"true":"false");
    return;
  }
  else if (cmd == "getLevel") {
    const String blk = doc["block"] | "";
    const int c = doc["ch"] | 1;
    if (blk.length()) ttpGetLevel(blk, c);
  }
  else if (cmd == "eqGetBand") {
    String alias = doc["alias"] | "";
    int band = doc["band"] | 1;
    if (alias.length() && band>=1 && band<=5) eqGetBandAll(alias.c_str(), band);
  }
  else if (cmd == "eqSet") {
    String alias = doc["alias"] | "";
    int band = doc["band"] | 1;
    String attr = doc["attr"] | "";
    float value = doc["value"] | 0.0f;
    if (alias.length() && attr.length()) {
      rememberCmd(alias, attr, band);
      ttpSend(alias + " set " + attr + " " + String(band) + " " + String(value, 3));
    }
  }
  else if (cmd == "eqSetBypass") {
    String alias = doc["alias"] | "";
    int band = doc["band"] | 1;
    bool bypass = doc["bypass"] | false;
    rememberCmd(alias, "bypass", band);
    ttpSend(alias + " set bypass " + String(band) + " " + (bypass ? "true" : "false"));
  }
  else if (cmd == "snapshot") {
    for (auto alias : {String("Analog_Level"), String("USB_Level")}) {
      ttpGetLevel(alias, 1);
      ttpGetLevel(alias, 2);
    }
  }
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED: {
      Serial.printf("[WS] Client %u connected from %s\n", num, ws.remoteIP(num).toString().c_str());
      DynamicJsonDocument d(256);
      d["cmd"]="hello"; d["host"]="dw821.local"; d["esp_ip"]=WiFi.localIP().toString();
      String out; serializeJson(d, out); ws.sendTXT(num, out);
      break;
    }
    case WStype_TEXT: handleMsg(num, String((char*)payload, length)); break;
    case WStype_DISCONNECTED: Serial.printf("[WS] Client %u disconnected\n", num); break;
    default: break;
  }
}

// ================= Arduino Core =================
void setup() {
  Serial.begin(115200);
  Serial.println("Booting...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  WiFi.setSleep(false);
  Serial.println(); Serial.print("IP: "); Serial.println(WiFi.localIP());

  WiFi.setHostname("dw821");
  if (MDNS.begin("dw821")) { Serial.println("[mDNS] Started as dw821.local"); MDNS.addService("ws","tcp",81); }
  else { Serial.println("[mDNS] Failed to start"); }

  ws.begin();
  ws.onEvent(onWsEvent);

  ensureTesiraConnected();
}

void loop() {
  ws.loop();
  ensureTesiraConnected();

  pumpTesira(6000);

  // time-slice สำรอง (กัน backlog)
  String line; unsigned long t0 = micros();
  while (tesira.connected() && ttpReadLine(line)) {
    handleTtpLine(line);
    if (micros() - t0 > 2000) break; // ~2ms/loop
  }

  updateMeters();

  if (retryLeft > 0 && millis() - lastRetryMs > RETRY_MS) {
    lastRetryMs = millis();
    for (auto alias : {String("Analog_Level"), String("USB_Level")}) {
      for (int ch = 1; ch <= 2; ++ch) {
        CtrlState *cs = pickCtrl(alias);
        if (cs && cs[ch].hasLevel && (millis() - cs[ch].levelMs) > 1000) {
          // ถ้า 1 วิแล้วยังเป็น seed (ไม่มี OK/EVENT ปักใหม่) ก็ถามซ้ำ
          ttpGetLevel(alias, ch);
        }
      }
    }
    retryLeft--;
  }
}