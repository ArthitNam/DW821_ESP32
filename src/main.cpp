// dw821_tesira_bridge.ino — token-only subs + strict EQ + no auto getLevel
// EQ GET !!!!ใช้ได้ดีแต่ช้า
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

// ================= WebSocket (App) =================
WebSocketsServer ws(81);

// ===== App state =====
String curBlock = "Analog_Level";
int curCh = 1;
float levelDb = -60.0;
bool isMute = false;

// ===== Logger =====
bool LOG_ERRORS = true;
bool LOG_RAW = false;

// ===== Meters =====
float analogLeft = -100.0f, analogRight = -100.0f;
float usbLeft = -100.0f, usbRight = -100.0f;
unsigned long lastSendAnalogVU = 0, lastSendUsbVU = 0;
const unsigned long vuInterval = 12; // ms

// ===== Guards =====
static unsigned long gLastEqOkMs = 0;
static const unsigned long EQ_QUIET_MS = 220;
static unsigned long gLastConnectedMs = 0;
static inline bool _isBootSettling(unsigned long winMs = 5000)
{
  return (gLastConnectedMs != 0) && (millis() - gLastConnectedMs < winMs);
}

// ===== Control cache =====
struct CtrlState
{
  float level;
  bool hasLevel;
  unsigned long levelMs;
  float lastSentLevel;
  unsigned long lastSentLevelMs;
  bool mute;
  bool hasMute;
  unsigned long muteMs;
  bool lastSentMute;
  unsigned long lastSentMuteMs;
};
CtrlState ctrlAnalog[3] = {
    {0, false, 0, 123456789.0f, 0, false, false, 0, false, 0},
    {0, false, 0, 123456789.0f, 0, false, false, 0, false, 0},
    {0, false, 0, 123456789.0f, 0, false, false, 0, false, 0}};
CtrlState ctrlUsb[3] = {
    {0, false, 0, 123456789.0f, 0, false, false, 0, false, 0},
    {0, false, 0, 123456789.0f, 0, false, false, 0, false, 0},
    {0, false, 0, 123456789.0f, 0, false, false, 0, false, 0}};
static inline CtrlState *pickCtrl(const String &alias)
{
  if (alias == "Analog_Level")
    return ctrlAnalog;
  if (alias == "USB_Level")
    return ctrlUsb;
  return nullptr;
}

// ===== Throttle WS =====
static const unsigned long CTRL_RESEND_MS = 250;
static const float CTRL_EPS = 0.05f;

// ===================== WS helpers =====================
void _broadcastLevelThrottled(const String &alias, int ch, float lv)
{
  CtrlState *cs = pickCtrl(alias);
  if (!cs)
    return;
  unsigned long now = millis();
  if (cs[ch].lastSentLevelMs != 0 &&
      now - cs[ch].lastSentLevelMs < CTRL_RESEND_MS &&
      fabsf(cs[ch].lastSentLevel - lv) <= CTRL_EPS)
    return;

  StaticJsonDocument<160> d;
  d["block"] = alias;
  d["ch"] = ch;
  d["level_db"] = lv;
  String s;
  serializeJson(d, s);
  Serial.printf("[WS TX][CTRL] %s\n", s.c_str());
  ws.broadcastTXT(s);
  cs[ch].lastSentLevel = lv;
  cs[ch].lastSentLevelMs = now;
}

void wsSendCtrl(const String &alias, int ch, bool sendLevel, float lv, bool sendMute, bool m)
{
  if (sendLevel)
    _broadcastLevelThrottled(alias, ch, lv);
  if (sendMute)
  {
    CtrlState *cs = pickCtrl(alias);
    unsigned long now = millis();
    if (cs)
    {
      if (cs[ch].lastSentMuteMs != 0 && cs[ch].lastSentMute == m && (now - cs[ch].lastSentMuteMs) < 120)
        return;
      cs[ch].lastSentMute = m;
      cs[ch].lastSentMuteMs = now;
    }
    StaticJsonDocument<96> d;
    d["block"] = alias;
    d["ch"] = ch;
    d["mute"] = m;
    String s;
    serializeJson(d, s);
    Serial.printf("[WS TX][CTRL] %s\n", s.c_str());
    ws.broadcastTXT(s);
  }
}

void sendVU(const char *alias, float left, float right, unsigned long &lastSendTime)
{
  const unsigned long now = millis();
  if (now - lastSendTime < vuInterval)
    return;
  float l = roundf(left * 10.0f) / 10.0f, r = roundf(right * 10.0f) / 10.0f;
  StaticJsonDocument<128> doc;
  doc["alias"] = alias;
  doc["block"] = alias;
  doc["left"] = l;
  doc["right"] = r;
  String s;
  serializeJson(doc, s);
  ws.broadcastTXT(s);
  lastSendTime = now;
}

void wsBroadcastTesiraOnline(bool online)
{
  StaticJsonDocument<64> d;
  d["tesira_connected"] = online;
  String s;
  serializeJson(d, s);
  ws.broadcastTXT(s);
}

// ===================== Tesira I/O =====================
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

// Telnet negotiate filter
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
    (cmd == 251 || cmd == 252) ? telnetSend(255, 254, opt) : telnetSend(255, 252, opt);
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
    return -2;
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

// ===================== Token map =====================
struct TokenMap
{
  String token;
  String alias;
  String attr;
  int ch;
};
static TokenMap gTok[32];
static int gTokN = 0;

void addTokenMap(const String &token, const String &alias, const String &attr, int ch)
{
  if (!token.length())
    return;
  for (int i = 0; i < gTokN; i++)
  {
    if (gTok[i].token == token)
    {
      gTok[i].alias = alias;
      gTok[i].attr = attr;
      gTok[i].ch = ch;
      return;
    }
  }
  if (gTokN < (int)(sizeof(gTok) / sizeof(gTok[0])))
    gTok[gTokN++] = TokenMap{token, alias, attr, ch};
}
bool findToken(const String &token, String &alias, String &attr, int &ch)
{
  for (int i = 0; i < gTokN; i++)
  {
    if (gTok[i].token == token)
    {
      alias = gTok[i].alias;
      attr = gTok[i].attr;
      ch = gTok[i].ch;
      return true;
    }
  }
  return false;
}

String makeToken(const String &alias, const String &attr, int ch)
{
  String pfx;
  if (alias == "Analog_Level")
    pfx = "AL";
  else if (alias == "USB_Level")
    pfx = "UL";
  else if (alias == "Analog_Meter")
    pfx = "AM";
  else if (alias == "USB_Meter")
    pfx = "UM";
  else
    pfx = "XX";
  String A = (attr == "mute") ? "M" : "V"; // V=level/meter, M=mute
  return pfx + A + String(ch);
}

bool gBootDone = false;
void subscribeWithTokenOnce(const String &alias, const String &attr, int ch)
{
  String tok = makeToken(alias, attr, ch);
  addTokenMap(tok, alias, attr, ch);
  ttpSend(alias + " subscribe " + attr + " " + String(ch) + " " + tok);
}

void bootSubscribeAllWithTokens()
{
  if (gBootDone)
    return;
  gBootDone = true;
  ttpSend("SESSION set verbose true");

  // Meters
  subscribeWithTokenOnce("USB_Meter", "level", 1);
  subscribeWithTokenOnce("USB_Meter", "level", 2);
  subscribeWithTokenOnce("Analog_Meter", "level", 1);
  subscribeWithTokenOnce("Analog_Meter", "level", 2);

  // Controls
  for (int ch = 1; ch <= 2; ++ch)
  {
    subscribeWithTokenOnce("Analog_Level", "level", ch);
    subscribeWithTokenOnce("Analog_Level", "mute", ch);
    subscribeWithTokenOnce("USB_Level", "level", ch);
    subscribeWithTokenOnce("USB_Level", "mute", ch);
  }
}

// ===================== EQ queue (strict +OK) =====================
struct EqPending
{
  String alias;
  String attr;
  int band;
};
static EqPending EQ_Q[32];
static int EQ_Q_N = 0;
static bool EQ_INFLIGHT = false;
static unsigned long EQ_CMD_MS = 0;
static const unsigned long EQ_TIMEOUT_MS = 900;

struct EqBand
{
  float f = 0;
  bool hasF = false;
  float g = 0;
  bool hasG = false;
  float bw = 0;
  bool hasBW = false;
  bool by = false;
  bool hasBy = false;
};
EqBand usbEq[6]; // 1..5
EqBand aEq[6];

static void eqq_push(const String &alias, const char *attr, int band)
{
  if (EQ_Q_N >= (int)(sizeof(EQ_Q) / sizeof(EQ_Q[0])))
    return;
  EQ_Q[EQ_Q_N++] = EqPending{alias, String(attr), band};
}
static bool eqq_empty() { return EQ_Q_N == 0; }
static EqPending eqq_pop_front()
{
  EqPending p = EQ_Q[0];
  for (int i = 1; i < EQ_Q_N; ++i)
    EQ_Q[i - 1] = EQ_Q[i];
  if (EQ_Q_N > 0)
    --EQ_Q_N;
  return p;
}
static void eq_kick()
{
  if (EQ_INFLIGHT || eqq_empty() || !tesira.connected())
    return;
  const EqPending &p = EQ_Q[0];
  String line = p.alias + " get " + p.attr + " " + String(p.band);
  EQ_INFLIGHT = true;
  EQ_CMD_MS = millis();
  Serial.printf("[EQ KICK] inflight=1 size=%d  >> %s\n", EQ_Q_N, line.c_str());
  ttpSend(line);
}
void eqGetBandAll(const char *alias, int band)
{
  eqq_push(alias, "frequency", band);
  eqq_push(alias, "gain", band);
  eqq_push(alias, "bandwidth", band);
  eqq_push(alias, "bypass", band);
  Serial.printf("[EQ] Request %s band %d (all attrs)\n", alias, band);
  eq_kick();
}
void eqEmitBand(const char *alias, int band)
{
  EqBand *table = (strcmp(alias, "USB_EQ") == 0) ? usbEq : aEq;
  EqBand &b = table[band];
  if (!(b.hasF && b.hasG && b.hasBW && b.hasBy))
    return;
  StaticJsonDocument<256> j;
  j["block"] = alias;
  j["band"] = band;
  j["frequency"] = b.f;
  j["gain"] = b.g;
  j["bandwidth"] = b.bw;
  j["bypass"] = b.by;
  String out;
  serializeJson(j, out);
  Serial.printf("[WS TX][EQ] %s\n", out.c_str());
  ws.broadcastTXT(out);
}
void eqUpdateAttr(const char *alias, const char *attr, int band, float numValue, bool boolValue, bool isBoolAttr)
{
  if (band < 1 || band > 5)
    return;
  // validate ranges hard
  if (!isBoolAttr)
  {
    if (!strcmp(attr, "frequency"))
    {
      if (!(numValue >= 20.0f && numValue <= 20000.0f))
        return;
    }
    else if (!strcmp(attr, "gain"))
    {
      if (!(numValue >= -15.0f && numValue <= 15.0f))
        return;
    }
    else if (!strcmp(attr, "bandwidth"))
    {
      if (!(numValue >= 0.01f && numValue <= 4.0f))
        return;
    }
  }
  EqBand *table = (strcmp(alias, "USB_EQ") == 0) ? usbEq : aEq;
  EqBand &b = table[band];
  if (!isBoolAttr)
  {
    if (!strcmp(attr, "frequency"))
    {
      b.f = numValue;
      b.hasF = true;
    }
    else if (!strcmp(attr, "gain"))
    {
      b.g = numValue;
      b.hasG = true;
    }
    else if (!strcmp(attr, "bandwidth"))
    {
      b.bw = numValue;
      b.hasBW = true;
    }
  }
  else
  {
    b.by = boolValue;
    b.hasBy = true;
  }
  eqEmitBand(alias, band);
}

// ===================== Point-in-time control (compat only) =====================
String gLastAlias, gLastAttr;
int gLastCh = 0;
inline void rememberCmd(const String &a, const String &at, int ch)
{
  gLastAlias = a;
  gLastAttr = at;
  gLastCh = ch;
}

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
  if (CtrlState *cs = pickCtrl(alias))
  {
    cs[ch].mute = on;
    cs[ch].hasMute = true;
    cs[ch].muteMs = millis();
    wsSendCtrl(alias, ch, false, 0, true, on);
  }
  ttpSend(alias + " set mute " + String(ch) + " " + (on ? "true" : "false"));
}

// ===================== Parser =====================
static inline float parseValueFromOk(const String &line)
{
  // strict: must contain "value"
  int iv = line.indexOf("\"value\"");
  if (iv < 0)
    return NAN;
  int colon = line.indexOf(':', iv);
  if (colon < 0)
    return NAN;
  return line.substring(colon + 1).toFloat();
}
inline bool parse_mute_token_line_hasTF(const String &line, bool fallback)
{
  int t = line.lastIndexOf("true");
  int f = line.lastIndexOf("false");
  if (t >= 0 && (f < 0 || t > f))
    return true;
  if (f >= 0 && (t < 0 || f > t))
    return false;
  return fallback;
}

// ดึง float ตัว "สุดท้าย" จากบรรทัด (ทนทานกับรูปแบบหลากหลาย)
static float extractLastFloat(const String &s, bool *ok)
{
  int i = s.length() - 1;
  // ข้ามช่องว่าง/คอมมา/ปีกกา/quote ปลายบรรทัด
  while (i >= 0 && (s[i] == ' ' || s[i] == '\t' || s[i] == ',' || s[i] == '"' || s[i] == '}' || s[i] == ']'))
    i--;
  if (i < 0)
  {
    if (ok)
      *ok = false;
    return NAN;
  }
  // เก็บช่วงตัวเลขย้อนกลับ (รองรับ - + . e E)
  int end = i, start = i;
  auto isNumChar = [](char c)
  {
    return (c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E';
  };
  while (start >= 0 && isNumChar(s[start]))
    start--;
  start++;
  if (start > end)
  {
    if (ok)
      *ok = false;
    return NAN;
  }
  String sub = s.substring(start, end + 1);
  // ตรวจว่ามีตัวเลขจริง ๆ ไหม
  bool hasDigit = false;
  for (size_t k = 0; k < sub.length(); ++k)
    if (sub[k] >= '0' && sub[k] <= '9')
    {
      hasDigit = true;
      break;
    }
  if (!hasDigit)
  {
    if (ok)
      *ok = false;
    return NAN;
  }
  if (ok)
    *ok = true;
  return sub.toFloat();
}

// ดึงค่า boolean จากบรรทัด (หา true/false แบบไม่เคร่ง JSON)
static bool extractBoolTF(const String &s, bool *has)
{
  int t = s.lastIndexOf("true");
  int f = s.lastIndexOf("false");
  if (t >= 0 && (f < 0 || t > f))
  {
    if (has)
      *has = true;
    return true;
  }
  if (f >= 0 && (t < 0 || f > t))
  {
    if (has)
      *has = true;
    return false;
  }
  if (has)
    *has = false;
  return false;
}

void handleTtpLine(const String &line)
{
  String l = line;
  l.trim();
  if (l.isEmpty())
    return;

  // 1) publishToken lines
  if (l.startsWith("!"))
  {
    int t1 = l.indexOf("\"publishToken\""), v1 = l.indexOf("\"value\"");
    if (t1 >= 0 && v1 >= 0)
    {
      int q1 = l.indexOf('"', t1 + 15), q2 = (q1 >= 0) ? l.indexOf('"', q1 + 1) : -1;
      String token = (q1 >= 0 && q2 > q1) ? l.substring(q1 + 1, q2) : "";
      int colon = l.indexOf(':', v1);
      float numVal = (colon > 0) ? l.substring(colon + 1).toFloat() : NAN;

      String alias, attr;
      int ch = 0;
      if (!findToken(token, alias, attr, ch))
      {
        Serial.printf("[TOK?] Unknown token=%s\n", token.c_str());
        return;
      }

      // meters
      if (alias == "USB_Meter" || alias == "Analog_Meter")
      {
        if (alias == "USB_Meter")
        {
          if (ch == 1)
            usbLeft = numVal;
          else if (ch == 2)
            usbRight = numVal;
          sendVU("USB_Meter", usbLeft, usbRight, lastSendUsbVU);
        }
        else
        {
          if (ch == 1)
            analogLeft = numVal;
          else if (ch == 2)
            analogRight = numVal;
          sendVU("Analog_Meter", analogLeft, analogRight, lastSendAnalogVU);
        }
        return;
      }

      // controls (range clamp & sanity)
      if (CtrlState *cs = pickCtrl(alias))
      {
        const unsigned long now = millis();
        if (attr == "level")
        {
          if (isnan(numVal))
            return;
          if (!(numVal >= -100.0f && numVal <= 20.0f))
            return;
          if (alias == "USB_Level" && ch == 2 && _isBootSettling() && (numVal <= 0.5f || numVal >= 160.0f))
            return;
          cs[ch].level = numVal;
          cs[ch].hasLevel = true;
          cs[ch].levelMs = now;
          _broadcastLevelThrottled(alias, ch, numVal);
          return;
        }
        if (attr == "mute")
        {
          bool m;
          if (l.indexOf("true") >= 0 || l.indexOf("false") >= 0)
            m = parse_mute_token_line_hasTF(l, cs[ch].hasMute ? cs[ch].mute : false);
          else
            m = (!isnan(numVal) ? (numVal != 0.0f) : (cs[ch].hasMute ? cs[ch].mute : false));
          cs[ch].mute = m;
          cs[ch].hasMute = true;
          cs[ch].muteMs = now;
          wsSendCtrl(alias, ch, false, 0, true, m);
          return;
        }
      }
    }
    return;
  }

  // 2) +OK — for EQ inflight (robust parsing, no attr-name requirement)
  if (l.startsWith("+OK"))
  {
    if (!(EQ_INFLIGHT && EQ_Q_N > 0))
      return; // ignore unrelated +OK
    const EqPending p = eqq_pop_front();
    EQ_INFLIGHT = false;

    if (p.attr == "bypass")
    {
      bool hasB = false;
      bool bv = extractBoolTF(l, &hasB);
      if (!hasB)
      {
        // บางรุ่นอาจให้ "value":0/1
        float vMaybe = parseValueFromOk(l);
        if (!isnan(vMaybe))
        {
          bv = (vMaybe != 0.0f);
          hasB = true;
        }
      }
      if (hasB)
      {
        eqUpdateAttr(p.alias.c_str(), "bypass", p.band, 0.0f, bv, true);
        gLastEqOkMs = millis();
      }
      else
      {
        Serial.println("[EQ?] +OK bypass but no boolean found");
      }
      eq_kick();
      return;
    }

    // numeric attrs: frequency/gain/bandwidth
    float v = parseValueFromOk(l);
    if (isnan(v))
    {
      bool ok = false;
      v = extractLastFloat(l, &ok);
      if (!ok)
      {
        Serial.printf("[EQ?] +OK no numeric value for %s\n", p.attr.c_str());
        eq_kick();
        return;
      }
    }
    eqUpdateAttr(p.alias.c_str(), p.attr.c_str(), p.band, v, false, false);
    gLastEqOkMs = millis();
    eq_kick();
    return;
  }

  // others ignored
}

// ===================== Tesira connect =====================
bool lastTesiraOnline = false;

void pumpTesira(unsigned long budgetMicros = 6000)
{
  // EQ timeout
  if (EQ_INFLIGHT && (millis() - EQ_CMD_MS > EQ_TIMEOUT_MS))
  {
    Serial.println("[EQ TIMEOUT] drop inflight");
    EQ_INFLIGHT = false;
    if (!eqq_empty())
      (void)eqq_pop_front();
  }

  unsigned long t0 = micros();
  String line;
  while (tesira.connected() && ttpReadLine(line))
  {
    if (LOG_RAW)
    {
      Serial.print("[TTP <<] ");
      Serial.println(line);
    }
    handleTtpLine(line);
    if (micros() - t0 >= budgetMicros)
      break;
  }
}

void ensureTesiraConnected()
{
  bool was = tesira.connected();
  if (!tesira.connected())
  {
    unsigned long now = millis();
    if (now - lastTConnTry >= RECONN_MS)
    {
      lastTConnTry = now;
      Serial.printf("[TTP] Connecting %s:%u ...\n", TESIRA_IP, TESIRA_PORT);
      if (tesira.connect(TESIRA_IP, TESIRA_PORT))
      {
        tesira.setNoDelay(true);
        Serial.println("[TTP] Connected.");
        gLastConnectedMs = millis();
        bootSubscribeAllWithTokens();
      }
      else
      {
        Serial.println("[TTP] Connect failed.");
      }
    }
  }
  bool is = tesira.connected();
  if (is != was)
  {
    wsBroadcastTesiraOnline(is);
  }
}

// ===================== WebSocket (App) =====================
void handleMsg(uint8_t num, const String &msg)
{
  DynamicJsonDocument doc(256);
  if (deserializeJson(doc, msg))
    return;

  const String cmd = doc["cmd"] | "";
  const String block = doc["block"] | curBlock;
  const int ch = doc["ch"] | curCh;

  if (cmd == "subscribe")
  {
    curBlock = block;
    curCh = ch;
    if (CtrlState *cs = pickCtrl(block))
    {
      const unsigned long now = millis();

      // ใช้ค่า cache ถ้ามี (ไม่ต้องเช็ค 2s แล้ว)
      if (cs[ch].hasLevel)
      {
        _broadcastLevelThrottled(block, ch, cs[ch].level);
      }
      else
      {
        // ไม่มี cache → ลอง seed จากมิเตอร์ ถ้ามีจริง (> -99)
        float seed = -999.0f;
        if (block == "Analog_Level")
          seed = (ch == 1 ? analogLeft : analogRight);
        else if (block == "USB_Level")
          seed = (ch == 1 ? usbLeft : usbRight);

        if (seed > -99.0f)
        {
          seed = constrain(seed, -100.0f, 12.0f);
          // ส่งให้แอพดู แต่ "อย่า" mark เป็น hasLevel (เดี๋ยวค่าจริงจะตามมา)
          wsSendCtrl(block, ch, true, seed, false, false);
        }
        else
        {
          // ไม่มีทั้ง cache/meter → ขอค่าจริงจาก Tesira แล้ว "ยังไม่ส่ง"
          if (tesira.connected())
            ttpGetLevel(block, ch);
        }
      }

      // mute: ถ้ามี cache แล้วก็ส่งได้เลย ไม่ต้องเช็ค 2s
      if (cs[ch].hasMute)
      {
        wsSendCtrl(block, ch, false, 0, true, cs[ch].mute);
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
    Serial.printf("[APP >>] setLevel %s ch=%d db=%.1f\n", block.c_str(), ch, levelDb);
    return;
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
    Serial.printf("[APP >>] setMute %s ch=%d %s\n", block.c_str(), ch, isMute ? "true" : "false");
    return;
  }
  else if (cmd == "getLevel")
  {
    // compat only — แอพใหม่ควรพึ่ง subscribe เท่านั้น
    const String blk = doc["block"] | "";
    const int c = doc["ch"] | 1;
    if (blk.length() && tesira.connected())
      ttpGetLevel(blk, c);
    return;
  }
  else if (cmd == "snapshot")
  {
    // ส่งสถานะล่าสุดที่มีให้แอพ (ไม่ยิง get ใด ๆ)
    for (auto alias : {String("Analog_Level"), String("USB_Level")})
    {
      if (CtrlState *cs = pickCtrl(alias))
      {
        for (int c = 1; c <= 2; c++)
        {
          if (cs[c].hasLevel)
            wsSendCtrl(alias, c, true, cs[c].level, false, false);
          if (cs[c].hasMute)
            wsSendCtrl(alias, c, false, 0, true, cs[c].mute);
        }
      }
    }
    // meter จะถูกยิงต่อเนื่องอยู่แล้ว
    return;
  }
  else if (cmd == "eqGetBand")
  {
    String alias = doc["alias"] | "";
    int band = doc["band"] | 1;
    if (alias.length() && band >= 1 && band <= 5)
      eqGetBandAll(alias.c_str(), band);
    return;
  }
  else if (cmd == "eqSet")
  {
    String alias = doc["alias"] | "";
    int band = doc["band"] | 1;
    String attr = doc["attr"] | "";
    float value = doc["value"] | 0.0f;
    if (alias.length() && attr.length())
    {
      gLastAlias = alias;
      gLastAttr = attr;
      gLastCh = band;
      ttpSend(alias + " set " + attr + " " + String(band) + " " + String(value, 3));
    }
    return;
  }
  else if (cmd == "eqSetBypass")
  {
    String alias = doc["alias"] | "";
    int band = doc["band"] | 1;
    bool bypass = doc["bypass"] | false;
    gLastAlias = alias;
    gLastAttr = "bypass";
    gLastCh = band;
    ttpSend(alias + " set bypass " + String(band) + " " + (bypass ? "true" : "false"));
    return;
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
    // แจ้งสถานะ Tesira ปัจจุบันให้ไคลเอนต์ใหม่
    ws.sendTXT(num, String("{\"tesira_connected\":") + (tesira.connected() ? "true" : "false") + "}");
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

// ===================== Arduino Core =====================
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
  WiFi.setSleep(false);
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
    if (LOG_ERRORS)
      Serial.println("[mDNS] Failed to start");
  }

  ws.begin();
  ws.onEvent(onWsEvent);

  // เริ่มเชื่อม Tesira
  ensureTesiraConnected();
}

void loop()
{
  ws.loop();
  ensureTesiraConnected();
  pumpTesira(6000);

  // Throttle VU pair
  sendVU("Analog_Meter", analogLeft, analogRight, lastSendAnalogVU);
  sendVU("USB_Meter", usbLeft, usbRight, lastSendUsbVU);

  // ถ้า EQ คิวไม่ค้างและ Tesira พร้อม ให้ไหลคิว
  if (!EQ_INFLIGHT)
    eq_kick();
}