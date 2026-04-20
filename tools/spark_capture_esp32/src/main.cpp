/*
 * spark_capture_esp32 — Simulate a Spark 40 amp over Bluetooth Classic.
 *
 * The Spark Android app connects to this ESP32 as if it were a real amp.
 * "Send to Amp" captures the preset and prints a JSON: line over USB serial.
 *
 * Companion: tools/spark_capture.py reads the COM port and saves .json files.
 *
 * Board: ESP32 WROOM-32  (env: spark-capture)
 * Flash: pio run -e spark-capture --target upload   (from tools/spark_capture_esp32/)
 */

#include <Arduino.h>
#include <BluetoothSerial.h>

BluetoothSerial BT;

// ── 7-bit codec ──────────────────────────────────────────────────────────────

static size_t encode7bit(const uint8_t* in, size_t inLen, uint8_t* out) {
    size_t o = 0;
    for (size_t i = 0; i < inLen; i += 7) {
        size_t n = min((size_t)7, inLen - i);
        uint8_t prefix = 0;
        for (size_t j = 0; j < n; j++) prefix |= ((in[i+j] >> 7) & 1) << j;
        out[o++] = prefix;
        for (size_t j = 0; j < n; j++) out[o++] = in[i+j] & 0x7F;
    }
    return o;
}

static size_t decode7bit(const uint8_t* in, size_t inLen, uint8_t* out) {
    size_t o = 0, i = 0;
    while (i < inLen) {
        uint8_t prefix = in[i++];
        for (int b = 0; b < 7 && i < inLen; b++)
            out[o++] = in[i++] | (((prefix >> b) & 1) << 7);
    }
    return o;
}

// ── Frame builder (ESP32 → app) ───────────────────────────────────────────────

static uint8_t _msgNum = 0;

static void sendFrame(uint8_t cmd, uint8_t sub,
                      const uint8_t* payload = nullptr, size_t payLen = 0,
                      uint8_t msgNum = 0) {
    uint8_t enc[256]; size_t encLen = encode7bit(payload ? payload : (const uint8_t*)"", payLen, enc);
    uint8_t chk = 0; for (size_t i = 0; i < encLen; i++) chk ^= enc[i];

    if (msgNum == 0) msgNum = (++_msgNum) & 0x7F;

    // SysEx
    uint8_t sx[300]; size_t sl = 0;
    sx[sl++]=0xF0; sx[sl++]=0x01; sx[sl++]=msgNum;
    sx[sl++]=chk&0x7F; sx[sl++]=cmd; sx[sl++]=sub;
    memcpy(sx+sl, enc, encLen); sl += encLen;
    sx[sl++]=0xF7;

    // BLE-style block header
    uint8_t hdr[16] = {0x01,0xFE,0x00,0x00,0x53,0xFE,(uint8_t)(sl+16),0,0,0,0,0,0,0,0,0};
    BT.write(hdr, 16);
    BT.write(sx, sl);
}

// ACK echoing incoming msgNum and subCmd — cmd always 0x04 (Ignitron protocol)
static void sendAck(uint8_t incomingMsgNum, uint8_t subCmd) {
    sendFrame(0x04, subCmd, nullptr, 0, incomingMsgNum);
}

// ── Connection-sequence responses ─────────────────────────────────────────────

static void respSerial() {
    const char* s = "S999C999B999";
    uint8_t p[24]; size_t n = strlen(s);
    p[0]=(uint8_t)n; p[1]=(uint8_t)(0xA0+n); memcpy(p+2,s,n); p[2+n]=0xF7;
    sendFrame(0x03, 0x23, p, 2+n+1);
}
static void respFirmware()  { uint8_t p[]={0xCE,0x01,0x0A,0x08,0x19}; sendFrame(0x03,0x2F,p,5); }
static void respChecksums() { uint8_t p[]={0x94,0xCC,0x50,0x16,0x8F,0x58}; sendFrame(0x03,0x2A,p,6); }
static void respAmpStatus() { uint8_t p[]={0x0F,0x02,0x00,0x00,0xCD,0x0F,0xA9,0xCD,0x08,0x33,0x14}; sendFrame(0x03,0x71,p,11); }
static void respReady()     { uint8_t p[]={0xC3,0x1E,0x00,0x0F}; sendFrame(0x03,0x72,p,4); }
static void respPresetNum() { uint8_t p[]={0x00,0x00}; sendFrame(0x03,0x10,p,2); }

// ── Incoming stream parser ────────────────────────────────────────────────────

#define SBUF_SIZE 1024
static uint8_t _sbuf[SBUF_SIZE];
static size_t  _slen = 0;

struct Frame { uint8_t cmd, sub, msgNum, payload[512]; size_t payLen; };
static Frame _parsed[4];

static int streamFeed(const uint8_t* data, size_t len) {
    if (_slen + len > SBUF_SIZE) _slen = 0;
    memcpy(_sbuf + _slen, data, len); _slen += len;

    int count = 0; size_t i = 0;
    while (i + 16 <= _slen) {
        if (_sbuf[i] != 0x01 || _sbuf[i+1] != 0xFE) { i++; continue; }
        size_t totalLen = _sbuf[i+6];   // total packet length (header + SysEx)
        if (totalLen < 16 || i + totalLen > _slen) break;
        size_t flen = totalLen - 16;    // SysEx portion only

        const uint8_t* sx = _sbuf + i + 16;
        if (flen >= 7 && sx[0]==0xF0 && sx[flen-1]==0xF7 && count < 4) {
            Frame& f = _parsed[count++];
            f.msgNum = sx[2]; f.cmd = sx[4]; f.sub = sx[5];
            f.payLen = decode7bit(sx+6, flen-7, f.payload);
        }
        i += totalLen;
    }
    if (i) { memmove(_sbuf, _sbuf+i, _slen-i); _slen -= i; }
    return count;
}

// ── Multi-chunk assembler ─────────────────────────────────────────────────────

#define MAX_CHUNKS 8
static uint8_t _cdata[MAX_CHUNKS][200];
static size_t  _clens[MAX_CHUNKS];
static bool    _cpres[MAX_CHUNKS];
static int     _ctotal = -1;
static uint8_t _assembled[2048];

static void chunkReset() { _ctotal = -1; memset(_cpres,0,sizeof(_cpres)); }

// Returns assembled length in _assembled[], or -1 if incomplete.
static int chunkFeed(const uint8_t* payload, size_t len) {
    if (len < 3) return -1;
    int num=payload[0], idx=payload[1]; size_t cl=payload[2];
    if (_ctotal < 0) { _ctotal=num; memset(_cpres,0,sizeof(_cpres)); }
    if (idx >= MAX_CHUNKS || cl > 200) { chunkReset(); return -1; }
    memcpy(_cdata[idx], payload+3, cl); _clens[idx]=cl; _cpres[idx]=true;
    for (int k=0; k<_ctotal; k++) if (!_cpres[k]) return -1;
    size_t pos=0;
    for (int k=0; k<_ctotal; k++) { memcpy(_assembled+pos,_cdata[k],_clens[k]); pos+=_clens[k]; }
    chunkReset();
    return (int)pos;
}

// ── Preset → JSON ─────────────────────────────────────────────────────────────

static void jsonStr(const char* s) {
    Serial.print('"');
    for (; *s; s++) {
        if (*s=='"') Serial.print("\\\"");
        else if (*s=='\\') Serial.print("\\\\");
        else Serial.print(*s);
    }
    Serial.print('"');
}

static bool rdStr(const uint8_t* d, size_t dl, size_t& p, char* out, size_t max) {
    if (p >= dl) return false;
    uint8_t b=d[p]; size_t n=0;
    if (b==0xD9) { if (p+1>=dl) return false; n=d[p+1]; p+=2; }
    else if (b>=0xA0) { n=b-0xA0; p++; }
    else { out[0]=0; return true; }
    if (p+n>dl || n>=max) return false;
    memcpy(out,d+p,n); out[n]=0; p+=n; return true;
}

static bool rdFloat(const uint8_t* d, size_t dl, size_t& p, float& v) {
    if (p+4>=dl || d[p]!=0xCA) return false; p++;
    uint8_t b[4]={d[p+3],d[p+2],d[p+1],d[p]}; memcpy(&v,b,4); p+=4; return true;
}

static void printPresetJSON(const uint8_t* d, size_t dl) {
    size_t p=1; if (p>=dl) return;
    int pnum=d[p++];
    char uuid[64],name[64],ver[32],desc[128],icon[64];
    if (!rdStr(d,dl,p,uuid,64)) return;
    if (!rdStr(d,dl,p,name,64)) return;
    if (!rdStr(d,dl,p,ver, 32)) return;
    if (!rdStr(d,dl,p,desc,128)) return;
    if (!rdStr(d,dl,p,icon,64)) return;
    float bpm; if (!rdFloat(d,dl,p,bpm)) return;
    if (p>=dl) return;
    int nPed=d[p++]-0x90;

    Serial.print("JSON:{\"PresetNumber\":"); Serial.print(pnum);
    Serial.print(",\"UUID\":"); jsonStr(uuid);
    Serial.print(",\"Name\":"); jsonStr(name);
    Serial.print(",\"Version\":"); jsonStr(ver);
    Serial.print(",\"Description\":"); jsonStr(desc);
    Serial.print(",\"Icon\":"); jsonStr(icon);
    Serial.print(",\"BPM\":"); Serial.print(bpm,1);
    Serial.print(",\"Pedals\":[");

    for (int i=0; i<nPed; i++) {
        char pname[64]; if (!rdStr(d,dl,p,pname,64)) return;
        if (p>=dl) return;
        bool isOn=(d[p++]==0xC3);
        if (p>=dl) return;
        int nPar=d[p++]-0x90;
        if (i) Serial.print(',');
        Serial.print("{\"Name\":"); jsonStr(pname);
        Serial.print(",\"IsOn\":"); Serial.print(isOn?"true":"false");
        Serial.print(",\"Parameters\":[");
        for (int j=0; j<nPar; j++) {
            if (p>=dl) return;
            p++; // paramIdx
            if (p<dl && d[p]!=0xCA) p++; // skip 0x91 fixarray marker
            float v; if (!rdFloat(d,dl,p,v)) return;
            if (j) Serial.print(',');
            Serial.print(v,6);
        }
        Serial.print("]}");
    }
    uint8_t chk = (dl > 0) ? d[dl-1] : 0;
    Serial.print("],\"Checksum\":\""); Serial.print(chk); Serial.println("\"}");
}

// ── Setup & loop ──────────────────────────────────────────────────────────────

static void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
    if (event == ESP_SPP_SRV_OPEN_EVT)
        Serial.println("STATUS:Client connected");
    else if (event == ESP_SPP_CLOSE_EVT)
        Serial.println("STATUS:Client disconnected");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    BT.register_callback(btCallback);
    BT.begin("Spark 40 Audio", false);  // false = slave/server mode
    Serial.println("STATUS:Advertising as 'Spark 40 Audio' — waiting for Spark app...");
    Serial.println("STATUS:In the app: My Tones -> tap preset -> Send to Amp");
}

void loop() {
    if (!BT.available()) return;

    uint8_t buf[1024]; size_t n=0;
    while (BT.available() && n<sizeof(buf)) buf[n++]=(uint8_t)BT.read();
    if (!n) return;

    int nf = streamFeed(buf, n);
    for (int fi=0; fi<nf; fi++) {
        Frame& f = _parsed[fi];

        if (f.cmd == 0x02) {
            switch (f.sub) {
                case 0x23: respSerial();                              break;
                case 0x11: sendFrame(0x03, 0x11);                    break;
                case 0x2F: respFirmware();                           break;
                case 0x2A: respChecksums();                          break;
                case 0x71: respAmpStatus();                          break;
                case 0x72: respReady();                              break;
                case 0x10: respPresetNum();                          break;
                case 0x01: sendFrame(0x03, 0x01);                    break;
            }
            continue;
        }

        if (f.cmd==0x01 && f.sub==0x01) {
            int aLen = chunkFeed(f.payload, f.payLen);
            sendAck(f.msgNum, f.sub);
            if (aLen > 0) {
                Serial.println("STATUS:Preset received — parsing...");
                printPresetJSON(_assembled, (size_t)aLen);
            }
            continue;
        }

        if (f.cmd==0x01 && f.sub != 0x04) {
            sendAck(f.msgNum, f.sub);
            continue;
        }
    }
}
