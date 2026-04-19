/*
__________           .___      .__  .__                 _____  .__       .__     ___ ________________    ___
\______   \ ____   __| _/____  |  | |__| ____   ____   /     \ |__| ____ |__|   /  / \__    ___/     \   \  \
 |     ___// __ \ / __ |\__  \ |  | |  |/    \ /  _ \ /  \ /  \|  |/    \|  |  /  /    |    | /  \ /  \   \  \
 |    |   \  ___// /_/ | / __ \|  |_|  |   |  (  <_> )    Y    \  |   |  \  | (  (     |    |/    Y    \   )  )
 |____|    \___  >____ |(____  /____/__|___|  /\____/\____|__  /__|___|  /__|  \  \    |____|\____|__  /  /  /
               \/     \/     \/             \/               \/        \/       \__\                 \/  /__/
                                                                                   (c) 2018-2024 alf45star
                                                                       https://github.com/alf45tar/PedalinoMini
 */

#pragma once

#if defined(SPARK_AMP)

#include <Arduino.h>
#include <functional>
#include <vector>
#include <algorithm>
#include <cstring>
#include "SparkMessage.h"

// ── Callback types ─────────────────────────────────────────────────────────────

using SparkPresetCb    = std::function<void(const SparkPreset&)>;
using SparkPresetNumCb = std::function<void(uint8_t)>;
using SparkStringCb    = std::function<void(const char*)>;
using SparkAckCb       = std::function<void(uint8_t subCmd)>;
using SparkEffectCb    = std::function<void(const char* name, bool isOn)>;

// ── SparkStreamReader ──────────────────────────────────────────────────────────
//
// Parses raw BLE notification blocks from the Spark amp into structured events.
// Feed each notification payload into feed(); registered callbacks fire when a
// complete, decoded message is available.

class SparkStreamReader {
public:
    SparkPresetCb    onPreset;           // cmd=0x03 subCmd=0x01 — full preset received
    SparkPresetNumCb onPresetNum;        // cmd=0x03 subCmd=0x10 — current HW preset 0-3
    SparkStringCb    onAmpName;          // cmd=0x03 subCmd=0x11
    SparkStringCb    onSerialNumber;     // cmd=0x03 subCmd=0x23
    SparkStringCb    onFirmwareVersion;  // cmd=0x03 subCmd=0x2F
    SparkAckCb       onFinalAck;         // cmd=0x04 — final ACK, proceed to next step
    SparkAckCb       onInterimAck;       // cmd=0x05 — interim ACK, send next chunk
    SparkEffectCb    onEffectToggle;     // cmd=0x01 subCmd=0x15 — effect toggled on amp

    void feed(const uint8_t* data, size_t len);

private:
    std::vector<uint8_t> _rawBuf;    // BLE-level cross-notification reassembly
    std::vector<uint8_t> _chunkBuf;
    uint8_t _expectedChunks = 0;
    uint8_t _lastCmd = 0, _lastSubCmd = 0;

    static std::vector<uint8_t> decode7bit(const uint8_t* enc, size_t len);
    void processDecoded(uint8_t cmd, uint8_t subCmd, const std::vector<uint8_t>& raw);
    void dispatchPayload(uint8_t cmd, uint8_t subCmd, const uint8_t* data, size_t len);

    static size_t parseStr(const uint8_t* d, size_t i, size_t len, char* out, size_t outLen);
    static size_t parseFloat(const uint8_t* d, size_t i, size_t len, float& out);
    static bool   parsePreset(const uint8_t* d, size_t len, SparkPreset& out);
};

// ── feed() ─────────────────────────────────────────────────────────────────────
//
// Block layout (16-byte header then one or more F0…F7 SysEx frames):
//   01 FE 00 00 [dir0] [dir1] [frameLen] 00×9  [F0 … F7]

// Last raw notification bytes (for debug via status endpoint)
static uint8_t  _lastNotifyBuf[20] = {};
static size_t   _lastNotifyLen     = 0;

// Raw decoded bytes for last ampName / serial dispatches (debug)
static uint8_t  _dbgAmpNameRaw[24] = {};
static uint8_t  _dbgAmpNameRawLen  = 0;
static uint8_t  _dbgSerialRaw[24]  = {};
static uint8_t  _dbgSerialRawLen   = 0;

void SparkStreamReader::feed(const uint8_t* data, size_t len) {
    if (len == 0) return;

    // Capture first 20 bytes for debug
    _lastNotifyLen = std::min(len, (size_t)20);
    memcpy(_lastNotifyBuf, data, _lastNotifyLen);

    // Accumulate bytes across BLE notifications — the Spark GO (and other models)
    // fragment SysEx frames across multiple 20-byte ATT notifications, so a single
    // F0…F7 frame may arrive in pieces.
    _rawBuf.insert(_rawBuf.end(), data, data + len);
    if (_rawBuf.size() > 4096) { _rawBuf.clear(); return; }  // sanity cap

    size_t p            = 0;
    size_t consumeUpTo  = 0;
    bool   hasIncomplete = false;

    while (p < _rawBuf.size()) {
        if (_rawBuf[p] != 0xF0) { p++; continue; }

        size_t fStart = p;
        size_t fEnd   = fStart + 1;
        while (fEnd < _rawBuf.size() && _rawBuf[fEnd] != 0xF7) fEnd++;

        if (fEnd >= _rawBuf.size()) {
            consumeUpTo   = fStart;   // keep from this F0 onwards
            hasIncomplete = true;
            break;
        }

        // Complete frame: F0 01 msgNum checksum cmd subCmd [encoded…] F7
        if (fEnd - fStart >= 6) {
            uint8_t cmd    = _rawBuf[fStart + 4];
            uint8_t subCmd = _rawBuf[fStart + 5];
            auto decoded   = decode7bit(_rawBuf.data() + fStart + 6, fEnd - fStart - 6);
            processDecoded(cmd, subCmd, decoded);
        }
        p           = fEnd + 1;
        consumeUpTo = p;
    }

    if (!hasIncomplete) consumeUpTo = _rawBuf.size();
    _rawBuf.erase(_rawBuf.begin(), _rawBuf.begin() + consumeUpTo);
}

// ── 7-bit decode ───────────────────────────────────────────────────────────────
//
// Inverse of SparkMessage::encode7bit.
// Each group: 1 prefix byte (bit 6..0 = MSBs of bytes 0..6) followed by up to
// 7 data bytes with MSBs cleared.

std::vector<uint8_t> SparkStreamReader::decode7bit(const uint8_t* enc, size_t len) {
    std::vector<uint8_t> out;
    out.reserve(len);
    size_t i = 0;
    while (i < len) {
        uint8_t prefix = enc[i++];
        for (int bit = 0; bit < 7 && i < len; bit++, i++) {  // LSB-first: bit j = MSB of byte j
            uint8_t b = enc[i];
            if (prefix & (1 << bit)) b |= 0x80;
            out.push_back(b);
        }
    }
    return out;
}

// ── processDecoded() ───────────────────────────────────────────────────────────
//
// Handles multi-chunk reassembly before dispatching.
// Multi-chunk sub-header: [numChunks][chunkIdx][chunkDataLen][data…]
// Detected when numChunks > 1 and chunkDataLen == decoded.size() - 3.

void SparkStreamReader::processDecoded(uint8_t cmd, uint8_t subCmd,
                                       const std::vector<uint8_t>& raw) {
    if (raw.empty()) {
        if (cmd == 0x04 && onFinalAck)   onFinalAck(subCmd);
        if (cmd == 0x05 && onInterimAck) onInterimAck(subCmd);
        return;
    }

    uint8_t numChunks = raw[0], chunkIdx = raw[1];
    uint8_t chunkDataLen = raw.size() >= 3 ? raw[2] : 0;
    bool isMultiChunk = (numChunks > 1 && numChunks <= 20 &&
                         raw.size() >= 3 &&
                         chunkDataLen == (uint8_t)(raw.size() - 3));

    if (isMultiChunk) {
        if (chunkIdx == 0) {
            _chunkBuf.clear();
            _expectedChunks = numChunks;
            _lastCmd        = cmd;
            _lastSubCmd     = subCmd;
        }
        _chunkBuf.insert(_chunkBuf.end(), raw.begin() + 3, raw.end());

        if (chunkIdx < numChunks - 1) {
            if (onInterimAck) onInterimAck(subCmd);
            return;
        }
        dispatchPayload(_lastCmd, _lastSubCmd, _chunkBuf.data(), _chunkBuf.size());
        _chunkBuf.clear();
        _expectedChunks = 0;
        return;
    }

    dispatchPayload(cmd, subCmd, raw.data(), raw.size());
}

// ── dispatchPayload() ──────────────────────────────────────────────────────────

void SparkStreamReader::dispatchPayload(uint8_t cmd, uint8_t subCmd,
                                        const uint8_t* data, size_t len) {
    switch (cmd) {
        case 0x03:
            switch (subCmd) {
                case 0x01: {
                    SparkPreset p = {};
                    DPRINT("[Spark] preset data arrived len=%d  [%02X %02X %02X %02X %02X %02X %02X %02X]\n",
                           (int)len,
                           len>0?data[0]:0, len>1?data[1]:0, len>2?data[2]:0, len>3?data[3]:0,
                           len>4?data[4]:0, len>5?data[5]:0, len>6?data[6]:0, len>7?data[7]:0);
                    bool ok = parsePreset(data, len, p);
                    DPRINT("[Spark] parsePreset=%d name='%s'\n", (int)ok, ok ? p.name : "?");
                    if (ok && onPreset) onPreset(p);
                    break;
                }
                case 0x10:
                    // payload: [0x00, presetNum]
                    if (len >= 2 && onPresetNum) onPresetNum(data[1]);
                    break;
                case 0x11: {
                    _dbgAmpNameRawLen = (uint8_t)std::min(len, (size_t)24);
                    memcpy(_dbgAmpNameRaw, data, _dbgAmpNameRawLen);
                    char s[64] = {};
                    parseStr(data, 0, len, s, sizeof(s));
                    if (s[0] == '\0' && len > 1) parseStr(data, 1, len, s, sizeof(s));  // skip GO raw-length prefix
                    if (s[0] == '\0' && len > 3) parseStr(data, 3, len, s, sizeof(s));  // skip chunk sub-header
                    if (onAmpName) onAmpName(s);
                    break;
                }
                case 0x23: {
                    _dbgSerialRawLen = (uint8_t)std::min(len, (size_t)24);
                    memcpy(_dbgSerialRaw, data, _dbgSerialRawLen);
                    char s[64] = {};
                    parseStr(data, 0, len, s, sizeof(s));
                    if (s[0] == '\0' && len > 1) parseStr(data, 1, len, s, sizeof(s));  // skip GO raw-length prefix
                    if (s[0] == '\0' && len > 3) parseStr(data, 3, len, s, sizeof(s));  // skip chunk sub-header
                    if (onSerialNumber) onSerialNumber(s);
                    break;
                }
                case 0x2F: {
                    // Firmware version: 0xCE major minor revision build
                    char v[32] = {};
                    if (len >= 5 && data[0] == 0xCE)
                        snprintf(v, sizeof(v), "%d.%d.%d.%d", data[1], data[2], data[3], data[4]);
                    else
                        parseStr(data, 0, len, v, sizeof(v));  // fallback: try string
                    if (onFirmwareVersion) onFirmwareVersion(v);
                    break;
                }
            }
            break;

        case 0x04:
            if (onFinalAck) onFinalAck(subCmd);
            break;

        case 0x05:
            if (onInterimAck) onInterimAck(subCmd);
            break;

        case 0x01:
            if (subCmd == 0x15 && len >= 2) {
                char name[64] = {};
                size_t i = parseStr(data, 0, len, name, sizeof(name));
                if (i < len && onEffectToggle)
                    onEffectToggle(name, data[i] == SP_BOOL_TRUE);
            }
            break;
    }
}

// ── String / Float parsers ─────────────────────────────────────────────────────

size_t SparkStreamReader::parseStr(const uint8_t* d, size_t i, size_t len,
                                    char* out, size_t outLen) {
    if (i >= len) return i;
    uint8_t sLen;
    if (d[i] == SP_LONGSTR_MARKER) {
        if (++i >= len) return i;
        sLen = d[i++];
    } else if (d[i] >= SP_STR_OFFSET) {
        sLen = d[i++] - SP_STR_OFFSET;
    } else {
        return i;
    }
    if (i + sLen > len) return len;
    size_t cp = std::min((size_t)sLen, outLen - 1);
    memcpy(out, d + i, cp);
    out[cp] = '\0';
    return i + sLen;
}

size_t SparkStreamReader::parseFloat(const uint8_t* d, size_t i, size_t len, float& out) {
    if (i + 5 > len || d[i] != SP_FLOAT_MARKER) return i;
    i++;
    uint32_t bits = ((uint32_t)d[i] << 24) | ((uint32_t)d[i+1] << 16) |
                    ((uint32_t)d[i+2] << 8) |  (uint32_t)d[i+3];
    memcpy(&out, &bits, 4);
    return i + 4;
}

// ── Preset parser ──────────────────────────────────────────────────────────────

bool SparkStreamReader::parsePreset(const uint8_t* d, size_t len, SparkPreset& out) {
    if (len < 4) return false;
    size_t i = 0;
    i++;                          // skip direction byte (0x00 app→amp, 0x01 amp→app)
    out.presetNumber = d[i++];   // preset number

    // UUID — always LongString
    if (i >= len || d[i] != SP_LONGSTR_MARKER) return false;
    i++;
    if (i >= len) return false;
    uint8_t uuidLen = d[i++];
    if (i + uuidLen > len) return false;
    size_t ucp = std::min((size_t)uuidLen, sizeof(out.uuid) - 1);
    memcpy(out.uuid, d + i, ucp);
    out.uuid[ucp] = '\0';
    i += uuidLen;

    i = parseStr(d, i, len, out.name,        sizeof(out.name));
    i = parseStr(d, i, len, out.version,     sizeof(out.version));
    i = parseStr(d, i, len, out.description, sizeof(out.description));
    i = parseStr(d, i, len, out.icon,        sizeof(out.icon));
    i = parseFloat(d, i, len, out.bpm);

    // 7-pedal array marker: 0x97
    if (i >= len || d[i] != (SP_ARR_OFFSET + 7)) return false;
    i++;

    for (int p = 0; p < 7 && i < len; p++) {
        i = parseStr(d, i, len, out.pedals[p].name, sizeof(out.pedals[p].name));
        if (i >= len) return false;
        out.pedals[p].isOn = (d[i++] == SP_BOOL_TRUE);
        if (i >= len) return false;
        uint8_t nParams = d[i++] - SP_ARR_OFFSET;
        out.pedals[p].numParameters = std::min(nParams, (uint8_t)10);
        for (uint8_t j = 0; j < nParams; j++) {
            if (i + 2 > len) return false;
            i++;                               // param index byte
            if (d[i++] != 0x91) return false;  // separator
            float v = 0.0f;
            i = parseFloat(d, i, len, v);
            if (j < 10) out.pedals[p].parameters[j] = v;
        }
    }
    return true;
}

#endif // SPARK_AMP
