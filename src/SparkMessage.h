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
#include <vector>
#include <cstring>
#include <algorithm>

// ── BLE protocol constants ─────────────────────────────────────────────────────

#define SPARK_DIR_TO_AMP_B0     0x53
#define SPARK_DIR_TO_AMP_B1     0xFE

// Spark 40 / GO / NEO: 173-byte BLE writes, 128-byte raw chunk limit.
// Each 128-byte chunk encodes to ≤150 bytes + 7-byte frame header = 157-byte
// SysEx frame, which fits in the 173 - 16 = 157 bytes available after the
// outer block header. These numbers are tight by design.
#define SPARK_CHUNK_MAX         0x80    // 128: max raw payload bytes per chunk
#define SPARK_BLE_WRITE_SIZE    0xAD    // 173: max bytes per BLE write

#define SPARK_PRESET_CUSTOM     0x7F    // presetNumber for on-the-fly custom presets

// msgpack-like type markers used in Spark's binary data encoding
#define SP_FLOAT_MARKER         0xCA
#define SP_BOOL_TRUE            0xC3
#define SP_BOOL_FALSE           0xC2
#define SP_LONGSTR_MARKER       0xD9    // strings > 31 chars: 0xD9 len bytes...
#define SP_STR_OFFSET           0xA0    // short strings: (len + 0xA0) bytes...
#define SP_ARR_OFFSET           0x90    // array/count header: (count + 0x90)

// ── Data structures ────────────────────────────────────────────────────────────

struct SparkPedal {
    char    name[64];
    bool    isOn;
    float   parameters[10];
    uint8_t numParameters;
};

struct SparkPreset {
    uint8_t    presetNumber;    // 0x7F for custom, 0-3 for HW slots
    char       uuid[40];
    char       name[33];
    char       version[8];
    char       description[128];
    char       icon[64];
    float      bpm;
    SparkPedal pedals[7];       // fixed order: gate, comp, drive, amp, mod, delay, reverb
};

// Each element is one complete, ready-to-write BLE payload
using SparkBlocks = std::vector<std::vector<uint8_t>>;

// ── SparkMessage ───────────────────────────────────────────────────────────────

class SparkMessage {
public:
    // Commands → amp
    static SparkBlocks switchHWPreset(uint8_t presetNum);
    static SparkBlocks sendPreset(const SparkPreset& preset);
    static SparkBlocks toggleEffect(const char* name, bool isOn);
    static SparkBlocks changeEffectParam(const char* name, uint8_t paramIdx, float value);
    static SparkBlocks changeEffectModel(const char* oldName, const char* newName);
    static SparkBlocks sendAck(uint8_t subCmd);

    // Queries (amp replies via BLE notify → SparkStreamReader)
    static SparkBlocks getSerialNumber();
    static SparkBlocks getAmpName();
    static SparkBlocks getHWChecksums();
    static SparkBlocks getCurrentPreset();
    static SparkBlocks getCurrentPresetNum();
    static SparkBlocks getFirmwareVersion();
    static SparkBlocks getAmpStatus();

private:
    static uint8_t _msgNum;

    static void appendStr(std::vector<uint8_t>& b, const char* s);
    static void appendPrefixedStr(std::vector<uint8_t>& b, const char* s);
    static void appendFloat(std::vector<uint8_t>& b, float v);
    static void appendBool(std::vector<uint8_t>& b, bool v);

    static std::vector<uint8_t> buildPresetData(const SparkPreset& p);
    static std::vector<uint8_t> encode7bit(const std::vector<uint8_t>& raw);
    static SparkBlocks          buildBlocks(uint8_t cmd, uint8_t subCmd,
                                            const std::vector<uint8_t>& payload);
};

uint8_t SparkMessage::_msgNum = 1;

// ── Private: data type helpers ─────────────────────────────────────────────────

void SparkMessage::appendStr(std::vector<uint8_t>& b, const char* s) {
    uint8_t len = (uint8_t)strlen(s);
    if (len <= 31) {
        b.push_back(SP_STR_OFFSET + len);
    } else {
        b.push_back(SP_LONGSTR_MARKER);
        b.push_back(len);
    }
    for (uint8_t i = 0; i < len; i++) b.push_back((uint8_t)s[i]);
}

void SparkMessage::appendFloat(std::vector<uint8_t>& b, float v) {
    uint32_t bits;
    memcpy(&bits, &v, 4);
    b.push_back(SP_FLOAT_MARKER);
    b.push_back((bits >> 24) & 0xFF);
    b.push_back((bits >> 16) & 0xFF);
    b.push_back((bits >>  8) & 0xFF);
    b.push_back( bits        & 0xFF);
}

// Prefixed-string format used in effect commands (toggleEffect, changeEffectParam, changeEffectModel).
// Matches Ignitron addPrefixedString: [rawLen][0xA0+rawLen][bytes…]
void SparkMessage::appendPrefixedStr(std::vector<uint8_t>& b, const char* s) {
    uint8_t len = (uint8_t)strlen(s);
    b.push_back(len);
    b.push_back(SP_STR_OFFSET + len);
    for (uint8_t i = 0; i < len; i++) b.push_back((uint8_t)s[i]);
}

void SparkMessage::appendBool(std::vector<uint8_t>& b, bool v) {
    b.push_back(v ? SP_BOOL_TRUE : SP_BOOL_FALSE);
}

// ── Private: preset data builder ───────────────────────────────────────────────

std::vector<uint8_t> SparkMessage::buildPresetData(const SparkPreset& p) {
    std::vector<uint8_t> b;

    b.push_back(0x00);           // direction marker
    b.push_back(p.presetNumber); // 0x7F for custom

    // UUID always as LongString regardless of length
    uint8_t uuidLen = (uint8_t)strlen(p.uuid);
    b.push_back(SP_LONGSTR_MARKER);
    b.push_back(uuidLen);
    for (uint8_t i = 0; i < uuidLen; i++) b.push_back((uint8_t)p.uuid[i]);

    appendStr(b, p.name);
    appendStr(b, p.version);
    appendStr(b, p.description);
    appendStr(b, p.icon);
    appendFloat(b, p.bpm);

    b.push_back(SP_ARR_OFFSET + 7); // 0x97 — always 7 pedals

    for (int i = 0; i < 7; i++) {
        appendStr(b, p.pedals[i].name);
        appendBool(b, p.pedals[i].isOn);
        b.push_back(SP_ARR_OFFSET + p.pedals[i].numParameters);
        for (uint8_t j = 0; j < p.pedals[i].numParameters; j++) {
            b.push_back(j);     // parameter index
            b.push_back(0x91);  // separator
            appendFloat(b, p.pedals[i].parameters[j]);
        }
    }

    // Preset checksum: sum of all bytes after the first 2 (direction + presetNumber), mod 256
    uint16_t sum = 0;
    for (size_t i = 2; i < b.size(); i++) sum += b[i];
    b.push_back((uint8_t)(sum & 0xFF));

    return b;
}

// ── Private: 7-bit encoding ────────────────────────────────────────────────────
//
// Packs groups of up to 7 bytes: a prefix byte holds the MSBs (bit 6 = MSB of
// byte 0, bit 5 = MSB of byte 1, … bit 0 = MSB of byte 6), then the 7 data
// bytes follow with their MSBs cleared.  This keeps all payload bytes below
// 0x80 so they can be carried inside the F0…F7 SysEx-style frame.

std::vector<uint8_t> SparkMessage::encode7bit(const std::vector<uint8_t>& raw) {
    std::vector<uint8_t> out;
    out.reserve(raw.size() + raw.size() / 7 + 2);
    size_t i = 0;
    while (i < raw.size()) {
        size_t n = std::min(raw.size() - i, (size_t)7);
        uint8_t prefix = 0;
        for (size_t j = 0; j < n; j++) {
            if (raw[i + j] & 0x80) prefix |= (uint8_t)(1 << j);  // LSB-first: bit j = MSB of byte j
        }
        out.push_back(prefix);
        for (size_t j = 0; j < n; j++) out.push_back(raw[i + j] & 0x7F);
        i += n;
    }
    return out;
}

// ── Private: block builder ─────────────────────────────────────────────────────
//
// Pipeline per chunk:
//   [sub-header if multi-chunk] + raw_chunk
//     → encode7bit
//     → SysEx frame:  F0 01 [msgNum] [checksum] [cmd] [subCmd] [encoded] F7
//     → BLE block:    01 FE 00 00 53 FE [frameLen] 00×9 [frame]

SparkBlocks SparkMessage::buildBlocks(uint8_t cmd, uint8_t subCmd,
                                      const std::vector<uint8_t>& payload) {
    SparkBlocks blocks;

    size_t numChunks = payload.empty() ? 1
                                       : (payload.size() + SPARK_CHUNK_MAX - 1) / SPARK_CHUNK_MAX;

    for (size_t ci = 0; ci < numChunks; ci++) {
        size_t start = ci * SPARK_CHUNK_MAX;
        size_t end   = std::min(start + (size_t)SPARK_CHUNK_MAX, payload.size());

        std::vector<uint8_t> chunk;
        chunk.reserve((end - start) + 3);

        // 3-byte sub-header only for multi-chunk messages
        if (numChunks > 1) {
            chunk.push_back((uint8_t)numChunks);
            chunk.push_back((uint8_t)ci);
            chunk.push_back((uint8_t)(end - start));
        }

        chunk.insert(chunk.end(), payload.begin() + start, payload.begin() + end);

        auto encoded = encode7bit(chunk);

        // Message checksum: XOR of all 7-bit encoded bytes
        uint8_t checksum = 0;
        for (uint8_t byte : encoded) checksum ^= byte;

        // SysEx frame
        std::vector<uint8_t> frame;
        frame.reserve(6 + encoded.size() + 1);
        frame.push_back(0xF0);
        frame.push_back(0x01);
        frame.push_back(_msgNum);
        frame.push_back(checksum);
        frame.push_back(cmd);
        frame.push_back(subCmd);
        frame.insert(frame.end(), encoded.begin(), encoded.end());
        frame.push_back(0xF7);

        _msgNum = (_msgNum >= 0xFF) ? 0x01 : _msgNum + 1;

        // Outer BLE block
        std::vector<uint8_t> block;
        block.reserve(16 + frame.size());
        block.push_back(0x01);
        block.push_back(0xFE);
        block.push_back(0x00);
        block.push_back(0x00);
        block.push_back(SPARK_DIR_TO_AMP_B0);   // 0x53
        block.push_back(SPARK_DIR_TO_AMP_B1);   // 0xFE
        block.push_back((uint8_t)frame.size());
        for (int f = 0; f < 9; f++) block.push_back(0x00); // filler
        block.insert(block.end(), frame.begin(), frame.end());

        blocks.push_back(std::move(block));
    }

    return blocks;
}

// ── Public: commands ───────────────────────────────────────────────────────────

SparkBlocks SparkMessage::switchHWPreset(uint8_t presetNum) {
    return buildBlocks(0x01, 0x38, {0x00, presetNum});
}

SparkBlocks SparkMessage::sendPreset(const SparkPreset& preset) {
    return buildBlocks(0x01, 0x01, buildPresetData(preset));
}

SparkBlocks SparkMessage::toggleEffect(const char* name, bool isOn) {
    std::vector<uint8_t> payload;
    appendPrefixedStr(payload, name);
    appendBool(payload, isOn);
    payload.push_back(0x00);  // trailing null required by amp (Ignitron commit 0d49c1b)
    return buildBlocks(0x01, 0x15, payload);
}

SparkBlocks SparkMessage::changeEffectParam(const char* name, uint8_t paramIdx, float value) {
    std::vector<uint8_t> payload;
    appendPrefixedStr(payload, name);
    payload.push_back(paramIdx);
    appendFloat(payload, value);
    return buildBlocks(0x01, 0x04, payload);
}

SparkBlocks SparkMessage::changeEffectModel(const char* oldName, const char* newName) {
    std::vector<uint8_t> payload;
    appendPrefixedStr(payload, oldName);
    appendPrefixedStr(payload, newName);
    return buildBlocks(0x01, 0x06, payload);
}

SparkBlocks SparkMessage::sendAck(uint8_t subCmd) {
    return buildBlocks(0x04, subCmd, {});
}

// ── Public: queries ────────────────────────────────────────────────────────────

SparkBlocks SparkMessage::getSerialNumber()     { return buildBlocks(0x02, 0x23, {}); }
SparkBlocks SparkMessage::getAmpName()          { return buildBlocks(0x02, 0x11, {}); }
SparkBlocks SparkMessage::getHWChecksums()      { return buildBlocks(0x02, 0x2A, {0x94, 0x00, 0x01, 0x02, 0x03}); }
SparkBlocks SparkMessage::getCurrentPreset()    { return buildBlocks(0x02, 0x01, {0x01, 0x00}); }
SparkBlocks SparkMessage::getCurrentPresetNum() { return buildBlocks(0x02, 0x10, {}); }
SparkBlocks SparkMessage::getFirmwareVersion()  { return buildBlocks(0x02, 0x2F, {}); }
SparkBlocks SparkMessage::getAmpStatus()        { return buildBlocks(0x02, 0x71, {}); }

#endif // SPARK_AMP
