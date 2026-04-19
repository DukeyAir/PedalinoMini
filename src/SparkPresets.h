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
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <vector>
#include <algorithm>
#include <cstring>
#include "SparkMessage.h"

// ── Storage layout ─────────────────────────────────────────────────────────────
//
// Preset files:  /spark/<name>.json   (standard Spark preset JSON format)
// Bank map:      /spark/banks.json    ([["file1.json", null, ...], ...])

#define SPARK_PRESET_DIR  "/spark"
#define SPARK_BANKS_FILE  "/spark/banks.json"
#define SPARK_BANKS       20
#define SPARK_SLOTS        4

// Bank assignment table — bankMap[bank][slot] = filename (empty = unassigned)
char sparkBankMap[SPARK_BANKS][SPARK_SLOTS][32];

// ── Init ───────────────────────────────────────────────────────────────────────

void sparkPresets_init() {
    memset(sparkBankMap, 0, sizeof(sparkBankMap));

    if (!SPIFFS.exists(SPARK_BANKS_FILE)) return;

    File f = SPIFFS.open(SPARK_BANKS_FILE, "r");
    if (!f) return;

    JsonDocument doc;
    if (deserializeJson(doc, f) != DeserializationError::Ok) { f.close(); return; }
    f.close();

    JsonArray banks = doc.as<JsonArray>();
    for (int b = 0; b < SPARK_BANKS && b < (int)banks.size(); b++) {
        JsonArray slots = banks[b].as<JsonArray>();
        for (int s = 0; s < SPARK_SLOTS && s < (int)slots.size(); s++) {
            const char* name = slots[s];
            if (name) strlcpy(sparkBankMap[b][s], name, 32);
        }
    }
}

// ── Bank persistence ───────────────────────────────────────────────────────────

void sparkPresets_saveBanks() {
    File f = SPIFFS.open(SPARK_BANKS_FILE, "w");
    if (!f) return;

    JsonDocument doc;
    JsonArray banks = doc.to<JsonArray>();
    for (int b = 0; b < SPARK_BANKS; b++) {
        JsonArray slots = banks.add<JsonArray>();
        for (int s = 0; s < SPARK_SLOTS; s++) {
            if (sparkBankMap[b][s][0]) slots.add(sparkBankMap[b][s]);
            else                       slots.add(nullptr);
        }
    }
    serializeJson(doc, f);
    f.close();
}

// ── Preset load ────────────────────────────────────────────────────────────────

bool sparkPresets_load(const char* filename, SparkPreset& out) {
    char path[72];
    snprintf(path, sizeof(path), "%s/%s", SPARK_PRESET_DIR, filename);

    File f = SPIFFS.open(path, "r");
    if (!f) return false;

    JsonDocument doc;
    auto err = deserializeJson(doc, f);
    f.close();
    if (err) return false;

    memset(&out, 0, sizeof(out));
    out.presetNumber = doc["PresetNumber"] | (uint8_t)SPARK_PRESET_CUSTOM;
    strlcpy(out.uuid,        doc["UUID"]        | "", sizeof(out.uuid));
    strlcpy(out.name,        doc["Name"]        | "", sizeof(out.name));
    strlcpy(out.version,     doc["Version"]     | "", sizeof(out.version));
    strlcpy(out.description, doc["Description"] | "", sizeof(out.description));
    strlcpy(out.icon,        doc["Icon"]        | "", sizeof(out.icon));
    out.bpm = doc["BPM"] | 120.0f;

    JsonArray pedals = doc["Pedals"].as<JsonArray>();
    for (int i = 0; i < 7 && i < (int)pedals.size(); i++) {
        JsonObject p = pedals[i].as<JsonObject>();
        strlcpy(out.pedals[i].name, p["Name"] | "", sizeof(out.pedals[i].name));
        out.pedals[i].isOn = p["IsOn"] | false;
        JsonArray params = p["Parameters"].as<JsonArray>();
        out.pedals[i].numParameters = (uint8_t)std::min((int)params.size(), 10);
        for (int j = 0; j < out.pedals[i].numParameters; j++)
            out.pedals[i].parameters[j] = params[j] | 0.0f;
    }
    return true;
}

// ── Preset save ────────────────────────────────────────────────────────────────

bool sparkPresets_save(const SparkPreset& p, const char* filename) {
    char path[72];
    snprintf(path, sizeof(path), "%s/%s", SPARK_PRESET_DIR, filename);

    File f = SPIFFS.open(path, "w");
    if (!f) return false;

    JsonDocument doc;
    doc["PresetNumber"] = p.presetNumber;
    doc["UUID"]         = p.uuid;
    doc["Name"]         = p.name;
    doc["Version"]      = p.version;
    doc["Description"]  = p.description;
    doc["Icon"]         = p.icon;
    doc["BPM"]          = p.bpm;

    JsonArray pedals = doc["Pedals"].to<JsonArray>();
    for (int i = 0; i < 7; i++) {
        JsonObject po = pedals.add<JsonObject>();
        po["Name"] = p.pedals[i].name;
        po["IsOn"] = p.pedals[i].isOn;
        JsonArray params = po["Parameters"].to<JsonArray>();
        for (int j = 0; j < p.pedals[i].numParameters; j++)
            params.add(p.pedals[i].parameters[j]);
    }

    serializeJson(doc, f);
    f.close();
    return true;
}

// ── Preset delete ──────────────────────────────────────────────────────────────

bool sparkPresets_delete(const char* filename) {
    // Remove from bank map
    for (int b = 0; b < SPARK_BANKS; b++)
        for (int s = 0; s < SPARK_SLOTS; s++)
            if (strcmp(sparkBankMap[b][s], filename) == 0)
                sparkBankMap[b][s][0] = '\0';
    sparkPresets_saveBanks();

    char path[72];
    snprintf(path, sizeof(path), "%s/%s", SPARK_PRESET_DIR, filename);
    return SPIFFS.remove(path);
}

// ── Preset list ────────────────────────────────────────────────────────────────

std::vector<String> sparkPresets_list() {
    std::vector<String> files;
    File root = SPIFFS.open("/");
    if (!root) return files;

    File f = root.openNextFile();
    while (f) {
        // Normalise: ensure leading slash (ESP-IDF v4+ omits it)
        String fullPath = f.name();
        if (!fullPath.startsWith("/")) fullPath = "/" + fullPath;

        // Extract base filename (last path component)
        int sep = fullPath.lastIndexOf('/');
        String base = (sep >= 0) ? fullPath.substring(sep + 1) : fullPath.substring(1);

        if (base.endsWith(".json") && base != "banks.json" && !base.isEmpty()) {
            // Accept if the full path is in our directory, OR verify via exists()
            // (handles SPIFFS variants that strip the directory from f.name())
            char expected[72];
            snprintf(expected, sizeof(expected), "%s/%s", SPARK_PRESET_DIR, base.c_str());
            if (fullPath.startsWith(SPARK_PRESET_DIR "/") || SPIFFS.exists(expected)) {
                files.push_back(base);
            }
        }
        f = root.openNextFile();
    }
    return files;
}

// ── Load by bank/slot index ────────────────────────────────────────────────────

bool sparkPresets_loadByIndex(uint8_t bank, uint8_t slot, SparkPreset& out) {
    if (bank >= SPARK_BANKS || slot >= SPARK_SLOTS) return false;
    if (sparkBankMap[bank][slot][0] == '\0') return false;
    return sparkPresets_load(sparkBankMap[bank][slot], out);
}

// ── Save uploaded preset from raw JSON bytes ───────────────────────────────────

bool sparkPresets_saveRaw(const char* filename, const uint8_t* data, size_t len) {
    // Validate it's valid JSON with the expected shape before storing
    JsonDocument doc;
    if (deserializeJson(doc, data, len) != DeserializationError::Ok) return false;
    if (!doc["Name"].is<const char*>()) return false;
    if (!doc["Pedals"].is<JsonArray>()) return false;

    char path[72];
    snprintf(path, sizeof(path), "%s/%s", SPARK_PRESET_DIR, filename);
    File f = SPIFFS.open(path, "w");
    if (!f) return false;
    size_t written = f.write(data, len);
    f.close();
    return (written == len);
}

#endif // SPARK_AMP
