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

#if defined(SPARK_AMP) && defined(BLE)

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <cstring>
#include "SparkMessage.h"
#include "SparkStreamReader.h"
#include "SparkPresets.h"
#include "SparkBLE.h"

// ── Command types (fire_action → Spark task via queue) ─────────────────────────

enum SparkCmdType : uint8_t {
    SPARK_CMD_PRESET,         // load preset from bank/slot and send to amp
    SPARK_CMD_HW_PRESET,      // switch to one of the amp's 4 built-in presets
    SPARK_CMD_EFFECT_TOGGLE,  // toggle one effect on/off
    SPARK_CMD_EFFECT_PARAM,   // change a single effect parameter value
    SPARK_CMD_EFFECT_MODEL,   // swap an effect model
    SPARK_CMD_RECONNECT,      // trigger a reconnect attempt
};

struct SparkCmd {
    SparkCmdType type;
    uint8_t      bank;
    uint8_t      slot;
    char         effectName[32];
    char         effectNewName[32]; // for EFFECT_MODEL
    bool         effectIsOn;
    uint8_t      paramIdx;
    float        paramValue;
};

// ── Shared state (readable from any task/core) ─────────────────────────────────

bool     sparkConnected       = false;
uint8_t  sparkCurrentBank     = 0;     // active Spark bank (0-based)
uint8_t  sparkCurrentSlot     = 0;     // active slot within bank (0-3)
char     sparkAmpName[32]     = "";
char     sparkSerial[32]      = "";
char     sparkFirmware[32]    = "";
SparkPreset sparkCurrentPreset      = {};    // last preset loaded on the amp
bool        sparkHasPreset          = false;
char        sparkCurrentPresetName[33] = ""; // always synced with sparkCurrentPreset.name
uint8_t     sparkCurrentHWPreset    = 0xFF;  // 0-3 = HW slot active, 0xFF = custom
char        sparkHWPresetNames[4][33]  = {}; // amp-reported names, cached per HW slot
bool        sparkHWPresetNameKnown[4]  = {};
char        sparkCustomSlotNames[4][33]= {}; // preset names for current sparkCurrentBank

// ── Private ────────────────────────────────────────────────────────────────────

static QueueHandle_t     _sparkQueue;
static SemaphoreHandle_t _sparkFinalAckSem;
static SemaphoreHandle_t _sparkInterimAckSem;
static volatile uint8_t  _sparkAckSubCmd    = 0;
static volatile uint8_t  _sparkExpectedSlot = 0xFF; // slot set before getCurrentPreset() after HW switch

// ── ACK wait helpers ───────────────────────────────────────────────────────────

static void sparkUpdateCustomSlotNames() {
    for (int s = 0; s < SPARK_SLOTS; s++) {
        sparkCustomSlotNames[s][0] = '\0';
        if (sparkBankMap[sparkCurrentBank][s][0]) {
            SparkPreset p;
            if (sparkPresets_loadByIndex(sparkCurrentBank, s, p))
                strlcpy(sparkCustomSlotNames[s], p.name, sizeof(sparkCustomSlotNames[s]));
        }
    }
}

static bool sparkWaitInterimAck(uint8_t expectSubCmd, uint32_t timeoutMs = 3000) {
    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(timeoutMs);
    while (xTaskGetTickCount() < end) {
        if (xSemaphoreTake(_sparkInterimAckSem, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (_sparkAckSubCmd == expectSubCmd) return true;
        }
    }
    return false;
}

static bool sparkWaitFinalAck(uint8_t expectSubCmd, uint32_t timeoutMs = 3000) {
    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(timeoutMs);
    while (xTaskGetTickCount() < end) {
        if (xSemaphoreTake(_sparkFinalAckSem, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (_sparkAckSubCmd == expectSubCmd) return true;
            DPRINT("[Spark] waitFinalAck: got 0x%02X, want 0x%02X — discarding\n",
                   _sparkAckSubCmd, expectSubCmd);
        }
    }
    return false;
}

// ── Send a full preset with multi-chunk ACK handshake ─────────────────────────

static bool sparkSendPreset(const SparkPreset& preset) {
    SparkBlocks blocks = SparkMessage::sendPreset(preset);

    // Drain any stale interimAck signal left over from previous amp responses
    // (amp floods interimAcks during getCurrentPreset multi-chunk replies).
    xSemaphoreTake(_sparkInterimAckSem, 0);

    // Amp sends interimAck (cmd=0x05, subCmd=0x01) for all but the last block,
    // and finalAck (cmd=0x04, subCmd=0x01) for the last block.
    for (size_t i = 0; i < blocks.size(); i++) {
        SparkBlocks single = {blocks[i]};
        if (!sparkBLE_writeBlocks(single)) return false;

        if (i < blocks.size() - 1) {
            if (!sparkWaitInterimAck(0x01, 3000)) {
                DPRINT("Spark: timeout waiting for interim ACK on chunk %d\n", (int)i);
                return false;
            }
        }
    }

    if (!sparkWaitFinalAck(0x01, 3000)) {
        DPRINT("Spark: timeout waiting for final preset ACK\n");
        return false;
    }
    DPRINT("[Spark] preset send complete, activating custom slot\n");

    // Tell amp to activate the custom edit slot (0x7F)
    sparkBLE_writeBlocks(SparkMessage::switchHWPreset(SPARK_PRESET_CUSTOM));
    if (!sparkWaitFinalAck(0x38, 2000)) {
        DPRINT("[Spark] no ACK for custom slot activation\n");
    }

    sparkCurrentPreset = preset;
    sparkHasPreset     = true;
    strlcpy(sparkCurrentPresetName, preset.name, sizeof(sparkCurrentPresetName));
    return true;
}

// ── Execute one command from the queue ─────────────────────────────────────────

static void sparkExecuteCmd(const SparkCmd& cmd) {
    switch (cmd.type) {
        case SPARK_CMD_PRESET: {
            DPRINT("[Spark] preset bank=%d slot=%d file='%s'\n",
                   cmd.bank, cmd.slot, sparkBankMap[cmd.bank][cmd.slot]);
            SparkPreset preset;
            if (sparkPresets_loadByIndex(cmd.bank, cmd.slot, preset)) {
                DPRINT("[Spark] preset loaded: '%s'\n", preset.name);
                if (sparkSendPreset(preset)) {
                    sparkCurrentBank     = cmd.bank;
                    sparkCurrentSlot     = cmd.slot;
                    sparkCurrentHWPreset = 0xFF;
                    sparkUpdateCustomSlotNames();
                }
            } else {
                DPRINT("[Spark] preset load failed — bank/slot empty or file missing\n");
            }
            break;
        }
        case SPARK_CMD_HW_PRESET:
            sparkBLE_writeBlocks(SparkMessage::switchHWPreset(cmd.slot));
            if (sparkWaitFinalAck(0x38, 1500)) {
                sparkCurrentHWPreset = cmd.slot;
                sparkHasPreset       = true;
                _sparkExpectedSlot   = cmd.slot;
                // 300ms pre-wait before name fetch — abort immediately if another command queued
                for (int t = 0; t < 30 && uxQueueMessagesWaiting(_sparkQueue) == 0; t++)
                    vTaskDelay(pdMS_TO_TICKS(10));
                if (uxQueueMessagesWaiting(_sparkQueue) == 0) {
                    sparkBLE_writeBlocks(SparkMessage::getCurrentPreset());
                    for (int t = 0; t < 20 && _sparkExpectedSlot != 0xFF
                                    && uxQueueMessagesWaiting(_sparkQueue) == 0; t++)
                        vTaskDelay(pdMS_TO_TICKS(50));
                }
            }
            break;

        case SPARK_CMD_EFFECT_TOGGLE:
            sparkBLE_writeBlocks(SparkMessage::toggleEffect(cmd.effectName, cmd.effectIsOn));
            // Update our local preset copy
            for (int i = 0; i < 7; i++) {
                if (strcmp(sparkCurrentPreset.pedals[i].name, cmd.effectName) == 0) {
                    sparkCurrentPreset.pedals[i].isOn = cmd.effectIsOn;
                    break;
                }
            }
            break;

        case SPARK_CMD_EFFECT_PARAM:
            sparkBLE_writeBlocks(SparkMessage::changeEffectParam(
                cmd.effectName, cmd.paramIdx, cmd.paramValue));
            break;

        case SPARK_CMD_EFFECT_MODEL:
            sparkBLE_writeBlocks(SparkMessage::changeEffectModel(
                cmd.effectName, cmd.effectNewName));
            break;

        case SPARK_CMD_RECONNECT:
            sparkBLE_disconnect();
            sparkConnected = false;
            break;
    }
}

// ── Connection sequence after BLE link is up ───────────────────────────────────

static void sparkRunConnectionSequence() {
    DPRINT("Spark: running connection sequence\n");

    // 1. Get serial number — first message, triggers amp name response
    sparkBLE_writeBlocks(SparkMessage::getSerialNumber());
    sparkWaitFinalAck(0x23, 2000);

    vTaskDelay(pdMS_TO_TICKS(100));

    // 2. Get amp name → sets BLE write size based on model
    sparkBLE_writeBlocks(SparkMessage::getAmpName());
    sparkWaitFinalAck(0x11, 2000);

    vTaskDelay(pdMS_TO_TICKS(100));

    // 3. Get firmware version
    sparkBLE_writeBlocks(SparkMessage::getFirmwareVersion());
    sparkWaitFinalAck(0x2F, 2000);

    vTaskDelay(pdMS_TO_TICKS(100));

    // 4. Get HW checksums (cache validation)
    sparkBLE_writeBlocks(SparkMessage::getHWChecksums());
    sparkWaitFinalAck(0x2A, 2000);

    vTaskDelay(pdMS_TO_TICKS(100));

    // 5. Get amp status (triggers amp to report readiness; Ignitron sends this before preset)
    sparkBLE_writeBlocks(SparkMessage::getAmpStatus());
    vTaskDelay(pdMS_TO_TICKS(200));

    // 6. Get current preset number
    sparkBLE_writeBlocks(SparkMessage::getCurrentPresetNum());
    vTaskDelay(pdMS_TO_TICKS(100));

    // 7. Get current preset (response is cmd=0x03, not cmd=0x04 — don't wait for final ACK)
    sparkBLE_writeBlocks(SparkMessage::getCurrentPreset());
    vTaskDelay(pdMS_TO_TICKS(500));  // allow onPreset callback to fire

    DPRINT("Spark: connection sequence done — amp: %s  serial: %s  fw: %s\n",
           sparkAmpName, sparkSerial, sparkFirmware);

    sparkConnected = true;
}

// ── FreeRTOS task ──────────────────────────────────────────────────────────────

static void sparkTask(void* param) {
    static bool connectionSequenceDone = false;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));

        if (!sparkBLEConnected) {
            connectionSequenceDone = false;
            sparkConnected = false;

            DPRINT("Spark: scanning for amp...\n");
            if (sparkBLE_scanAndConnect(15)) {
                // Give the amp a moment to settle after connection
                vTaskDelay(pdMS_TO_TICKS(1500));
                sparkRunConnectionSequence();
                connectionSequenceDone = true;
            } else {
                // Retry after a pause
                vTaskDelay(pdMS_TO_TICKS(10000));
            }
            continue;
        }

        // Drain command queue
        SparkCmd cmd;
        while (xQueueReceive(_sparkQueue, &cmd, 0) == pdTRUE) {
            sparkExecuteCmd(cmd);
        }
    }
}

// ── StreamReader callback setup ────────────────────────────────────────────────

static void sparkSetupReaderCallbacks() {
    sparkReader.onPreset = [](const SparkPreset& p) {
        sparkCurrentPreset = p;
        sparkHasPreset     = true;
        strlcpy(sparkCurrentPresetName, p.name, sizeof(sparkCurrentPresetName));

        if (_sparkExpectedSlot <= 3) {
            // Response to an explicit HW switch: trust our own slot counter, not
            // p.presetNumber (real amp always reports 0 regardless of active slot).
            strlcpy(sparkHWPresetNames[_sparkExpectedSlot], p.name, 33);
            sparkHWPresetNameKnown[_sparkExpectedSlot] = true;
            sparkCurrentHWPreset = _sparkExpectedSlot;
            _sparkExpectedSlot   = 0xFF;
        } else if (p.presetNumber == SPARK_PRESET_CUSTOM) {
            sparkCurrentHWPreset = 0xFF;
        }
        // Otherwise (connection-time getCurrentPreset, presetNumber unreliable):
        // leave sparkCurrentHWPreset and HW name cache untouched.
    };

    sparkReader.onPresetNum = [](uint8_t n) {
        DPRINT("Spark: current HW preset = %d\n", n);
    };

    sparkReader.onAmpName = [](const char* name) {
        strlcpy(sparkAmpName, name, sizeof(sparkAmpName));
        DPRINT("Spark: amp name = %s\n", sparkAmpName);
        _sparkAckSubCmd = 0x11;
        xSemaphoreGive(_sparkFinalAckSem);
    };

    sparkReader.onSerialNumber = [](const char* s) {
        strlcpy(sparkSerial, s, sizeof(sparkSerial));
        DPRINT("Spark: serial = %s\n", sparkSerial);
        _sparkAckSubCmd = 0x23;
        xSemaphoreGive(_sparkFinalAckSem);
    };

    sparkReader.onFirmwareVersion = [](const char* v) {
        strlcpy(sparkFirmware, v, sizeof(sparkFirmware));
        DPRINT("Spark: firmware = %s\n", sparkFirmware);
        _sparkAckSubCmd = 0x2F;
        xSemaphoreGive(_sparkFinalAckSem);
    };

    sparkReader.onFinalAck = [](uint8_t subCmd) {
        DPRINT("[Spark] finalAck subCmd=0x%02X\n", subCmd);
        _sparkAckSubCmd = subCmd;
        xSemaphoreGive(_sparkFinalAckSem);
    };

    sparkReader.onInterimAck = [](uint8_t subCmd) {
        DPRINT("[Spark] interimAck subCmd=0x%02X\n", subCmd);
        _sparkAckSubCmd = subCmd;
        xSemaphoreGive(_sparkInterimAckSem);
    };

    sparkReader.onEffectToggle = [](const char* name, bool isOn) {
        DPRINT("Spark: effect %s -> %s\n", name, isOn ? "ON" : "OFF");
        for (int i = 0; i < 7; i++) {
            if (strcmp(sparkCurrentPreset.pedals[i].name, name) == 0) {
                sparkCurrentPreset.pedals[i].isOn = isOn;
                break;
            }
        }
    };
}

// ── Public API ─────────────────────────────────────────────────────────────────

void spark_task_start() {
    _sparkQueue         = xQueueCreate(16, sizeof(SparkCmd));
    _sparkFinalAckSem   = xSemaphoreCreateBinary();
    _sparkInterimAckSem = xSemaphoreCreateBinary();

    sparkPresets_init();
    sparkUpdateCustomSlotNames();
    sparkSetupReaderCallbacks();

    xTaskCreatePinnedToCore(
        sparkTask,     // task function
        "sparkTask",   // name
        8192,          // stack (preset JSON parsing needs headroom)
        nullptr,       // parameter
        1,             // priority
        nullptr,       // handle (not needed)
        0              // core 0 (same as web server, away from MIDI on core 1)
    );

    DPRINT("Spark task started on core 0\n");
}

// Push a command from any task/core (safe via FreeRTOS queue)
static bool sparkPushCmd(const SparkCmd& cmd) {
    if (!_sparkQueue) return false;
    return xQueueSend(_sparkQueue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

void spark_queue_preset(uint8_t bank, uint8_t slot) {
    DPRINT("[Spark] queue preset bank=%d slot=%d\n", bank, slot);
    SparkCmd cmd = {};
    cmd.type = SPARK_CMD_PRESET;
    cmd.bank = bank;
    cmd.slot = slot;
    bool ok = sparkPushCmd(cmd);
    DPRINT("[Spark] queue preset queued=%d\n", (int)ok);
}

void spark_queue_hw_preset(uint8_t hwPreset) {
    DPRINT("[Spark] queue HW preset %d  connected=%d\n", hwPreset, (int)sparkConnected);
    SparkCmd cmd = {};
    cmd.type = SPARK_CMD_HW_PRESET;
    cmd.slot = hwPreset;
    bool ok = sparkPushCmd(cmd);
    DPRINT("[Spark] queue HW preset queued=%d\n", (int)ok);
}

void spark_queue_toggle_effect(const char* name, bool isOn) {
    SparkCmd cmd = {};
    cmd.type       = SPARK_CMD_EFFECT_TOGGLE;
    cmd.effectIsOn = isOn;
    strlcpy(cmd.effectName, name, sizeof(cmd.effectName));
    sparkPushCmd(cmd);
}

void spark_queue_effect_param(const char* name, uint8_t paramIdx, float value) {
    SparkCmd cmd = {};
    cmd.type      = SPARK_CMD_EFFECT_PARAM;
    cmd.paramIdx  = paramIdx;
    cmd.paramValue= value;
    strlcpy(cmd.effectName, name, sizeof(cmd.effectName));
    sparkPushCmd(cmd);
}

// Returns current known on/off state for a named effect
bool spark_get_effect_state(const char* name) {
    if (!sparkHasPreset) return false;
    for (int i = 0; i < 7; i++) {
        if (strcmp(sparkCurrentPreset.pedals[i].name, name) == 0)
            return sparkCurrentPreset.pedals[i].isOn;
    }
    return false;
}

// ── Web-layer helpers (called from WebConfigAsync.h) ──────────────────────────

// Serialise current Spark status to JSON string
String spark_status_json() {
    JsonDocument doc;
    doc["connected"] = sparkConnected;
    doc["scanning"]  = sparkBLEScanning;
    doc["ampName"]   = sparkAmpName;
    doc["serial"]    = sparkSerial;
    doc["firmware"]  = sparkFirmware;
    doc["bank"]      = sparkCurrentBank;
    doc["slot"]      = sparkCurrentSlot;
    if (sparkHasPreset) doc["preset"] = sparkCurrentPreset.name;

    doc["dbgWriteCount"]      = _sparkWriteCount;
    doc["dbgWriteOk"]         = _sparkWriteOkCount;
    doc["dbgFFC1WriteNR"]     = _sparkFFC1WriteNR;
    doc["dbgFFC2CanNotify"]   = _sparkFFC2CanNotify;
    doc["dbgFFC2CanIndicate"] = _sparkFFC2CanIndicate;
    doc["dbgFFC2SubOk"]       = _sparkFFC2SubOk;

    // Debug: raw decoded bytes for last ampName / serial dispatches
    auto hexDump = [](const uint8_t* buf, uint8_t n) -> String {
        String h; h.reserve(n * 3);
        for (uint8_t i = 0; i < n; i++) {
            if (i) h += ' ';
            char b[3]; snprintf(b, sizeof(b), "%02X", buf[i]);
            h += b;
        }
        return h;
    };
    if (_dbgAmpNameRawLen > 0) doc["dbgAmpNameRaw"] = hexDump(_dbgAmpNameRaw, (uint8_t)std::min((int)_dbgAmpNameRawLen, 24));
    if (_dbgSerialRawLen  > 0) doc["dbgSerialRaw"]  = hexDump(_dbgSerialRaw,  (uint8_t)std::min((int)_dbgSerialRawLen,  24));

    // Debug: last BLE notification bytes (hex)
    if (_lastNotifyLen > 0) {
        String hex;
        hex.reserve(_lastNotifyLen * 3);
        for (size_t i = 0; i < _lastNotifyLen; i++) {
            if (i) hex += ' ';
            char buf[3];
            snprintf(buf, sizeof(buf), "%02X", _lastNotifyBuf[i]);
            hex += buf;
        }
        doc["dbgLastNotify"] = hex;
        doc["dbgNotifyLen"]  = _lastNotifyLen;
    }

    String out;
    serializeJson(doc, out);
    return out;
}

// Serialise bank map to JSON string
String spark_banks_json() {
    JsonDocument doc;
    JsonArray banks = doc.to<JsonArray>();
    for (int b = 0; b < SPARK_BANKS; b++) {
        JsonArray slots = banks.add<JsonArray>();
        for (int s = 0; s < SPARK_SLOTS; s++) {
            if (sparkBankMap[b][s][0]) slots.add(sparkBankMap[b][s]);
            else                       slots.add(nullptr);
        }
    }
    String out;
    serializeJson(doc, out);
    return out;
}

#endif // SPARK_AMP && BLE
