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
#include <NimBLEDevice.h>
#include "SparkMessage.h"
#include "SparkStreamReader.h"

// ── Spark BLE UUIDs ────────────────────────────────────────────────────────────

#define SPARK_SERVICE_UUID  "FFC0"
#define SPARK_WRITE_UUID    "FFC1"   // write commands TO the amp
#define SPARK_NOTIFY_UUID   "FFC2"   // receive notifications FROM the amp

// ── State exposed to SparkControl ─────────────────────────────────────────────

bool    sparkBLEConnected  = false;
bool    sparkBLEScanning   = false;

// ── Internal state ─────────────────────────────────────────────────────────────

static NimBLEClient*               _sparkClient     = nullptr;
static NimBLERemoteCharacteristic* _sparkFFC1       = nullptr;
static NimBLEAddress               _sparkAddress;
static bool                        _sparkAddrFound  = false;
static uint32_t                    _sparkWriteCount   = 0;
static uint32_t                    _sparkWriteOkCount = 0;
static bool                        _sparkFFC1WriteNR  = false;
static bool                        _sparkFFC2CanNotify  = false;
static bool                        _sparkFFC2CanIndicate = false;
static bool                        _sparkFFC2SubOk    = false;

SparkStreamReader sparkReader;  // owned here, callbacks set by SparkControl

// ── Scan callbacks ─────────────────────────────────────────────────────────────

class SparkScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* dev) override {
        if (dev->isAdvertisingService(NimBLEUUID(SPARK_SERVICE_UUID))) {
            DPRINT("Spark amp found: %s\n", dev->getAddress().toString().c_str());
            _sparkAddress  = dev->getAddress();
            _sparkAddrFound = true;
            NimBLEDevice::getScan()->stop();
        }
    }
};

static SparkScanCallbacks _scanCb;

// ── Client callbacks ───────────────────────────────────────────────────────────

class SparkClientCallbacks : public NimBLEClientCallbacks {
    void onDisconnect(NimBLEClient* client) override {
        DPRINT("Spark amp disconnected\n");
        sparkBLEConnected = false;
        _sparkFFC1        = nullptr;
    }
    bool onConfirmPIN(uint32_t pin) override { return true; }
};

static SparkClientCallbacks _clientCb;

// ── BLE notification handler ───────────────────────────────────────────────────

static void onSparkNotify(NimBLERemoteCharacteristic* c,
                           uint8_t* data, size_t len, bool isNotify) {
    sparkReader.feed(data, len);
}

// ── Scan ───────────────────────────────────────────────────────────────────────

bool sparkBLE_scan(uint32_t timeoutSec = 15) {
    DPRINT("Scanning for Spark amp (service %s)...\n", SPARK_SERVICE_UUID);
    sparkBLEScanning = true;
    _sparkAddrFound  = false;

    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(&_scanCb);
    pScan->setActiveScan(true);
    pScan->setInterval(100);
    pScan->setWindow(99);
    pScan->start((uint32_t)timeoutSec, false);  // blocking until found or timeout

    sparkBLEScanning = false;
    return _sparkAddrFound;
}

// ── Connect ────────────────────────────────────────────────────────────────────

bool sparkBLE_connect() {
    if (!_sparkAddrFound) return false;

    DPRINT("Connecting to Spark amp %s...\n", _sparkAddress.toString().c_str());

    // Clean up stale client
    if (_sparkClient) {
        NimBLEDevice::deleteClient(_sparkClient);
        _sparkClient = nullptr;
    }

    _sparkClient = NimBLEDevice::createClient();
    _sparkClient->setClientCallbacks(&_clientCb, false);
    _sparkClient->setConnectionParams(6, 10, 0, 600);
    _sparkClient->setConnectTimeout(10);

    if (!_sparkClient->connect(_sparkAddress)) {
        DPRINT("Spark BLE connect failed\n");
        NimBLEDevice::deleteClient(_sparkClient);
        _sparkClient = nullptr;
        return false;
    }

    NimBLERemoteService* pSvc = _sparkClient->getService(SPARK_SERVICE_UUID);
    if (!pSvc) {
        DPRINT("Spark service FFC0 not found\n");
        _sparkClient->disconnect();
        return false;
    }

    _sparkFFC1 = pSvc->getCharacteristic(SPARK_WRITE_UUID);
    NimBLERemoteCharacteristic* pFFC2 = pSvc->getCharacteristic(SPARK_NOTIFY_UUID);

    if (!_sparkFFC1 || !pFFC2) {
        DPRINT("Spark characteristics not found\n");
        _sparkClient->disconnect();
        return false;
    }

    _sparkFFC2CanNotify   = pFFC2->canNotify();
    _sparkFFC2CanIndicate = pFFC2->canIndicate();
    DPRINT("FFC2 canNotify=%d canIndicate=%d\n", (int)_sparkFFC2CanNotify, (int)_sparkFFC2CanIndicate);

    // Try notifications first; fall back to indications if not supported
    bool useNotify = _sparkFFC2CanNotify;
    _sparkFFC2SubOk = pFFC2->subscribe(useNotify, onSparkNotify, true);
    if (!_sparkFFC2SubOk && _sparkFFC2CanIndicate) {
        DPRINT("Notify subscribe failed, trying indications\n");
        _sparkFFC2SubOk = pFFC2->subscribe(false, onSparkNotify, true);
    }
    if (!_sparkFFC2SubOk) {
        DPRINT("Failed to subscribe to FFC2 (notify=%d indicate=%d)\n",
               (int)_sparkFFC2CanNotify, (int)_sparkFFC2CanIndicate);
        _sparkClient->disconnect();
        return false;
    }

    _sparkFFC1WriteNR = _sparkFFC1->canWriteNoResponse();
    DPRINT("Spark amp connected. FFC1 canWrite=%d canWriteNR=%d  FFC2 subOk=%d\n",
           (int)_sparkFFC1->canWrite(), (int)_sparkFFC1WriteNR, (int)_sparkFFC2SubOk);
    sparkBLEConnected = true;
    return true;
}

// ── Disconnect ─────────────────────────────────────────────────────────────────

void sparkBLE_disconnect() {
    if (_sparkClient && _sparkClient->isConnected())
        _sparkClient->disconnect();
    sparkBLEConnected = false;
    _sparkFFC1        = nullptr;
}

// ── Write blocks to FFC1 ──────────────────────────────────────────────────────
//
// Returns false if not connected or write fails.
// delayMs: inter-block delay (0 for Spark 40/GO/NEO, 80 for Mini/Spark 2).

bool sparkBLE_writeBlocks(const SparkBlocks& blocks, uint32_t delayMs = 0) {
    if (!sparkBLEConnected || !_sparkFFC1) return false;
    for (const auto& block : blocks) {
        _sparkWriteCount++;
        // Always use write-without-response — Spark protocol requires it
        bool ok = _sparkFFC1->writeValue(block.data(), block.size(), false);
        if (ok) _sparkWriteOkCount++;
        else DPRINT("Spark BLE write failed\n");
        if (delayMs > 0) vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
    return true;
}

// ── Scan + connect helper ──────────────────────────────────────────────────────

bool sparkBLE_scanAndConnect(uint32_t scanSec = 15) {
    if (!sparkBLE_scan(scanSec)) return false;
    return sparkBLE_connect();
}

#endif // SPARK_AMP && BLE
