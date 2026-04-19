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

#include "SparkPresets.h"
#include "SparkControl.h"

// ── Spark web handler functions ────────────────────────────────────────────────
// Included at file scope in WebConfigAsync.h, before http_setup().

void http_handle_spark(AsyncWebServerRequest* req) {
  AsyncWebServerResponse *response = req->beginChunkedResponse("text/html", get_spark_page_chunked);
  response->addHeader("Connection", "close");
  req->send(response);
}

void http_handle_spark_status(AsyncWebServerRequest* req) {
  req->send(200, "application/json", spark_status_json());
}

void http_handle_spark_presets(AsyncWebServerRequest* req) {
  auto files = sparkPresets_list();
  JsonDocument doc;
  JsonArray arr = doc.to<JsonArray>();
  for (const auto& f : files) arr.add(f.c_str());
  String out;
  serializeJson(doc, out);
  req->send(200, "application/json", out);
}

void http_handle_spark_banks(AsyncWebServerRequest* req) {
  req->send(200, "application/json", spark_banks_json());
}

static String _banksBody;

void http_body_spark_banks(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
  if (index == 0) _banksBody = "";
  _banksBody += String(reinterpret_cast<char*>(data)).substring(0, len);
}

void http_handle_post_spark_banks(AsyncWebServerRequest* req) {
  JsonDocument doc;
  if (deserializeJson(doc, _banksBody) != DeserializationError::Ok) { req->send(400); return; }
  JsonArray banks = doc.as<JsonArray>();
  for (int b = 0; b < SPARK_BANKS && b < (int)banks.size(); b++) {
    JsonArray slots = banks[b].as<JsonArray>();
    for (int s = 0; s < SPARK_SLOTS && s < (int)slots.size(); s++) {
      const char* name = slots[s];
      if (name) strlcpy(sparkBankMap[b][s], name, 32);
      else      sparkBankMap[b][s][0] = '\0';
    }
  }
  sparkPresets_saveBanks();
  req->send(200);
}

void http_handle_delete_spark_preset(AsyncWebServerRequest* req) {
  if (!req->hasParam("name")) { req->send(400); return; }
  sparkPresets_delete(req->getParam("name")->value().c_str());
  req->send(200);
}

void http_handle_post_spark_connect(AsyncWebServerRequest* req) {
  SparkCmd cmd = {};
  cmd.type = SPARK_CMD_RECONNECT;
  sparkPushCmd(cmd);
  req->send(200);
}

static String            _uploadFilename;
static std::vector<uint8_t> _uploadBuf;
static bool              _uploadOk = false;

void http_handle_spark_upload(AsyncWebServerRequest* req,
                               const String& filename, size_t index,
                               uint8_t* data, size_t len, bool final) {
  if (index == 0) {
    _uploadFilename = filename;
    int slash = _uploadFilename.lastIndexOf('/');
    if (slash >= 0) _uploadFilename = _uploadFilename.substring(slash + 1);
    if (!_uploadFilename.endsWith(".json")) _uploadFilename += ".json";
    if (_uploadFilename.length() > 36)
      _uploadFilename = _uploadFilename.substring(0, 31) + ".json";
    _uploadBuf.clear();
    _uploadBuf.reserve(4096);
    _uploadOk = false;
  }
  _uploadBuf.insert(_uploadBuf.end(), data, data + len);
  if (final) {
    _uploadOk = sparkPresets_saveRaw(_uploadFilename.c_str(),
                                     _uploadBuf.data(), _uploadBuf.size());
    _uploadBuf.clear();
    // Response is sent by the onRequest lambda below
  }
}

#endif // SPARK_AMP && BLE
