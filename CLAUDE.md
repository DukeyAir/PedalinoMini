# PedalinoMini — Claude Context

## Hardware

- **Device:** LilyGo T-Display-S3 (ESP32-S3, onboard ST7789 TFT 320×170)
- **Flash env:** ALWAYS use `-e lilygo-t-display-s3`. The `bpi-leaf-s3` env also exists but has NO display — flashing it here makes the device appear dead.
- **Flash command:** `~/.platformio/penv/Scripts/pio run -e lilygo-t-display-s3 --target upload`

---

## Goal

Full Spark amp preset switching support:
- Connect to Spark amp via BLE
- Switch between 4 hardware presets on the amp
- Send unlimited custom presets organized in banks of 4
- Toggle individual effects on/off, change parameters
- Upload and manage preset JSON files via web UI
- Assign presets/effects to physical pedal buttons

Not implementing: AMP emulation mode, keyboard mode, looper control.

---

## Execution Architecture

```
Core 1: loop()   — controller_run() (button polling, action dispatch) + MTC
Core 1: loop1()  — MIDI input reading every 1ms
Core 0: loop0()  — http_run() + OTA + display_update() every 10ms
Core 0: sparkTask (FreeRTOS) — BLE writes, ACK waits, connection sequence
```

Action dispatch: Physical input → AceButton → `controller_event_handler_button()` → `fire_action()` → `spark_queue_*()`→ FreeRTOS queue → `sparkTask` → BLE write

---

## BLE Protocol

**Source of truth:** [stangreg/Ignitron](https://github.com/stangreg/Ignitron)

### Service & Characteristics

| Item | UUID |
|------|------|
| Service | `FFC0` |
| Write (commands TO amp) | `FFC1` |
| Notify (responses FROM amp) | `FFC2` |

### Command Table

| Command | cmd | subCmd | Payload |
|---------|-----|--------|---------|
| Switch HW Preset | `0x01` | `0x38` | `{0x00, presetNum}` |
| Send Full Preset | `0x01` | `0x01` | preset data (presetNumber=`0x7F` for custom) |
| Change Effect Parameter | `0x01` | `0x04` | prefixedStr(name) + paramIdx + float |
| Change Effect Model | `0x01` | `0x06` | prefixedStr(oldName) + prefixedStr(newName) |
| Toggle Effect On/Off | `0x01` | `0x15` | prefixedStr(name) + bool + `0x00` |
| Get Current Preset Num | `0x02` | `0x10` | `{}` |
| Get Current Preset | `0x02` | `0x01` | `{0x01, 0x00}` |
| Get Amp Name | `0x02` | `0x11` | `{}` |
| Get Serial Number | `0x02` | `0x23` | `{}` |
| Get HW Checksums | `0x02` | `0x2A` | `{0x94, 0x00, 0x01, 0x02, 0x03}` |
| Get Firmware Version | `0x02` | `0x2F` | `{}` |
| Get Amp Status | `0x02` | `0x71` | `{}` |

### String Encoding — Two Formats, NOT Interchangeable

| Format | Bytes | Used in |
|--------|-------|---------|
| `appendStr` | `[0xA0+len][bytes]` or `[0xD9][len][bytes]` for >31 chars | Preset data: name, version, description, icon, pedal names |
| `appendPrefixedStr` | `[len][0xA0+len][bytes]` | Effect commands: toggleEffect, changeEffectParam, changeEffectModel |

### Encoding Pipeline

```
Raw payload bytes
  → If multi-chunk: add sub-header [numChunks][chunkIdx][chunkLen] per chunk (max 128 bytes each)
  → 7-bit encode: groups of 7 bytes; prefix byte holds MSBs (bit j = MSB of byte j, LSB-first)
  → SysEx frame: F0 01 [msgNum] [checksum] [cmd] [subCmd] [encoded] F7
  → BLE block:   01 FE 00 00 53 FE [frameLen] 00×9 [frame]
```

Checksum = XOR of all 7-bit encoded bytes in the frame.

### Preset Data Format (buildPresetData)

```
[0x00][presetNumber]           direction marker + preset num (0x7F = custom)
[0xD9][uuidLen][uuid bytes]    UUID always as LongString
[name string]                  appendStr
[version string]               appendStr
[description string]           appendStr
[icon string]                  appendStr
[0xCA][b3][b2][b1][b0]         BPM as IEEE 754 float big-endian
[0x97]                         0x90+7 = 7-pedal array
For each of 7 pedals:
  [pedal name]                 appendStr
  [0xC3 or 0xC2]               isOn bool
  [0x90+N]                     N parameters
  For each parameter:
    [paramIdx byte]
    [0xCA][b3][b2][b1][b0]     float value
[checksum byte]                sum of all bytes after first 2, mod 256
```

### parsePreset (receiving from amp)

The amp sends `d[0] = 0x01` (from-amp direction byte). Our `parsePreset` skips `d[0]` without checking its value — do NOT add `d[0] != 0x00` back. `d[1]` is the preset number.

### ACKs

PedalinoMini is always in **APP/controller mode** — it **never sends ACKs to the amp**. The amp sends `cmd=0x04` ACKs to us; we only receive them.

### sendPreset ACK Protocol (confirmed by log)

For `sendPreset` (cmd=0x01, subCmd=0x01) with N BLE blocks:
- Blocks 0…N-2: amp sends **interimAck** (`cmd=0x05, subCmd=0x01`) after each
- Block N-1 (last): amp sends **finalAck** (`cmd=0x04, subCmd=0x01`)
- After finalAck: send `switchHWPreset(0x7F)` to activate the custom edit slot; amp responds with finalAck(`cmd=0x04, subCmd=0x38`)

**Critical:** drain `_sparkInterimAckSem` with `xSemaphoreTake(_sparkInterimAckSem, 0)` before starting the send. The amp floods interimAcks during `getCurrentPreset` multi-chunk responses, leaving a stale signal that would be consumed as a false ack for block 0, shifting all acks off by one and causing the last block to never be acked.

### Connection Sequence

1. Scan for BLE service `FFC0`
2. Connect + subscribe to `FFC2` notifications
3. `getSerialNumber()` — first message, triggers amp name response
4. `getAmpName()` — identifies model
5. `getFirmwareVersion()`
6. `getHWChecksums()` — cache validation
7. `getAmpStatus()`
8. `getCurrentPresetNum()`
9. `getCurrentPreset()` — load active preset; `onPreset` fires asynchronously

**BLE MIDI must always run when BLE is enabled.** Skipping it to free NimBLE causes a boot loop after WiFi join on ESP32-S3 — the BT controller stays uninitialized and WiFi COEX crashes. Boot sequence in `PedalinoMini.cpp`:

```cpp
if (bleEnabled) ble_midi_start_service();         // always — initializes NimBLE + BT controller
if (sparkEnabled) spark_task_start();              // after BLE MIDI, reuses NimBLE stack
```

---

## Spark Action Types

| Constant | Value | midiCode | midiChannel | oscAddress |
|---|---|---|---|---|
| `PED_ACTION_SPARK_PRESET` | 60 | slot (0-3) | — | — |
| `PED_ACTION_SPARK_EFFECT_TOGGLE` | 63 | — | — | effect name |
| `PED_ACTION_SPARK_HW_PRESET` | 64 | HW preset (0-3) | — | — |
| `PED_ACTION_SPARK_EFFECT_PARAM` | 65 | — | param index (0-9) | effect name |

Bank+/Bank- actions were removed — PedalinoMini's own bank navigation is used instead.

Web UI field mapping for action editor:

| Action | Code field | Channel div | OSC Address |
|---|---|---|---|
| HW Preset | enabled "HW Preset (0-3)" | hidden | disabled |
| Spark Preset | enabled "Slot (0-3)" | hidden | disabled |
| Effect Toggle | disabled | hidden | enabled "Effect Name" |
| Effect Param | disabled | shown "Param Index (0-9)" | enabled "Effect Name" |

---

## Display

### Top Bar Layout (non-scrolling mode, left → right)
- WiFi signal arc — top-left corner, color by signal strength
- BLE MIDI ♦ symbol — x≈36, blue when connected
- Profile A/B/C badge — x=52+24×profile, 22px wide
- **Spark "S" badge** — x=124, 22px wide; green=connected, dark grey=not connected (`extern bool sparkConnected` in DisplayTFT.h)

### `@` Bank Display Mode

Triggered when `banknames[currentBank][0] == '@'`. Shows 8 preset slots in two bar rows (24px each) with the current preset name centered between them.

**Layout:** `[top bar][sep line][preset name area][sep line][bottom bar]`  
Vertical dividers between the 4 slots in each bar. FreeSans9pt7b for slot names, FreeSans18pt7b for center name.

**Row assignment (hardware-agnostic — `TFT_eSPI::setRotation` handles the coordinate transform, no manual flip compensation):**
- Top row → custom presets (`PED_ACTION_SPARK_PRESET`), sorted **descending** so slot 0 is rightmost
- Bottom row → HW presets (`PED_ACTION_SPARK_HW_PRESET`), sorted **ascending** so slot 0 is leftmost

**Physical switch layout (user's device with `flipScreen=true`):**
- Bottom row L→R: footswitches 1–4 (HW presets 0–3)
- Top row R→L: footswitches 5–8 (custom preset slots 0–3, so slot 0 = F/S 5 = top-right)

**HW slot placeholders:** Shows `"HW1"`–`"HW4"` until the name is learned by switching to that preset. Name is cached in `sparkHWPresetNames[slot]` / `sparkHWPresetNameKnown[slot]`.

### HW Preset Name Fetch (after switch ACK)

After `cmd=0x04 subCmd=0x38` ACK:
1. Spark task waits up to 300ms in 10ms ticks — **aborts immediately if new command arrives in queue**
2. If no new command: sends `getCurrentPreset()`
3. Polls `_sparkExpectedSlot != 0xFF` in 50ms ticks up to 1s — **also aborts if queue non-empty**
4. `onPreset` fires asynchronously, caches name at `sparkHWPresetNames[_sparkExpectedSlot]`, clears `_sparkExpectedSlot`

Do **not** use `sparkWaitFinalAck` for `getCurrentPreset` response — amp replies with `cmd=0x03`, not `cmd=0x04`.

---

## Preset JSON Format

```json
{
  "PresetNumber": 127,
  "UUID": "DEFBB271-B3EE-4C7E-A623-2E5CA53B6DDA",
  "Name": "Studio Session",
  "Version": "0.7",
  "Description": "Description",
  "Icon": "icon.png",
  "BPM": 120.0,
  "Pedals": [
    { "Name": "bias.noisegate",  "IsOn": false, "Parameters": [0.5, 0.35] },
    { "Name": "BBEOpticalComp",  "IsOn": true,  "Parameters": [0.76, 0.26, 0.0] },
    { "Name": "DistortionTS9",   "IsOn": false, "Parameters": [0.14, 0.41, 0.69] },
    { "Name": "Acoustic",        "IsOn": true,  "Parameters": [0.64, 0.39, 0.38, 0.60, 0.52] },
    { "Name": "ChorusAnalog",    "IsOn": true,  "Parameters": [0.84, 0.23, 0.94, 0.35] },
    { "Name": "DelayMono",       "IsOn": false, "Parameters": [0.22, 0.21, 0.49, 0.60, 1.0] },
    { "Name": "bias.reverb",     "IsOn": true,  "Parameters": [0.72, 0.33, 0.28, 0.36, 0.34, 0.49, 0.40] }
  ],
  "Checksum": "23"
}
```

Always exactly 7 pedals in order: Noise Gate, Compressor, Drive, Amp, Modulation, Delay, Reverb.
File names max 31 chars including `.json`.

### Effect Chain Reference

| Slot | Type | Examples |
|------|------|---------|
| 0 | Noise Gate | `bias.noisegate` |
| 1 | Compressor | `LA2AComp`, `BlueComp`, `BBEOpticalComp` |
| 2 | Drive | `Booster`, `KlonCentaurSilver`, `DistortionTS9`, `ProCoRat` |
| 3 | Amp | `Twin`, `Plexi`, `Rectifier`, `EVH`, `Acoustic`, `RolandJC120` |
| 4 | Modulation | `Tremolo`, `ChorusAnalog`, `Flanger`, `Phaser` |
| 5 | Delay | `DelayMono`, `VintageDelay`, `DelayReverse` |
| 6 | Reverb | `bias.reverb` |

### Amp-Specific BLE Settings

| Model | Write size | Inter-block delay |
|-------|-----------|-------|
| Spark 40 / GO / NEO | 173 bytes | none |
| Spark Mini / 2 | 100 bytes | 80ms between writes |

**Connection params:** `setConnectionParams(6, 10, 0, 600)` — requests 7.5–12.5ms interval. Takes effect on next BLE connect. Do **not** add an unconditional post-write delay for Spark 40 — the ACK handshake provides natural pacing between blocks.

---

## Web UI Notes

- Bootstrap 5.3.2 dark mode: `data-bs-theme='dark'` on `<html>`, `navbar-dark bg-dark` on nav
- ESPAsyncWebServer does **not** auto-collect `application/json` bodies — always register an `onBody` callback as the 5th arg of `server.on()` for JSON POST endpoints
- Spark nav item is hidden when `sparkEnabled == false`
- Bank editor JS: only re-renders bank table on page load / after upload / after delete — **not** on the 5s status poll (to avoid wiping unsaved selections)

---

## Debug DPRINTs (remove when stable)

- `SparkControl.h`: `[Spark] queue preset`, `[Spark] preset bank/slot/file`, `[Spark] preset loaded`, `[Spark] finalAck subCmd=`, `[Spark] interimAck subCmd=`, `[Spark] preset send complete`, `[Spark] no ACK for custom slot activation`
- `SparkStreamReader.h`: `[Spark] preset data arrived len=…`, `[Spark] parsePreset=…`

---

## Known Incidents

### Boot loop after WiFi join
BLE MIDI was conditionally skipped when Spark was enabled. BT controller stayed uninitialized. WiFi COEX requires it initialized — crash happens after WiFi join, not at startup.
**Fix:** BLE MIDI always runs unconditionally.

### Device appeared dead (no display)
Wrong env `-e bpi-leaf-s3` flashed to LilyGo T-Display-S3.
**Fix:** Erase with `bpi-leaf-s3 --target erase`, reflash with `lilygo-t-display-s3`.

### HW Preset 3 always ACK timeout
`switchHWPreset` sent 1-byte payload `{presetNum}`. Amp requires `{0x00, presetNum}`.
Preset 0 worked by coincidence (single `0x00`).

### parsePreset always failed on amp responses
Checked `d[0] == 0x00` but amp sends `d[0] = 0x01` (from-amp direction byte).
Diagnosed via hex dump: `[01 00 D9 24 ...]` — `0xD9 0x24` is UUID LongString marker + len=36.

### Bank editor POST returned 400
ESPAsyncWebServer ignores JSON bodies without explicit `onBody` callback in `server.on()`.

### Bank editor selections wiped every 5s
`setInterval` called `renderBanks()` every 5s, discarding unsaved DOM state.

### `@` display rows inverted / slots in wrong order
Manual `flipScreen` coordinate compensation was added, but `TFT_eSPI::setRotation()` already transforms the coordinate system — `(0,0)` is always the user-visible top-left regardless of rotation. Adding compensation double-inverts both axes.
**Fix:** No flipScreen logic in the `@` block. Always: custSlots→top row descending, hwSlots→bottom row ascending.

### Custom preset send consumed stale interimAck, sent incomplete data
`_sparkInterimAckSem` accumulates signals from the amp's multi-chunk `getCurrentPreset` response. First call to `waitInterimAck` in the send loop consumed the stale signal (false ack for block 0), shifting all subsequent acks off by one — last block was never acked, amp received incomplete preset and didn't switch.
**Fix:** `xSemaphoreTake(_sparkInterimAckSem, 0)` drain before starting every `sparkSendPreset`.

### Custom preset send timed out waiting for finalAck — never came
The stale semaphore caused all blocks to appear acked early, then `sparkWaitFinalAck(0x01)` waited for a finalAck that had already been consumed as the shifted "last interimAck".
**Fix:** With the drain in place, the correct protocol is visible: interimAck per block, finalAck for the last block, then `switchHWPreset(0x7F)` to activate.

### HW preset name fetch blocked queue for up to 1.3s
300ms `vTaskDelay` + 1s polling loop ran synchronously in `sparkTask`, preventing any queued command from executing.
**Fix:** Both waits broken into 10ms / 50ms ticks each checking `uxQueueMessagesWaiting(_sparkQueue)`, aborting immediately if a new command arrives.
