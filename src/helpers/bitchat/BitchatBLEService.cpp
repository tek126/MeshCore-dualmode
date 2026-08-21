#include "BitchatBLEService.h"

#ifdef ESP32

#include <Arduino.h>
#include <esp_bt.h>  // For esp_ble_tx_power_set

#if BITCHAT_DEBUG
  #define BITCHAT_DEBUG_PRINTLN(F, ...) Serial.printf("BITCHAT: " F "\n", ##__VA_ARGS__)
#else
  #define BITCHAT_DEBUG_PRINTLN(...) {}
#endif

// Verbose packet hex dump macro (separate from BITCHAT_DEBUG for optional verbosity)
#if BITCHAT_DEBUG_PACKETDUMP
static void dumpPacketHex(const char* label, const uint8_t* data, size_t len) {
    Serial.printf("PACKETDUMP [%s] (%zu bytes):\n", label, len);
    for (size_t i = 0; i < len; i++) {
        Serial.printf("%02X ", data[i]);
        if ((i + 1) % 16 == 0) Serial.println();
    }
    if (len % 16 != 0) Serial.println();
}
  #define BITCHAT_PACKETDUMP(label, data, len) dumpPacketHex(label, data, len)
#else
  #define BITCHAT_PACKETDUMP(label, data, len) {}
#endif

BitchatBLEService::BitchatBLEService()
    : _server(nullptr)
    , _service(nullptr)
    , _characteristic(nullptr)
    , _callback(nullptr)
    , _serviceActive(false)
    , _bitchatClientCount(0)
    , _lastKnownServerCount(0)
    , _clientSubscribed(false)
    , _pendingConnect(false)
    , _pendingData(false)
    , _writeBufferOffset(0)
    , _lastWriteTime(0)
    , _queueHead(0)
    , _queueTail(0)
    , _txHead(0)
    , _txTail(0)
    , _lastNotifyTime(0)
    , _notifiesSent(0)
    , _txQueueDrops(0)
{
    memset(_writeBuffer, 0, sizeof(_writeBuffer));
    memset(_deviceName, 0, sizeof(_deviceName));
    strcpy(_deviceName, "Bitchat");
    for (size_t i = 0; i < MESSAGE_QUEUE_SIZE; i++) {
        _messageQueue[i].valid = false;
    }
    for (size_t i = 0; i < TX_QUEUE_SIZE; i++) {
        _txQueue[i].valid = false;
        _txQueue[i].len = 0;
    }
}

bool BitchatBLEService::attachToServer(BLEServer* server, BitchatBLECallback* callback) {
    if (server == nullptr || callback == nullptr) {
        BITCHAT_DEBUG_PRINTLN("attachToServer: null server or callback");
        return false;
    }

    _server = server;
    _callback = callback;

    // Create Bitchat service
    _service = _server->createService(BITCHAT_SERVICE_UUID);
    if (_service == nullptr) {
        BITCHAT_DEBUG_PRINTLN("Failed to create Bitchat service");
        return false;
    }

    // Create characteristic with READ, WRITE, WRITE_NR, NOTIFY, and INDICATE properties
    _characteristic = _service->createCharacteristic(
        BITCHAT_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_WRITE_NR |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_INDICATE
    );

    if (_characteristic == nullptr) {
        BITCHAT_DEBUG_PRINTLN("Failed to create Bitchat characteristic");
        return false;
    }

    // Bitchat uses open security (no PIN required)
    _characteristic->setAccessPermissions(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);

    // Add descriptor for notifications
    _characteristic->addDescriptor(new BLE2902());

    // Set callbacks
    _characteristic->setCallbacks(this);

    BITCHAT_DEBUG_PRINTLN("Bitchat BLE service attached to server");
    return true;
}

// MeshCore UART service UUID (for scan response)
#define MESHCORE_UART_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"

void BitchatBLEService::start() {
    if (_service == nullptr) {
        BITCHAT_DEBUG_PRINTLN("Cannot start: service not created");
        return;
    }

    // Request larger MTU to support Bitchat messages (up to 512 bytes padded)
    // This overrides the default MAX_FRAME_SIZE (172) used by MeshCore
    BLEDevice::setMTU(517);  // Max BLE MTU

    _service->start();
    _serviceActive = true;

    // NOTE: In shared BLE mode (with SerialBLEInterface), we set Bitchat UUID in scan response
    // to coexist with MeshCore UUID in main advertisement.
    // In standalone mode, caller should use startAdvertising() instead which puts
    // Bitchat UUID in main advertisement (required for Bitchat app discovery).
    if (_server != nullptr) {
        BLEAdvertising* advertising = _server->getAdvertising();

        // Set scan response data (will be used when advertising starts)
        BLEAdvertisementData scanResponse;
        scanResponse.setCompleteServices(BLEUUID(BITCHAT_SERVICE_UUID));
        advertising->setScanResponseData(scanResponse);

        BITCHAT_DEBUG_PRINTLN("Bitchat BLE service started (shared mode)");
    } else {
        BITCHAT_DEBUG_PRINTLN("Bitchat BLE service started");
    }
}

void BitchatBLEService::startServiceOnly() {
    if (_service == nullptr) {
        return;
    }

    // Request larger MTU to support Bitchat messages (up to 512 bytes padded)
    BLEDevice::setMTU(517);

    _service->start();
    _serviceActive = true;
    BITCHAT_DEBUG_PRINTLN("Bitchat BLE service started (standalone)");
}

void BitchatBLEService::setDeviceName(const char* name) {
    strncpy(_deviceName, name, sizeof(_deviceName) - 1);
    _deviceName[sizeof(_deviceName) - 1] = '\0';
}

void BitchatBLEService::startAdvertising() {
    if (_server == nullptr) {
        return;
    }

    BLEAdvertising* advertising = _server->getAdvertising();

    // Set max TX power for better range (+9 dBm is max for ESP32)
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL1, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL2, ESP_PWR_LVL_P9);

    // Set moderate advertising parameters for battery efficiency
    // 200-250ms interval balances discovery speed and power consumption
    advertising->setMinInterval(0x140);  // 200ms minimum (0x140 * 0.625ms)
    advertising->setMaxInterval(0x190);  // 250ms maximum (0x190 * 0.625ms)

    // Set Bitchat UUID in MAIN advertisement (required for Bitchat app discovery)
    // The Bitchat Android app filters on service UUID in main advertisement packet
    // BLE advertisement packet is max 31 bytes:
    //   Flags: 3 bytes, 128-bit UUID: 18 bytes = 21 bytes used
    //   Remaining for name: 10 bytes (2 header + 8 chars max)
    // Put full name in scan response instead
    BLEAdvertisementData advData;
    advData.setFlags(ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
    advData.setCompleteServices(BLEUUID(BITCHAT_SERVICE_UUID));
    // Don't set name in main adv - no room with 128-bit UUID
    advertising->setAdvertisementData(advData);

    // Put device name in scan response (ASCII only, no emoji - they break BLE)
    char safeName[20];
    size_t j = 0;
    for (size_t i = 0; _deviceName[i] != '\0' && j < sizeof(safeName) - 1; i++) {
        // Only copy ASCII printable characters (skip emoji/unicode)
        if (_deviceName[i] >= 0x20 && _deviceName[i] <= 0x7E) {
            safeName[j++] = _deviceName[i];
        }
    }
    safeName[j] = '\0';
    if (j == 0) strcpy(safeName, "Bitchat");  // Fallback if name was all emoji
    BLEAdvertisementData scanResponse;
    scanResponse.setName(safeName);
    advertising->setScanResponseData(scanResponse);

    advertising->start();
    BITCHAT_DEBUG_PRINTLN("BLE advertising started: %s", safeName);
}

void BitchatBLEService::onServerDisconnect() {
    checkForDisconnects();

    // Always restart advertising to remain discoverable
    // ESP32 BLE stops advertising when a connection is made
    if (_server != nullptr) {
        BLEAdvertising* advertising = _server->getAdvertising();
        advertising->start();
        BITCHAT_DEBUG_PRINTLN("Restarted BLE advertising (clients: %d)", _bitchatClientCount);
    }
}

void BitchatBLEService::onServerConnect() {
    // Restart advertising to allow additional clients to discover us
    // ESP32 BLE stops advertising when a connection is established
    if (_server != nullptr) {
        BLEAdvertising* advertising = _server->getAdvertising();
        advertising->start();
        BITCHAT_DEBUG_PRINTLN("Restarted BLE advertising after connect");
    }
}

void BitchatBLEService::clearWriteBufferLocked() {
    _writeBufferOffset = 0;
    _writeBuffer[0] = 0;
}

void BitchatBLEService::checkForDisconnects() {
    if (_server == nullptr) return;

    bool fireDisconnect = false;
    uint32_t currentServerCount = _server->getConnectedCount();
    portENTER_CRITICAL(&_mux);
    if (currentServerCount < _lastKnownServerCount) {
        uint8_t disconnected = _lastKnownServerCount - currentServerCount;
        _bitchatClientCount = (disconnected >= _bitchatClientCount)
            ? 0 : _bitchatClientCount - disconnected;
        _lastKnownServerCount = currentServerCount;

        if (_bitchatClientCount == 0) {
            _clientSubscribed = false;
            clearWriteBufferLocked();
            fireDisconnect = true;
        }
    }
    portEXIT_CRITICAL(&_mux);

    if (fireDisconnect && _callback != nullptr) {
        _callback->onBitchatClientDisconnect();
    }
}

bool BitchatBLEService::queueMessage(const BitchatMessage& msg) {
    size_t nextTail = (_queueTail + 1) % MESSAGE_QUEUE_SIZE;

    // Check if queue is full
    if (nextTail == _queueHead) {
        BITCHAT_DEBUG_PRINTLN("Message queue full, dropping message");
        return false;
    }

    _messageQueue[_queueTail].msg = msg;
    _messageQueue[_queueTail].valid = true;
    _queueTail = nextTail;

    return true;
}

void BitchatBLEService::processQueue() {
    while (_queueHead != _queueTail) {
        if (_messageQueue[_queueHead].valid) {
            _messageQueue[_queueHead].valid = false;

            if (_callback != nullptr) {
                _callback->onBitchatMessageReceived(_messageQueue[_queueHead].msg);
            }
        }
        _queueHead = (_queueHead + 1) % MESSAGE_QUEUE_SIZE;
    }
}

void BitchatBLEService::processIncomingLocked() {
    // Snapshot the shared write buffer under the lock, parse the snapshot with
    // the lock released (parsing copies whole BitchatMessage structs — too much
    // work to do with interrupts masked), then remove the consumed prefix from
    // the live buffer, which may have grown in the meantime.
    portENTER_CRITICAL(&_mux);
    size_t snapLen = _writeBufferOffset;
    if (snapLen > sizeof(_parseBuffer)) snapLen = sizeof(_parseBuffer);
    memcpy(_parseBuffer, _writeBuffer, snapLen);
    portEXIT_CRITICAL(&_mux);

    BITCHAT_DEBUG_PRINTLN("Processing %zu buffered bytes", snapLen);

    // Parse ALL complete messages in the snapshot (not just the first one)
    size_t consumed = 0;
    int msgCount = 0;
    while (consumed < snapLen) {
        BitchatMessage& msg = _parseMsg;
        size_t remaining = snapLen - consumed;

        if (!BitchatProtocol::parseMessage(_parseBuffer + consumed, remaining, msg)) {
            break;  // No more complete messages
        }

        size_t msgSize = BitchatProtocol::getMessageSize(msg);
        if (msgSize == 0 || msgSize > remaining) {
            break;  // Incomplete message
        }

        if (BitchatProtocol::validateMessage(msg)) {
            BITCHAT_DEBUG_PRINTLN("Received Bitchat message: type=%02X, len=%d", msg.type, msg.payloadLength);
            BITCHAT_PACKETDUMP("BLE_SERVICE_RX", msg.payload, msg.payloadLength);
            queueMessage(msg);
            msgCount++;
        } else {
            BITCHAT_DEBUG_PRINTLN("Invalid Bitchat message received");
            BITCHAT_PACKETDUMP("BLE_SERVICE_RX_INVALID", _parseBuffer + consumed, msgSize);
        }

        consumed += msgSize;
    }

    BITCHAT_DEBUG_PRINTLN("Parsed %d messages, consumed %zu bytes", msgCount, consumed);

    // Decide what survives of the snapshot's remainder. Android pads writes to
    // 256 bytes, so a leftover that doesn't start with a valid version byte is
    // padding/garbage and gets dropped along with the consumed prefix.
    size_t drop = consumed;
    if (consumed < snapLen && _parseBuffer[consumed] != BITCHAT_VERSION
        && _parseBuffer[consumed] != BITCHAT_VERSION_2) {
        BITCHAT_DEBUG_PRINTLN("Remainder starts with 0x%02X (not a known version), clearing garbage",
                              _parseBuffer[consumed]);
        drop = snapLen;
    }

    if (drop > 0) {
        portENTER_CRITICAL(&_mux);
        if (drop >= _writeBufferOffset) {
            clearWriteBufferLocked();
        } else {
            memmove(_writeBuffer, _writeBuffer + drop, _writeBufferOffset - drop);
            _writeBufferOffset -= drop;
        }
        portEXIT_CRITICAL(&_mux);
    }
}

void BitchatBLEService::drainTxQueue() {
    if (_txHead == _txTail && !_txQueue[_txHead].valid) {
        return;  // empty
    }
    uint32_t now = millis();
    if (now - _lastNotifyTime < BLE_NOTIFY_SPACING_MS) {
        return;  // pace notifies — bursts can wedge Bluedroid
    }
    if (!_serviceActive || _characteristic == nullptr) {
        // Service went away: drop what's queued rather than hold it forever
        _txQueue[_txHead].valid = false;
        _txHead = (_txHead + 1) % TX_QUEUE_SIZE;
        return;
    }

    TxSlot& slot = _txQueue[_txHead];
    if (slot.valid) {
        slot.valid = false;
        BITCHAT_PACKETDUMP("BLE_SERVICE_TX", slot.data, slot.len);
        _characteristic->setValue(slot.data, slot.len);
        _characteristic->notify(true);
        _notifiesSent++;
        _lastNotifyTime = now;
    }
    _txHead = (_txHead + 1) % TX_QUEUE_SIZE;
}

void BitchatBLEService::loop() {
    // Check for disconnections (detect when clients drop without callback)
    checkForDisconnects();

    uint32_t now = millis();

    // Handle deferred connect callback (from BLE callback)
    bool fireConnect = false;
    portENTER_CRITICAL(&_mux);
    if (_pendingConnect) {
        _pendingConnect = false;
        fireConnect = true;
    }
    portEXIT_CRITICAL(&_mux);
    if (fireConnect && _callback != nullptr) {
        _callback->onBitchatClientConnect();
    }

    // Handle deferred data processing (parsing moved out of BLE callback)
    // Wait 300ms after last write before processing to allow multi-fragment messages to arrive
    // (Increased from 100ms to handle BLE timing variations with large messages)
    bool doProcess = false;
    portENTER_CRITICAL(&_mux);
    if (_pendingData && (now - _lastWriteTime >= 300)) {
        _pendingData = false;
        doProcess = true;
    }
    // Check for write buffer timeout
    if (_writeBufferOffset > 0 && (now - _lastWriteTime > WRITE_TIMEOUT_MS)) {
        clearWriteBufferLocked();
    }
    portEXIT_CRITICAL(&_mux);

    if (doProcess) {
        processIncomingLocked();
    }

    // Process queued messages
    processQueue();

    // Drain one paced notify per loop pass
    drainTxQueue();

    // Safety: periodically restart advertising if no clients connected
    // This handles cases where ESP32 BLE stack silently stops advertising
    if (_server != nullptr && _bitchatClientCount == 0 && _server->getConnectedCount() == 0) {
        static uint32_t lastAdvRestart = 0;
        if (now - lastAdvRestart > 30000) {  // Every 30 seconds
            BLEAdvertising* advertising = _server->getAdvertising();
            advertising->start();
            lastAdvRestart = now;
        }
    }
}

void BitchatBLEService::onWrite(BLECharacteristic* pCharacteristic) {
    // MINIMAL WORK IN CALLBACK - BLE stack has limited space!
    // Just buffer data and set flags; all processing happens in loop()

    std::string value = pCharacteristic->getValue();
    if (value.empty()) {
        return;
    }

    const uint8_t* data = reinterpret_cast<const uint8_t*>(value.data());
    size_t length = value.length();

    uint32_t serverCount = (_server != nullptr) ? _server->getConnectedCount() : 0;

    portENTER_CRITICAL(&_mux);
    _lastWriteTime = millis();
    _pendingData = true;  // Flag for loop() to process

    // Detect new Bitchat clients by comparing to server's total connection count
    if (_server != nullptr && serverCount > _lastKnownServerCount) {
        uint8_t newClients = serverCount - _lastKnownServerCount;
        _bitchatClientCount += newClients;
        _lastKnownServerCount = serverCount;
        _pendingConnect = true;  // Defer callback to loop()
    }

    // Append to write buffer (increased size for large messages)
    size_t copyLen = length;
    if (_writeBufferOffset + copyLen > sizeof(_writeBuffer)) {
        clearWriteBufferLocked();
        copyLen = (length > sizeof(_writeBuffer)) ? sizeof(_writeBuffer) : length;
    }

    memcpy(&_writeBuffer[_writeBufferOffset], data, copyLen);
    _writeBufferOffset += copyLen;
    portEXIT_CRITICAL(&_mux);
}

void BitchatBLEService::onRead(BLECharacteristic* pCharacteristic) {
    // Currently unused - reads return the last written value
    // NO Serial output here - BLE callback has limited stack
}

void BitchatBLEService::onStatus(BLECharacteristic* pCharacteristic, Status s, uint32_t code) {
    // Called when CCCD is written (client subscribes/unsubscribes to notifications)
    // NO Serial output here - BLE callback has limited stack
    if (s == Status::SUCCESS_NOTIFY || s == Status::SUCCESS_INDICATE) {
        _clientSubscribed = true;
    } else if (s == Status::ERROR_NOTIFY_DISABLED) {
        _clientSubscribed = false;
    }
}

bool BitchatBLEService::broadcastMessage(const BitchatMessage& msg) {
    if (!_serviceActive || _characteristic == nullptr) {
        return false;
    }

    // Serialize into a queue slot; the notify itself happens paced from loop().
    // Slot sized for a full single-notification message: a decompressed payload can exceed
    // BITCHAT_MAX_WIRE_PAYLOAD_SIZE (e.g. a long #mesh message re-synced from history), so
    // 512 covers anything that fits one BLE notification (MTU 517); larger messages are
    // safely refused by serializeMessage() (len==0).
    size_t nextTail = (_txTail + 1) % TX_QUEUE_SIZE;
    if (nextTail == _txHead) {
        _txQueueDrops++;
        BITCHAT_DEBUG_PRINTLN("Notify queue full, dropping message type=0x%02X", msg.type);
        return false;
    }

    TxSlot& slot = _txQueue[_txTail];
    size_t len = BitchatProtocol::serializeMessage(msg, slot.data, sizeof(slot.data));
    if (len == 0) {
        return false;
    }
    slot.len = (uint16_t)len;
    slot.valid = true;
    _txTail = nextTail;

    BITCHAT_DEBUG_PRINTLN("TX queued: type=0x%02X, len=%zu", msg.type, len);
    return true;
}

#endif // ESP32
