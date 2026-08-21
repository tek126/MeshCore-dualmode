#include "BitchatBLEService.h"

#ifdef NRF52_PLATFORM

#include <Arduino.h>

#if BITCHAT_DEBUG
  #define BITCHAT_DEBUG_PRINTLN(...) do { Serial.printf("BITCHAT_BLE: "); Serial.printf(__VA_ARGS__); Serial.println(); } while(0)
#else
  #define BITCHAT_DEBUG_PRINTLN(...) {}
#endif

// Bitchat service UUID: F47B5E2D-4A9E-4C5A-9B3F-8E1D2C3A4B5C
// Bluefruit uses little-endian byte order for UUIDs
static const uint8_t BITCHAT_SERVICE_UUID_BYTES[] = {
    0x5C, 0x4B, 0x3A, 0x2C, 0x1D, 0x8E, 0x3F, 0x9B,
    0x5A, 0x4C, 0x9E, 0x4A, 0x2D, 0x5E, 0x7B, 0xF4
};

// Bitchat characteristic UUID: A1B2C3D4-E5F6-4A5B-8C9D-0E1F2A3B4C5D
// (Same as ESP32 BITCHAT_CHARACTERISTIC_UUID in BitchatProtocol.h)
// Bluefruit uses little-endian byte order for UUIDs
static const uint8_t BITCHAT_CHARACTERISTIC_UUID_BYTES[] = {
    0x5D, 0x4C, 0x3B, 0x2A, 0x1F, 0x0E, 0x9D, 0x8C,
    0x5B, 0x4A, 0xF6, 0xE5, 0xD4, 0xC3, 0xB2, 0xA1
};

// Singleton instance for static callback access
BitchatBLEService* BitchatBLEService::_instance = nullptr;

// Static buffers to keep everything out of heap allocation
BitchatBLEService::QueuedMessage BitchatBLEService::_messageQueue[MESSAGE_QUEUE_SIZE];
uint8_t BitchatBLEService::_writeBuffer[1024];
uint8_t BitchatBLEService::_parseBuffer[1024];
BitchatBLEService::TxSlot BitchatBLEService::_txQueue[TX_QUEUE_SIZE];

BitchatBLEService::BitchatBLEService()
    : _service(BITCHAT_SERVICE_UUID_BYTES)
    , _characteristic(BITCHAT_CHARACTERISTIC_UUID_BYTES)
    , _callback(nullptr)
    , _serviceActive(false)
    , _bitchatClientCount(0)
    , _clientSubscribed(false)
    , _pendingSubscribeFlush(false)
    , _pendingConnect(false)
    , _pendingData(false)
    , _writeBufferOffset(0)
    , _lastWriteTime(0)
    , _queueHead(0)
    , _queueTail(0)
    , _txHead(0)
    , _txTail(0)
    , _lastNotifyTime(0)
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
    _instance = this;
}

bool BitchatBLEService::beginStandalone(const char* deviceName, BitchatBLECallback* callback) {
    if (callback == nullptr) {
        return false;
    }

    _callback = callback;
    strncpy(_deviceName, deviceName, sizeof(_deviceName) - 1);
    _deviceName[sizeof(_deviceName) - 1] = '\0';

    // Configure connection parameters BEFORE begin()
    // MTU 517 is standard max BLE MTU - allows writes up to 514 bytes
    // setMaxLen(512) is the characteristic buffer size
    // Parameters: mtu_max, event_len, hvn_qsize, wrcmd_qsize
    Bluefruit.configPrphConn(517, BLE_GAP_EVENT_LENGTH_DEFAULT, BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);

    // Initialize Bluefruit
    Bluefruit.begin();

    Bluefruit.setTxPower(8);  // Max power (+8 dBm) for better range

    // Set up connection callbacks
    Bluefruit.Periph.setConnectCallback(onConnect);
    Bluefruit.Periph.setDisconnectCallback(onDisconnect);

    // Bitchat uses open security (no PIN required)
    Bluefruit.Security.setMITM(false);
    Bluefruit.Security.setIOCaps(false, false, false);

    // Set device name (filter out non-ASCII characters for BLE)
    char safeName[32];
    size_t j = 0;
    for (size_t i = 0; deviceName[i] != '\0' && j < sizeof(safeName) - 1; i++) {
        if (deviceName[i] >= 0x20 && deviceName[i] <= 0x7E) {
            safeName[j++] = deviceName[i];
        }
    }
    safeName[j] = '\0';
    if (j == 0) strcpy(safeName, "Bitchat");

    Bluefruit.setName(safeName);

    // Configure the Bitchat service
    _service.begin();

    // Configure the characteristic with READ, WRITE, WRITE_NR, NOTIFY properties
    _characteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP | CHR_PROPS_NOTIFY | CHR_PROPS_INDICATE);
    _characteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);  // Open security
    // CRITICAL: Must be ≤512 or BLE service discovery breaks on NRF52
    _characteristic.setMaxLen(512);
    _characteristic.setWriteCallback(onCharacteristicWrite);
    _characteristic.setCccdWriteCallback(onCharacteristicCccdWrite);
    _characteristic.begin();

    _serviceActive = true;
    BITCHAT_DEBUG_PRINTLN("Bitchat BLE service initialized: %s", safeName);

    return true;
}

void BitchatBLEService::startAdvertising() {
    // Clear any previous advertising data
    Bluefruit.Advertising.clearData();
    Bluefruit.ScanResponse.clearData();

    // Set Bitchat UUID in MAIN advertisement (required for Bitchat app discovery)
    // The Bitchat Android app filters on service UUID in main advertisement packet
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addService(_service);

    // Put device name in scan response (not main adv - no room with 128-bit UUID)
    Bluefruit.ScanResponse.addName();

    // Configure advertising parameters
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);    // in units of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);      // seconds in fast mode
    Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising

    BITCHAT_DEBUG_PRINTLN("BLE advertising started");
}

void BitchatBLEService::onServerDisconnect() {
    bool fireDisconnect = false;
    taskENTER_CRITICAL();
    if (_bitchatClientCount > 0) {
        _bitchatClientCount--;
    }
    if (_bitchatClientCount == 0) {
        _clientSubscribed = false;
        clearWriteBufferLocked();
        fireDisconnect = true;
    }
    taskEXIT_CRITICAL();

    if (fireDisconnect && _callback != nullptr) {
        _callback->onBitchatClientDisconnect();
    }
}

void BitchatBLEService::clearWriteBufferLocked() {
    _writeBufferOffset = 0;
    _writeBuffer[0] = 0;
}

bool BitchatBLEService::queueMessage(const BitchatMessage& msg) {
    size_t nextTail = (_queueTail + 1) % MESSAGE_QUEUE_SIZE;

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
    // Snapshot under the lock, parse outside it (a BitchatMessage copy is too
    // much work to do with the scheduler masked), then drop the consumed
    // prefix from the live buffer, which may have grown in the meantime.
    taskENTER_CRITICAL();
    size_t snapLen = _writeBufferOffset;
    if (snapLen > sizeof(_parseBuffer)) snapLen = sizeof(_parseBuffer);
    memcpy(_parseBuffer, _writeBuffer, snapLen);
    taskEXIT_CRITICAL();

    BITCHAT_DEBUG_PRINTLN("Processing %u buffered bytes", (unsigned)snapLen);

    // Parse ALL complete messages in the snapshot
    size_t consumed = 0;
    while (consumed < snapLen) {
        BitchatMessage msg;
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
            queueMessage(msg);
        } else {
            BITCHAT_DEBUG_PRINTLN("Invalid Bitchat message received");
        }

        consumed += msgSize;
    }

    // Android pads writes to 256 bytes: a leftover that doesn't start with a
    // known version byte is padding/garbage — drop it with the consumed prefix.
    size_t drop = consumed;
    if (consumed < snapLen && _parseBuffer[consumed] != BITCHAT_VERSION
        && _parseBuffer[consumed] != BITCHAT_VERSION_2) {
        drop = snapLen;
    }

    if (drop > 0) {
        taskENTER_CRITICAL();
        if (drop >= _writeBufferOffset) {
            clearWriteBufferLocked();
        } else {
            memmove(_writeBuffer, _writeBuffer + drop, _writeBufferOffset - drop);
            _writeBufferOffset -= drop;
        }
        taskEXIT_CRITICAL();
    }
}

void BitchatBLEService::drainTxQueue() {
    if (_txHead == _txTail && !_txQueue[_txHead].valid) {
        return;  // empty
    }
    uint32_t now = millis();
    if (now - _lastNotifyTime < BLE_NOTIFY_SPACING_MS) {
        return;  // pace notifies; the SoftDevice HVN queue is shallow
    }
    if (!_serviceActive) {
        _txQueue[_txHead].valid = false;
        _txHead = (_txHead + 1) % TX_QUEUE_SIZE;
        return;
    }

    TxSlot& slot = _txQueue[_txHead];
    if (slot.valid) {
        slot.valid = false;
        // Always set the characteristic value so it can be read even without
        // a subscription; notify only reaches subscribed clients.
        _characteristic.write(slot.data, slot.len);
        if (_clientSubscribed) {
            uint16_t result = _characteristic.notify(slot.data, slot.len);
            if (!result) {
                BITCHAT_DEBUG_PRINTLN("notify failed (HVN queue?)");
            }
        }
        _lastNotifyTime = now;
    }
    _txHead = (_txHead + 1) % TX_QUEUE_SIZE;
}

void BitchatBLEService::loop() {
    uint32_t now = millis();

    // Handle deferred connect callback
    bool fireConnect = false;
    taskENTER_CRITICAL();
    if (_pendingConnect) {
        _pendingConnect = false;
        fireConnect = true;
    }
    taskEXIT_CRITICAL();
    if (fireConnect && _callback != nullptr) {
        _callback->onBitchatClientConnect();
    }

    // Handle deferred data processing
    // Wait 100ms after last write before processing to allow multi-chunk messages to arrive
    bool doProcess = false;
    taskENTER_CRITICAL();
    if (_pendingData && (now - _lastWriteTime >= 100)) {
        _pendingData = false;
        doProcess = true;
    }
    // Check for write buffer timeout
    if (_writeBufferOffset > 0 && (now - _lastWriteTime > WRITE_TIMEOUT_MS)) {
        clearWriteBufferLocked();
    }
    taskEXIT_CRITICAL();

    if (doProcess) {
        processIncomingLocked();
    }

    // Client just subscribed: the bridge will re-announce shortly anyway; just
    // clear the flag (queued notifies from now on reach the subscriber).
    if (_pendingSubscribeFlush) {
        _pendingSubscribeFlush = false;
    }

    // Process queued messages
    processQueue();

    // Drain one paced notify per loop pass
    drainTxQueue();
}

bool BitchatBLEService::broadcastMessage(const BitchatMessage& msg) {
    if (!_serviceActive) {
        return false;
    }

    size_t nextTail = (_txTail + 1) % TX_QUEUE_SIZE;
    if (nextTail == _txHead) {
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

    BITCHAT_DEBUG_PRINTLN("TX queued: type=0x%02X, len=%u", msg.type, (unsigned)len);
    return true;
}

// Static callbacks — these run on the Bluefruit callback task, NOT the
// Arduino loop task. Keep them minimal: set flags, buffer bytes, get out.

void BitchatBLEService::onConnect(uint16_t conn_handle) {
    if (_instance != nullptr) {
        taskENTER_CRITICAL();
        _instance->_bitchatClientCount++;
        _instance->_pendingConnect = true;
        taskEXIT_CRITICAL();
    }
}

void BitchatBLEService::onDisconnect(uint16_t conn_handle, uint8_t reason) {
    (void)reason;
    if (_instance != nullptr) {
        _instance->onServerDisconnect();
    }
}

void BitchatBLEService::onCharacteristicWrite(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    if (_instance == nullptr || len == 0) {
        return;
    }

    taskENTER_CRITICAL();
    _instance->_lastWriteTime = millis();
    _instance->_pendingData = true;

    // Append to write buffer
    size_t copyLen = len;
    if (_instance->_writeBufferOffset + copyLen > sizeof(_writeBuffer)) {
        _instance->clearWriteBufferLocked();
        copyLen = (len > sizeof(_writeBuffer)) ? sizeof(_writeBuffer) : len;
    }

    memcpy(&_writeBuffer[_instance->_writeBufferOffset], data, copyLen);
    _instance->_writeBufferOffset += copyLen;
    taskEXIT_CRITICAL();
}

void BitchatBLEService::onCharacteristicCccdWrite(uint16_t conn_handle, BLECharacteristic* chr, uint16_t cccd_value) {
    if (_instance != nullptr) {
        bool wasSubscribed = _instance->_clientSubscribed;
        _instance->_clientSubscribed = (cccd_value & BLE_GATT_HVX_NOTIFICATION) != 0;
        // No serialize/notify work here — this runs on the BLE task. Flag it;
        // loop() handles the transition (the bridge re-announces on its own).
        if (!wasSubscribed && _instance->_clientSubscribed) {
            _instance->_pendingSubscribeFlush = true;
        }
    }
}

#endif // NRF52_PLATFORM
