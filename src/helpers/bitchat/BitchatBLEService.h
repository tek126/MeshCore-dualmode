#pragma once

#include "BitchatProtocol.h"

#ifdef ESP32
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEAdvertising.h>
#include <freertos/FreeRTOS.h>

/**
 * Callback interface for Bitchat BLE events
 */
class BitchatBLECallback {
public:
    virtual ~BitchatBLECallback() {}

    /**
     * Called when a Bitchat message is received via BLE
     * @param msg The received message
     */
    virtual void onBitchatMessageReceived(const BitchatMessage& msg) = 0;

    /**
     * Called when a Bitchat BLE client connects
     */
    virtual void onBitchatClientConnect() {}

    /**
     * Called when a Bitchat BLE client disconnects
     */
    virtual void onBitchatClientDisconnect() {}
};

/**
 * Bitchat BLE Service
 * Provides a GATT service for Bitchat protocol communication
 * Can be attached to an existing BLE server
 *
 * Concurrency model: onWrite()/onStatus() run on the Bluedroid BLE task while
 * loop() runs on the Arduino task. Everything both sides touch (the write
 * reassembly buffer, its offset, the pending flags and the client counters) is
 * guarded by _mux; loop() snapshots the buffer under the lock and parses the
 * snapshot outside it, so the lock is only ever held for memcpy-scale work.
 *
 * Outgoing notifies go through a paced queue drained from loop() (one notify
 * per BLE_NOTIFY_SPACING_MS) instead of being fired back-to-back — bursts of
 * notify() calls are a known way to wedge the Bluedroid stack.
 */
class BitchatBLEService : public BLECharacteristicCallbacks {
public:
    BitchatBLEService();

    /**
     * Attach to an existing BLE server
     * Must be called after BLEDevice::init() and server creation
     * @param server The BLE server to attach to
     * @param callback Callback for Bitchat events
     * @return true if service was created successfully
     */
    bool attachToServer(BLEServer* server, BitchatBLECallback* callback);

    /**
     * Start the Bitchat service (shared BLE mode)
     * Sets Bitchat UUID in scan response for coexistence with MeshCore UUID
     * Call after attachToServer() and before advertising
     */
    void start();

    /**
     * Start the Bitchat service only, without touching advertising
     * Use this for standalone mode where startAdvertising() will be called separately
     */
    void startServiceOnly();

    /**
     * Set the device name for BLE advertising
     * Call before startAdvertising()
     */
    void setDeviceName(const char* name);

    /**
     * Start BLE advertising with Bitchat UUID in main advertisement
     * This is required for Bitchat app discovery (it filters on main adv UUID)
     */
    void startAdvertising();

    /**
     * Check if the service is active
     */
    bool isActive() const { return _serviceActive; }

    /**
     * Check if a Bitchat client is connected
     * Note: This tracks clients that have interacted with the Bitchat characteristic
     */
    bool hasConnectedClient() const { return _bitchatClientCount > 0; }

    /**
     * Queue a message for broadcast to connected Bitchat clients.
     * The actual notify happens from loop(), paced to avoid wedging Bluedroid.
     * @param msg Message to send
     * @return true if the message was queued (false: serialize failed or queue full)
     */
    bool broadcastMessage(const BitchatMessage& msg);

    /**
     * Process loop - call from main loop
     * Handles deferred message processing and drains the paced notify queue
     */
    void loop();

    /**
     * Mark client as disconnected (call from server disconnect callback)
     */
    void onServerDisconnect();

    /**
     * Restart advertising after connection (call from server connect callback)
     * ESP32 BLE stops advertising when a connection is made
     */
    void onServerConnect();

    /**
     * Diagnostics for `bitchat status`
     */
    uint32_t getNotifiesSent() const { return _notifiesSent; }
    uint32_t getTxQueueDrops() const { return _txQueueDrops; }

protected:
    // BLECharacteristicCallbacks
    void onWrite(BLECharacteristic* pCharacteristic) override;
    void onRead(BLECharacteristic* pCharacteristic) override;
    void onStatus(BLECharacteristic* pCharacteristic, Status s, uint32_t code) override;

private:
    BLEServer* _server;
    BLEService* _service;
    BLECharacteristic* _characteristic;
    BitchatBLECallback* _callback;
    char _deviceName[48];

    bool _serviceActive;
    uint8_t _bitchatClientCount;      // Number of Bitchat clients that have written
    uint8_t _lastKnownServerCount;    // Track server's total connection count
    volatile bool _clientSubscribed;  // True when client has subscribed to notifications

    // Guards state shared between the Bluedroid callback task and loop():
    // write buffer + offset, pending flags, client counters.
    portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

    // Flags for deferred processing (to avoid work in BLE callbacks)
    volatile bool _pendingConnect;  // Client connected, callback not yet called
    volatile bool _pendingData;     // Data received, not yet parsed

    // Write buffer for reassembling fragmented BLE writes
    // Size 1024 to handle long messages that compress to ~615 bytes (2 fragments)
    uint8_t _writeBuffer[1024];
    size_t _writeBufferOffset;
    uint32_t _lastWriteTime;
    static const uint32_t WRITE_TIMEOUT_MS = 5000;

    // Parse-side snapshot of the write buffer (loop() context only). Filled
    // under _mux, parsed with the lock released.
    uint8_t _parseBuffer[1024];
    BitchatMessage _parseMsg;   // parse scratch: keeps the big struct off the loop stack

    // Message queue for deferred processing (filled and drained on the loop
    // task only — parsing happens in loop(), never in the BLE callback)
    static const size_t MESSAGE_QUEUE_SIZE = 8;
    struct QueuedMessage {
        BitchatMessage msg;
        bool valid;
    };
    QueuedMessage _messageQueue[MESSAGE_QUEUE_SIZE];
    size_t _queueHead;
    size_t _queueTail;

    // Paced outgoing-notify queue. Entries hold the serialized wire form so a
    // queued send costs 512 bytes, not a whole BitchatMessage.
    static const size_t TX_QUEUE_SIZE = 12;
    static const uint32_t BLE_NOTIFY_SPACING_MS = 40;
    struct TxSlot {
        uint8_t data[512];
        uint16_t len;
        bool valid;
    };
    TxSlot _txQueue[TX_QUEUE_SIZE];
    size_t _txHead;
    size_t _txTail;
    uint32_t _lastNotifyTime;
    uint32_t _notifiesSent;
    uint32_t _txQueueDrops;

    void clearWriteBufferLocked();   // caller must hold _mux
    bool queueMessage(const BitchatMessage& msg);
    void processQueue();
    void processIncomingLocked();    // snapshot + parse; takes _mux internally
    void drainTxQueue();
    void checkForDisconnects();
};

#endif // ESP32
