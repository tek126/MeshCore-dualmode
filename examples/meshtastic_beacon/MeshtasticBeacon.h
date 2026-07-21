#pragma once

// MeshtasticBeacon — emit a Meshtastic-formatted text packet from a MeshCore radio.
//
// A native MeshCore advert is invisible to Meshtastic: even on a matching
// frequency the two use different sync words and incompatible framing, so a
// Meshtastic node receives the RF energy and drops it. To make a Meshtastic
// phone actually *display* something, we have to speak Meshtastic for one
// packet: build its header, AES-CTR encrypt a text-message protobuf with the
// channel key, retune the radio to the Meshtastic modem preset + sync word,
// transmit, then restore the MeshCore PHY.
//
// This is a ONE-WAY beacon. The repeater appears as a named node + map pin
// (NodeInfo + Position) and posts a text line, but it does not route Meshtastic
// traffic or rebroadcast.
//
// The text path is verified on-air; the interop-critical math (channel hash,
// region/preset frequency, AES-CTR nonce, NodeInfo/Position encoders) is pinned
// by host unit tests in test/ (run test/run.sh). Those pure helpers live in
// MeshtasticProto.h; this file adds AES-CTR packet assembly + radio control.

#include <Arduino.h>
#include <string.h>
#include <CTR.h>     // rweather Crypto (already a MeshCore dependency)
#include <AES.h>
#include <RadioLib.h>          // RADIOLIB_IRQ_TX_DONE (hardware TxDone check)
#include "MeshtasticProto.h"   // pure tables/hash/freq/protobuf (host-testable)

namespace meshtastic {

// ---------------------------------------------------------------------------
// Modem preset. Frequency is REGION-SPECIFIC; this is the US 915 LongFast slot.
// For EU change freq_mhz (e.g. 869.525) to your region's LongFast default.
// Meshtastic LongFast = BW 250 kHz, SF 11, CR 4/5, sync word 0x2B.
// ---------------------------------------------------------------------------
struct ModemPreset {
  float    freq_mhz;
  float    bw_khz;
  uint8_t  sf;
  uint8_t  cr;        // pass the denominator: 5 == 4/5
  uint16_t preamble;  // symbols (Meshtastic uses 16)
  uint8_t  sync_word; // RadioLib 1-byte LoRa sync word; Meshtastic public = 0x2B
};

static const ModemPreset LONGFAST_US = { 906.875f, 250.0f, 11, 5, 16, 0x2B };
static const ModemPreset LONGFAST_EU = { 869.525f, 250.0f, 11, 5, 16, 0x2B };

// Tables (PRESETS/REGIONS), hashing, frequency calc and the protobuf payload
// builders live in MeshtasticProto.h (included above) so they can be host-tested.

// ---------------------------------------------------------------------------
// Build a Meshtastic broadcast packet (Data{portnum,payload} -> AES-CTR -> wire).
// Returns total wire length, or 0 if it would overflow `out`.
//
// Wire format (all multi-byte fields little-endian):
//   header (16 bytes):
//     [0..3]   to        = 0xFFFFFFFF (broadcast)
//     [4..7]   from      = this node's Meshtastic node number
//     [8..11]  id        = packet id (nonzero; varies per packet)
//     [12]     flags     = hop_limit[2:0] | want_ack<<3 | via_mqtt<<4 | hop_start[7:5]
//     [13]     channel   = channel hash
//     [14]     next_hop  = 0
//     [15]     relay_node= 0
//   payload: AES-CTR( Data{ portnum, payload } )  -- see PortNum in MeshtasticProto.h
// ---------------------------------------------------------------------------
inline int buildDataPacket(uint8_t* out, size_t out_cap,
                           uint32_t from_node, uint32_t packet_id,
                           uint8_t portnum, const uint8_t* payload, size_t plen,
                           const uint8_t* key, size_t key_len,
                           uint8_t chan_hash, uint8_t hop_limit = 3) {
  // ---- plaintext: protobuf Data { portnum (field 1), payload (field 2) } ----
  if (plen > 230) return 0;                 // keep us under the 256B PHY cap
  uint8_t pt[240];
  size_t p = 0;
  p += pbUint32(&pt[p], 1, portnum);        // field 1 portnum
  pt[p++] = 0x12;                           // field 2 payload, wire type 2
  p += pbVarint(&pt[p], plen);
  memcpy(&pt[p], payload, plen); p += plen;

  if (out_cap < 16 + p) return 0;

  // ---- header ----
  size_t o = 0;
  auto put32 = [&](uint32_t v) {
    out[o++] = (uint8_t)v;        out[o++] = (uint8_t)(v >> 8);
    out[o++] = (uint8_t)(v >> 16); out[o++] = (uint8_t)(v >> 24);
  };
  put32(0xFFFFFFFFu);                         // to: broadcast
  put32(from_node);                           // from
  put32(packet_id);                           // id
  uint8_t hl = hop_limit & 0x07;
  out[o++] = hl | (hl << 5);                  // flags: hop_limit + hop_start, no ack/mqtt
  out[o++] = chan_hash;                       // channel hash
  out[o++] = 0x00;                            // next_hop
  out[o++] = 0x00;                            // relay_node

  // ---- nonce: packetId (64-bit LE) | fromNode (32-bit LE) | 0 ----
  uint8_t nonce[16];
  memset(nonce, 0, sizeof(nonce));
  nonce[0] = packet_id;        nonce[1] = packet_id >> 8;
  nonce[2] = packet_id >> 16;  nonce[3] = packet_id >> 24;
  nonce[8] = from_node;        nonce[9] = from_node >> 8;
  nonce[10] = from_node >> 16; nonce[11] = from_node >> 24;

  // ---- AES-CTR encrypt plaintext directly into the wire buffer ----
  // NOTE: 16-byte key -> AES-128 (the default public channel). For a 32-byte
  // PSK swap CTR<AES128> for CTR<AES256>.
  CTR<AES128> ctr;
  ctr.setKey(key, key_len);
  ctr.setIV(nonce, sizeof(nonce));
  ctr.encrypt(&out[o], pt, p);
  o += p;

  return (int)o;
}

// Convenience: a TEXT_MESSAGE_APP packet (unchanged API).
inline int buildTextPacket(uint8_t* out, size_t out_cap,
                           uint32_t from_node, uint32_t packet_id,
                           const char* text,
                           const uint8_t* key, size_t key_len,
                           uint8_t chan_hash, uint8_t hop_limit = 3) {
  return buildDataPacket(out, out_cap, from_node, packet_id, PORT_TEXT,
                         (const uint8_t*)text, strlen(text),
                         key, key_len, chan_hash, hop_limit);
}

// ---------------------------------------------------------------------------
// Retune to Meshtastic, transmit one raw packet, restore the MeshCore PHY.
//
// Templated so it works for any concrete RadioLib radio (CustomLR1110 on the
// T1000-E, CustomSX1262, etc.) — it only needs standby()/setSyncWord()/
// setPreambleLength(), plus the MeshCore RadioLibWrapper for setParams/send.
//
//   driver      : the RadioLibWrapper (MeshCore's radio_driver)
//   radio       : the concrete RadioLib radio behind it
//   mt          : Meshtastic modem preset to transmit on
//   pkt,len     : bytes from buildTextPacket()
//   home_*      : MeshCore PHY to restore (your LORA_FREQ/BW/SF/CR + sync word)
//
// Returns true if the transmit completed (false on radio error or timeout).
// While this runs the radio is off the MeshCore channel, so MeshCore traffic in
// that window is missed — keep beacons infrequent.
// ---------------------------------------------------------------------------
template <class DriverT, class RadioT>
void radioEnterMeshtastic(DriverT& driver, RadioT& radio,
                          const ModemPreset& mt, int8_t tx_power) {
  radio.standby();                                  // freq/sf changes require standby
  driver.setParams(mt.freq_mhz, mt.bw_khz, mt.sf, mt.cr);
  radio.setSyncWord(mt.sync_word);
  radio.setPreambleLength(mt.preamble);
  driver.setTxPower(tx_power);
}

// What actually happened to one transmit. Callers mostly care about the
// did-it-go-out question (txDelivered); the finer grades exist so a node can
// say WHY on the console instead of guessing.
enum TxOutcome : uint8_t {
  TX_NOT_STARTED,   // the radio refused to begin the transmit
  TX_IRQ,           // normal path: the TxDone interrupt arrived
  TX_HW_CONFIRMED,  // interrupt missed, but the chip's TxDone flag is set
  TX_HW_FAILED,     // interrupt missed AND the chip's TxDone flag is clear
  TX_PRESUMED,      // interrupt missed, chip can't report -> airtime bound only
};

inline bool txDelivered(TxOutcome o) {
  return o == TX_IRQ || o == TX_HW_CONFIRMED || o == TX_PRESUMED;
}
inline bool txUsedFallback(TxOutcome o) {   // the TxDone interrupt didn't fire
  return o == TX_HW_CONFIRMED || o == TX_HW_FAILED || o == TX_PRESUMED;
}

// Send one raw packet while already tuned; cooperates with the TX state machine.
//
// Completion is normally signalled by the radio's TxDone interrupt (surfaced by
// the driver's isSendComplete()). After a Meshtastic retune that interrupt edge
// can occasionally be missed -- which, with a fixed multi-second timeout, would
// hang here and freeze the whole main loop while the node appears dead. So the
// wait is bounded by the packet's estimated airtime plus a margin: once that has
// elapsed, a transmit that started has physically finished.
//
// The bound alone can't say whether the packet went out, because
// RadioLibWrapper::isSendComplete() only reads a flag an ISR sets and never asks
// the hardware -- so "sent, interrupt lost" and "never transmitted" look
// identical from there (v0.2.4 assumed the former and could report a real
// failure as success). When the interrupt doesn't arrive we now ask the chip
// itself: RadioLib maps every family's TxDone bit onto the generic
// RADIOLIB_IRQ_TX_DONE, so one call covers SX126x/SX127x/LR11x0/STM32WLx.
//
// The read MUST happen before onSendFinished(), whose finishTransmit() clears
// the IRQ flags -- after that the evidence is gone.
template <class DriverT, class RadioT>
TxOutcome radioSendChecked(DriverT& driver, RadioT& radio,
                           const uint8_t* pkt, int len) {
  if (!driver.startSendRaw(pkt, len)) return TX_NOT_STARTED;

  uint32_t airtime = driver.getEstAirtimeFor(len);   // ms on air for this packet
  uint32_t cap = airtime * 2 + 500;                  // physical-done bound + margin
  uint32_t start = millis();
  while ((uint32_t)(millis() - start) <= cap) {
    if (driver.isSendComplete()) { driver.onSendFinished(); return TX_IRQ; }
    yield();
  }

  int16_t tx_done = radio.checkIrq(RADIOLIB_IRQ_TX_DONE);   // before finishTransmit()
  driver.onSendFinished();
  if (tx_done < 0) return TX_PRESUMED;   // chip doesn't expose it: assume airtime
  return tx_done > 0 ? TX_HW_CONFIRMED : TX_HW_FAILED;
}

template <class DriverT, class RadioT>
void radioRestoreMeshCore(DriverT& driver, RadioT& radio,
                          float home_freq, float home_bw, uint8_t home_sf,
                          uint8_t home_cr, uint8_t home_sync, int8_t home_tx_power) {
  radio.standby();
  driver.setParams(home_freq, home_bw, home_sf, home_cr);   // also restores preamble
  radio.setSyncWord(home_sync);
  driver.setTxPower(home_tx_power);
  driver.startRecv();   // re-arm RX now; otherwise the radio sits in standby until
                        // the next recvRaw(), and under powersaving the node can
                        // sleep first and stay deaf for a whole wake interval.
}

// Retune to Meshtastic, transmit one raw packet, restore the MeshCore PHY.
template <class DriverT, class RadioT>
bool transmitOnce(DriverT& driver, RadioT& radio,
                  const ModemPreset& mt,
                  const uint8_t* pkt, int len,
                  float home_freq, float home_bw, uint8_t home_sf, uint8_t home_cr,
                  uint8_t home_sync,
                  int8_t mt_tx_power, int8_t home_tx_power) {
  radioEnterMeshtastic(driver, radio, mt, mt_tx_power);
  TxOutcome r = radioSendChecked(driver, radio, pkt, len);
  radioRestoreMeshCore(driver, radio, home_freq, home_bw, home_sf, home_cr,
                       home_sync, home_tx_power);
  return txDelivered(r);
}

} // namespace meshtastic
