#pragma once

// Car-node marker carried in the advert's FEAT2 field (reserved and unused by
// upstream MeshCore, which parses and ignores it). Companions on this firmware
// use it to find the car nodes in range and show where each one parked.
//
//   feat2 = 0xCA00 | flags
//
// The advert's lat/lon is always the car's last *parked* position (the car
// node writes it on every park); CARNODE_ADV_PARKED says the car is still there.
// Costs 2 bytes of advert space, so the advertised name is limited to 21 bytes
// when a location is included (stock: 23).

#include <stdint.h>

#define CARNODE_ADV_MAGIC       0xCA00
#define CARNODE_ADV_MAGIC_MASK  0xFF00
#define CARNODE_ADV_PARKED      0x0001

static inline bool carNodeAdvert(uint16_t feat2) {
  return (feat2 & CARNODE_ADV_MAGIC_MASK) == CARNODE_ADV_MAGIC;
}
