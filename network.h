/// This file declares data types and values for out network layer. You may edit
/// this as you see fit (or remove it altogether).

#ifndef NETWORK_H
#define NETWORK_H

#include <cnet.h>
#include <stddef.h>
#include <stdint.h>

// Make this small for now, so that we don't need to worry about fragmenting
// packets.
//
#define NL_MAXDATA 1024

/// This struct defines the format for a network layer packet.
///
struct nl_packet {
  /// The node that this packet is destined for.
  CnetAddr dest;
  
  /// The node that this packet was created by.
  CnetAddr src;
  
  /// Checksum for this packet.
  uint32_t checksum;
  
  /// Length of this packet's payload.
  size_t length;
  
  /// The payload of this packet.
  char data[NL_MAXDATA];
};

// Determines the number of bytes used by a packet (the number of bytes used
// by the header plus the number of bytes used by the payload).
//
#define NL_PACKET_LENGTH(PKT) (offsetof(struct nl_packet, data) + PKT.length)

#endif // NETWORK_H
  
  
  