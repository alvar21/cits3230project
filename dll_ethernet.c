/// This file implements our Ethernet data link layer.

#include "dll_ethernet.h"

#include <cnet.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define ETH_MAXDATA 1500
#define ETH_MINFRAME 64

/// This struct type will hold the state for one instance of the Ethernet data
/// link layer. The definition of the type is not important for clients.
///
struct dll_eth_state {
  // The link that this instance of the Ethernet protocol is associated with.
  int link;
  
  // A pointer to the function that is called to pass data up to the next layer.
  up_from_dll_fn_ty nl_callback;
  
  // Add members to represent the Ethernet link's state here.
};

/// This struct specifies the format of an Ethernet frame. When using Ethernet
/// links in cnet, the first part of the frame must be the destination address.
///
struct eth_frame {
  // Ethernet address of the destination (receiver).
  CnetNICaddr dest;
  
  // Ethernet address of the source (sender).
  CnetNICaddr src;
  
  // For our protocol the type field will indicate the length of the payload.
  char type[2];
  
  // Data must be the last field, because we will truncate the unused area when
  // sending to the physical layer.
  char data[ETH_MAXDATA];
};

#define ETH_HEADER_LENGTH (offsetof(struct eth_frame, data))

/// Create a new state for an instance of the Ethernet data link layer.
///
struct dll_eth_state *dll_eth_new_state(int link, up_from_dll_fn_ty callback)
{
  // Ensure that the given link exists and is a LAN link.
  if (link > nodeinfo.nlinks || linkinfo[link].linktype != LT_LAN)
    return NULL;
  
  // Allocate memory for the state.
  struct dll_eth_state *state = calloc(1, sizeof(struct dll_eth_state));
  
  // Check whether or not the allocation was successful.
  if (state == NULL)
    return NULL;
  
  // Initialize the members of the structure.
  state->link = link;
  state->nl_callback = callback;
  
  return state;
}

/// Delete the given dll_eth_state. The given state pointer will be invalid
/// following a call to this function.
///
void dll_eth_delete_state(struct dll_eth_state *state)
{
  if (state == NULL)
    return;
  
  // Free any dynamic memory that is used by the members of the state.
  
  free(state);
}

/// Write a frame to the given Ethernet link.
///
void dll_eth_write(struct dll_eth_state *state,
                   CnetNICaddr dest,
                   const char *data,
                   uint16_t length)
{
  if (!data || length == 0)
    return;
  
  struct eth_frame frame;
  
  // Set the destination and source address.
  memcpy(frame.dest, dest, sizeof(CnetNICaddr));
  memcpy(frame.src, linkinfo[state->link].nicaddr, sizeof(CnetNICaddr));
  
  // Set the length of the payload.
  memcpy(frame.type, &length, sizeof(length));
  
  // Copy the payload into the frame.
  memcpy(frame.data, data, length);
  
  // Calculate the number of bytes to send.
  size_t frame_length = length + ETH_HEADER_LENGTH;
  if (frame_length < ETH_MINFRAME)
    frame_length = ETH_MINFRAME;
  
  CHECK(CNET_write_physical_reliable(state->link, &frame, &frame_length));
}

/// Called when a frame has been received on the Ethernet link. This function
/// will retrieve the payload, and then pass it to the callback function that
/// is associated with the given state struct.
///
void dll_eth_read(struct dll_eth_state *state,
                  const char *data,
                  size_t length)
{
  // printf("Ethernet: read frame of length %zd.\n", length);
  
  if (length > sizeof(struct eth_frame)) {
    // printf("\tFrame is too large!\n");
    return;
  }
  
  // Treat the data as an Ethernet frame.
  const struct eth_frame *frame = (const struct eth_frame *)data;
  
  // Extract the length of the payload from the Ethernet frame.
  uint16_t payload_length = 0;
  memcpy(&payload_length, frame->type, sizeof(payload_length));
  
  // Send the frame up to the next layer.
  if (state->nl_callback)
    (*(state->nl_callback))(state->link, frame->data, payload_length);
}
