/// This file implements our WiFi data link layer.

#include "dll_wifi.h"

#include <cnet.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define WIFI_MAXDATA 2312

/// This struct type will hold the state for one instance of the WiFi data
/// link layer. The definition of the type is not important for clients.
///
struct dll_wifi_state {
  // The link that this instance of the WiFi protocol is associated with.
  int link;
  
  // A pointer to the function that is called to pass data up to the next layer.
  up_from_dll_fn_ty nl_callback;
  
  // True iff this node is part of the DS (i.e. an access point).
  bool is_ds;
  
  // Add members to represent the WiFi link's state here.
};

/// This struct specifies the format of the control section of a WiFi frame.
struct wifi_control {
  unsigned from_ds : 1;
};

/// This struct specifies the format of a WiFi frame.
///
struct wifi_frame {
  // Control section.
  struct wifi_control control;
  
  // Number of bytes in the payload.
  uint16_t length;
  
  // Address of the receiver.
  CnetNICaddr dest;
  
  // Address of the transmitter.
  CnetNICaddr src;
  
  // CRC32 for the entire frame.
  uint32_t checksum;
  
  // Data must be the last field, because we will truncate the unused area when
  // sending to the physical layer.
  char data[WIFI_MAXDATA];
};

#define WIFI_HEADER_LENGTH (offsetof(struct wifi_frame, data))

/// Create a new state for an instance of the WiFi data link layer.
///
struct dll_wifi_state *dll_wifi_new_state(int link,
                                          up_from_dll_fn_ty callback,
                                          bool is_ds)
{
  // Ensure that the given link exists and is a WLAN link.
  if (link > nodeinfo.nlinks || linkinfo[link].linktype != LT_WLAN)
    return NULL;
  
  // Allocate memory for the state.
  struct dll_wifi_state *state = calloc(1, sizeof(struct dll_wifi_state));
  
  // Check whether or not the allocation was successful.
  if (state == NULL)
    return NULL;
  
  // Initialize the members of the structure.
  state->link = link;
  state->nl_callback = callback;
  state->is_ds = is_ds;
  
  return state;
}

/// Delete the given dll_wifi_state. The given state pointer will be invalid
/// following a call to this function.
///
void dll_wifi_delete_state(struct dll_wifi_state *state)
{
  if (state == NULL)
    return;
  
  // Free any dynamic memory that is used by the members of the state.
  
  free(state);
}

/// Write a frame to the given WiFi link.
///
void dll_wifi_write(struct dll_wifi_state *state,
                    CnetNICaddr dest,
                    const char *data,
                    uint16_t length)
{
  if (!data || length == 0 || length > WIFI_MAXDATA)
    return;
  
  // Create a frame and initialize the length field.
  struct wifi_frame frame = (struct wifi_frame){
    .control = (struct wifi_control){
      .from_ds = (state->is_ds ? 1 : 0)
    },
    .length = length
  };
  
  // Set the destination and source address.
  memcpy(frame.dest, dest, sizeof(CnetNICaddr));
  memcpy(frame.src, linkinfo[state->link].nicaddr, sizeof(CnetNICaddr));
  
  // Copy in the payload.
  memcpy(frame.data, data, length);
  
  // Set the checksum.
  frame.checksum = CNET_crc32((unsigned char *)&frame, sizeof(frame));
  
  // Calculate the number of bytes to send.
  size_t frame_length = WIFI_HEADER_LENGTH + length;
  
  CHECK(CNET_write_physical_reliable(state->link, &frame, &frame_length));
}

/// Called when a frame has been received on the WiFi link. This function will
/// retrieve the payload, and then pass it to the callback function that is
/// associated with the given state struct.
///
void dll_wifi_read(struct dll_wifi_state *state,
                   const char *data,
                   size_t length)
{
  // printf("WiFi: read from link %d with length %zd\n", state->link, length);
  
  if (length > sizeof(struct wifi_frame)) {
    // printf("\tFrame is too large!\n");
    return;
  }
  
  // Treat the data as a WiFi frame.
  const struct wifi_frame *frame = (const struct wifi_frame *)data;
  
  // Ignore WiFi frames received from other APs.
  if (frame->control.from_ds && state->is_ds) {
    printf("\tWiFi: Ignoring frame from access point.\n");
    return;
  }
  
  // Send the frame up to the next layer.
  if (state->nl_callback)
    (*(state->nl_callback))(state->link, frame->data, frame->length);
}
