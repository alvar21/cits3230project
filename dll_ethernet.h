/// This file declares the external interface to our Ethernet data link layer.

#ifndef DLL_ETHERNET_H
#define DLL_ETHERNET_H

#include "dll_shared.h"

#include <cnet.h>
#include <stdint.h>

/// This struct type will hold the state for one instance of the Ethernet data
/// link layer. The definition of the type is not important for clients.
///
struct dll_eth_state;

/// Create a new state for an instance of the Ethernet data link layer.
///
struct dll_eth_state *dll_eth_new_state(int link, up_from_dll_fn_ty callback);

/// Delete the given dll_eth_state. The given state pointer will be invalid
/// following a call to this function.
///
void dll_eth_delete_state(struct dll_eth_state *state);

/// Write a frame to the given Ethernet link.
///
void dll_eth_write(struct dll_eth_state *state,
                   CnetNICaddr dest,
                   const char *data,
                   uint16_t length);

/// Called when a frame has been received on the Ethernet link. This function
/// will retrieve the payload, and then pass it to the callback function that
/// is associated with the given state struct.
///
void dll_eth_read(struct dll_eth_state *state,
                  const char *data,
                  size_t length);

#endif // DLL_ETHERNET_H
