/// This file declares the external interface to our WiFi data link layer.

#ifndef DLL_WIFI_H
#define DLL_WIFI_H

#include "dll_shared.h"

#include <cnet.h>
#include <stdint.h>

/// This struct type will hold the state for one instance of the WiFi data
/// link layer. The definition of the type is not important for clients.
///
struct dll_wifi_state;

/// Create a new state for an instance of the WiFi data link layer.
///
struct dll_wifi_state *dll_wifi_new_state(int link,
                                          up_from_dll_fn_ty callback,
                                          bool is_ds);

/// Delete the given dll_wifi_state. The given state pointer will be invalid
/// following a call to this function.
///
void dll_wifi_delete_state(struct dll_wifi_state *state);

/// Write a frame to the given WiFi link.
///
void dll_wifi_write(struct dll_wifi_state *state,
                    CnetNICaddr dest,
                    const char *data,
                    uint16_t length);

/// Called when a frame has been received on the WiFi link. This function will
/// retrieve the payload, and then pass it to the callback function that is
/// associated with the given state struct.
///
void dll_wifi_read(struct dll_wifi_state *state,
                   const char *data,
                   size_t length);

#endif // DLL_WIFI_H
