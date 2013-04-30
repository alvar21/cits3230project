/// This file implements the functionality of our access points.

#include <cnet.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "ap.h"
#include "dll_ethernet.h"
#include "dll_wifi.h"
#include "mapping.h"
#include "network.h"

/// This enumerates the possible types of data link layers used by an AP.
///
enum dll_type {
  DLL_UNSUPPORTED,
  DLL_ETHERNET,
  DLL_WIFI
};

/// This holds the data link layer type and state for a single link on an AP.
///
struct dll_state {
  enum dll_type type;
  
  union {
    struct dll_eth_state *ethernet;
    struct dll_wifi_state *wifi;
  } data;
};

/// This holds the data link layer information for all links on this AP.
///
static struct dll_state *dll_states = NULL;

/// Raised when one of our physical links has received a frame.
///
static EVENT_HANDLER(physical_ready)
{
  // First we read the frame from the physical layer.
  char frame[DLL_MTU];
  size_t length	= sizeof(frame);
  int link;

  CHECK(CNET_read_physical(&link, frame, &length));
  
  // Now we forward this information to the correct data link layer.
  if (link > nodeinfo.nlinks) {
    // printf("AP: Received frame on unknown link %d.\n", link);
    return;
  }
  
  switch (dll_states[link].type) {
    case DLL_UNSUPPORTED:
      // printf("AP: Received frame on unsupported link.\n");
      break;
    
    case DLL_ETHERNET:
      // printf("AP: Received frame on Ethernet link %d.\n", link);
      dll_eth_read(dll_states[link].data.ethernet, frame, length);
      break;
    
    case DLL_WIFI:
      // printf("AP: Received frame on WiFi link %d.\n", link);
      dll_wifi_read(dll_states[link].data.wifi, frame, length);
      break;
  }
}

/// Called when we receive data from one of our data link layers.
///
static void up_from_dll(int link, const char *data, size_t length)
{
  if (length >= sizeof(struct nl_packet)) {
    printf("AP: %zu is larger than a nl_packet! ignoring.\n", length);
    return;
  }
  
  // Treat this frame as a network layer packet.
  const struct nl_packet *packet = (const struct nl_packet *)data;
  
  printf("AP: Received frame on link %d from node %" PRId32
         " for node %" PRId32 ".\n", link, packet->src, packet->dest);
  
  // We rebroadcast the packet on all of our links. If the packet came in on an
  // Ethernet link, then don't rebroadcast on that because all other nodes have
  // already seen it.
  CnetNICaddr broadcast;
  CHECK(CNET_parse_nicaddr(broadcast, "ff:ff:ff:ff:ff:ff"));
  
  for (int outlink = 1; outlink <= nodeinfo.nlinks; ++outlink) {
    switch (dll_states[outlink].type) {
      case DLL_UNSUPPORTED:
        break;
      
      case DLL_ETHERNET:
        if (outlink == link)
          break;
        printf("\tSending on Ethernet link %d\n", outlink);
        dll_eth_write(dll_states[outlink].data.ethernet,
                      broadcast,
                      data,
                      length);
        break;
      
      case DLL_WIFI:
        printf("\tSending on WiFi link %d\n", outlink);
        dll_wifi_write(dll_states[outlink].data.wifi,
                       broadcast,
                       data,
                       length);
        break;
    }
  }
}

/// Called when this access point is booted up.
///
void reboot_accesspoint()
{
  // We require each node to have a different stream of random numbers.
  CNET_srand(nodeinfo.time_of_day.sec + nodeinfo.nodenumber);
  
  // Provide the required event handlers.
  CHECK(CNET_set_handler(EV_PHYSICALREADY, physical_ready, 0));
  
  // Prepare to talk via our wireless connection.
  CHECK(CNET_set_wlan_model(my_WLAN_model));
  
  // Setup our data link layer instances.
  dll_states = calloc(nodeinfo.nlinks + 1, sizeof(struct dll_state));
  
  for (int link = 0; link <= nodeinfo.nlinks; ++link) {
    switch (linkinfo[link].linktype) {
      case LT_LOOPBACK:
        dll_states[link].type = DLL_UNSUPPORTED;
        break;
      
      case LT_WAN:
        dll_states[link].type = DLL_UNSUPPORTED;
        break;
      
      case LT_LAN:
        dll_states[link].type = DLL_ETHERNET;
        dll_states[link].data.ethernet = dll_eth_new_state(link, up_from_dll);
        break;
      
      case LT_WLAN:
        dll_states[link].type = DLL_WIFI;
        dll_states[link].data.wifi = dll_wifi_new_state(link,
                                                        up_from_dll,
                                                        true /* is_ds */);
        break;
    }
  }
  
  // printf("reboot_accesspoint() complete.\n");
}
