/// This file implements our WiFi data link layer.

#include "dll_wifi.h"

#include <cnet.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define WIFI_MAXDATA 2312

#define WIFI_RTS_TIMEOUT 1000
#define CTS_TIMER EV_TIMER4
#define TIME_PER_BYTE 10

enum wifi_packet_type {
	WIFI_PROBE,
	WIFI_RTS,
	WIFI_CTS,
	WIFI_DATA

}
/// This struct specifies the format of the control section of a WiFi frame.
struct wifi_control {
  unsigned from_ds : 1;
  enum wifi_frame_type type: 2;
};

/// This struct specifies the format of a WiFi frame.
///
struct wifi_frame {
  // Control section.
  struct wifi_control control;
  
  // Number of bytes in the payload, or in the case of request frames, the number of bytes that will be sent
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

typedef enum dll_wifi_state_status {
	WIFI_PROBING,
	WIFI_READY,
	WIFI_REQUESTING,
	WIFI_SENDING,
	WIFI_RECIEVING
}
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
  //associated access point if we are a mobile, otherwise current device we are trying to send to
  CnetNICaddr dest;  
  double strength;
  
  //state 0 = probing, 1 = connected, 2 = requesting to send, 3 = clear to send
  dll_wifi_state_status status;

  bool medium_ready;


  wifi_frame frame;
  wifi_frame control;
};

static EVENT_HANDLER(clear_to_send) {
	dll_wifi_state *state = (dll_wifi_state *)data;
	state->medium_ready = true;
	dll_wifi_send(state);
}


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
  
  state->status = WIFI_PROBING;
  if(is_ds)
	  state->status = WIFI_READY;
  state->medium_ready = true;

  CNET_set_handler(CTS_TIMER, clear_to_send);

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

void dll_wifi_send(struct dll_wifi_state *state) {	
	//cancelling timer
	if(!state)
		return;
	if(!state->medium_ready) {
		return;
		//will be called again when medium is ready
	}
	switch(state-status) {
	case WIFI_SENDING:
		CHECK(CNET_write_physical(state->link, &(state->frame), &frame_length));
		break;
	case WIFI_REQUESTING:
		CHECK(CNET_write_physical(state->link, &(state->control), &frame_length));
		break;
	case WIFI_RECIEVING:
		CHECK(CNET_write_physical(state->link, &(state->control), &frame_length));

	}

}
/// Write a frame to the given WiFi link.
///
void dll_wifi_write(struct dll_wifi_state *state,
                    CnetNICaddr dest,
                    const char *data,
                    uint16_t length)
{
  if (!data || length == 0 || length > WIFI_MAXDATA || !state->status == WIFI_READY)
    return;
  // Create a frame and initialize the length field.
  state->frame = (struct wifi_frame){
    .control = (struct wifi_control){
      .from_ds = (state->is_ds ? 1 : 0),
	  .wifi_packet_type = WIFI_DATA
    },
    .length = length
  };
  
  
  // Set the destination and source address.
  if(state->is_ds)  	 
	memcpy(state->ap, dest, sizeof(CnetNICaddr)); 

  memcpy(frame.dest, state->ap, sizeof(CnetNICaddr));
  memcpy(frame.src, linkinfo[state->link].nicaddr, sizeof(CnetNICaddr));
  
  // Copy in the payload.
  memcpy(frame.data, data, length);
  
  
  // Calculate the number of bytes to send.
  size_t frame_length = WIFI_HEADER_LENGTH + length;

  // Set the checksum.
  state->frame.checksum = CNET_crc32((unsigned char *)&(state->frame), frame_length);

  //send a RTS packet to associated access point
  dll_wifi_control(state, state->ap, WIFI_RTS)
  
}

//send control frames over the given WiFi link
void dll_wifi_control(struct dll_wifi_state *state,
					  CnetNICaddr dest,
					  wifi_frame_type type)
{
	if (state->status != WIFI_READY || state->status == WIFI_PROBING)
		return;
	if(type == WIFI_RTS)
		state->status = WIFI_REQUESTING;
	else if(type == WIFI_CTS)
		state->status = WIFI_RECIEVING;

  // Create a frame and initialize the length field.
  state->control = (struct wifi_frame){
    .control = (struct wifi_control){
      .from_ds = (state->is_ds ? 1 : 0),
	  .wifi_packet_type = type
    },
    .length = state->frame.length,
	.data = 0
  };  
  // Set the destination and source address.
  memcpy(state->control.dest, dest, sizeof(CnetNICaddr));
  memcpy(state->control.src, linkinfo[state->link].nicaddr, sizeof(CnetNICaddr));
   
  // Calculate the number of bytes to send.
  size_t frame_length = WIFI_HEADER_LENGTH;

  // Set the checksum.
  frame.checksum = CNET_crc32((unsigned char *)&(state->control), frame_length);
  

 dll_wifi_send(state);

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

	//length of other frametypes represents something else
	int length = 0;
	if ( frame->control.type == WIFI_DATA) 
		length = frame->length;

	size_t frame_length = WIFI_HEADER_LENGTH + length;
	if (frame_length > sizeof(struct wifi_frame)) {
		// printf("\tFrame Length Corrupt!\n");
		return;
	}
	int old_checksum = frame->checksum;
	frame->checksum = 0;
	int new_checksum = CNET_crc32((unsigned char *)frame, sizeof(*frame_length));

	if (old_checksum != new_checksum) {
		// printf("\tFrame Corrupt!\n");
		return;
	}
	
	CnetNICaddr broadcast;
	CHECK(CNET_parse_nicaddr(broadcast, "ff:ff:ff:ff:ff:ff"));

	if ( frame->dest != linkinfo[state->link].nicaddr ) {	  
		switch(frame->control.type) {
		case WIFI_RTS:
		case WIFI_CTS:
			state->medium_ready = false;
			CNET_start_timer(CTS_TIMER, frame->length()*TIME_PER_BYTE, (CnetData)state);
			return;
	} 
	// Mobile to Mobile or AP to AP communication disallowed
	if (state->is_ds == frame->control.is_ds)  {	  
		printf("\tWiFi: Ignoring frame from same device.\n");
		return;
	}


	if (state->is_ds) {
		if (frame->control.type == WIFI_PROBE) {
			dll_wifi_control(state, frame->src, WIFI_PROBE, 0);
		}
	} else {
		if (frame->control.type == WIFI_PROBE) {
			//find best
		}
	}

	switch (frame->control.type) {
		case WIFI_RTS:
			dll_wifi_control(state, frame->src, WIFI_CTS, frame->length);
		break;
		case WIFI_CTS:			
			state-status = WIFI_SENDING;
			dll_wifi_send(state);
		//respond with data
		break;
		case WIFI_DATA:			
		  // Send the frame up to the next layer.
		  if (state->nl_callback)
			(*(state->nl_callback))(state->link, frame->data, frame->length);

	}
  
}
