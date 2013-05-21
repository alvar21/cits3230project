/// This file implements our WiFi data link layer.

#include "dll_wifi.h"

#include <cnet.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define WIFI_MAXDATA 2312

#define CTS_TIMER EV_TIMER4
#define CONNECTED_TIMER EV_TIMER5
#define PROBING_TIMER EV_TIMER6
#define WIFI_TIMEOUT EV_TIMER7

#define TIME_PER_BYTE 10

#define MAX_COLLISIONS 16
#define BACKOFF 10000

typedef enum {
	WIFI_PROBE,
	WIFI_PROBE_ACK,
	WIFI_RTS,
	WIFI_CTS,
	WIFI_DATA
} wifi_frame_type;

/// This struct specifies the format of the control section of a WiFi frame.
struct wifi_control {
  unsigned from_ds;
  wifi_frame_type type;
};

/// This struct specifies the format of a WiFi frame.
///
struct wifi_frame {
    // Address of the receiver.
  CnetNICaddr dest;
  
  // Address of the transmitter.
  CnetNICaddr src;  
  // Control section.
  struct wifi_control control;
  
  // Number of bytes in the payload, or in the case of request frames, the number of bytes that will be sent
  uint16_t length;
  
  // CRC32 for the entire frame.
  uint32_t checksum;  
  // Data must be the last field, because we will truncate the unused area when
  // sending to the physical layer.
  char data[WIFI_MAXDATA];
};

#define WIFI_HEADER_LENGTH (offsetof(struct wifi_frame, data))

typedef enum  {
	WIFI_PROBING,
	WIFI_READY,
	WIFI_REQUESTING,
	WIFI_SENDING,
	WIFI_RECEIVING,
} dll_wifi_state_status;

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
  
  int collisions;
  
  dll_wifi_state_status status;

  bool medium_ready;

   CnetTimerID timeout;
   CnetTimerID clear;
   CnetTimerID connected;
   CnetTimerID start_probing;

  struct wifi_frame frame;
  struct wifi_frame control;
};

void backoff_time(struct dll_wifi_state *state) {
  
    
    CnetTime backoff_time;
  
    if(state->collisions < MAX_COLLISIONS) {
	    
	    backoff_time = (CNET_rand()%(int)(pow(2,state->collisions)) )*BACKOFF;
	     printf("choosing between %d slots\n", (int)(pow(2,state->collisions)) );
	    printf("Node %d, backing off for %d\n", nodeinfo.nodenumber, backoff_time);
	    if(state->timeout != NULLTIMER) {
	      CNET_stop_timer(state->timeout);
	    }
	    state->timeout = CNET_start_timer(WIFI_TIMEOUT, backoff_time+1000, (CnetData)state);
    }    
    
}

void dll_wifi_send(struct dll_wifi_state *state) {	
	//cancelling timer
	if(!state) {
		printf("No state passed\n");
		return;
	} else if(!state->medium_ready) {
	  
		printf("medium not ready\n");
		return;
		//will be called again when medium is ready
	} else if(state->status == WIFI_READY && state->control.control.type != WIFI_PROBE_ACK) {
	  
	    printf("nothing to send\n");
	    return;
	} else if(state->status == WIFI_RECEIVING) {
	    printf("waiting to receive\n");
	}
    if(state->control.control.type != WIFI_PROBE_ACK);
      backoff_time(state);
//       printf("sending probe frame to physical\n");
      size_t frame_length = WIFI_HEADER_LENGTH + state->frame.length;
  
	switch(state->status) {
	case WIFI_SENDING:
		CHECK(CNET_write_physical(state->link, &(state->frame), &frame_length));
		//state->status = WIFI_REQUESTING;
		//if it collides we must request again?
		break;
	case WIFI_REQUESTING:
	case WIFI_RECEIVING:		
	case WIFI_PROBING:
	case WIFI_READY:
// 		printf("Sending control frame: ");
		switch(state->control.control.type) {
		  case WIFI_PROBE:
// 		    printf("PROBE\n");
		    break;
		  default:
		    break;
		}
	      	frame_length = WIFI_HEADER_LENGTH;
		CHECK(CNET_write_physical(state->link, &(state->control), &frame_length));
		break;
	default: break;
	}

}

//send control frames over the given WiFi link
void dll_wifi_control(struct dll_wifi_state *state,
					  CnetNICaddr dest,
					  wifi_frame_type type)
{
	if (state->status != WIFI_READY && state->status != WIFI_PROBING) {
		printf( "Not ready to send packet\n");
		return;
	}
	if(type == WIFI_RTS) {
		printf( "Transitioning to WIFI_REQUESTING\n" );
		state->status = WIFI_REQUESTING;
	}
	else if(type == WIFI_CTS) {
		printf( "Transitioning to WIFI_RECEIVING\n" );
		state->status = WIFI_RECEIVING;
	} else {	      
// 		printf("sending probe frame to send\n");
	}

  // Create a frame and initialize the length field.
  state->control = (struct wifi_frame){
    .control = (struct wifi_control){
      .from_ds = (state->is_ds ? 1 : 0),
      .type = type
    },
    .length = state->frame.length,
    .data = {0}
  };  
  // Set the destination and source address.
  memcpy(state->control.dest, dest, sizeof(CnetNICaddr));
  memcpy(state->control.src, linkinfo[state->link].nicaddr, sizeof(CnetNICaddr));
   
  // Calculate the number of bytes to send.
  size_t frame_length = WIFI_HEADER_LENGTH;

  state->control.checksum = 0;
  // Set the checksum.
  state->control.checksum = CNET_crc32((unsigned char *)&(state->control), frame_length);
  

 dll_wifi_send(state);

}
/// Write a frame to the given WiFi link.
///
void dll_wifi_write(struct dll_wifi_state *state,
                    CnetNICaddr dest,
                    const char *data,
                    uint16_t length)
{
  if (!data || length == 0 || length > WIFI_MAXDATA || state->status != WIFI_READY)
    return;
  // Create a frame and initialize the length field.
  state->frame = (struct wifi_frame){
    .control = (struct wifi_control){
      .from_ds = (state->is_ds ? 1 : 0),
	   .type = WIFI_DATA
    },
    .length = length
  };
  
  
  // Set the destination and source address.
  if(state->is_ds)  	 
	memcpy(state->dest, dest, sizeof(CnetNICaddr)); 

  memcpy(state->frame.dest, state->dest, sizeof(CnetNICaddr));
  memcpy(state->frame.src, linkinfo[state->link].nicaddr, sizeof(CnetNICaddr));
  // Copy in the payload.
  memcpy(state->frame.data, data, length);
  
  
  // Calculate the number of bytes to send.
  size_t frame_length = WIFI_HEADER_LENGTH + length;

  state->frame.checksum = 0;
  // Set the checksum.
  state->frame.checksum = CNET_crc32((unsigned char *)&(state->frame), frame_length);

  //send a RTS packet to associated access point
  dll_wifi_control(state, state->dest, WIFI_RTS);
  
}




/// Called when a frame has been received on the WiFi link. This function will
/// retrieve the payload, and then pass it to the callback function that is
/// associated with the given state struct.
///
void dll_wifi_read(struct dll_wifi_state *state,
                   const char *data,
                   size_t length)
{
//      printf("WiFi: read from link %d with length %zd\n", state->link, length);
  
	if (length > sizeof(struct wifi_frame)) {
	 printf("\tFrame is too large!\n");
	return;
	}
  
	// Treat the data as a WiFi frame.
	struct wifi_frame *frame = (struct wifi_frame *)data;  

	int old_checksum = frame->checksum;
	frame->checksum = 0;
	int new_checksum = CNET_crc32((unsigned char *)frame, length);

	if (old_checksum != new_checksum) {
		printf("\tFrame Corrupt!\n");
		return;
	}

	if ( frame->dest != linkinfo[state->link].nicaddr ) {	  
		switch(frame->control.type) {
		case WIFI_RTS:
		case WIFI_CTS:
		  
			printf("CTS/RTS\n");
			state->medium_ready = false;
			state->clear = CNET_start_timer(CTS_TIMER, frame->length*TIME_PER_BYTE, (CnetData)state);
			return;
		default:
		  break;
		}
	} 
	// Mobile to Mobile or AP to AP communication disallowed
	if (state->is_ds == frame->control.from_ds)  {	  
		printf("\tWiFi: Ignoring frame from same device.\n");
		return;
	}
	
	double strength;
	double angle;
	CNET_wlan_arrival(state->link, &strength, &angle);
	

	if (state->is_ds) {
// 		printf("I am a ds\n");
		if (frame->control.type == WIFI_PROBE) {
			dll_wifi_control(state, frame->src, WIFI_PROBE_ACK);
			printf("Responding to probe frame\n");
		}
	} else {
		if (frame->control.type == WIFI_PROBE_ACK && state->status == WIFI_PROBING) {
			CNET_stop_timer(state->timeout);
			state->collisions = 0;
			
			printf("Found candidate \n");
			if (strength > state->strength) {
			    printf("Found better strength candidate %f\n", strength);
			     memcpy(state->dest, frame->src, sizeof(CnetNICaddr));			     
			     state->strength = strength;
			}
		}
		if ( frame->src == state->dest) {
		    state->strength = strength;
		}
	}
	
	switch (frame->control.type) {
		case WIFI_RTS:
			printf("RTS for us\n");
			dll_wifi_control(state, frame->src, WIFI_CTS);
		break;
		case WIFI_CTS:	
			CNET_stop_timer(state->timeout);
			state->collisions = 0;
			printf("CTS for us\n");	
			state->status = WIFI_SENDING;
			dll_wifi_send(state);
		//respond with data
		break;
		case WIFI_DATA:	
		  CNET_stop_timer(state->timeout);
		  state->collisions = 0;
		  printf("DATA for us\n");			
		  // Send the frame up to the next layer.
		  if (state->nl_callback)
			(*(state->nl_callback))(state->link, frame->data, frame->length);
		  state->status = WIFI_READY;
		default:
		  break;
	}
	
  
}


EVENT_HANDLER( wifi_timeout) {
   struct dll_wifi_state *state =  (struct dll_wifi_state *)data;
   
   state->collisions++;
   printf("Num collisions: %d\n", state->collisions);
    dll_wifi_send(state);
}

EVENT_HANDLER(clear_to_send) {
	struct dll_wifi_state *state = (struct dll_wifi_state *)data;
	state->medium_ready = true;
	dll_wifi_send(state);
	printf("Medium is now clear\n");
	
}

EVENT_HANDLER(finished_probing) {    
	struct dll_wifi_state *state = (struct dll_wifi_state *)data;
	if( state->strength != -1000000.0) {
	state->status = WIFI_READY;
	printf("Connected to an ap\n");
	} else {
	  printf("didnt find any ap\n");
	  state->start_probing = CNET_start_timer(PROBING_TIMER, 1000000, (CnetData)state);
	}
  
}

EVENT_HANDLER(start_probing) {
    struct dll_wifi_state *state = (struct dll_wifi_state *)data;
    state->status = WIFI_PROBING;    
    CnetNICaddr broadcast;
    CHECK(CNET_parse_nicaddr(broadcast, "ff:ff:ff:ff:ff:ff"));
//     printf("sending probe control\n");
    dll_wifi_control(state, broadcast, WIFI_PROBE);
    state->connected = CNET_start_timer(CONNECTED_TIMER, 1000000, (CnetData)state); 
  
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
  state->timeout = NULLTIMER;
  state->connected = NULLTIMER;
  state->start_probing = NULLTIMER;
  state->clear = NULLTIMER;
  state->medium_ready = true;
  state->strength = -1000000.0;
  state->collisions = 0;
  
  CNET_set_handler(WIFI_TIMEOUT, wifi_timeout, 0);
  CNET_set_handler(CONNECTED_TIMER, finished_probing, 0);
  CNET_set_handler(PROBING_TIMER, start_probing, 0);
  CNET_set_handler(CTS_TIMER, clear_to_send, 0);
  
  if(is_ds) {
//     printf("We are DS, WIFI_READY\n");
    state->status = WIFI_READY;
  }
  else {
    state->status = WIFI_PROBING;    
    state->start_probing = CNET_start_timer(PROBING_TIMER, 1000, (CnetData)state);    
  }

  
    CNET_srand(nodeinfo.time_of_day.sec + nodeinfo.nodenumber);
  
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
