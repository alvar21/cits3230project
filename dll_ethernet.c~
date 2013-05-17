/// This file implements our Ethernet data link layer.
#include "dll_ethernet.h"

#include <cnet.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#define ETH_MAXDATA 1500
#define ETH_MINFRAME 64

//maximum number of collisions
const int MAX_COL = 10;
//once we back off for 16 times and still unable to send. the network is considered too congested
const int MAX_BACKOFF = 16;
//TODO maybe edit the back off time
static CnetTime backoff_period = 5120000;//51.2microseconds

/// This struct specifies the format of an Ethernet frame. When using Ethernet
/// links in cnet, the first part of the frame must be the destination address.
struct eth_frame {
  // Ethernet address of the destination (receiver).
  CnetNICaddr dest;
  
  // Ethernet address of the source (sender).
  CnetNICaddr src;
    
  // For our protocol the type field will indicate the length of the payload.
  //might change to int
  char type[2];
    
  //checksum of the frame
  int checksum;
  
  // Data must be the last field, because we will truncate the unused area when
  // sending to the physical layer.
  char data[ETH_MAXDATA];
};

/// This struct type will hold the state for one instance of the Ethernet data
/// link layer. The definition of the type is not important for clients.  
struct dll_eth_state {
  // The link that this instance of the Ethernet protocol is associated with.
  int link;

	//number of backoff this state experienced in a row    
  int num_backoff;
  
  //number of col this state sensed in a row
  int num_col;
  
  //indicator that we are able to send out frame
  bool ready;
  
  // A pointer to the function that is called to pass data up to the next layer.????????????
  up_from_dll_fn_ty nl_callback;
  
  //for backoff period
  CnetTimerID timer;
  
  //keep a record of the last frame sent
  struct eth_frame sent_frame;
	
// Add members to represent the Ethernet link's state here.
};


#define ETH_HEADER_LENGTH (offsetof(struct eth_frame, data))

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

void dll_eth_backoff(dll_eth_state *state) 
{
	srand(time(NULL));
	CnetTime backoff_time;
	CnetTimerID timer;
	state->ready = false;
	if(state->num_backoff < MAX_BACKOFF) {
		state->num_backoff++;
		if(num_col < MAX_COL) {
			backoff_time = (rand()%(pow(2,num_col)))*backoff_period;
		} else {
			backoff_time = pow(2,num_col)*backoff_period;
		}
		state.timer = CNET_start_timer(EV_TIMER1, backofftime, state);
	} else {
		//ether is deemed to be too busy
	}
}

/// Write a frame to the given Ethernet link.
/// transmit frame function
void dll_eth_write(struct dll_eth_state *state,
                   CnetNICaddr dest,
                   const char *data,
                   uint16_t length)
{
  if (!data || length == 0)
    return;
 	if(state->ready) { 
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

		//checksum
		frame.checksum = 0;
		frame.checksum  = CNET_crc32((unsigned char *)&frame, (int)frame_length);
	
		//keep a copy of sent frame
		memcpy(state->sent_frame,frame,sizeof(eth_frame));
	
		int busy = CNET_carrier_sense(state->link);
		if(busy == -1) {
			//failure
		} else if(busy == 0) {
		/*	//reset the back off variables in nl
			state->num_backoff = 0;
			state->num_col = 0;*/
			CHECK(CNET_write_physical(state->link, &frame, &frame_length));
		} else {
		//there's transmission in ethernet, backoff
			dll_eth_backoff(state);
		}
  }
}

void dll_eth_timeouts(CnetEvent ev, CnetTimerID timer, CnetData data)
{
  struct dll_eth_state *state = (struct dll_eth_state *)data;
	struct eth_frame frame;
	frame = state->sent_frame;
	state->ready = true;
	dll_eth_write(state, frame.dest, frame.data, (uint16_t)frame.type);
}

/// Called when a frame has been received on the Ethernet link. This function
/// will retrieve the payload, and then pass it to the callback function that
/// is associated with the given state struct.
/// receive frame function
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
  struct eth_frame *frame = (struct eth_frame *)data;
  
  int old_sum = frame.checksum;
  frame.checksum = 0;
  int new_sum = CNET_crc32((unsigned char *)&f, (int)length);
  
  	//not corrupted
  if(new_sum == old_sum) {
  		// Extract the length of the payload from the Ethernet frame.
			uint16_t payload_length = 0;
			memcpy(&payload_length, frame->type, sizeof(payload_length));
  
  		// Send the frame up to the next layer.
  		if (state->nl_callback)
    		(*(state->nl_callback))(state->link, frame->data, payload_length);	
  } 
}

void dll_eth_collide(dll_eth_state *state) 
{
	if(state->num_col < MAX_COL) {
		state->num_col++;
	}
	dll_eth_backoff(state);
}

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
  state->timer = NULLTIMER;
  state->sent_frame = NULL;
  state->num_col = 0;
  state->num_backoff = 0;
	state->ready = true;
  CHECK(CNET_set_handler(EV_TIMER1, dll_eth_timeouts, 0));

  return state;
}

