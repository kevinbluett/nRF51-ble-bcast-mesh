#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "rbc_mesh.h"

#define	MESH_NOP 												(0)
#define	MESH_CONNECTION_REQUEST 				(1)
#define	MESH_CONNECTION_REQUEST_ACK			(2)
#define	MESH_DISCONNECT_SERVER					(3)
#define	MESH_DISCONNECT_CLIENT					(4)
#define	MESH_REQUEST_STATUS							(5)
#define	MESH_REQUEST_STATUS_ACK					(6)
#define	MESH_START_IMAGE_TRANSFER				(7)
#define	MESH_START_IMAGE_TRANSFER_ACK		(8)
#define	MESH_DATA_IMAGE_PACKET					(9)
#define	MESH_DATA_IMAGE_PACKET_ACK			(10)
#define	MESH_DATA_IMAGE_REQUEST					(11)
#define	MESH_IMAGE_TRANSFER_SUCCESS			(12)
#define	MESH_IMAGE_TRANSFER_SUCCESS_ACK	(13)
#define	MESH_IMAGE_ACTIVATE							(14)
#define	MESH_IMAGE_ACTIVATE_ACK					(15)
#define	MESH_CLIENT_ERROR								(16)

typedef uint16_t mesh_client_code;

typedef enum
{
	MESH_BROADCAST_NOP,
	MESH_BROADCAST_STATUS,
	MESH_BROADCAST_CLIENT_DIRECTION
} mesh_broadcast_code;

typedef uint16_t mesh_dfu_value_handle_t;

typedef struct
{
    mesh_client_code event_type;        /** See @ref rbc_mesh_event_type_t */
    mesh_dfu_value_handle_t value_handle;    /** Handle of the value the event is generated for */
	  uint16_t target_address;
    uint8_t* data;                      /** Current data array contained at the event handle location */
    uint8_t data_len;                   /** Length of data array */
} mesh_dfu_event_t;

void mesh_dfu_packet_handler(rbc_mesh_event_t * p_evt);
