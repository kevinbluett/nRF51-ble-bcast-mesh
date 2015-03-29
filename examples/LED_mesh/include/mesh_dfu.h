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

typedef enum
{
    MESH_STATE_INIT_ERROR,                                                           /**< State for: dfu_init(...) error. */
    MESH_STATE_IDLE,                                                                 /**< State for: idle. */
    MESH_STATE_PREPARING,                                                            /**< State for: preparing, indicates that the flash is being erased and no data packets can be processed. */
    MESH_STATE_RDY,                                                                  /**< State for: ready. */
    MESH_STATE_RX_INIT_PKT,                                                          /**< State for: receiving initialization packet. */
    MESH_STATE_RX_DATA_PKT,                                                          /**< State for: receiving data packet. */
    MESH_STATE_VALIDATE,                                                             /**< State for: validate. */
    MESH_STATE_WAIT_4_ACTIVATE                                                       /**< State for: waiting for dfu_image_activate(). */
} mesh_state_t;

/**@brief Structure holding a start packet containing update mode and image sizes.
 */
typedef struct
{
    uint8_t  mesh_update_mode;                                                                           /**< Packet type, used to identify the content of the received packet referenced by data packet. */
    uint32_t sd_image_size;                                                                             /**< Size of the SoftDevice image to be transferred. Zero if no SoftDevice image will be transfered. */
    uint32_t bl_image_size;                                                                             /**< Size of the Bootloader image to be transferred. Zero if no Bootloader image will be transfered. */
    uint32_t app_image_size;                                                                            /**< Size of the application image to be transmitted. Zero if no Bootloader image will be transfered. */
} mesh_start_packet_t;

/**@brief Structure holding a bootloader init/data packet received.
 */
typedef struct
{
    uint32_t   packet_length;                                                                           /**< Packet length of the data packet. Each data is word size, meaning length of 4 is 4 words, not bytes. */
    uint32_t * p_data_packet;                                                                           /**< Data Packet received. Each data is a word size entry. */
} mesh_data_packet_t;

typedef struct
{
    uint32_t   packet_type;                                                                             /**< Packet type, used to identify the content of the received packet referenced by data packet. */
    union
    {
        mesh_data_packet_t    data_packet;                                                               /**< Used when packet type is INIT_PACKET or DATA_PACKET. Packet contains data received for init or data. */
        mesh_start_packet_t * start_packet;                                                              /**< Used when packet type is START_DATA_PACKET. Will contain information on software to be updtaed, i.e. SoftDevice, Bootloader and/or Application along with image sizes. */
    } params;
} mesh_update_packet_t;

typedef struct
{
    mesh_client_code event_type;        /** See @ref rbc_mesh_event_type_t */
    mesh_dfu_value_handle_t value_handle;    /** Handle of the value the event is generated for */
	  uint16_t target_address;
    uint8_t* data;                      /** Current data array contained at the event handle location */
    uint8_t data_len;                   /** Length of data array */
} mesh_dfu_event_t;

uint32_t mesh_data_pkt_handle(mesh_update_packet_t * p_packet);
void mesh_dfu_send_response(uint8_t channel, char one, int addr);
void mesh_dfu_packet_handler(rbc_mesh_event_t * p_evt);
