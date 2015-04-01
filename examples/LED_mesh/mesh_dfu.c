#include "dfu_types.h"
#include <stddef.h>
#include <string.h>
#include "boards.h"
#include "nrf51.h"
#include "nrf_sdm.h"
#include "nrf_gpio.h"
#include "app_util.h"
#include "app_error.h"
#include "softdevice_handler.h"
#include "ble_hci.h"
#include "nordic_common.h"
#include "app_timer.h"
#include "ble_conn_params.h"
#include "hci_mem_pool.h"
#include "bootloader.h"
#include "pstorage.h"
#include "dfu_ble_svc_internal.h"
#include "nrf_delay.h"
#include "rbc_mesh.h"
#include "mesh_dfu.h"
#include "led_config.h"

static mesh_state_t                 m_mesh_state;                /**< Current DFU state. */
static uint32_t                     m_image_size;               /**< Size of the image that will be transmitted. */

static dfu_start_packet_t           m_start_packet;             /**< Start packet received for this update procedure. Contains update mode and image sizes information to be used for image transfer. */
static uint8_t                      m_init_packet[64];          /**< Init packet, can hold CRC, Hash, Signed Hash and similar, for image validation, integrety check and authorization checking. */ 
static uint8_t                      m_init_packet_length;       /**< Length of init packet received. */
static uint16_t                     m_image_crc;                /**< Calculated CRC of the image received. */
static uint32_t             				m_num_of_firmware_bytes_rcvd; 
static uint32_t                     m_data_received;

static pstorage_handle_t            m_storage_handle_app; 
static pstorage_handle_t            m_storage_handle_swap;      /**< Pstorage handle for the swap area (bank 1). Bank used when updating an application or bootloader without SoftDevice. */
static pstorage_handle_t          * mp_storage_handle_active;   /**< Pointer to the pstorage handle for the active bank for receiving of data packets. */

static uint16_t m_addr;
static uint8_t m_channel;
static uint8_t m_memory_clear = false;
static uint8_t m_block_write_progress = false;

static uint8_t * mp_rx_buffer;

/**@brief     Function for handling the callback events from the dfu module.
 *            Callbacks are expected when \ref dfu_data_pkt_handle has been executed.
 *
 * @param[in] packet    Packet type for which this callback is related. 
 * @param[in] result    Operation result code. NRF_SUCCESS when a queued operation was successful.
 * @param[in] p_data    Pointer to the data to which the operation is related.
 */
// static void mesh_callback_handler(uint32_t packet, uint32_t result, uint8_t * p_data)
// {
//     switch (packet)
//     {
//         uint32_t           err_code;

//         case DATA_PACKET:
//             if (result != NRF_SUCCESS)
//             {
//                 // ERROR CASE
//             }
//             else
//             {
//                 // Final stage in the circular buffer, frees the pointer
//                 err_code = hci_mem_pool_rx_consume(p_data);
//                 APP_ERROR_CHECK(err_code);
//             }
//             break;
        
//         case START_PACKET:
//             // Send procedure start message
//             break;
        
//         default:
//             // ignore.
//             break;
//     }
// }


// /**@brief     Function for processing start data written by the peer to the DFU Packet
//  *            Characteristic.
//  *
//  * @param[in] p_dfu     DFU Service Structure.
//  * @param[in] p_evt     Pointer to the event received from the S110 SoftDevice.
//  */
// static void mesh_start_download_process()
// {
//     uint32_t err_code;

//     // Extract the size of from the DFU Packet Characteristic.
//     uint8_t * length_data = p_evt->evt.ble_dfu_pkt_write.p_data;

    
//     err_code = mesh_start_packet_handler(&update_packet);
//     if (err_code != NRF_SUCCESS)
//     {
//         // Translate the err_code returned by the above function to DFU Response Value.
//     }
// }


// /**@brief     Function for processing initialization data written by the peer to the DFU Packet
//  *            Characteristic.
//  *
//  * @param[in] p_dfu     DFU Service Structure.
//  * @param[in] p_evt     Pointer to the event received from the S110 SoftDevice.
//  */
// static void init_data_process(ble_dfu_t * p_dfu, ble_dfu_evt_t * p_evt)
// {
//     uint32_t            err_code;
//     dfu_update_packet_t dfu_pkt;
//     uint8_t             num_of_padding_bytes = 0;    

//     dfu_pkt.packet_type   = INIT_PACKET;

//     dfu_pkt.params.data_packet.p_data_packet = (uint32_t *)p_evt->evt.ble_dfu_pkt_write.p_data;

//     // The DFU module accepts the dfu_pkt.packet_length to be in 'number of words'. And so if the
//     // received data does not have a size which is a multiple of four, it should be padded with
//     // zeros and the packet_length should be incremented accordingly before calling
//     // dfu_init_pkt_handle.
//     if ((p_evt->evt.ble_dfu_pkt_write.len & (sizeof(uint32_t) - 1)) != 0)
//     {
//          // Find out the number of bytes to be padded.
//          num_of_padding_bytes = sizeof(uint32_t)
//                                 -
//                                 (p_evt->evt.ble_dfu_pkt_write.len & (sizeof(uint32_t) - 1));

//          uint32_t i;
//          for (i = 0; i < num_of_padding_bytes; i++)
//          {
//              dfu_pkt.params.data_packet.p_data_packet[p_evt->evt.ble_dfu_pkt_write.len + i] = 0;
//          }
//     }

//     dfu_pkt.params.data_packet.packet_length = 
//         (p_evt->evt.ble_dfu_pkt_write.len + num_of_padding_bytes) / sizeof(uint32_t);

//     // Handle the packet
//     err_code = dfu_init_pkt_handle(&dfu_pkt);

//     // Translate the err_code returned by the above function to DFU Response Value.
//     if (err_code != NRF_SUCCESS)
//     {
//         ble_dfu_resp_val_t resp_val = nrf_err_code_translate(err_code, BLE_DFU_INIT_PROCEDURE);

//         err_code = ble_dfu_response_send(p_dfu, BLE_DFU_INIT_PROCEDURE, resp_val);
//         APP_ERROR_CHECK(err_code);
//     }
// }


// /**@brief     Function for processing application data written by the peer to the DFU Packet
//  *            Characteristic.
//  *
//  * @param[in] p_dfu     DFU Service Structure.
//  * @param[in] p_evt     Pointer to the event received from the S110 SoftDevice.
//  */
static void app_data_process(int addr, uint8_t packet_id, uint8_t * p_data_packet, uint32_t length) {
    uint32_t err_code;

    if ((length & (sizeof(uint32_t) - 1)) != 0) {
        // Data length is not a multiple of 4 (word size).
        return;
    }
    
    err_code = hci_mem_pool_rx_produce(length, (void**) &mp_rx_buffer);
    if (err_code != NRF_SUCCESS) {
        return;
    }

    memcpy(mp_rx_buffer, p_data_packet, length);

    err_code = hci_mem_pool_rx_data_size_set(length);
    if (err_code != NRF_SUCCESS) {
        return;
    }

		err_code = hci_mem_pool_rx_extract(&mp_rx_buffer, &length);
		if (err_code != NRF_SUCCESS) {
			return;
    }

    mesh_update_packet_t mesh_pkt;

		mesh_pkt.packet_type                      = DATA_PACKET;
		mesh_pkt.params.data_packet.packet_length = length / sizeof(uint32_t);
		mesh_pkt.params.data_packet.p_data_packet = (uint32_t*)mp_rx_buffer;
    
    err_code = mesh_data_pkt_handle(&mesh_pkt);

    if (err_code == NRF_SUCCESS)
    {
        // All the expected firmware data has been received and processed successfully.
        m_num_of_firmware_bytes_rcvd += length;

				mesh_dfu_send_response(packet_id, (unsigned short)MESH_IMAGE_TRANSFER_SUCCESS, addr);
     }
     else if (err_code == NRF_ERROR_INVALID_LENGTH)
     {
         // Firmware data packet was handled successfully. And more firmware data is expected.
         m_num_of_firmware_bytes_rcvd += length;

         mesh_dfu_send_response(packet_id, (unsigned short)MESH_DATA_IMAGE_PACKET_ACK, addr);
    }
    else
    {
				//Let's pretend errors don't exist for the time being
        uint32_t hci_error = hci_mem_pool_rx_consume(mp_rx_buffer);
        if (hci_error != NRF_SUCCESS)
        {
					// Process error
        }
    }
}

uint32_t mesh_data_pkt_handle(mesh_update_packet_t * p_packet)
{
    uint32_t   data_length;
    uint32_t   err_code;
    uint32_t * p_data;

    if (p_packet == NULL)
    {
        return NRF_ERROR_NULL;
    }

    // Check pointer alignment.
    if (!is_word_aligned(p_packet->params.data_packet.p_data_packet))
    {
        // The p_data_packet is not word aligned address.
        return NRF_ERROR_INVALID_ADDR;
    }

    switch (m_mesh_state)
    {
        case MESH_STATE_RDY:
        case MESH_STATE_RX_INIT_PKT:
            return NRF_ERROR_INVALID_STATE;

        case MESH_STATE_RX_DATA_PKT:
            data_length = p_packet->params.data_packet.packet_length * sizeof(uint32_t);

            if ((m_data_received + data_length) > m_image_size)
            {
                // The caller is trying to write more bytes into the flash than the size provided to
                // the dfu_image_size_set function. This is treated as a serious error condition and
                // an unrecoverable one. Hence point the variable mp_app_write_address to the top of
                // the flash area. This will ensure that all future application data packet writes
                // will be blocked because of the above check.
                m_data_received = 0xFFFFFFFF;

                return NRF_ERROR_DATA_SIZE;
            }

            // Valid peer activity detected. Hence restart the DFU timer.
            //err_code = dfu_timer_restart();
            //if (err_code != NRF_SUCCESS)
            //{
            //    return err_code;
            //}

            p_data = (uint32_t *)p_packet->params.data_packet.p_data_packet;

						pstorage_handle_t * p_dest = mp_storage_handle_active;
            //err_code = pstorage_raw_store(mp_storage_handle_active,
            //                              ,
            //                              data_length,
            //                              m_data_received);
						uint32_t star_addr = 0x00029000;
						
						err_code = sd_flash_write((uint32_t *)star_addr, (uint32_t *)p_data, data_length);

            if (err_code != NRF_SUCCESS)
            {
                return err_code;
            }
						
            m_data_received += data_length;

            if (m_data_received != m_image_size)
            {
                // The entire image is not received yet. More data is expected.
                err_code = NRF_ERROR_INVALID_LENGTH;
            }
            else
            {
                // The entire image has been received. Return NRF_SUCCESS.
                err_code = NRF_SUCCESS;
            }
            break;

        default:
            err_code = NRF_ERROR_INVALID_STATE;
            break;
    }

    return err_code;
}

static void rbc_event_to_mesh_packet(mesh_dfu_event_t * packet, rbc_mesh_event_t * p_evt) 
{
				//unsigned long mask;
				//mask = ((1 << 28) - 1) << 27;
	
				//uint32_t masked = *(uint32_t *)p_evt->data & mask;
				//masked = masked >> 27;
	
				//if (strstr("wtf lad", reinterpret_cast<const char*>(p_evt->data)) != NULL) {
				//	unsigned lol = 0;
				//}
				//unsigned isolatedXbits = p_evt->data & mask;
				unsigned short one = *p_evt->data;
				one = one & 0xF8;
				one = one >> 3;
	
				uint16_t t_addr = ((uint16_t) *(p_evt->data+1) << 8) | *(p_evt->data+2);
				
        packet->event_type = one;
				packet->target_address = t_addr;
        packet->data_len = p_evt->data_len;
        packet->value_handle = p_evt->value_handle;
        packet->data = p_evt->data;
}

void mesh_dfu_send_response(uint8_t channel, char one, int addr)//, uint16_t addr)
{
	one = one << 3;
	uint8_t resp[28];
	
	resp[0] = one;
	resp[1] = (addr >> 8);
	resp[2] = addr;
	
	for (int i = 3; i < 28; i++) {
			resp[i] = 0xFF;
	}
	
	rbc_mesh_value_set(channel, resp, 28);
}

static void pstorage_callback_handler(pstorage_handle_t * p_handle,
                                      uint8_t             op_code,
                                      uint32_t            result,
                                      uint8_t           * p_data,
                                      uint32_t            data_len)
{
			switch (op_code)
			{
					case PSTORAGE_STORE_OP_CODE:
							m_block_write_progress = false;
							//if(result == NRF_SUCCESS) {
								mesh_dfu_send_response(m_channel, (unsigned short)MESH_DATA_IMAGE_PACKET_ACK, m_addr);
								int err = hci_mem_pool_rx_consume(p_data);
								APP_ERROR_CHECK(err);
							//}
							
							break;

					case PSTORAGE_CLEAR_OP_CODE:
							//if (result == NRF_SUCCESS) {
								mesh_dfu_send_response(m_channel, (unsigned short)MESH_START_IMAGE_TRANSFER_ACK, m_addr);
							//} else {
							//	led_config(3, 1);
							//}
							m_memory_clear = false;
							break;

					default:
							break;
			}
}
	
/**@brief     Function for the Device Firmware Update Service event handler.
 *
 * @details   This function will be called for all Device Firmware Update Service events which
 *            are passed to the application.
 *
 * @param[in] p_dfu     Device Firmware Update Service structure.
 * @param[in] p_evt     Event received from the Device Firmware Update Service.
 */
void mesh_dfu_packet_handler(rbc_mesh_event_t * p_evt)
{
    uint32_t           err_code;
		mesh_dfu_event_t mesh_packet;

		rbc_event_to_mesh_packet(&mesh_packet, p_evt);
	
		ble_gap_addr_t my_addr;
    sd_ble_gap_address_get(&my_addr);
    int addr = ((uint16_t) my_addr.addr[4] << 8) | (my_addr.addr[5]);
	
		if (mesh_packet.target_address != addr) {
			return;
		}
		
		m_addr = mesh_packet.target_address;
		m_channel = p_evt->value_handle;

    switch (mesh_packet.event_type)
    {
        case MESH_NOP:
            // No operation, channel unused.
            break;
        case MESH_CONNECTION_REQUEST:
            // Initialise connection
            // Confirmation connection: MESH_CONNECTION_REQUEST_ACK
					led_config(1, 1);
				  mesh_dfu_send_response(p_evt->value_handle, (unsigned short)MESH_CONNECTION_REQUEST_ACK, mesh_packet.target_address);

					break;
        case MESH_IMAGE_TRANSFER_SUCCESS:

            // Validate image
            //err_code = dfu_image_validate();

            // Send Ack
            // MESH_IMAGE_TRANSFER_SUCCESS_ACK
            break;

        case MESH_IMAGE_ACTIVATE:
            // Send ACK
            // MESH_IMAGE_ACTIVATE_ACK

            // Switch to bootloader & request BANK swap

            break;
        case MESH_DISCONNECT_SERVER:
            // Error condition, server disconnecting from mesh imminently.
						led_config(1, 0);
            break;

        case MESH_REQUEST_STATUS:
            // Server interrogation, return software information & whatever else if needed
            // MESH_REQUEST_STATUS_ACK
            break;
        case MESH_START_IMAGE_TRANSFER:
            // Prepare for the transfer of the new firmware
            // MESH_START_IMAGE_TRANSFER_ACK
				
						if (m_memory_clear) 
							return;
								
						// Total Image size to be recieved
						m_image_size = ((uint32_t) *(mesh_packet.data+3) << 24) | ((uint32_t) *(mesh_packet.data+4) << 16) | ((uint32_t) *(mesh_packet.data+5) << 8) | ((uint32_t) *(mesh_packet.data+6));
				
						pstorage_module_param_t storage_module_param = {.cb = pstorage_callback_handler};
						err_code = pstorage_raw_register(&storage_module_param, &m_storage_handle_app);
						APP_ERROR_CHECK(err_code);
						
						// Init image transfer variables
						m_storage_handle_app.block_id = DFU_BANK_0_REGION_START;
						m_storage_handle_swap = m_storage_handle_app;
						m_storage_handle_swap.block_id = DFU_BANK_1_REGION_START;
						m_mesh_state = MESH_STATE_RX_DATA_PKT;
						
						mp_storage_handle_active = &m_storage_handle_swap;
						
						//err_code = pstorage_raw_clear(&m_storage_handle_swap, DFU_IMAGE_MAX_SIZE_BANKED);
						//APP_ERROR_CHECK(err_code);
						m_memory_clear = false;// = true;
						mesh_dfu_send_response(m_channel, (unsigned short)MESH_START_IMAGE_TRANSFER_ACK, m_addr);
						
            break;
        case MESH_DATA_IMAGE_PACKET:
            // Acknowledge & write recieved packet
            // MESH_DATA_IMAGE_PACKET_ACK
						if (m_block_write_progress) {
							return;
						}

						// Pretend processing
						mesh_dfu_send_response(p_evt->value_handle, (unsigned short)MESH_DATA_IMAGE_PACKET_ACK, mesh_packet.target_address);			
						m_block_write_progress = false;
						//app_data_process(mesh_packet.target_address, (uint8_t)p_evt->value_handle, (uint8_t *) (mesh_packet.data+3), 16);
				
            break;

        default:
            // Unsupported event or client side event.
            break;
    }
}
