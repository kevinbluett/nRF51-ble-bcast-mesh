#include "dfu_transport.h"
#include "dfu.h"
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
#include "ble_stack_handler_types.h"
#include "ble_advdata.h"
#include "ble_l2cap.h"
#include "ble_gap.h"
#include "ble_gatt.h"
#include "ble_hci.h"
#include "ble_dfu.h"
#include "nordic_common.h"
#include "app_timer.h"
#include "ble_conn_params.h"
#include "hci_mem_pool.h"
#include "bootloader.h"
#include "dfu_ble_svc_internal.h"
#include "nrf_delay.h"
#include "rbc_mesh.h"
#include "mesh_dfu.h"
#include "led_config.h"


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
// static void app_data_process(ble_dfu_t * p_dfu, ble_dfu_evt_t * p_evt)
// {
//     uint32_t err_code;

//     if ((p_evt->evt.ble_dfu_pkt_write.len & (sizeof(uint32_t) - 1)) != 0)
//     {
//         // Data length is not a multiple of 4 (word size).
//         return;
//     }

//     uint32_t length = p_evt->evt.ble_dfu_pkt_write.len;
    
//     err_code = hci_mem_pool_rx_produce(length, (void**) &mp_rx_buffer);
//     if (err_code != NRF_SUCCESS)
//     {
//         return;
//     }
    
//     uint8_t * p_data_packet = p_evt->evt.ble_dfu_pkt_write.p_data;

//     memcpy(mp_rx_buffer, p_data_packet, length);

//     err_code = hci_mem_pool_rx_data_size_set(length);
//     if (err_code != NRF_SUCCESS)
//     {
//         return;
//     }

//     err_code = hci_mem_pool_rx_extract(&mp_rx_buffer, &length);
//     if (err_code != NRF_SUCCESS)
//     {
//         return;
//     }

//     dfu_update_packet_t dfu_pkt;

//     dfu_pkt.packet_type                      = DATA_PACKET;
//     dfu_pkt.params.data_packet.packet_length = length / sizeof(uint32_t);
//     dfu_pkt.params.data_packet.p_data_packet = (uint32_t*)mp_rx_buffer;
    
//     err_code = dfu_data_pkt_handle(&dfu_pkt);

//     if (err_code == NRF_SUCCESS)
//     {
//         // All the expected firmware data has been received and processed successfully.

//         m_num_of_firmware_bytes_rcvd += p_evt->evt.ble_dfu_pkt_write.len;

//         // Notify the DFU Controller about the success about the procedure.
//         err_code = ble_dfu_response_send(p_dfu,
//                                          BLE_DFU_RECEIVE_APP_PROCEDURE,
//                                          BLE_DFU_RESP_VAL_SUCCESS);
//         APP_ERROR_CHECK(err_code);
//     }
//     else if (err_code == NRF_ERROR_INVALID_LENGTH)
//     {
//         // Firmware data packet was handled successfully. And more firmware data is expected.
//         m_num_of_firmware_bytes_rcvd += p_evt->evt.ble_dfu_pkt_write.len;

//         // Check if a packet receipt notification is needed to be sent.
//         if (m_pkt_rcpt_notif_enabled)
//         {
//             // Decrement the counter for the number firmware packets needed for sending the
//             // next packet receipt notification.
//             m_pkt_notif_target_cnt--;

//             if (m_pkt_notif_target_cnt == 0)
//             {
//                 err_code = ble_dfu_pkts_rcpt_notify(p_dfu, m_num_of_firmware_bytes_rcvd);
//                 APP_ERROR_CHECK(err_code);

//                 // Reset the counter for the number of firmware packets.
//                 m_pkt_notif_target_cnt = m_pkt_notif_target;
//             }
//         }
//     }
//     else
//     {
//         uint32_t hci_error = hci_mem_pool_rx_consume(mp_rx_buffer);
//         if (hci_error != NRF_SUCCESS)
//         {
//             dfu_error_notify(p_dfu, hci_error);
//         }
        
//         dfu_error_notify(p_dfu, err_code);
//     }
// }

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

void mesh_dfu_send_response(char one)
{
	one = one << 3;
	uint8_t* resp;
	*resp = one;
	rbc_mesh_value_set(1, resp, 0);
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
    ble_dfu_resp_val_t resp_val;
		mesh_dfu_event_t mesh_packet;

		rbc_event_to_mesh_packet(&mesh_packet, p_evt);
	
		ble_gap_addr_t my_addr;
    sd_ble_gap_address_get(&my_addr);
    int addr = ((uint16_t) my_addr.addr[4] << 8) | (my_addr.addr[5]);
	
		if (mesh_packet.target_address != addr) {
			return;
		}

    switch (mesh_packet.event_type)
    {
        case MESH_NOP:
            // No operation, channel unused.
            break;
        case MESH_CONNECTION_REQUEST:
            // Initialise connection
            // Confirmation connection: MESH_CONNECTION_REQUEST_ACK
					led_config(2, 1);
				  //mesh_dfu_send_response((unsigned short)MESH_CONNECTION_REQUEST_ACK);
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
            break;

        case MESH_REQUEST_STATUS:
            // Server interrogation, return software information & whatever else if needed
            // MESH_REQUEST_STATUS_ACK
            break;
        case MESH_START_IMAGE_TRANSFER:
            // Prepare for the transfer of the new firmware
            // MESH_START_IMAGE_TRANSFER_ACK

            // Will recieve CRC & Size information here
            break;
        case MESH_DATA_IMAGE_PACKET:
            // Acknowledge & write recieved packet
            // MESH_DATA_IMAGE_PACKET_ACK
            break;

        default:
            // Unsupported event or client side event.
            break;
    }
}
