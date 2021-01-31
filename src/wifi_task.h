/******************************************************************************
 * File Name:   wifi_task.h
 *
 * Description: This file is the public interface of wifi_task.c source file.
 *
 *******************************************************************************
 * Created by:  Bastiaan Slee
 *****************************************​**************************************/

/*******************************************************************************
 *  Include guard
 ******************************************************************************/
#ifndef WIFI_TASK_H_
#define WIFI_TASK_H_

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/*******************************************************************************
 * Global constants
 *******************************************************************************/
/* Wi-Fi Credentials: Modify WIFI_SSID, WIFI_PASSWORD and WIFI_SECURITY_TYPE
 * to match your Wi-Fi network credentials.
 * Note: Maximum length of the Wi-Fi SSID and password is set to
 * CY_WCM_MAX_SSID_LEN and CY_WCM_MAX_PASSPHRASE_LEN as defined in cy_wcm.h file.
 */
#define WIFI_SSID                         "YOUR_SSID"
#define WIFI_PASSWORD                     "YOUR_PASSWORD"

/* Security type of the Wi-Fi access point. See 'cy_wcm_security_t' structure
 * in "cy_wcm.h" for more details. */
#define WIFI_SECURITY_TYPE                 CY_WCM_SECURITY_WPA2_AES_PSK

/* Maximum number of connection retries to the Wi-Fi network. */
#define MAX_WIFI_CONN_RETRIES             (20u)

/* Wi-Fi re-connection time interval in milliseconds */
#define WIFI_CONN_RETRY_INTERVAL_MSEC     (100u)

#define MAKE_IPV4_ADDRESS(a, b, c, d)     ((((uint32_t) d) << 24) | \
                                          (((uint32_t) c) << 16) | \
                                          (((uint32_t) b) << 8) |\
                                          ((uint32_t) a))

/* The size of the cy_wcm_ip_address_t array that is passed to
 * cy_wcm_get_ip_addr API. In the case of stand-alone AP or STA mode, the size of
 * the array is 1. In concurrent AP/STA mode, the size of the array is 2 where
 * the first index stores the IP address of the STA and the second index
 * stores the IP address of the AP.
 */
#define SIZE_OF_IP_ARRAY_STA              (1u)

/* Change the server IP address to match the UDP server address (IP address
 * of the PC).
 */
#define NTP_SERVER_IP_ADDRESS             MAKE_IPV4_ADDRESS(132, 163, 97, 6)
#define NTP_SERVER_PORT                   (123)

#define NTP_TIMESTAMP_DELTA 			  2208988800ull

/* Buffer size to store the incoming messages from server, in bytes. */
#define MAX_UDP_RECV_BUFFER_SIZE          (100)

#define LI(packet)   (uint8_t) ((packet.li_vn_mode & 0xC0) >> 6) // (li   & 11 000 000) >> 6
#define VN(packet)   (uint8_t) ((packet.li_vn_mode & 0x38) >> 3) // (vn   & 00 111 000) >> 3
#define MODE(packet) (uint8_t) ((packet.li_vn_mode & 0x07) >> 0) // (mode & 00 000 111) >> 0

#define RTOS_TASK_TICKS_TO_WAIT           (1000)

/*******************************************************************************
 * Data structure and enumeration
 ******************************************************************************/

// Structure that defines the 48 byte NTP packet protocol.
typedef struct {
	uint8_t li_vn_mode;      // Eight bits. li, vn, and mode.
							 // li.   Two bits.   Leap indicator.
							 // vn.   Three bits. Version number of the protocol.
							 // mode. Three bits. Client will pick mode 3 for client.

	uint8_t stratum;         // Eight bits. Stratum level of the local clock.
	uint8_t poll;   // Eight bits. Maximum interval between successive messages.
	uint8_t precision;       // Eight bits. Precision of the local clock.

	uint32_t rootDelay;      // 32 bits. Total round trip delay time.
	uint32_t rootDispersion; // 32 bits. Max error aloud from primary clock source.
	uint32_t refId;          // 32 bits. Reference clock identifier.

	uint32_t refTm_s;        // 32 bits. Reference time-stamp seconds.
	uint32_t refTm_f;     // 32 bits. Reference time-stamp fraction of a second.

	uint32_t origTm_s;       // 32 bits. Originate time-stamp seconds.
	uint32_t origTm_f;    // 32 bits. Originate time-stamp fraction of a second.

	uint32_t rxTm_s;         // 32 bits. Received time-stamp seconds.
	uint32_t rxTm_f;       // 32 bits. Received time-stamp fraction of a second.

	uint32_t txTm_s; // 32 bits and the most important field the client cares about. Transmit time-stamp seconds.
	uint32_t txTm_f;       // 32 bits. Transmit time-stamp fraction of a second.

} ntp_packet;              // Total: 384 bits or 48 bytes.

/*******************************************************************************
 * Function Prototype
 ********************************************************************************/
void wifi_task(void *arg);

#endif /* WIFI_TASK_H_ */

/* END OF FILE [] */
