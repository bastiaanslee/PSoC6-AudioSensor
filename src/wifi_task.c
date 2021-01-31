/******************************************************************************
 * File Name:   wifi_task.c
 *
 * Description: This file contains task and functions related to WIFI operation.
 *
 *******************************************************************************
 * Created by:  Bastiaan Slee
 *****************************************​**************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cy_retarget_io.h"
#include "string.h"
#include "time.h"
#include "cy_wcm.h" /* WiFi connection manager header files. */
#include "cy_wcm_error.h" /* WiFi connection manager header files. */
#include "cy_secure_sockets.h" /* Cypress secure socket header file. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* My related tasks */
#include "wifi_task.h"

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
cy_rslt_t wifi_connect_to_ap(void);
void wifi_network_event_callback(cy_wcm_event_t event,
		cy_wcm_event_data_t *event_data);

cy_rslt_t ntp_get_time(void);
static cy_rslt_t ntp_client_recv_handler(cy_socket_t socket_handle, void *arg);

uint32_t ntohl(uint32_t const net);

/*******************************************************************************
 * Global Variables
 ********************************************************************************/
TaskHandle_t xHandle_Audio;
TaskHandle_t xHandle_Influx;
TaskHandle_t xHandle_CapSense;
TaskHandle_t xHandle_WIFI;

/* UDP client socket handle */
cy_socket_t udp_client_handle;
cy_socket_sockaddr_t peer_addr;

// Create and zero out the packet. All 48 bytes worth.
ntp_packet packet = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

/* Flag to indicate that message is received from the TCP server. */
bool received_time_from_server = false;

/* The RTC object */
cyhal_rtc_t rtc_obj;

/*******************************************************************************
 * Function Name: wifi_task
 *******************************************************************************
 * Summary:
 *  Task used to establish a connection to a remote TCP server and
 *  control the LED state (ON/OFF) based on the command received from TCP server.
 *
 * Parameters:
 *  void *args : Task parameter defined during task creation (unused).
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void wifi_task(void *arg) {
	/* wait a little bit to give the other tasks time to suspend on their own */
	vTaskDelay(pdMS_TO_TICKS(1000u));

	/* Obtain the handle of a task from its name and resume the tasks. */
	xHandle_Audio = xTaskGetHandle("Audio Task");
	xHandle_Influx = xTaskGetHandle("Influx Task");
	xHandle_CapSense = xTaskGetHandle("CapSense Task");
	xHandle_WIFI = xTaskGetHandle("WIFI Task");

	/* "Repeatedly" running part of the task; after 1 run it does suspend, but a resume could wake it up, suspending other tasks temporarily */
	for (;;) {

		/* As Wifi needs full system resources to successfully start, suspend all other tasks for a moment. */
		vTaskSuspend(xHandle_Audio);
		vTaskSuspend(xHandle_Influx);
		vTaskSuspend(xHandle_CapSense);

		vTaskDelay(pdMS_TO_TICKS(1000u));

		/* Connect to WiFi AP */
		if (wifi_connect_to_ap() != CY_RSLT_SUCCESS) {
			printf("\033[91mWIFI: Failed to connect to AP.\033[m\n");
			CY_ASSERT(0);
		}

		/* Get current time from NTP server and store in RTC */
		if (ntp_get_time() != CY_RSLT_SUCCESS) {
			printf("\033[91mNTP: Failed to get current time.\033[m\n");
			CY_ASSERT(0);
		}

		printf("\n");

		/* Wifi and NTP/RTC completed, now resume the tasks. */
		vTaskResume(xHandle_CapSense);
		vTaskResume(xHandle_Influx);

		vTaskDelay(pdMS_TO_TICKS(1000u));
		vTaskResume(xHandle_Audio);

		/* Wifi task can suspend now, waiting for a renewed connection attempt */
		vTaskSuspend(xHandle_WIFI);
	}

}

/*******************************************************************************
 * Function Name: wifi_connect_to_ap()
 *******************************************************************************
 * Summary:
 *  Connects to WiFi AP using the user-configured credentials, retries up to a
 *  configured number of times until the connection succeeds.
 *
 *******************************************************************************/
cy_rslt_t wifi_connect_to_ap(void) {
	/* Variables used by WiFi connection manager.*/
	cy_wcm_connect_params_t wifi_conn_param;
	cy_wcm_config_t wifi_config = { .interface = CY_WCM_INTERFACE_TYPE_STA };
	cy_wcm_ip_address_t ip_address;
	cy_rslt_t result;

	/* Initialize WiFi connection manager. */
	result = cy_wcm_init(&wifi_config);
	if (result != CY_RSLT_SUCCESS) {
		printf("\033[91mWIFI: Connection Manager initialization failed!\033[m\n");
		return result;
	}

	printf("\n");

	/* Set the WiFi SSID, password and security type. */
	memset(&wifi_conn_param, 0, sizeof(cy_wcm_connect_params_t));
	memset(&ip_address, 0, sizeof(cy_wcm_ip_address_t));
	memcpy(wifi_conn_param.ap_credentials.SSID, WIFI_SSID, sizeof(WIFI_SSID));
	memcpy(wifi_conn_param.ap_credentials.password, WIFI_PASSWORD,
			sizeof(WIFI_PASSWORD));
	wifi_conn_param.ap_credentials.security = WIFI_SECURITY_TYPE;

	/* Join the WiFi AP. */
	for (uint32_t conn_retries = 0; conn_retries < MAX_WIFI_CONN_RETRIES;
			conn_retries++) {
		result = cy_wcm_connect_ap(&wifi_conn_param, &ip_address);

		if (result == CY_RSLT_SUCCESS) {
			printf("\033[94mWIFI: Successfully connected to network '%s'.\033[m\n",
					wifi_conn_param.ap_credentials.SSID);

			/* Callback event that shows disconnects and reconnects. */
			cy_wcm_register_event_callback(wifi_network_event_callback);

			/* We are successfully connected, exit the function with a success message */
			return CY_RSLT_SUCCESS;
		}

		printf("\033[91mWIFI: Connection to network failed with error code %d. Retrying in %d ms...\033[m\n",
				(int) result, WIFI_CONN_RETRY_INTERVAL_MSEC);

		vTaskDelay(pdMS_TO_TICKS(WIFI_CONN_RETRY_INTERVAL_MSEC));
	}

	/* Stop retrying after maximum retry attempts. */
	printf("\033[91mWIFI: Exceeded maximum connection attempts.\033[m\n");

	return CY_RSLT_MW_ERROR;
}

/*******************************************************************************
 * Function Name: wifi_network_event_callback()
 *******************************************************************************
 * Summary:
 *  Network event callback function
 *
 *******************************************************************************/
void wifi_network_event_callback(cy_wcm_event_t event,
		cy_wcm_event_data_t *event_data) {
	if (event == CY_WCM_EVENT_DISCONNECTED) {
		printf("\033[94mWIFI: Disconnected from network.\033[m\n");

		/* We lost connection :( Try to create a new connection by running the WIFI task again */
		vTaskResume(xHandle_WIFI);
	}

	else if (event == CY_WCM_EVENT_RECONNECTED) {
		printf("\033[94mWIFI: Reconnected to network.\033[m\n");
	}

	/* This event corresponds to the event when the IP address of the device changes. */
	else if (event == CY_WCM_EVENT_IP_CHANGED) {
		cy_wcm_ip_address_t ip_addr;
		cy_wcm_get_ip_addr(CY_WCM_INTERFACE_TYPE_STA, &ip_addr,
				SIZE_OF_IP_ARRAY_STA);
		if (event_data->ip_addr.version == CY_WCM_IP_VER_V4) {
			printf("\033[94mWIFI: IP Address Assigned: %d.%d.%d.%d\033[m\n",
					(uint8_t) event_data->ip_addr.ip.v4,
					(uint8_t) (event_data->ip_addr.ip.v4 >> 8),
					(uint8_t) (event_data->ip_addr.ip.v4 >> 16),
					(uint8_t) (event_data->ip_addr.ip.v4 >> 24));
		} else if (event_data->ip_addr.version == CY_WCM_IP_VER_V6) {
			printf("\033[94mWIFI: IP Address Assigned: %lX:%lX:%lX:%lX\033[m\n",
					event_data->ip_addr.ip.v6[0],
					(event_data->ip_addr.ip.v6[1]),
					(event_data->ip_addr.ip.v6[2]),
					(event_data->ip_addr.ip.v6[3]));
		}
	}
}

/*******************************************************************************
 * Function Name: ntp_get_time
 *******************************************************************************
 * Summary:
 *  Function to create an UDP socket and send a request to the NTP server.
 *  NTP requests are handled via UDP protocol on port 123
 *
 * Parameters:
 *  none
 *
 * Return:
 *  cy_result result: Result of the operation
 *
 *******************************************************************************/
cy_rslt_t ntp_get_time(void) {
	cy_rslt_t result;

	/* Variable to store the number of bytes sent to the UDP server. */
	uint32_t bytes_sent = 0;

	/* To hold the RTC time */
	struct tm date_time;

	/* IP address and UDP port number of the UDP server */
	cy_socket_sockaddr_t udp_server_addr = { .ip_address.ip.v4 =
			NTP_SERVER_IP_ADDRESS, .ip_address.version = CY_SOCKET_IP_VER_V4,
			.port = NTP_SERVER_PORT };

	/* Prepare the NTP packet that we will send:
	 * Set the first byte's bits to 01,100,011;
	 * LI = 1 (01) (Leap Indicator "last minute of the day has 61 seconds", ignored in request)
	 * VN = 4 (100) (Version Number)
	 * Mode = 3 (011) (Mode = 3: client)
	 */
	memset(&packet, 0, sizeof(ntp_packet));
	*((char *) &packet + 0) = 0b01100011;

	/* Sockets initialization */
	result = cy_socket_init();
	if (result != CY_RSLT_SUCCESS) {
		printf("\033[91mUDP: Sockets initialization failed!\033[m\n");
		return result;
	}

	/* Create a UDP socket. */
	result = cy_socket_create(CY_SOCKET_DOMAIN_AF_INET, CY_SOCKET_TYPE_DGRAM,
			CY_SOCKET_IPPROTO_UDP, &udp_client_handle);
	if (result != CY_RSLT_SUCCESS) {
		printf("\033[91mUDP: Client Socket creation failed!\033[m\n");
		return result;
	}

	/* Variable used to set socket receive callback function. */
	cy_socket_opt_callback_t udp_recv_option = { .callback =
			ntp_client_recv_handler, .arg = NULL };

	/* Register the callback function to handle messages received from UDP client. */
	result = cy_socket_setsockopt(udp_client_handle, CY_SOCKET_SOL_SOCKET,
			CY_SOCKET_SO_RECEIVE_CALLBACK, &udp_recv_option,
			sizeof(cy_socket_opt_callback_t));
	if (result != CY_RSLT_SUCCESS) {
		printf("\033[91mUDP: Registring receive callback failed!\033[m\n");
		return result;
	}

	/* Send NTP data to Server */
	result = cy_socket_sendto(udp_client_handle, (char*) &packet,
			sizeof(ntp_packet), CY_SOCKET_FLAGS_NONE, &udp_server_addr,
			sizeof(cy_socket_sockaddr_t), &bytes_sent);
	if (result != CY_RSLT_SUCCESS) {
		printf("\033[91mUDP: Failed to send data to server. Error : %ld\033[m\n",
				result);
		return result;
	}

	/* When we are here, we did successfully send a message to the NTP server. Now we wait for a returning message */
	for (;;) {
		if (received_time_from_server) {
			/* Return message is received and time is extracted, so jump out of the FOR loop */
			break;
		}
		vTaskDelay(RTOS_TASK_TICKS_TO_WAIT);
	}

	/* Read the time stored in the RTC */
	result = cyhal_rtc_read(&rtc_obj, &date_time);
	if (CY_RSLT_SUCCESS != result) {
		printf("\033[91mRTC: Failed to read.\033[m\n");
		return result;
	}
	time_t t_of_day = mktime(&date_time);
	printf("\033[94mRTC: %s\033[m", asctime(&date_time));
	printf("\033[94mRTC: seconds since the Epoch: %ld\033[m\n",
			(long) t_of_day);

	/* All done for the UDP/NTP/RTC functions! */

	/* Disconnect the UDP client. */
	cy_socket_disconnect(udp_client_handle, 0);

	/* Free the resources allocated to the socket. */
	cy_socket_delete(udp_client_handle);

	return CY_RSLT_SUCCESS;

}

/*******************************************************************************
 * Function Name: ntp_client_recv_handler
 *******************************************************************************
 * Summary:
 *  Callback function to handle incoming UDP server messages. Translated the
 *  received package from NTP protocol into a readable time. And then store the
 *  time on the RTC.
 *
 * Parameters:
 *  cy_socket_t socket_handle: Connection handle for the UDP client socket
 *  void *args : Parameter passed on to the function (unused)
 *
 * Return:
 *  cy_result result: Result of the operation
 *
 *******************************************************************************/
cy_rslt_t ntp_client_recv_handler(cy_socket_t udp_client_handle, void *arg) {
	cy_rslt_t result;

	/* Variable to store the number of bytes received. */
	uint32_t bytes_received = 0;

	/* To hold the NTP time */
	struct tm * new_time = { 0 };

	/* Receive incoming message from UDP server. */
	result = cy_socket_recvfrom(udp_client_handle, &packet,
			MAX_UDP_RECV_BUFFER_SIZE,
			CY_SOCKET_FLAGS_RECVFROM_NONE, NULL, 0, &bytes_received);

	if (bytes_received < 0) {
		printf("\033[91mUDP: error reading from socket\033[m\n");
		return result;
	}

	// These two fields contain the time-stamp seconds as the packet left the NTP server.
	// The number of seconds correspond to the seconds passed since 1900.
	// ntohl() converts the bit/byte order from the network's to host's "endianness".
	packet.txTm_s = ntohl(packet.txTm_s); // Time-stamp seconds.
	packet.txTm_f = ntohl(packet.txTm_f); // Time-stamp fraction of a second.

	// Extract the 32 bits that represent the time-stamp seconds (since NTP epoch) from when the packet left the server.
	// Subtract 70 years worth of seconds from the seconds since 1900.
	// This leaves the seconds since the UNIX epoch of 1970.
	// (1900)------------------(1970)**************************************(Time Packet Left the Server)
	time_t txTm = (time_t) (packet.txTm_s - NTP_TIMESTAMP_DELTA);

	// Print the time we got from the server, in UTC time.
	printf("\033[94mNTP: time received: %s\033[m",
			ctime((const time_t*) &txTm));

	/* Initialize RTC */
	result = cyhal_rtc_init(&rtc_obj);
	if (CY_RSLT_SUCCESS != result) {
		printf("\033[91mRTC: Failed to init.\033[m\n");
		return result;
	}

	new_time = gmtime(&txTm);

	/* Write the time to the RTC */
	result = cyhal_rtc_write(&rtc_obj, new_time);
	if (CY_RSLT_SUCCESS != result) {
		printf("\033[91mRTC: Writing time failed.\033[m\n");
		return result;
	}

	printf("\033[94mRTC: Time updated.\033[m\n");
	received_time_from_server = true;

	return CY_RSLT_SUCCESS;
}

/*******************************************************************************
 * Function Name: ntohl()
 *******************************************************************************
 * Summary:
 *  Function to convert the unsigned integer netlong from network byte order to
 *  host byte order
 *
 *******************************************************************************/
uint32_t ntohl(uint32_t const net) {
	uint8_t data[4] = { };
	memcpy(&data, &net, sizeof(data));

	return ((uint32_t) data[3] << 0) | ((uint32_t) data[2] << 8)
			| ((uint32_t) data[1] << 16) | ((uint32_t) data[0] << 24);
}

/* [] END OF FILE */
