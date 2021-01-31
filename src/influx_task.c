/******************************************************************************
 * File Name:   influx_task.c
 *
 * Description: This file contains task and functions related to TCP client
 * operation, which is used for the InfluxDB API connection.
 *
 *******************************************************************************
 * Created by:  Bastiaan Slee
 *****************************************​**************************************/

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cy_retarget_io.h"
#include "string.h"
#include "cy_secure_sockets.h" /* Cypress secure socket header file. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* My related tasks */
#include "influx_task.h"

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
cy_rslt_t tcp_send_to_server(cy_socket_sockaddr_t tcp_server_address,
		char tx_buffer[(10 * 1024)]);

/*******************************************************************************
 * Global Variables
 ********************************************************************************/
/* TCP client socket handle */
cy_socket_t tcp_socket_handle;

QueueHandle_t influx_command_data_q;

/* Buffer to hold the data to be sent to the client. */
char tx_buffer[(16 * 1024)];
char tx_body[(15 * 1024)];

/*******************************************************************************
 * Function Name: influx_task
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
void influx_task(void *arg) {
	cy_rslt_t result;

	/* Immediately suspend the task to give WIFI connection priority */
	vTaskSuspend( NULL);
	printf("\033[94mInflux: task started!\033[m\n");

	BaseType_t rtos_api_result;
	influx_command_data_t influx_cmd_data;

	/* IP address and TCP port number of the TCP server to which the TCP client connects to. */
	cy_socket_sockaddr_t tcp_server_address = { .ip_address.ip.v4 =
			TCP_SERVER_IP_ADDRESS, .ip_address.version = CY_SOCKET_IP_VER_V4,
			.port = TCP_SERVER_PORT };

	tx_buffer[0] = '\0';
	tx_body[0] = '\0';

	/* Empty the queue before starting */
	xQueueReset(influx_command_data_q);

	/* Set the start of the loop to -4, as first couple of audio readings have an invalid data reading */
	int i_upload = -4;

	/* Repeatedly running part of the task */
	for (;;) {
		/* Block until a command has been received over queue */
		rtos_api_result = xQueueReceive(influx_command_data_q, &influx_cmd_data,
				portMAX_DELAY);

		/* Command has been received from queue */
		if (rtos_api_result == pdTRUE) {
			switch (influx_cmd_data.influx_command) {
			/* Check the command and process the steps for it. */
			case INFLUX_DATA: {
				/* InfluxDB line protocol note: https://docs.influxdata.com/influxdb/v2.0/write-data/developer-tools/api/ */
				sprintf(tx_body,
						"%sPSoc6,sensor=audio,weighting=%s tot=%.1f,oct01=%.1f,oct02=%.1f,oct03=%.1f,oct04=%.1f,oct05=%.1f,oct06=%.1f,oct07=%.1f,oct08=%.1f,oct09=%.1f,oct10=%.1f,oct11=%.1f %ld",
						tx_body, influx_cmd_data.audio_weighting,
						influx_cmd_data.audio_volume,
						influx_cmd_data.audio_octave01,
						influx_cmd_data.audio_octave02,
						influx_cmd_data.audio_octave03,
						influx_cmd_data.audio_octave04,
						influx_cmd_data.audio_octave05,
						influx_cmd_data.audio_octave06,
						influx_cmd_data.audio_octave07,
						influx_cmd_data.audio_octave08,
						influx_cmd_data.audio_octave09,
						influx_cmd_data.audio_octave10,
						influx_cmd_data.audio_octave11,
						influx_cmd_data.sec_since_epoch);

				/* At 20, we are going to upload! (2 spectrums per second, so this is 10 seconds of data) */
				if (i_upload == 20) {
					/* InfluxDB header protocol note: https://docs.influxdata.com/influxdb/v2.0/write-data/developer-tools/api/ */
					tx_buffer[0] = '\0';
					sprintf(tx_buffer,
							"POST /api/v2/write?org=%s&bucket=%s&precision=%s HTTP/1.0\r\nAuthorization: Token %s\r\nContent-Type: text/plain\r\nContent-Length: %d\r\n\r\n%s",
							INFLUX_ORGANISATION, INFLUX_BUCKET,
							INFLUX_PRECISSION, INFLUX_TOKEN, strlen(tx_body),
							tx_body);

					result = tcp_send_to_server(tcp_server_address, tx_buffer);
					if (result == CY_RSLT_SUCCESS) {
						tx_body[0] = '\0';
					}

					i_upload = 0;
				}

				/* Above 0, we assume this is just a reading to be added to previous readings */
				else if (i_upload >= 0) {
					sprintf(tx_body, "%s\n", tx_body);
					i_upload++;
				}

				/* Below 0, this is a reading just after start, ignore it! */
				else {
					tx_body[0] = '\0';
					i_upload++;
				}

				break;
			}

			}
		}

		/* delay for a very short time, to let the other tasks go ahead */
		vTaskDelay(pdMS_TO_TICKS(1u));
	}

}

/*******************************************************************************
 * Function Name: tcp_send_to_server
 *******************************************************************************
 * Summary:
 *  Function to connect to TCP server and send the prepared packet.
 *
 * Parameters:
 *  cy_socket_sockaddr_t address: Address of TCP server socket
 *  char tx_buffer: the packet to send
 *
 * Return:
 *  cy_result result: Result of the operation
 *
 *******************************************************************************/
cy_rslt_t tcp_send_to_server(cy_socket_sockaddr_t tcp_server_address,
		char tx_buffer[(10 * 1024)]) {
	cy_rslt_t result = CY_RSLT_MODULE_SECURE_SOCKETS_TIMEOUT;
	cy_rslt_t conn_result;

	/* Variable to store the number of bytes sent to the TCP server. */
	uint32_t bytes_sent = 0;

	for (uint32_t conn_retries = 0; conn_retries < MAX_TCP_SERVER_CONN_RETRIES;
			conn_retries++) {
		/* Initialize the Sockets Library. */
		result = cy_socket_init();
		if (result != CY_RSLT_SUCCESS) {
			printf("\033[91mTCP: Socket init failed! Trying to reconnect...\033[m\n");
			return result;
		}

		/* Create a new TCP socket. */
		result = cy_socket_create(CY_SOCKET_DOMAIN_AF_INET,
				CY_SOCKET_TYPE_STREAM,
				CY_SOCKET_IPPROTO_TCP, &tcp_socket_handle);
		if (result != CY_RSLT_SUCCESS) {
			printf("\033[91mTCP: Failed to create socket! Trying to reconnect...\033[m\n");
			return result;
		}

		/* Set the SEND timeout to 3000 milliseconds */
		result = cy_socket_setsockopt(tcp_socket_handle, CY_SOCKET_SOL_SOCKET,
		CY_SOCKET_SO_SNDTIMEO, 3000, sizeof(3000));
		if (result != CY_RSLT_SUCCESS) {
			printf("\033[91mSet socket option: CY_SOCKET_SO_SNDTIMEO failed. Trying to reconnect...\033[m\n");
			return result;
		}

		/* Set the RECEIVE timeout to 5000 milliseconds */
		result = cy_socket_setsockopt(tcp_socket_handle, CY_SOCKET_SOL_SOCKET,
		CY_SOCKET_SO_RCVTIMEO, 5000, sizeof(5000));
		if (result != CY_RSLT_SUCCESS) {
			printf("\033[91mSet socket option: CY_SOCKET_SO_RCVTIMEO failed. Trying to reconnect...\033[m\n");
			return result;
		}

		/* No need for a receive callback */
		/* No need for a disconnection callback */

		result = cy_socket_connect(tcp_socket_handle, &tcp_server_address,
				sizeof(cy_socket_sockaddr_t));
		if (result == CY_RSLT_SUCCESS) {
			/* Send a message to the TCP server. */
			conn_result = cy_socket_send(tcp_socket_handle, tx_buffer,
					strlen(tx_buffer), CY_SOCKET_FLAGS_NONE, &bytes_sent);
			if (conn_result == CY_RSLT_SUCCESS) {
				printf("\033[32mTCP: sent %ld bytes to InfluxDB server.\033[m\n",
						bytes_sent);
				//printf("%s\n", tx_buffer);
			}

			/* Disconnect the TCP client. */
			cy_socket_disconnect(tcp_socket_handle, 0);

			/* Free the resources allocated to the socket. */
			cy_socket_delete(tcp_socket_handle);

			return conn_result;
		}

		printf("\033[91mTCP: Could not connect to TCP server. Trying to reconnect...\033[m\n");

		/* The resources allocated during the socket creation (cy_socket_create) should be deleted. */
		cy_socket_delete(tcp_socket_handle);
	}

	/* Stop retrying after maximum retry attempts. */
	printf("\033[91mTCP: Exceeded maximum connection attempts to the TCP server. Package is lost...\033[m\n");

	return result;
}

/* [] END OF FILE */
