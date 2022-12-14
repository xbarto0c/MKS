/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Adam Dunkels <adam@sics.se>
 *
 */


#include "lwip/opt.h"

#if LWIP_NETCONN

#include "lwip/sys.h"
#include "lwip/api.h"
#include "string.h"

#define telnet_THREAD_PRIO  ( tskIDLE_PRIORITY + 4 )
#define CMD_BUFFER_LEN 512 // length of the support buffer

char s[CMD_BUFFER_LEN];

static void telnet_process_command(char *cmd, struct netconn *conn)
{

	sprintf(s, "Prijato: %s\r\n", cmd); // sends and processes data \n is for the Termite to show us the data
	char *token;
	token = strtok(cmd, " "); // strtok -> cmd is a pointer to the beginning of the buffer, second parameter is a string with a list of words separating parameters,
	// if it fins the parameter, it return a pointer to the word before the parameter

	if (strcasecmp(token, "HELLO") == 0) // compares a string with another string, case insensitive
	{
		sprintf(s, "Komunikace OK\r\n");
	}
	else if (strcasecmp(token, "LED1") == 0)
	{
		token = strtok(NULL, " "); // if the first parameter is NULL, continue, where left off the last time (to return the next string before the separating parameter)
		if (strcasecmp(token, "ON") == 0) HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 1); // if the string says "led1 on", turn it on, if it says led1 "off", turn it off
		else if (strcasecmp(token, "OFF") == 0)
		{
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);
			sprintf(s, "OK\r\n");
		}
	}
	else if (strcasecmp(token, "LED2") == 0)
	{
		token = strtok(NULL, " "); // if the first parameter is NULL, continue, where left off the last time (to return the next string before the separating parameter)
		if (strcasecmp(token, "ON") == 0) HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1); // if the string says "led2 on", turn it on, if it says led2 "off", turn it off
		else if (strcasecmp(token, "OFF") == 0)
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
			sprintf(s, "OK\r\n");
		}
	}
	else if (strcasecmp(token, "LED3") == 0)
	{
		token = strtok(NULL, " "); // if the first parameter is NULL, continue, where left off the last time (to return the next string before the separating parameter)
		if (strcasecmp(token, "ON") == 0) HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1); // if the string says "led2 on", turn it on, if it says led2 "off", turn it off
		else if (strcasecmp(token, "OFF") == 0)
		{
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
			sprintf(s, "OK\r\n");
		}
	}
	else s = "";
	netconn_write(conn, s, strlen(s), NETCONN_COPY);
	if (strcasecmp(token, "STATUS") == 0) // if the string says "status", return the status of both leds
	{
		if(HAL_GPIO_ReadPin(LD1_GPIO_Port, LD1_Pin)) sprintf(s, "LED1 is turned on\r\n");
		else sprintf(s, "LED1 is turned off\r\n");
		netconn_write(conn, s, strlen(s), NETCONN_COPY);

		if(HAL_GPIO_ReadPin(LD2_GPIO_Port, LD2_Pin)) sprintf(s, "LED2 is turned on\r\n");
		else sprintf(s, "LED2 is turned off\r\n");
		netconn_write(conn, s, strlen(s), NETCONN_COPY);

		if(HAL_GPIO_ReadPin(LD3_GPIO_Port, LD3_Pin)) sprintf(s, "LED3 is turned on\r\n");
		else sprintf(s, "LED3 is turned off\r\n");
		netconn_write(conn, s, strlen(s), NETCONN_COPY);
	}
}

static void telnet_byte_available(uint8_t c, struct netconn *conn)
{
	static uint16_t cnt;
	static char data[CMD_BUFFER_LEN];
	if (cnt < CMD_BUFFER_LEN && c >= 32 && c <= 127) data[cnt++] = c;
	if (c == '\n' || c == '\r') {
		data[cnt] = '\0';
		telnet_process_command(data, conn);
		cnt = 0;
	}
}
/*-----------------------------------------------------------------------------------*/
static void telnet_thread(void *arg)
{
	struct netconn *conn, *newconn;
	err_t err, accept_err;
	struct netbuf *buf;
	uint8_t *data;
	u16_t len;

	LWIP_UNUSED_ARG(arg);

	/* Create a new connection identifier. */
	conn = netconn_new(NETCONN_TCP);

	if (conn!=NULL)
	{
		/* Bind connection to well known port number 23, which is common for telnet. */
		err = netconn_bind(conn, NULL, 23);

		if (err == ERR_OK)
		{
			/* Tell connection to go into listening mode. */
			netconn_listen(conn);

			while (1)
			{
				/* Grab new connection. */
				accept_err = netconn_accept(conn, &newconn);

				/* Process the new connection. */
				if (accept_err == ERR_OK)
				{

					while (netconn_recv(newconn, &buf) == ERR_OK)
					{
						netbuf_data(buf, (void**)&data, &len);
						while (len--) telnet_byte_available(*data++, newconn);
						netbuf_delete(buf);
					}

					/* Close connection and discard connection identifier. */
					netconn_close(newconn);
					netconn_delete(newconn);
				}
			}
		}
		else
		{
			netconn_delete(newconn);
		}
	}
}
/*-----------------------------------------------------------------------------------*/

void telnet_init(void)
{
	sys_thread_new("telnet_thread", telnet_thread, NULL, DEFAULT_THREAD_STACKSIZE, telnet_THREAD_PRIO);
}
/*-----------------------------------------------------------------------------------*/



#endif /* LWIP_NETCONN */
