/** @file ssi_comms.c */

/*==========================================================
 * Copyright 2021 QuickLogic Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *==========================================================*/
#include "app_config.h"
#if SENSIML_SIMPLE_STREAM_BUILD
#include <inttypes.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "ssi_comms.h"

static uint32_t ssi_conn_seqnum[SSI_MAX_CHANNELS] = {0};
#define CONNECT_STRING "connect"
#define CONNECT_CHARS 7
#define DISCONNECT_STRING "disconnect"
#define DISCONNECT_CHARS 10
static int chars_read;

static char connection_buf[DISCONNECT_CHARS];

static ssi_io_funcs_t ssi_interface;

void ssi_init(bool (*read)(void*, const size_t),  bool (*write)(void*, const size_t))
{
    ssi_interface.ssi_read = read;
    ssi_interface.ssi_write = write;
    ssi_interface.initialized = true;
    ssi_interface.connected = false;
}

bool ssi_connected(void)
{
    return ssi_interface.initialized && ssi_interface.connected;
}

void ssi_try_connect(void)
{
    chars_read = 0;
    memset(connection_buf, 0,DISCONNECT_CHARS );
    while(chars_read < CONNECT_CHARS)
    {
        SERCOM5_USART_Read(&connection_buf[chars_read], 1);
        chars_read++;
    }
    if(strncmp(connection_buf, CONNECT_STRING, CONNECT_CHARS) == 0)
    {
         ssi_interface.connected = true;
    }
}

void ssi_seqnum_init(uint8_t channel)
{
    if (channel >= SSI_MAX_CHANNELS)
      return;
    ssi_conn_seqnum[channel] = 0;
}

void ssi_seqnum_reset(uint8_t channel)
{
    if (channel >= SSI_MAX_CHANNELS)
      return;
    ssi_conn_seqnum[channel] = 0;
}

uint32_t ssi_seqnum_update(uint8_t channel)
{
    if (channel >= SSI_MAX_CHANNELS)
      return 0;
    ssi_conn_seqnum[channel]++;
    return ssi_conn_seqnum[channel];
}

uint32_t ssi_seqnum_get(uint8_t channel)
{
    if (channel >= SSI_MAX_CHANNELS)
      return 0;
    return ssi_conn_seqnum[channel];
}

uint8_t ssi_payload_checksum_get(uint8_t *p_data, uint16_t len)
{
    uint8_t crc8 = p_data[0];
    for (int k = 1; k < len; k++)
    {
        crc8 ^= p_data[k];
    }
    return crc8;
}

void ssiv2_publish_sensor_data(uint8_t channel, uint8_t* buffer, int size)
{
    if(ssi_interface.initialized == false)
    {
        return;
    }
    uint8_t ssiv2header[SSI_HEADER_SIZE];
    uint8_t sync = SSI_SYNC_DATA;
    uint8_t rsvd = 0;
    uint16_t u16len = (size + 6);
    uint32_t seqnum = ssi_seqnum_update(channel);
    uint8_t crc8 = 0;

    ssiv2header[0] = sync;
    ssiv2header[1] = (u16len >> 0) & 0xff;
    ssiv2header[2] = (u16len >> 8) & 0xff;
    ssiv2header[3] = rsvd;
    ssiv2header[4] = channel;
    ssiv2header[5] = (seqnum >> 0) & 0xff;
    ssiv2header[6] = (seqnum >> 8) & 0xff;
    ssiv2header[7] = (seqnum >> 16) & 0xff;
    ssiv2header[8] = (seqnum >> 24) & 0xff;

    // compute 8-bit checksum
    crc8 = crc8 ^ ssi_payload_checksum_get(ssiv2header+3, SSI_HEADER_SIZE-3);
    crc8 = crc8 ^ ssi_payload_checksum_get(buffer, size);

    // Send SSI v2 header information
    ssi_interface.ssi_write(ssiv2header, SSI_HEADER_SIZE);

    // Send sensor data
    ssi_interface.ssi_write(buffer, size);

    // Add 8-bit checksum
    ssi_interface.ssi_write(&crc8, 1);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

void ssiv1_publish_sensor_data(uint8_t* buffer, int size)
{
    if(ssi_interface.initialized == false)
    {
        return;
    }
    ssi_interface.ssi_write(buffer, size);
}
#pragma GCC diagnostic pop

#endif //SENSIML_SIMPLE_STREAM_BUILD
