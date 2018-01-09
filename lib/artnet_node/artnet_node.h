/***************************************************************************
*
* Title          : Arduino ArtNet Node
* Version        : v1
* Last updated   : 16.05.2015
* Web            : https://newfangled.me, https://alexforey.com
* Target         : Arduino Mega 2560, Arduino Mega 1280, Arduino Uno

*** Arduino IDE v0023 MUST BE USED ***

* Based on code from
  * Toni Merino, http://www.deskontrol.net/blog  merino.toni@gmail.com
  * Christoph Guillermet, http://www.le-chat-noir-numerique.fr  karistouf@yahoo.fr

* Structures and definitions (common.h and packet.h) from libartnet (c)Simon Newton and Lutz Hillebrand (ilLUTZminator), www.opendmx.net
*
* Art-Net™ Designed by and Copyright Artistic Licence Holdings Ltd.
*
***************************************************************************
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 as published by the Free Software Foundation; either version2 of
 the License, or (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 General Public License for more details.

 If you have no copy of the GNU General Public License, write to the
 Free Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 For other license models, please contact the author.

;***************************************************************************/
#ifndef ARTNET_NODE_H
#define ARTNET_HODE_H

#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "common.h"
#include "packets.h"

#define ARTNET_PORT           0x1936
#define MAX_BUFFER_UDP        1650

#define short_get_high_byte(x)((HIGH_BYTE & x) >> 8)
#define short_get_low_byte(x)(LOW_BYTE & x)
#define bytes_to_short(high, low)( ((high << 8) & 0xFF00) | (low & 0x00FF) );

uint8_t* get_packet_buffer();
int get_max_buffer_size();
void handle_packet();
void fill_art_node(uint8_t* mac, uint32_t* ip, uint32_t* gateway, uint32_t* subnetmask);
void fill_art_poll_reply(artnet_reply_t *poll_reply);

void set_poll_callback(void (*ptr)(artnet_poll_t *packetBuffer));
void set_dmx_callback(void (*ptr)(artnet_dmx_t *packetBuffer));

struct artnet_node_s {
  uint8_t  id           [8];
  uint16_t opCode;
  uint8_t  localIp      [4];
  uint16_t localPort;
  uint8_t  verH;
  uint8_t  ver;
  uint8_t  subH;
  uint8_t  sub;
  uint8_t  oemH;
  uint8_t  oem;
  uint8_t  ubea;
  uint8_t  status;
  uint8_t  etsaman      [2];
  uint8_t  shortname    [ARTNET_SHORT_NAME_LENGTH];
  uint8_t  longname     [ARTNET_LONG_NAME_LENGTH];
  uint8_t  nodereport   [ARTNET_REPORT_LENGTH];
  uint8_t  numbportsH;
  uint8_t  numbports;
  uint8_t  porttypes    [ARTNET_MAX_PORTS];
  uint8_t  goodinput    [ARTNET_MAX_PORTS];
  uint8_t  goodoutput   [ARTNET_MAX_PORTS];
  uint8_t  swin         [ARTNET_MAX_PORTS];
  uint8_t  swout        [ARTNET_MAX_PORTS];
  uint8_t  swvideo;
  uint8_t  swmacro;
  uint8_t  swremote;
  uint8_t  style;
  uint8_t  remoteIp     [4];
  uint16_t remotePort;
  uint8_t  broadcastIp  [4];
  uint8_t  gateway      [4];
  uint8_t  subnetMask   [4];
  uint8_t  mac          [6];
  uint8_t  ProVerH;
  uint8_t  ProVer;
  uint8_t  ttm;
} __attribute__((packed));

typedef struct artnet_node_s artnet_node_t;

#endif
