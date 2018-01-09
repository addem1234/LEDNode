/***************************************************************************
*
* Title          : Arduino ArtNet Node
* Version        : v1.1
* Last updated   : 18.05.2015
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

#include "artnet_node.h"

/************************************
        ArtNet Definitions
************************************/

artnet_node_t             art_node;
artnet_packet_type_t      packet_type;

void (*artnet_dmx_callback)(artnet_dmx_t *packetBuffer);
void (*artnet_poll_callback)(artnet_poll_t *packetBuffer);

uint8_t packetBuffer[MAX_BUFFER_UDP]; // buffer to store incoming UDP data

uint8_t* get_packet_buffer() {
  return packetBuffer;
}

int get_max_buffer_size() {
  return MAX_BUFFER_UDP;
}

void set_poll_callback(void (*ptr)(artnet_poll_t *packetBuffer)) {
  artnet_poll_callback = ptr;
}

void set_dmx_callback(void (*ptr)(artnet_dmx_t *packetBuffer)) {
  artnet_dmx_callback = ptr;
}

/************************************
    ArtNet Function Definitions
************************************/

// Find out the ArtNet packet type
uint16_t get_packet_type(uint8_t *packet) {
  if (!memcmp(packet, art_node.id, sizeof(art_node.id))) {
    return bytes_to_short(packet[9], packet[8]);
  }
  return 0;  // bad packet
}

void handle_packet()
{
  packet_type = (artnet_packet_type_t) get_packet_type((uint8_t *)&packetBuffer);

  // Ignore bad packets
  if(packet_type == 0) {
    return;
  }

  // Handle DMX data packets
  if(packet_type == ARTNET_DMX) {
    if (sizeof(packetBuffer) < sizeof(artnet_dmx_t)) {
      return;
    } else if(artnet_dmx_callback) {
      artnet_dmx_callback((artnet_dmx_t*) &packetBuffer);
    }
  }

  else if(packet_type == ARTNET_POLL) {
    if(sizeof(packetBuffer) < sizeof(artnet_poll_t))
      return;
    else if(artnet_poll_callback) {
      artnet_poll_callback((artnet_poll_t*) &packetBuffer);
    }
  }
}

void fill_art_node(uint8_t* mac, uint32_t* ip, uint32_t* gateway, uint32_t* subnetmask)
{
  //fill to 0's
  memset (&art_node, 0, sizeof(art_node));

  //fill data
  memcpy(art_node.mac, mac, 6);                   // the mac address of node
  memcpy(art_node.localIp, ip, 4);                // the IP address of node
  memcpy(art_node.gateway, gateway, 4);           // gateway IP address
  memcpy(art_node.subnetMask, subnetmask, 4);     // network mask (art-net use 'A' network type)
  
  art_node.localPort = ARTNET_PORT;               // artnet UDP port is always 6454 (0x1936)

  sprintf((char*) &art_node.id, "Art-Net");
  sprintf((char*) &art_node.shortname, "CUBE");
  sprintf((char*) &art_node.longname, "Art-Net LED CUBE 10x10");

  art_node.numbports = 6;
  art_node.numbportsH = 0;

  for(int i = 0; i < 4; i++) {
    art_node.swout[i] = 0;
    art_node.swin[i] = 0;
    art_node.goodoutput[i] = 0x80;
  }

  memset(&art_node.porttypes,  0x80, 4);
  memset(&art_node.goodinput,  0x08, 4);

  art_node.subH           = 0x00;        // high byte of the Node Subnet Address (This field is currently unused and set to zero. It is
                                         // provided to allow future expansion.) (art-net III)
  art_node.sub            = 0x00;        // low byte of the Node Subnet Address


  art_node.etsaman[0] = 0;        // The ESTA manufacturer code.
  art_node.etsaman[1] = 0;        // The ESTA manufacturer code.
  art_node.verH       = 1;        // high byte of Node firmware revision number.
  art_node.ver        = 1;        // low byte of Node firmware revision number.
  art_node.ProVerH    = 0;        // high byte of the Art-Net protocol revision number.
  art_node.ProVer     = 14;       // low byte of the Art-Net protocol revision number.
  art_node.oemH       = 0;        // high byte of the oem value.
  art_node.oem        = 0xFF;     // low byte of the oem value. (0x00FF = developer code)
  art_node.ubea       = 0;        // This field contains the firmware version of the User Bios Extension Area (UBEA). 0 if not used
  art_node.status     = 0x08;
  art_node.swvideo    = 0;
  art_node.swmacro    = 0;
  art_node.swremote   = 0;
  art_node.style      = 0;        // StNode style - A DMX to/from Art-Net device
}

void fill_art_poll_reply(artnet_reply_t *poll_reply)
{
  //copy data from node
  memcpy(poll_reply->id, &art_node.id, sizeof(poll_reply->id));
  memcpy(poll_reply->ip, &art_node.localIp, sizeof(poll_reply->ip));
  memcpy(poll_reply->mac, &art_node.mac, sizeof(poll_reply->mac));
  memcpy(poll_reply->shortname, &art_node.shortname, sizeof(poll_reply->shortname));
  memcpy(poll_reply->longname, &art_node.longname, sizeof(poll_reply->longname));
  memcpy(poll_reply->porttypes, &art_node.porttypes, sizeof(poll_reply->porttypes));
  memcpy(poll_reply->goodinput, &art_node.goodinput, sizeof(poll_reply->goodinput));
  memcpy(poll_reply->goodoutput, &art_node.goodoutput, sizeof(poll_reply->goodoutput));
  memcpy(poll_reply->swin, &art_node.swin, sizeof(poll_reply->swin));
  memcpy(poll_reply->swout, &art_node.swout, sizeof(poll_reply->swout));
  memcpy(poll_reply->etsaman, &art_node.etsaman, sizeof(poll_reply->etsaman));

  sprintf((char*) poll_reply->nodereport, "%i DMX output universes active.\0", &art_node.numbports);

  poll_reply->opCode          = ARTNET_REPLY;  // ARTNET_REPLY
  poll_reply->port            = art_node.localPort;
  poll_reply->verH            = art_node.verH;
  poll_reply->ver             = art_node.ver;
  poll_reply->subH            = art_node.subH;
  poll_reply->sub             = art_node.sub;
  poll_reply->oemH            = art_node.oemH;
  poll_reply->oem             = art_node.oem;
  poll_reply->status          = art_node.status;
  poll_reply->numbportsH      = art_node.numbportsH;
  poll_reply->numbports       = art_node.numbports;
  poll_reply->swvideo         = art_node.swvideo;
  poll_reply->swmacro         = art_node.swmacro;
  poll_reply->swremote        = art_node.swremote;
  poll_reply->style           = art_node.style;
}

