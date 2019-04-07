#ifndef __CONTIKI_CONF_H__CDBB4VIH3I__
#define __CONTIKI_CONF_H__CDBB4VIH3I__

#include <stdint.h>
#include "main.h"


#define CCIF
#define CLIF

#define WITH_UIP 1
#define WITH_ASCII 1

#define CLOCK_CONF_SECOND    100

typedef unsigned int clock_time_t;
typedef unsigned int uip_stats_t;

#ifndef BV
#define BV(x) (1<<(x))
#endif

/* Size of LoRa FIFO is 256 bytes. */
#define PACKETBUF_CONF_SIZE    256

#define AUTOSTART_ENABLE    1 /* Enable Auto Start Processes */

/* Select protocols for network stack */
#define NETSTACK_CONF_RADIO    sx1278_radio_driver
#define NETSTACK_CONF_RDC    cxmac_driver
#define NETSTACK_CONF_MAC    csma_driver
#define NETSTACK_CONF_NETWORK    rime_driver
#define NETSTACK_CONF_FRAMER    framer_nullmac

#define QUEUEBUF_CONF_NUM    1

/* uIP configuration */
#define UIP_CONF_LLH_LEN         0
#define UIP_CONF_BROADCAST       1
#define UIP_CONF_LOGGING 1
#define UIP_CONF_BUFFER_SIZE 116

#define UIP_CONF_TCP_FORWARD 1

/* Prefix for relocation sections in ELF files */
#define REL_SECT_PREFIX ".rel"

#define CC_BYTE_ALIGNED __attribute__ ((packed, aligned(1)))

#define USB_EP1_SIZE 64
#define USB_EP2_SIZE 64

#define RAND_MAX 0x7fff

/* Set for CX-MAC */
#define WITH_ACK_OPTIMIZATION    0
#define WITH_ENCOUNTER_OPTIMIZATION    0
#define WITH_STREAMING    0
#define WITH_STROBE_BROADCAST    1

/* Enable energy estimation */
#define ENERGEST_CONF_ON    0

/* LoRa start-newwork: 0xFFFF=broadcast, 0xFFFE=sink, 0x0001~0xFFFD=node, 0x0000=null. */
#define RIMEADDR_CONF_SIZE    2
#define RIMEADDR_SINK    0xFE
#define RIMEADDR_BROADCAST    0xFF


#endif /* __CONTIKI_CONF_H__CDBB4VIH3I__ */

