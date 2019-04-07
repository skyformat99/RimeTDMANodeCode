/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         A simple power saving MAC protocol based on X-MAC [SenSys 2006]
 * \author
 *         Adam Dunkels <adam@sics.se>
 *         Niclas Finne <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 */

#include "dev/leds.h"
#include "dev/radio.h"
#include "dev/watchdog.h"
#include "net/netstack.h"
#include "lib/random.h"
#include "net/mac/cxmac.h"
#include "net/rime.h"
#include "net/rime/timesynch.h"
#include "sys/compower.h"
#include "sys/pt.h"
#include "sys/rtimer.h"

#include "contiki-conf.h"
#include "Network.h"

#ifdef EXPERIMENT_SETUP
#include "experiment-setup.h"
#endif

#include <string.h>

#ifndef WITH_ACK_OPTIMIZATION
#define WITH_ACK_OPTIMIZATION        1
#endif
#ifndef WITH_ENCOUNTER_OPTIMIZATION
#define WITH_ENCOUNTER_OPTIMIZATION  1
#endif
#ifndef WITH_STREAMING
#define WITH_STREAMING               1
#endif
#ifndef WITH_STROBE_BROADCAST
#define WITH_STROBE_BROADCAST        0
#endif

struct announcement_data {
  uint16_t id;
  uint16_t value;
};

/* The maximum number of announcements in a single announcement
   message - may need to be increased in the future. */
#define ANNOUNCEMENT_MAX 10

/* The structure of the announcement messages. */
struct announcement_msg {
  uint16_t num;
  struct announcement_data data[ANNOUNCEMENT_MAX];
};

/* The length of the header of the announcement message, i.e., the
   "num" field in the struct. */
#define ANNOUNCEMENT_MSG_HEADERLEN (sizeof (uint16_t))

#define DISPATCH          0
#define TYPE_STROBE       0x10
/* #define TYPE_DATA         0x11 */
#define TYPE_ANNOUNCEMENT 0x12
#define TYPE_STROBE_ACK   0x13

struct cxmac_hdr {
  uint8_t type;
};

#define MAX_STROBE_SIZE 4

#define DEFAULT_PERIOD (cxmac_time.off_time + cxmac_time.on_time)

/* Delay between RX strobe and TX strobe ack is 600us */
#define WAIT_TIME_BEFORE_STROBE_ACK (0ul * RTIMER_ARCH_SECOND / 1000) /* 0 ms */

/* The cycle time for announcements. */
#define ANNOUNCEMENT_PERIOD 4 * CLOCK_SECOND

/* The time before sending an announcement within one announcement
   cycle. */
#define ANNOUNCEMENT_TIME (random_rand() % (ANNOUNCEMENT_PERIOD))

static struct cxmac_config cxmac_time = {
  .on_time = 41ul * RTIMER_ARCH_SECOND / 1000, /* 41 ms */
  .off_time = 4 * 41ul * RTIMER_ARCH_SECOND / 1000, /* OFF = 4 * ON */
  .strobe_time = (1 + 4) * 41ul * RTIMER_ARCH_SECOND / 1000, /* STROBE = OFF + 1 * ON */
  .strobe_wait_time = 15ul * RTIMER_ARCH_SECOND / 1000, /* 15 ms */
};

#include <stdio.h>

static struct pt pt;

static volatile uint8_t cxmac_is_on = 0;

static volatile unsigned char waiting_for_packet = 0;
static volatile unsigned char someone_is_sending = 0;
static volatile unsigned char we_are_sending = 0;
static volatile unsigned char radio_is_on = 0;

#undef LEDS_ON
#undef LEDS_OFF
#undef LEDS_TOGGLE

#define LEDS_ON(x) leds_on(x)
#define LEDS_OFF(x) leds_off(x)
#define LEDS_TOGGLE(x) leds_toggle(x)

#define DEBUG 1
#if DEBUG
#include <Dbg.h>
#undef PRINTF
#define PRINTF(...)    dbg_PrintfArg(__VA_ARGS__)
#define PRINTDEBUG(...)    dbg_PrintfArg(__VA_ARGS__)
#else
#undef LEDS_ON
#undef LEDS_OFF
#undef LEDS_TOGGLE
#define LEDS_ON(x)
#define LEDS_OFF(x)
#define LEDS_TOGGLE(x)
#define PRINTF(...)
#define PRINTFF(...)
#define PRINTDEBUG(...)
#endif

#if CXMAC_CONF_ANNOUNCEMENTS
/* Timers for keeping track of when to send announcements. */
static struct ctimer announcement_cycle_ctimer, announcement_ctimer;

static int announcement_radio_txpower;
#endif /* CXMAC_CONF_ANNOUNCEMENTS */

/* Flag that is used to keep track of whether or not we are listening
   for announcements from neighbors. */
static uint8_t is_listening;

#if CXMAC_CONF_COMPOWER
static struct compower_activity current_packet;
#endif /* CXMAC_CONF_COMPOWER */

#if WITH_ENCOUNTER_OPTIMIZATION

#include "lib/list.h"
#include "lib/memb.h"

struct encounter {
  struct encounter *next;
  rimeaddr_t neighbor;
  rtimer_clock_t time;
};

#define MAX_ENCOUNTERS 4
LIST(encounter_list);
MEMB(encounter_memb, struct encounter, MAX_ENCOUNTERS);
#endif /* WITH_ENCOUNTER_OPTIMIZATION */

static uint8_t is_streaming;
static rimeaddr_t is_streaming_to, is_streaming_to_too;
static rtimer_clock_t stream_until;
#define DEFAULT_STREAM_TIME (RTIMER_ARCH_SECOND)

#ifndef MIN
#define MIN(a, b) ((a) < (b)? (a) : (b))
#endif /* MIN */

/*---------------------------------------------------------------------------*/
static void
on(void)
{
  if(cxmac_is_on && radio_is_on == 0) {
    radio_is_on = 1;
    NETSTACK_RADIO.on();
    LEDS_ON(LEDS_RED);
    PRINTFF("cxmac: RF on time=%u\r\n", clock_time());
  }
}
/*---------------------------------------------------------------------------*/
static void
off(void)
{
  if(cxmac_is_on && radio_is_on != 0 && is_listening == 0 &&
     is_streaming == 0) {
    radio_is_on = 0;
    NETSTACK_RADIO.off();
    LEDS_OFF(LEDS_RED);
    PRINTFF("cxmac: RF off time=%u\r\n", clock_time());
  }
}
/*---------------------------------------------------------------------------*/
static void
powercycle_turn_radio_off(void)
{
  if(we_are_sending == 0 &&
     waiting_for_packet == 0) {
    off();
  }
#if CXMAC_CONF_COMPOWER
  compower_accumulate(&compower_idle_activity);
#endif /* CXMAC_CONF_COMPOWER */
}
static void
powercycle_turn_radio_on(void)
{
  if(we_are_sending == 0 &&
     waiting_for_packet == 0) {
    on();
  }
}
/*---------------------------------------------------------------------------*/
static struct ctimer cpowercycle_ctimer;
/* EXPLAIN: Add "RTIMER_ARCH_SECOND / 2" for ceil to integer, like as ceil 3.5 to 4. */
#define CSCHEDULE_POWERCYCLE(rtime)    \
    cschedule_powercycle((1ul * CLOCK_SECOND * (rtime) + RTIMER_ARCH_SECOND / 2) / RTIMER_ARCH_SECOND)
static char cpowercycle(void *ptr);
static void
cschedule_powercycle(clock_time_t time)
{

  if(cxmac_is_on) {
    if(time == 0) {
      time = 1;
    }
    ctimer_set(&cpowercycle_ctimer, time,
               (void (*)(void *))cpowercycle, NULL);
  }
}
/*---------------------------------------------------------------------------*/
static char
cpowercycle(void *ptr)
{
  if(is_streaming) {
    if(!RTIMER_CLOCK_LT(RTIMER_NOW(), stream_until)) {
      is_streaming = 0;
      rimeaddr_copy(&is_streaming_to, &rimeaddr_null);
      rimeaddr_copy(&is_streaming_to_too, &rimeaddr_null);
    }
  }

  PT_BEGIN(&pt);

  while(1) {
    /* Only wait for some cycles to pass for someone to start sending */
    if(someone_is_sending > 0) {
      someone_is_sending--;
    }

    /* If there were a strobe in the air, turn radio on */
    powercycle_turn_radio_on();
    CSCHEDULE_POWERCYCLE(cxmac_time.on_time);
    PT_YIELD(&pt);

    if(cxmac_time.off_time > 0) {
      powercycle_turn_radio_off();
      if(waiting_for_packet != 0) {
	waiting_for_packet++;
	if(waiting_for_packet > 3) {
	  /* We should not be awake for more than 3 consecutive
	     power cycles without having heard a packet, so we turn off
	     the radio. */
	  waiting_for_packet = 0;
	  powercycle_turn_radio_off();
	}
      }
      CSCHEDULE_POWERCYCLE(cxmac_time.off_time);
      PT_YIELD(&pt);
    }
  }

  PT_END(&pt);
}
/*---------------------------------------------------------------------------*/
#if CXMAC_CONF_ANNOUNCEMENTS
static int
parse_announcements(const rimeaddr_t *from)
{
  /* Parse incoming announcements */
  struct announcement_msg adata;
  int i;

  memcpy(&adata, packetbuf_dataptr(), MIN(packetbuf_datalen(), sizeof(adata)));

  /*  printf("%d.%d: probe from %d.%d with %d announcements\r\n",
	 rimeaddr_node_addr.u8[0], rimeaddr_node_addr.u8[1],
	 from->u8[0], from->u8[1], adata->num);*/
  /*  for(i = 0; i < packetbuf_datalen(); ++i) {
    printf("%02x ", ((uint8_t *)packetbuf_dataptr())[i]);
  }
  printf("\r\n");*/

  for(i = 0; i < adata.num; ++i) {
    /*   printf("%d.%d: announcement %d: %d\r\n",
	  rimeaddr_node_addr.u8[0], rimeaddr_node_addr.u8[1],
	  adata->data[i].id,
	  adata->data[i].value);*/

    announcement_heard(from,
		       adata.data[i].id,
		       adata.data[i].value);
  }
  return i;
}
/*---------------------------------------------------------------------------*/
static int
format_announcement(char *hdr)
{
  struct announcement_msg adata;
  struct announcement *a;

  /* Construct the announcements */
  /*  adata = (struct announcement_msg *)hdr;*/

  adata.num = 0;
  for(a = announcement_list();
      a != NULL && adata.num < ANNOUNCEMENT_MAX;
      a = list_item_next(a)) {
    adata.data[adata.num].id = a->id;
    adata.data[adata.num].value = a->value;
    adata.num++;
  }

  memcpy(hdr, &adata, sizeof(struct announcement_msg));

  if(adata.num > 0) {
    return ANNOUNCEMENT_MSG_HEADERLEN +
      sizeof(struct announcement_data) * adata.num;
  } else {
    return 0;
  }
}
#endif /* CXMAC_CONF_ANNOUNCEMENTS */
/*---------------------------------------------------------------------------*/
#if WITH_ENCOUNTER_OPTIMIZATION
static void
register_encounter(const rimeaddr_t *neighbor, rtimer_clock_t time)
{
  struct encounter *e;

  /* If we have an entry for this neighbor already, we renew it. */
  for(e = list_head(encounter_list); e != NULL; e = list_item_next(e)) {
    if(rimeaddr_cmp(neighbor, &e->neighbor)) {
      e->time = time;
      break;
    }
  }
  /* No matching encounter was found, so we allocate a new one. */
  if(e == NULL) {
    e = memb_alloc(&encounter_memb);
    if(e == NULL) {
      /* We could not allocate memory for this encounter, so we just drop it. */
      return;
    }
    rimeaddr_copy(&e->neighbor, neighbor);
    e->time = time;
    list_add(encounter_list, e);
  }
}
#endif /* WITH_ENCOUNTER_OPTIMIZATION */

/*---------------------------------------------------------------------------*/
int cxmac_send_packet(const rimeaddr_t *p_tNodeAddr, const void *p_vBuf, uint16_t wSize)
{
  uint8_t    collisions, strobe[MAX_STROBE_SIZE];
  rtimer_clock_t    t, t0, encounter_time, tPrepareRxStart;
  int    strobes, strobe_len, len, got_strobe_ack, is_broadcast, is_already_streaming;
  struct queuebuf    *packet;

#if WITH_ENCOUNTER_OPTIMIZATION
  struct encounter *e;
#endif

  encounter_time = 0;
  got_strobe_ack = 0;
  is_broadcast = 0;
  is_already_streaming = 0;

  /* Copy sent data into packetbuf. */
  packetbuf_clear();
  packetbuf_copyfrom(p_vBuf, wSize);
  packetbuf_set_datalen(wSize);

  packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, p_tNodeAddr);    

  /* Is broadcast? */
  if(rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER), &rimeaddr_broadcast)) {
    is_broadcast = 1;
    PRINTDEBUG("cxmac: send broadcast\r\n");
  }

/* is_reliable = packetbuf_attr(PACKETBUF_ATTR_RELIABLE) ||
    packetbuf_attr(PACKETBUF_ATTR_ERELIABLE);*/
  len = NETSTACK_FRAMER.create();
  strobe_len = len;
  if(len < 0 || strobe_len > (int)sizeof(strobe)) {
    /* Failed to send */
    PRINTF("cxmac: send failed, too large header\r\n");
    return MAC_TX_ERR_FATAL;
  }
  memcpy(strobe, packetbuf_hdrptr(), len);

  packetbuf_compact();
  packet = queuebuf_new_from_packetbuf();
  if(packet == NULL) {
    /* No buffer available */
    PRINTF("cxmac: send failed, no queue buffer available (of %u)\r\n",
           QUEUEBUF_CONF_NUM);
    return MAC_TX_ERR;
  }

#if WITH_STREAMING
  if(is_streaming == 1 &&
     (rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
		   &is_streaming_to) ||
      rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
		   &is_streaming_to_too))) {
    is_already_streaming = 1;
  }
  if(packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE) ==
     PACKETBUF_ATTR_PACKET_TYPE_STREAM) {
    is_streaming = 1;
    if(rimeaddr_cmp(&is_streaming_to, &rimeaddr_null)) {
      rimeaddr_copy(&is_streaming_to, packetbuf_addr(PACKETBUF_ADDR_RECEIVER));
    } else if(!rimeaddr_cmp(&is_streaming_to, packetbuf_addr(PACKETBUF_ADDR_RECEIVER))) {
      rimeaddr_copy(&is_streaming_to_too, packetbuf_addr(PACKETBUF_ADDR_RECEIVER));
    }
    stream_until = RTIMER_NOW() + DEFAULT_STREAM_TIME;
  }
#endif /* WITH_STREAMING */

  off();

#if WITH_ENCOUNTER_OPTIMIZATION
  /* We go through the list of encounters to find if we have recorded
     an encounter with this particular neighbor. If so, we can compute
     the time for the next expected encounter and setup a ctimer to
     switch on the radio just before the encounter. */
  for(e = list_head(encounter_list); e != NULL; e = list_item_next(e)) {
    const rimeaddr_t *neighbor = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);

    if(rimeaddr_cmp(neighbor, &e->neighbor)) {
      rtimer_clock_t wait, now, expected;

      /* We expect encounters to happen every DEFAULT_PERIOD time
	 units. The next expected encounter is at time e->time +
	 DEFAULT_PERIOD. To compute a relative offset, we subtract
	 with clock_time(). Because we are only interested in turning
	 on the radio within the DEFAULT_PERIOD period, we compute the
	 waiting time with modulo DEFAULT_PERIOD. */

      now = RTIMER_NOW();
      wait = ((rtimer_clock_t)(e->time - now)) % (DEFAULT_PERIOD);
      expected = now + wait - 2 * cxmac_time.on_time;

#if WITH_ACK_OPTIMIZATION
      /* Wait until the receiver is expected to be awake */
      if(packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE) !=
	 PACKETBUF_ATTR_PACKET_TYPE_ACK &&
	 is_streaming == 0) {
	/* Do not wait if we are sending an ACK, because then the
	   receiver will already be awake. */
	while(RTIMER_CLOCK_LT(RTIMER_NOW(), expected));
      }
#else /* WITH_ACK_OPTIMIZATION */
      /* Wait until the receiver is expected to be awake */
      while(RTIMER_CLOCK_LT(RTIMER_NOW(), expected));
#endif /* WITH_ACK_OPTIMIZATION */
    }
  }
#endif /* WITH_ENCOUNTER_OPTIMIZATION */

  /* By setting we_are_sending to one, we ensure that the rtimer
     powercycle interrupt do not interfere with us sending the packet. */
  we_are_sending = 1;
  LEDS_ON(LEDS_BLUE);

  /* Send a train of strobes until the receiver answers with an ACK. */
  strobes = collisions = 0;
  if (!is_already_streaming) 
  {
    watchdog_stop();
    t0 = RTIMER_NOW();
    for ( got_strobe_ack = 0;
            got_strobe_ack == 0 && collisions == 0 &&
            RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + cxmac_time.strobe_time);
	     strobes++ )
    {
      /* Send the strobe packet. */
      if (got_strobe_ack == 0 && collisions == 0) 
      {
        if (is_broadcast) 
        {
        #if WITH_STROBE_BROADCAST
          NETSTACK_RADIO.send(strobe, strobe_len);
        #else
          /* restore the packet to send */
          queuebuf_to_packetbuf(packet);
          NETSTACK_RADIO.send(packetbuf_hdrptr(), packetbuf_totlen());
        #endif
          off();
        } 
        else 
        {
          rtimer_clock_t    wt;
          NETSTACK_RADIO.send(strobe, strobe_len);
          tPrepareRxStart = RTIMER_NOW();		  
          /* Turn off the radio for a while to let the other side respond. We don't need to keep our 
               radio on when we know that the other side needs some time to produce a reply. */
          wt = RTIMER_NOW() + WAIT_TIME_BEFORE_STROBE_ACK;
          off();
          while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt)) ;
          on();
        }/*if (is_broadcast)*/
      }/*if (got_strobe_ack == 0 && collisions == 0)*/

      /* Turn on the radio to listen for the strobe ACK. */
      on();
      t = RTIMER_NOW();
      while ( got_strobe_ack == 0 &&
                 RTIMER_CLOCK_LT(RTIMER_NOW(), t + cxmac_time.strobe_wait_time) )
      {
        rtimer_clock_t now = RTIMER_NOW();

        /* See if we got an ACK */
        packetbuf_clear();
        len = NETSTACK_RADIO.read(packetbuf_dataptr(), PACKETBUF_SIZE);
        if(len > 0) 
        {
          packetbuf_set_datalen(len);
          if (NETSTACK_FRAMER.parse() >= 0) 
          {
            if (rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER), p_tNodeAddr)) 
            {
              /* We got an ACK from the receiver, so we can immediately send the packet. */
              got_strobe_ack = 1;
              encounter_time = now;
            } 
            else /*if (rimeaddr_cmp(......))*/ 
            {
              PRINTDEBUG("cxmac: collision, isn't awake ack.\r\n");
              collisions++;
            } /*if (rimeaddr_cmp(......))*/
          }
          else 
          {
            PRINTF("cxmac: send failed to parse %u\r\n", len);
          }/*if (NETSTACK_FRAMER.parse() >= 0)*/
        }/*if(len > 0)*/
      }/*while (...)*/
    }/*for (...)*/
  }/*if (!is_already_streaming)*/

  /* Automatic calculate "PrepareRx" */
  bool    bNeedCalc;
  bNeedCalc = radio_sx1278_NeedCalcPrepareRx();
  if (got_strobe_ack && bNeedCalc)
  {
    radio_sx1278_CalcPrepareRx(tPrepareRxStart, encounter_time);
  }

#if WITH_ACK_OPTIMIZATION
  /* If we have received the strobe ACK, and we are sending a packet
     that will need an upper layer ACK (as signified by the
     PACKETBUF_ATTR_RELIABLE packet attribute), we keep the radio on. */
  if(got_strobe_ack && (packetbuf_attr(PACKETBUF_ATTR_RELIABLE) ||
			packetbuf_attr(PACKETBUF_ATTR_ERELIABLE) ||
			packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE) ==
			PACKETBUF_ATTR_PACKET_TYPE_STREAM)) {
    on(); /* Wait for ACK packet */
    waiting_for_packet = 1;
  } else {
    off();
  }
#else /* WITH_ACK_OPTIMIZATION */
  off();
#endif /* WITH_ACK_OPTIMIZATION */

  /* restore the packet to send */
  queuebuf_to_packetbuf(packet);
  queuebuf_free(packet);

  /* Send the data packet. */
  if((is_broadcast || got_strobe_ack || is_streaming) && collisions == 0) {
    NETSTACK_RADIO.send(packetbuf_hdrptr(), packetbuf_totlen());
  }

#if WITH_ENCOUNTER_OPTIMIZATION
  if(got_strobe_ack && !is_streaming) {
    register_encounter(packetbuf_addr(PACKETBUF_ADDR_RECEIVER), encounter_time);
  }
#endif /* WITH_ENCOUNTER_OPTIMIZATION */
  watchdog_start();

  PRINTF("cxmac: send (strobes=%u,len=%u,%s), time=%u\r\n", strobes,
	 packetbuf_totlen(), got_strobe_ack ? "ack" : "no ack", clock_time());

#if CXMAC_CONF_COMPOWER
  /* Accumulate the power consumption for the packet transmission. */
  compower_accumulate(&current_packet);

  /* Convert the accumulated power consumption for the transmitted
     packet to packet attributes so that the higher levels can keep
     track of the amount of energy spent on transmitting the
     packet. */
  compower_attrconv(&current_packet);

  /* Clear the accumulated power consumption so that it is ready for
     the next packet. */
  compower_clear(&current_packet);
#endif /* CXMAC_CONF_COMPOWER */

  we_are_sending = 0;

  LEDS_OFF(LEDS_BLUE);
  if(collisions == 0) {
    if(!is_broadcast && !got_strobe_ack) {
      return MAC_TX_NOACK;
    } else {
      return MAC_TX_OK;
    }
  } else {
    someone_is_sending++;
    return MAC_TX_COLLISION;
  }
}
/*---------------------------------------------------------------------------*/
static void
qsend_packet(mac_callback_t sent, void *ptr)
{
  int ret;
  if(someone_is_sending) {
    PRINTFF("cxmac: should queue packet, now just dropping %d %d %d %d.\r\n",
	   waiting_for_packet, someone_is_sending, we_are_sending, radio_is_on);
    RIMESTATS_ADD(sendingdrop);
    ret = MAC_TX_COLLISION;
  } else {
    PRINTFF("cxmac: send immediately.\r\n");
//    ret = send_packet();
  }

  mac_call_sent_callback(sent, ptr, ret, 1);
}
/*---------------------------------------------------------------------------*/
static void
qsend_list(mac_callback_t sent, void *ptr, struct rdc_buf_list *buf_list)
{
  if(buf_list != NULL) {
    queuebuf_to_packetbuf(buf_list->buf);
    qsend_packet(sent, ptr);
  }
}

#if 0
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
static void PrintPacketBuf(void)
{
    uint8_t    a_byPrintBuf[4 * 6 + 20], *p_byData;

    p_byData = packetbuf_dataptr();

    #include "stdio.h"
    sprintf( (char *)a_byPrintBuf, "data: %02x, %02x, %02x, %02x, %02x, %02x\r\n",
               p_byData[0], p_byData[1], p_byData[2], p_byData[3], p_byData[4], p_byData[5] );
    PRINTF(a_byPrintBuf, 0, PRINTF_FORMAT_NONE);

    return;
}
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
#endif

void nodeawake_RxAwakeTone(void);
void nodeawake_RxData(void);
void nodeawake_RxNot2Us(void);


/*---------------------------------------------------------------------------*/
static void input_packet(void)
{
  int    iResult;
#if 0
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
  PrintPacketBuf();
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
#endif

  #include "main.h"

  if (NETSTACK_FRAMER.parse() >= 0)
  {
  #if (ROLE_SINK == PRODUCT_ROLE)
    //if (rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER), &rimeaddr_sink))
    {
      /* RX a frame from Node. */
      PRINTF("Node=%s\r\n", packetbuf_dataptr());
    }
  #else
    if (rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER), network_GetRxRimeAddr()))
    {
      if (packetbuf_datalen() >= 1) /* Is data */
      {
          /* Deliver this data frame to Application-Process. */
          nodeawake_RxData();
      }
      else /* Is Awake Tone */
      {
          iResult = NETSTACK_RADIO.send(network_GetRxRimeAddr(), sizeof(rimeaddr_t)); /* Send Awake Ack */
          if (RADIO_TX_OK != iResult)
          {
              nop(); /* For debug */
          }

          NETSTACK_RADIO.on(); /* Listen for data frame. */
          nodeawake_RxAwakeTone();
      }
    }
    else
    {
      /* Frame is not to us */
      nodeawake_RxNot2Us();	  
    }
  #endif
  }
  
#if 0
  if(NETSTACK_FRAMER.parse() >= 0) {
    hdr = packetbuf_dataptr();
	
    if(hdr->type == TYPE_STROBE) {
      someone_is_sending = 2;

      if(rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                                &rimeaddr_node_addr)) {
	  /* This is a strobe packet for us. */

         /* If the sender address is someone else, we should
	       acknowledge the strobe and wait for the packet. By using
	       the same address as both sender and receiver, we flag the
	       message is a strobe ack. */
        hdr->type = TYPE_STROBE_ACK;
        packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, &rimeaddr_node_addr);
        packetbuf_compact();
        if(NETSTACK_FRAMER.create() >= 0) {
          NETSTACK_RADIO.send(packetbuf_hdrptr(), packetbuf_totlen());
          /* We turn on the radio in anticipation of the incoming packet. */
          on(); /* For flag of turn on radio */
          NETSTACK_RADIO.on(); /* Insure the radio is turned on */
          someone_is_sending = 1;
          waiting_for_packet = 1;
          PRINTF("cxmac: send strobe ack %u, time=%u\r\n", packetbuf_totlen(), clock_time());
        } else {
          PRINTDEBUG("cxmac: failed to send strobe ack\r\n");
        }
      }else if(rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                                       &rimeaddr_null)) {
        /* If the receiver address is null, the strobe is sent to
            prepare for an incoming broadcast packet. If this is the
            case, we turn on the radio and wait for the incoming
            broadcast packet. */
        waiting_for_packet = 1;
        on();
      } else {
        PRINTDEBUG("cxmac: strobe not for us\r\n");
      }

      /* We are done processing the strobe and we therefore return to the caller. */
      return;
#if CXMAC_CONF_ANNOUNCEMENTS
    } else if(hdr->type == TYPE_ANNOUNCEMENT) {
      packetbuf_hdrreduce(sizeof(struct cxmac_hdr));
      parse_announcements(packetbuf_addr(PACKETBUF_ADDR_SENDER));
#endif /* CXMAC_CONF_ANNOUNCEMENTS */
    } else if(hdr->type == TYPE_STROBE_ACK) {
      PRINTDEBUG("cxmac: stray strobe ack\r\n");
    } else {
      someone_is_sending = 0;
      if(rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                                &rimeaddr_node_addr) ||
         rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                                &rimeaddr_null)) {
         /* This is a regular packet that is destined to us or to the broadcast address. */

         /* We have received the final packet, so we can go back to being asleep. */
         off();
      #if CXMAC_CONF_COMPOWER
        /* Accumulate the power consumption for the packet reception. */
        compower_accumulate(&current_packet);
        /* Convert the accumulated power consumption for the received
	     packet to packet attributes so that the higher levels can
	     keep track of the amount of energy spent on receiving the
	     packet. */
        compower_attrconv(&current_packet);

        /* Clear the accumulated power consumption so that it is ready for the next packet. */
        compower_clear(&current_packet);
      #endif /* CXMAC_CONF_COMPOWER */

        waiting_for_packet = 0;

        PRINTFF("cxmac: data(%u), time=%u\r\n", packetbuf_datalen(), clock_time());
        NETSTACK_MAC.input();
        return;
      } else {
        PRINTDEBUG("cxmac: data not for us\r\n");
      }
    }	
  } else {
    PRINTF("cxmac: failed to parse (%u)\r\n", packetbuf_totlen());
  }
#endif
}
/*---------------------------------------------------------------------------*/
#if CXMAC_CONF_ANNOUNCEMENTS
static void
send_announcement(void *ptr)
{
  struct cxmac_hdr *hdr;
  int announcement_len;

  /* Set up the probe header. */
  packetbuf_clear();
  hdr = packetbuf_dataptr();

  announcement_len = format_announcement((char *)hdr +
					 sizeof(struct cxmac_hdr));

  if(announcement_len > 0) {
    packetbuf_set_datalen(sizeof(struct cxmac_hdr) + announcement_len);
    hdr->dispatch = DISPATCH;
    hdr->type = TYPE_ANNOUNCEMENT;

    packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &rimeaddr_node_addr);
    packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, &rimeaddr_null);
    packetbuf_set_attr(PACKETBUF_ATTR_RADIO_TXPOWER, announcement_radio_txpower);
    if(NETSTACK_FRAMER.create() >= 0) {
      NETSTACK_RADIO.send(packetbuf_hdrptr(), packetbuf_totlen());
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
cycle_announcement(void *ptr)
{
  ctimer_set(&announcement_ctimer, ANNOUNCEMENT_TIME,
	     send_announcement, NULL);
  ctimer_set(&announcement_cycle_ctimer, ANNOUNCEMENT_PERIOD,
	     cycle_announcement, NULL);
  if(is_listening > 0) {
    is_listening--;
    /*    printf("is_listening %d\r\n", is_listening);*/
  }
}
/*---------------------------------------------------------------------------*/
static void
listen_callback(int periods)
{
  is_listening = periods + 1;
}
#endif /* CXMAC_CONF_ANNOUNCEMENTS */
/*---------------------------------------------------------------------------*/
void
cxmac_set_announcement_radio_txpower(int txpower)
{
#if CXMAC_CONF_ANNOUNCEMENTS
  announcement_radio_txpower = txpower;
#endif /* CXMAC_CONF_ANNOUNCEMENTS */
}
/*---------------------------------------------------------------------------*/
void
cxmac_init(void)
{
  radio_is_on = 0;
  waiting_for_packet = 0;
  PT_INIT(&pt);
  /*  rtimer_set(&rt, RTIMER_NOW() + cxmac_time.off_time, 1,
      (void (*)(struct rtimer *, void *))powercycle, NULL);*/

  cxmac_is_on = 1;

#if WITH_ENCOUNTER_OPTIMIZATION
  list_init(encounter_list);
  memb_init(&encounter_memb);
#endif /* WITH_ENCOUNTER_OPTIMIZATION */

#if CXMAC_CONF_ANNOUNCEMENTS
  announcement_register_listen_callback(listen_callback);
  ctimer_set(&announcement_cycle_ctimer, ANNOUNCEMENT_TIME,
	     cycle_announcement, NULL);
#endif /* CXMAC_CONF_ANNOUNCEMENTS */

/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
#if (ROLE_SINK == PRODUCT_ROLE)
  on();
#else
  CSCHEDULE_POWERCYCLE(cxmac_time.off_time);
#endif
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
}
/*---------------------------------------------------------------------------*/
static int
turn_on(void)
{
  cxmac_is_on = 1;
  /*  rtimer_set(&rt, RTIMER_NOW() + cxmac_time.off_time, 1,
      (void (*)(struct rtimer *, void *))powercycle, NULL);*/
  CSCHEDULE_POWERCYCLE(cxmac_time.off_time);
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
turn_off(int keep_radio_on)
{
  cxmac_is_on = 0;
  if(keep_radio_on) {
    return NETSTACK_RADIO.on();
  } else {
    return NETSTACK_RADIO.off();
  }
}
/*---------------------------------------------------------------------------*/
static unsigned short
channel_check_interval(void)
{
  return (1ul * CLOCK_SECOND * DEFAULT_PERIOD) / RTIMER_ARCH_SECOND;
}
/*---------------------------------------------------------------------------*/
const struct rdc_driver cxmac_driver =
  {
    "CX-MAC",
    cxmac_init,
    qsend_packet,
    qsend_list,
    input_packet,
    turn_on,
    turn_off,
    channel_check_interval,
  };

/*---------------------------------------------------------------------------*/
void cxmac_set_config(rtimer_clock_t on, rtimer_clock_t off, rtimer_clock_t strobe, rtimer_clock_t strobe_wait)
{
    cxmac_time.on_time = on;
    cxmac_time.off_time = off;
    cxmac_time.strobe_time = strobe;
    cxmac_time.strobe_wait_time = strobe_wait;
	
    return;
}

