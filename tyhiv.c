/*
 * Copyright (c) 2016, CEA IoT.
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
 *
 */



#include "contiki.h"
#include "lib/random.h"
#include "net/rime.h"
#include "net/rime/collect.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"
#include "net/netstack.h"
#include <stdio.h>
#include <math.h>


#if defined (PLATFORM_HAS_LIGHT)
#include "dev/light-sensor.h"
#endif
#if defined (PLATFORM_HAS_BATT)
#include "dev/battery-sensor.h"
#endif
#if defined (PLATFORM_HAS_SHT11)
#include "dev/sht11-sensor.h"
#endif
#if defined (PLATFORM_HAS_LEDS)
#include "dev/leds.h"
#endif

#define Settleing_time 50
#define Sending_interval 120 

static struct collect_conn tc;

/*---------------------------------------------------------------------------*/
PROCESS(example_collect_process, "Test collect process");
AUTOSTART_PROCESSES(&example_collect_process);
/*---------------------------------------------------------------------------*/
static void
recv(const rimeaddr_t *originator, uint8_t seqno, uint8_t hops)
{
  printf("S%d%s\n",
	 originator->u8[0],
	 (char *)packetbuf_dataptr());
}
/*---------------------------------------------------------------------------*/
static const struct collect_callbacks callbacks = { recv };
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_collect_process, ev, data)
{
  static struct etimer periodic;
  static struct etimer et;
      static int val,Sec=0;
      static struct sensors_sensor *sensor;
      static float s = 0;
      static int dec, Temp, Lum, Hum;
      static float frac, Tfrac,Lfrac,Hfrac;
	

  
  PROCESS_BEGIN();

  collect_open(&tc, 130, COLLECT_ROUTER, &callbacks);

  if(rimeaddr_node_addr.u8[0] == 1 &&
     rimeaddr_node_addr.u8[1] == 0) {
	//printf("I am sink\n");
	collect_set_sink(&tc, 1);
  }

  /* Allow some time for the network to settle. */
  etimer_set(&et, Settleing_time * CLOCK_SECOND);
  PROCESS_WAIT_UNTIL(etimer_expired(&et));

  while(1) {

    /* Send a packet every 30 seconds. */

    if(etimer_expired(&periodic)) {
	if(Sec==1) {
     	  etimer_set(&periodic, CLOCK_SECOND * 1);
      	  etimer_set(&et, random_rand() % (CLOCK_SECOND * 5));
      	  Sec=0;}
	else 
	if(Sec==0) {
     	  etimer_set(&periodic, CLOCK_SECOND * 60*Sending_interval);
      	  etimer_set(&et, random_rand() % (CLOCK_SECOND * 60*Sending_interval));
      	  Sec=1;}
    }

    PROCESS_WAIT_EVENT();


    if(etimer_expired(&et)) {
      static rimeaddr_t oldparent;
      const rimeaddr_t *parent;
      SENSORS_ACTIVATE(light_sensor);
      SENSORS_ACTIVATE(sht11_sensor);
           val = sht11_sensor.value(SHT11_SENSOR_TEMP);
      	   if(val != -1) 
      	   {
		s= ((0.01*val) - 39.60);
      	  	dec = s;
		Temp=dec;
      	  	frac = s - dec;
		Tfrac=frac;
      	  	//printf("\nTemperature=%d.%02u C (%d)\n", dec, (unsigned int)(frac * 100),val);               
           }

           val = light_sensor.value(LIGHT_SENSOR_TOTAL_SOLAR);
      	   if(val != -1) 
      	   {
      		s = (float)(val * 0.4071);
      	  	dec = s;
		Lum=dec;
      	  	frac = s - dec;
		Lfrac=frac;
      	  //printf("Light=%d.%02u lux (%d)\n", dec, (unsigned int)(frac * 100),val);               
           } 
	   val=sht11_sensor.value(SHT11_SENSOR_HUMIDITY);
	   if(val != -1) 
      	   {
		s= (((0.0405*val) - 4) + ((-2.8 * 0.000001)*(pow(val,2))));  
      	  	dec = s;
		Hum=dec;
      	  	frac = s - dec;
		Hfrac=frac;
      	  	//printf("Humidity=%d.%02u % (%d)\n", dec, (unsigned int)(frac * 100),val);               
           }

    	   SENSORS_DEACTIVATE(light_sensor);
    	   SENSORS_DEACTIVATE(sht11_sensor);


      packetbuf_clear();
      packetbuf_set_datalen(sprintf(packetbuf_dataptr(),"T%d.%0uL%d.%02uH%d.%02uI%d", Temp, (unsigned int)(Tfrac * 100),Lum, (unsigned int)(Lfrac * 100),Hum, (unsigned int)(Hfrac * 100),Sec) + 1);
      collect_send(&tc, 15);

      parent = collect_parent(&tc);
      if(!rimeaddr_cmp(parent, &oldparent)) {
        if(!rimeaddr_cmp(&oldparent, &rimeaddr_null)) {
          printf("#L %d 0\n", oldparent.u8[0]);
        }
        if(!rimeaddr_cmp(parent, &rimeaddr_null)) {
          printf("#L %d 1\n", parent->u8[0]);
        }
        rimeaddr_copy(&oldparent, parent);
      }
    }

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
