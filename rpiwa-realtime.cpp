/*

rawMCP3208.c
Public Domain
2021-05-03

gcc -Wall -pthread -o rawMCP3208 rawMCP3208.c -lpigpio

This code shows how to bit bang SPI using DMA.

Using DMA to bit bang allows for two advantages

1) the time of the SPI transaction can be guaranteed to within a
   microsecond or so.

2) multiple devices of the same type can be read or written
  simultaneously.

This code shows how to read more than one MCP3202 at a time.

Each MCP3208 shares the SPI clock, MOSI, and slave select lines but has
a unique MISO line.
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pigpio.h>
#include "RPIWAlib.h"
#include <string.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <signal.h>

#include "cfg_parse.h"


#define SPI_SS 8 // GPIO for slave select.

#define ADCS 1    // Number of connected MCP3208.
#define N_CHANNELS 8

#define BITS 12            // Bits per reading.
#define BX 8               // Bit position of data bit B11.
#define B0 (BX + BITS - 1) // Bit position of data bit B0.

#define MISO 9   // ADC 1 MISO.

#define BUFFER 256       // Generally make this buffer as large as possible.

#define REPEAT_MICROS 40 // Reading every x microseconds.

#define SAMPLES 500000  // Number of samples to take,
//#define SAMPLES 3125  // Number of samples to take,

#define OUTPUT_FIFO "/tmp/RPIWA.fifo"

rawSPI_t rawSPI =
{
   .clk     =  11, // GPIO for SPI clock.
   .mosi    =  10, // GPIO for SPI MOSI.
   .ss_pol  =  1, // Slave select resting level.
   .ss_us   =  1, // Wait 1 micro after asserting slave select.
   .clk_pol =  0, // Clock resting level.
   .clk_pha =  0, // 0 sample on first edge, 1 sample on second edge.
   .clk_us  =  1, // 2 clocks needed per bit so 500 kbps.
};


SignalNode SNCT8;
SignalNode SNCT7;
SignalNode SNCT6;
SignalNode SNCT5;
SignalNode SNCT4;
SignalNode SNCT3;
SignalNode SNCT2;
SignalNode SNCT1;

PowerNode PNV1CT1;
PowerNode PNV1CT2;
PowerNode PNV1CT3;
PowerNode PNV1CT4;
PowerNode PNV1CT5;
PowerNode PNV1CT6;
PowerNode PNV1CT7;

InstantNode INV1;
InstantNode INCT7;
InstantNode INCT6;
InstantNode INCT5;
InstantNode INCT4;
InstantNode INCT3;
InstantNode INCT2;
InstantNode INCT1;

#define OUTPUT_TYPE_RAW 0
#define OUTPUT_TYPE_INSTANT 1
#define OUTPUT_TYPE_CT8 2
#define OUTPUT_TYPE_CT7V1 3

int output_type = OUTPUT_TYPE_RAW;

struct cfg_struct *cfg;
char tmp_str[1024];
float KCAL[8];

static volatile int keepRunning = 1;
void sig_handler(int signo) {

    keepRunning = 0;
}

int main(int argc, char *argv[])
{

   if (signal(SIGINT, sig_handler) == SIG_ERR)
        printf("\ncan't catch SIGINT\n");
   if (signal(SIGTERM, sig_handler) == SIG_ERR)
        printf("\ncan't catch SIGTERM\n");
  
   int i, wid, toffset;
   char buf[2];
   gpioPulse_t final[2];
   char rx[8];
   int sample;
   int val[N_CHANNELS];
   int cb, botCB, topOOL, reading, now_reading, read_delay;
   float cbs_per_reading;
   rawWaveInfo_t rwi;

   int channel;
   int p;
   uint32_t level;
   
   int output_rate = 3125;
   char output_str[128];
   int outstr_len;
   
   mkfifo(OUTPUT_FIFO, 0666);
   int fifod = open(OUTPUT_FIFO, O_WRONLY);
   signal(SIGPIPE, SIG_IGN);
   
   // DEALING WITH CONFIGURATION FILE
   cfg = cfg_init();
   
   if (cfg_load(cfg,"/etc/rpiwa.conf") < 0)
   {
   	fprintf(stderr,"Unable to load rpiwa.conf\n");
   	return -1;
   }
   
   sprintf(tmp_str, cfg_get(cfg,"RunType"));
   output_type = atoi(tmp_str);
   
   sprintf(tmp_str, cfg_get(cfg,"KCAL"));
   char * token = strtok(tmp_str, " ");
   i = 0;
   while (token !=  NULL)
   {
   	KCAL[i] = atof(token);
   	token = strtok(NULL, " ");
   	i++;
   	if (i==8) break;
   }
   
   sprintf(tmp_str, cfg_get(cfg,"OutputRate"));
   output_rate = 1000*atof(tmp_str)/REPEAT_MICROS/N_CHANNELS;
   printf("Output rate every %d samples. Every %.d milliseconds.\n", output_rate, output_rate*REPEAT_MICROS*N_CHANNELS/1000);
   
   // INPUT ARGUMENTS
   if (argc > 1){}

   if (gpioInitialise() < 0) return 1;

   // Need to set GPIO as outputs otherwise wave will have no effect.

   gpioSetMode(rawSPI.clk,  PI_OUTPUT);
   gpioSetMode(rawSPI.mosi, PI_OUTPUT);
   gpioSetMode(SPI_SS,      PI_OUTPUT);

   gpioWaveAddNew(); // Flush any old unused wave data.

   toffset = 0;
   channel = 0;

   /*
   MCP3208 12-bit ADC 8 channels

   1  2  3  4  5  6   7   8  9  10 11 12 13 14 15 16 17 18 19
   SB SD D2 D1 D0 NA NA B11 B10 B9 B8 B7 B6 B5 B4 B3 B2 B1 B0

   SB  1  1
   SD  1  0=differential 1=single
   D2
   D1
   D0
   */

   /*
      Now construct lots of bit banged SPI reads.  Each ADC reading
      will be stored separately.  We need to ensure that the
      buffer is big enough to cope with any reasonable rescehdule.

      In practice make the buffer as big as you can.
   */

   for (i=0; i<BUFFER; i++)
   {
      buf[0] = 0xC0 + (channel<<3); // Start bit, single ended, channel 0.

      rawWaveAddSPI(&rawSPI, toffset, SPI_SS, buf, 5, BX, B0, B0);

      /*
         REPEAT_MICROS must be more than the time taken to
         transmit the SPI message.
      */

      toffset += REPEAT_MICROS;
      if (++channel>=N_CHANNELS) channel=0;
   }

   // Force the same delay after the last reading.

   final[0].gpioOn = 0;
   final[0].gpioOff = 0;
   final[0].usDelay = toffset;

   final[1].gpioOn = 0; // Need a dummy to force the final delay.
   final[1].gpioOff = 0;
   final[1].usDelay = 0;

   gpioWaveAddGeneric(2, final);

   wid = gpioWaveCreate(); // Create the wave from added data.

   if (wid < 0)
   {
      fprintf(stderr, "Can't create wave, %d too many?\n", BUFFER);
      return 1;
   }

   /*
      The wave resources are now assigned,  Get the number
      of control blocks (CBs) so we can calculate which reading
      is current when the program is running.
   */

   rwi = rawWaveInfo(wid);

   printf("# cb %d-%d ool %d-%d del=%d ncb=%d nb=%d nt=%d\n",
      rwi.botCB, rwi.topCB, rwi.botOOL, rwi.topOOL, rwi.deleted,
      rwi.numCB,  rwi.numBOOL,  rwi.numTOOL);

   /*
      CBs are allocated from the bottom up.  As the wave is being
      transmitted the current CB will be between botCB and topCB
      inclusive.
   */

   botCB = rwi.botCB;

   /*
      Assume each reading uses the same number of CBs (which is
      true in this particular example).
   */

   cbs_per_reading = (float)rwi.numCB / (float)BUFFER;

   printf("# cbs=%d per read=%.1f base=%d\n",
      rwi.numCB, cbs_per_reading, botCB);

   /*
      OOL are allocated from the top down. There are BITS bits
      for each ADC reading and BUFFER ADC readings.  The readings
      will be stored in topOOL - 1 to topOOL - (BITS * BUFFER).
   */

   topOOL = rwi.topOOL;

   printf("starting dma.\n");
   fflush(stdout);

   gpioWaveTxSend(wid, PI_WAVE_MODE_REPEAT);

   reading = 0;
   sample = 0;

   channel = 0;
   
   //float KCAL[8] = {200.43, 74.074, 74.074, 74.074, 74.074, 74.074, 74.074, 74.074};
   
   if (output_type == OUTPUT_TYPE_CT8)
   {
   	SNCT8.begin(0, KCAL[0]);
   	SNCT7.begin(1, KCAL[1]);
   	SNCT6.begin(2, KCAL[2]);
   	SNCT5.begin(3, KCAL[3]);
   	SNCT4.begin(4, KCAL[4]);
   	SNCT3.begin(5, KCAL[5]);
   	SNCT2.begin(6, KCAL[6]);
   	SNCT1.begin(7, KCAL[7]);
   }
   else if (output_type == OUTPUT_TYPE_CT7V1)
   {
   	PNV1CT1.begin(7, KCAL[7], 0, KCAL[0]);
   	PNV1CT2.begin(6, KCAL[6], 0, KCAL[0]);
   	PNV1CT3.begin(5, KCAL[5], 0, KCAL[0]);
   	PNV1CT4.begin(4, KCAL[4], 0, KCAL[0]);
   	PNV1CT5.begin(3, KCAL[3], 0, KCAL[0]);
   	PNV1CT6.begin(2, KCAL[2], 0, KCAL[0]);
   	PNV1CT7.begin(1, KCAL[1], 0, KCAL[0]);
   }
   else if (output_type == OUTPUT_TYPE_INSTANT)
   {   
   	INV1.begin(KCAL[0]);
   	INCT7.begin(KCAL[1]);
   	INCT6.begin(KCAL[2]);
   	INCT5.begin(KCAL[3]);
   	INCT4.begin(KCAL[4]);
   	INCT3.begin(KCAL[5]);
   	INCT2.begin(KCAL[6]);
   	INCT1.begin(KCAL[7]);
   }

   // while (sample<SAMPLES)
   while (keepRunning)
   {
      // Which reading is current?

      cb = rawWaveCB() - botCB;

      now_reading = (float) cb / cbs_per_reading;

      if (now_reading >= reading){
	      read_delay = now_reading - reading;
      }
      else {
	      read_delay = BUFFER + now_reading - reading;
      }

      if (read_delay > BUFFER/2){
      	fprintf(stderr, "Can't keep up with reading speed. Read delay is %d. =%d-%d\n", read_delay, now_reading, reading);
      	//break;
      }
      //printf("\n%d Read delay: %d. =%d-%d", sample, read_delay,  now_reading, reading);

      // Loop gettting the fresh readings.
      while (now_reading != reading)
      {
      	
         /*
            Each reading uses BITS OOL.  The position of this readings
            OOL are calculated relative to the waves top OOL.
         */
	p = topOOL - ((reading%BUFFER)*BITS) - 1;

	for (i=0; i<BITS; i++)
   	{
      		level = rawWaveGetOut(p);
      		putBitInBytes(i, rx, level & (1<<MISO));
      		p--;
	}

        //   7   6  5  4  3  2  1  0 |  7  6  5  4  3  2  1  0
        // B11 B10 B9 B8 B7 B6 B5 B4 | B3 B2 B1 B0  X  X  X  X

        val[channel] = (rx[0]<<4) + (rx[1]>>4);
	

      if (++channel>=N_CHANNELS)
      {
      
      	if (output_type == OUTPUT_TYPE_CT8){
      		SNCT8.updateRMS(val[0]);
		SNCT7.updateRMS(val[1]);
		SNCT6.updateRMS(val[2]);
		SNCT5.updateRMS(val[3]);
		SNCT4.updateRMS(val[4]);
		SNCT3.updateRMS(val[5]);
		SNCT2.updateRMS(val[6]);
		SNCT1.updateRMS(val[7]);

		if (SNCT8.warm && (sample % output_rate == 0))
		{	 
			outstr_len = sprintf(output_str, "%d %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",\
							sample, SNCT1.RMS, SNCT2.RMS, SNCT3.RMS, SNCT4.RMS, SNCT5.RMS, SNCT6.RMS, SNCT7.RMS, SNCT8.RMS);	 
			write(fifod, output_str, outstr_len);
		}
      	}
      	else if (output_type == OUTPUT_TYPE_RAW){
      		outstr_len = sprintf(output_str, "%d %d %d %d %d %d %d %d %d\n",\
						 sample, val[0], val[1], val[2],\
						 val[3], val[4], val[5], val[6], val[7]);
		write(fifod, output_str, outstr_len);
      	}
      	else if (output_type == OUTPUT_TYPE_INSTANT){
      		
		INV1.convert(val[0]);
		INCT7.convert(val[1]);
		INCT6.convert(val[2]);
		INCT5.convert(val[3]);
		INCT4.convert(val[4]);
		INCT3.convert(val[5]);
		INCT2.convert(val[6]);
		INCT1.convert(val[7]);
		
		
		outstr_len = sprintf(output_str, "%d %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",\
						 sample, INV1.value, INCT7.value, INCT6.value,\
						 INCT5.value, INCT4.value, INCT3.value,\
						 INCT2.value, INCT1.value);
		write(fifod, output_str, outstr_len);
      	}
	else if (output_type == OUTPUT_TYPE_CT7V1){
		PNV1CT1.updateRMS(val[7], val[0]);
		PNV1CT2.updateRMS(val[6], val[0]);
		PNV1CT3.updateRMS(val[5], val[0]);
		PNV1CT4.updateRMS(val[4], val[0]);
		PNV1CT5.updateRMS(val[3], val[0]);
		PNV1CT6.updateRMS(val[2], val[0]);
		PNV1CT7.updateRMS(val[1], val[0]);
		if (PNV1CT1.warm && (sample % output_rate == 0))
		{
			outstr_len = sprintf(output_str, "%d %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", sample,\
						 PNV1CT1.Vrms,\
						 PNV1CT1.Irms, PNV1CT2.Irms, PNV1CT3.Irms, PNV1CT4.Irms, PNV1CT5.Irms, PNV1CT6.Irms, PNV1CT7.Irms,\
						 PNV1CT1.P, PNV1CT2.P, PNV1CT3.P, PNV1CT4.P, PNV1CT5.P, PNV1CT6.P, PNV1CT7.P);
			write(fifod, output_str, outstr_len);
		}
	}
	
	
	
	
	sample+=1;
	channel=0;
      }

         if (++reading >= BUFFER) reading = 0;
      }
      usleep(100);
      if (!keepRunning) break;
   }
   
   printf("ended cleanly.\n");
   close(fifod);
   gpioTerminate();

   return 0;
}

