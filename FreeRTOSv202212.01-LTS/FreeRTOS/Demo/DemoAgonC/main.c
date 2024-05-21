/*
 * FreeRTOS Kernel V10.5.1
 * Copyright (C) 2024 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 *  Please ensure to read the configuration and relevant port sections of the
 *  online documentation.
 *
 *  http://www.FreeRTOS.org - Documentation, latest information, license and
 *  contact details.
 *
 *  Demo application for EZ80 Agon Light
*/
#pragma asm "\tDEFINE TASKS, SPACE = RAM, ALIGN = 10000h"


#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"


extern unsigned int _heaptop;  // defined in the linker directive file
extern unsigned int _heapbot;  //   "


	/* Fudge factor to delay rate of putch's
	   Without slowing down the rate of putchars we get lockups or system 
	   resets. It doesn't appear to be a corrupt stack - but check further. 
	   
	   As putch invokes soft reset into MOS which blocks, we only return to 
	   the app after the character has been transimitted over UART0 to VDP.
	   Is it the ESP32 VDP Terminal Processor not keeping up; running out of
	   buffer space?? 
		  UART0 initialised in github.com/breakintoprogram/agon-mos/blob/main/main.c
	      to either 1152000 or if that fails then downto 384000 bps. UART0 CTS 
		  flow control in github.com/breakintoprogram/agon-mos/blob/main/src/serial.asm
  	      384000 / 8 (+2 stop bits) = 38,400 Bps.
	   With FUDGE < 200 we see corruption in the output - looks like letter 'Y' 
	   is displayed. If we see these are we at risk of a system reset??
	   Use a FUDGE factor + margin delay, such that no corruption is seen */
#define FUDGE	250


static unsigned int idlecnt = 0;

void Task1( void *pvParameters );
void Task2( void *pvParameters );
void vApplicationIdleHook( void );


int main( void )
{
	BaseType_t r;
	int tskcnt = 2;

#	if defined( _DEBUG )
	{
		( void )printf( "main %d\r\n", __LINE__ );
		( void )printf( "&_heaptop = 0x%p\r\n", &_heaptop );
		( void )printf( "&_heapbot = 0x%p\r\n", &_heapbot );
		( void )printf( "Heap size 0x%x\r\n", configTOTAL_HEAP_SIZE );
	}
#	endif

    /* Create the tasks */
    r = xTaskCreate( Task1, "Task1", configMINIMAL_STACK_SIZE, (void *)10, tskIDLE_PRIORITY + 2, NULL );
	if( pdPASS != r )
	{
		tskcnt--;
		( void )printf( "Failed to allocate Task1: %d\r\n", r );
	}

	r = xTaskCreate( Task2, "Task2", configMINIMAL_STACK_SIZE, (void *)3,  tskIDLE_PRIORITY + 2, NULL );
	if( pdPASS != r )
	{
		tskcnt--;
		( void )printf( "Failed to allocate Task2: %d\r\n", r );
	}

	if( 2 == tskcnt )
	{
		/* Start FreeRTOS */
		( void )printf( "\r\nEntering scheduler\r\n" );
			
		vTaskStartScheduler( );
		
		// should never get here
		( void )printf( "Back from scheduler\r\n" );
	}

    return( 0 );
}


#pragma asm "\tSEGMENT TASKS"
void Task1( void *pvParameters )
{
    unsigned int const p =( unsigned int )pvParameters;
	unsigned short int cnt = 0;
	char ch = '-';
	int i;

	( void )printf( "\r\nStarting %s\r\n", pcTaskGetName( NULL ));
    while( 1 )
    {
		if( 0 ==( cnt++ % 80 ))
		{
			cnt = 1;

			if( '\\' == ch )
				ch = '-';
			else
				ch = '\\';
			
			portYIELD( );
		}

		for( i = 0; FUDGE > i; i++ );  // slow the rate of putchars
		portEnterMOS( );
        putchar( ch );
		portExitMOS( );
    }
	
	( void )p;
}


#pragma asm "\tSEGMENT TASKS"
void Task2( void *pvParameters )
{
    unsigned int const p =( unsigned int )pvParameters;
	unsigned short cnt = 0;
	char ch = '|';
	int i;

	( void )printf( "\r\nStarting %s\r\n", pcTaskGetName( NULL ));
    while( 1 )
    {
		if( 0 ==( cnt++ % 80 ))
		{
			cnt = 1;

			if( '/' == ch )
				ch = '|';
			else
				ch = '/';
			
			portYIELD( );
		}

		for( i = 0; FUDGE > i; i++ );  // slow the rate of putchars
		portEnterMOS( );
        putchar( ch );
		portExitMOS( );
    }
	
	( void )p;
}


void vApplicationIdleHook( void )
{
	/* DO NOT call a BLOCKING function from within the IDLE task;
       IDLE must always be in either the READY or the RUN state, and no other. */
	
	//Machen mit ein blinken light would be excellent

	idlecnt++;
}
