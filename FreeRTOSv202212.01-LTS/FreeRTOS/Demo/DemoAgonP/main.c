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


extern unsigned int _heaptop;  // defined in the linker directive file
extern unsigned int _heapbot;  //   "

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
		( void )printf( "&_heaptop = 0x%p\r\n", &_heaptop );
		( void )printf( "&_heapbot = 0x%p\r\n", &_heapbot );
		( void )printf( "Heap size 0x%x\r\n", configTOTAL_HEAP_SIZE );
	}
#	endif

    /* Create the sample tasks. */
    r = xTaskCreate( Task1, "Task1", configMINIMAL_STACK_SIZE, (void *)10, tskIDLE_PRIORITY + 3, NULL );
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
    unsigned int const ticks =( unsigned int )pvParameters;
	char ch = '|';
	int i;

	( void )printf( "\r\nStarting %s : delay ticks = %d\r\n", 
					pcTaskGetName( NULL ), ticks );

    while( 1 )
    {
		/* The ZDSII math library is non-reentrant. Should've read the manual. 
		   You will get random resets if you do a loop using unguarded '%'.
		if( 0 ==( i++ % 80 ))
		   Math library functions must be called within crit_enter and crit_exit */

		for( i = 0; 80 > i; i++ )
		{
			/* when calling standard C library functions that will access MOS,
			   need to perform MOS enter / exit critical region */
			portEnterMOS( );
			putchar( ch );
			portExitMOS( );
			
			vTaskDelay( ticks );
		}

		if( '|' == ch )
		{
			ch = '-';
		}
		else
		{
			ch = '|';
		}
    }
}


#pragma asm "\tSEGMENT TASKS"
void Task2( void *pvParameters )
{
    unsigned int const ticks =( unsigned int )pvParameters;
	char ch = '/';
	int i;

	( void )printf( "\r\nStarting %s : delay ticks = %d\r\n", 
					pcTaskGetName( NULL ), ticks );

    while( 1 )
    {
		for( i = 0; 80 > i; i++ )
		{
			/* when calling standard C library functions that will access MOS,
			   need to perform MOS enter / exit critical region */
			portEnterMOS( );
			putchar( ch );
			portExitMOS( );
			
			vTaskDelay( ticks );
		}

		if( '/' == ch )
		{
			ch = '\\';
		}
		else
		{
			ch = '/';
		}
	}
}


void vApplicationIdleHook( void )
{
	/* DO NOT call a BLOCKING function from within the IDLE task;
       IDLE must always be in either the READY or the RUN state, and no other. */
	
	//Machen mit ein blinken light would be excellent

	idlecnt++;
}
